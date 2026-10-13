"""Testing-GUI panel for the full impact SEQUENCE (pre-retract +
conditioning pulse, Current Profile impact, dwell, retract, two-segment
recording) and for profile files.

Kept out of app.py on purpose: everything here talks to the app only
through a small, explicit surface (the shared Current Profile / Motion
PID / Rest PID / Amp StringVars, the status line, the controller and
_show_capture), and the whole Arm/Fire path goes through
sequence/impact_sequence.py -- the same code the user GUI runs. So the
testing GUI can keep evolving its own motion fields while this panel,
and the profile files it writes, stay the single integration point.

Also provides the LVDT conditioning buttons (handoff notes section 4)
for the LVDT Calibration section.
"""

import datetime
import tkinter as tk
from pathlib import Path
from tkinter import filedialog

import gclib

from analysis.capture_analysis import build_sequence_figure
from analysis.capture_io import capture_document, save_capture
from analysis.sequence_metrics import format_metrics, impact_metrics
from controller.controller import (
    Controller,
    FULL_STROKE_MM,
    TORQUE_VOLTS_FULL_SCALE,
)
from gui.figure_popup import show_figure
from program.conditioning_program import (
    MODE1_PULSE_CURRENT_A,
    MODE1_PULSE_MS,
    MODE1_PULSE_REPEATS,
    MODE2_PULSE_CURRENT_A,
    MODE2_PULSE_MS,
    build_conditioning_program,
)
from recording.recording_sequence import wait_for_motion_complete, wait_for_program_complete
from sequence.impact_sequence import (
    compile_sequence,
    fetch_sequence_recording,
    info_text,
    prepare_controller,
)
from sequence.profile import (
    DWELL_MAX_MS,
    DWELL_MIN_MS,
    PROFILE_SUFFIX,
    default_profile,
    load_profile,
    profile_impact_position_mm,
    save_profile,
)

PROFILES_DIR = Path(__file__).resolve().parent.parent.parent / "profiles"


def _labeled_entry(row, label, var, width=8):
    tk.Label(row, text=label).pack(side="left")
    entry = tk.Entry(row, textvariable=var, width=width)
    entry.pack(side="left", padx=(2, 8))
    return entry


class SequencePanel:
    """Builds the "Impact Sequence" LabelFrame inside `parent` and owns
    its state. `app` is the testing GUI App (see module docstring for
    what is used)."""

    def __init__(self, app, parent):
        self.app = app
        self.controller = app.controller
        self.root = app.root
        self.status_var = app.status_var
        self._armed = None
        self._connected = False
        self._servo_enabled = False
        self._busy = False

        d = default_profile()
        seq = d["sequence"]
        self.name_var = tk.StringVar(value="")
        self.tip_var = tk.StringVar(value="19 mm")
        self.notes_var = tk.StringVar(value="")
        c = seq["condition"]
        self.condition_enabled_var = tk.BooleanVar(value=c["enabled"])
        self.condition_current_var = tk.StringVar(value=str(c["current_a"]))
        self.condition_ms_var = tk.StringVar(value=str(c["duration_ms"]))
        self.condition_settle_var = tk.StringVar(value=str(c["settle_ms"]))
        self.condition_position_var = tk.StringVar(value=str(c["position_mm"]))
        dw = seq["dwell"]
        self.dwell_ms_var = tk.StringVar(value=str(dw["duration_ms"]))
        self.hold_current_var = tk.StringVar(value=str(dw["hold_current_a"]))
        self.hold_servo_var = tk.BooleanVar(value=dw["hold_servo"])
        self.fast_tail_var = tk.StringVar(value=str(dw["fast_tail_ms"]))
        r = seq["retract"]
        self.kick_current_var = tk.StringVar(value=str(r["kick_current_a"]))
        self.kick_ms_var = tk.StringVar(value=str(r["kick_ms"]))
        self.retract_kp_var = tk.StringVar(value=str(r["pid"]["kp"]))
        self.retract_kd_var = tk.StringVar(value=str(r["pid"]["kd"]))
        self.retract_ki_var = tk.StringVar(value=str(r["pid"]["ki"]))
        self.retract_offset_var = tk.StringVar(value=str(r["pid"]["offset_v"]))
        self.retract_speed_var = tk.StringVar(value=str(r["speed_m_s"]))
        self.retract_accel_var = tk.StringVar(value=str(r["accel_m_s2"]))
        self.retract_decel_var = tk.StringVar(value=str(r["decel_m_s2"]))
        self.retract_target_var = tk.StringVar(value=str(r["target_mm"]))
        self.circular_var = tk.BooleanVar(value=seq["recording"]["circular"])
        self.disable_at_end_var = tk.BooleanVar(value=seq["disable_at_end"])
        self.plan_var = tk.StringVar(value="")

        self._build(parent)

    # ------------------------------------------------------------ widgets
    def _build(self, parent):
        frame = tk.LabelFrame(parent, text="Impact Sequence (Current Profile + conditioning/dwell/retract)")
        frame.pack(padx=10, pady=(0, 10), fill="x")
        self.frame = frame

        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        _labeled_entry(row, "Profile Name:", self.name_var, 18)
        _labeled_entry(row, "Tip:", self.tip_var, 8)
        _labeled_entry(row, "Notes:", self.notes_var, 28)

        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        tk.Checkbutton(row, text="LVDT condition pulse", variable=self.condition_enabled_var).pack(side="left")
        _labeled_entry(row, "Current (A, +=into retract stop):", self.condition_current_var, 6)
        _labeled_entry(row, "Duration (ms):", self.condition_ms_var, 6)
        _labeled_entry(row, "Settle (ms):", self.condition_settle_var, 6)
        _labeled_entry(row, "At (mm):", self.condition_position_var, 6)

        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        _labeled_entry(row, f"Dwell (ms) [{DWELL_MIN_MS:g}-{DWELL_MAX_MS:g}]:", self.dwell_ms_var, 7)
        _labeled_entry(row, "Hold Current (A, +=into extend stop):", self.hold_current_var, 6)
        tk.Checkbutton(row, text="Keep position loop during dwell", variable=self.hold_servo_var).pack(side="left", padx=(0, 8))
        _labeled_entry(row, "Fast Capture Tail (ms):", self.fast_tail_var, 6)

        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        _labeled_entry(row, "Retract Kick (A, +=toward retract):", self.kick_current_var, 6)
        _labeled_entry(row, "Kick (ms):", self.kick_ms_var, 6)
        _labeled_entry(row, "Retract Speed (m/s):", self.retract_speed_var, 7)
        _labeled_entry(row, "Accel (m/s²):", self.retract_accel_var, 7)
        _labeled_entry(row, "Decel (m/s²):", self.retract_decel_var, 7)
        _labeled_entry(row, "Target (mm):", self.retract_target_var, 6)

        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        _labeled_entry(row, "Retract KP:", self.retract_kp_var, 6)
        _labeled_entry(row, "KD:", self.retract_kd_var, 6)
        _labeled_entry(row, "KI:", self.retract_ki_var, 6)
        _labeled_entry(row, "Offset (V):", self.retract_offset_var, 6)
        tk.Checkbutton(row, text="Rolling (circular) fast capture", variable=self.circular_var).pack(side="left", padx=(0, 8))
        tk.Checkbutton(row, text="Disable servo at end", variable=self.disable_at_end_var).pack(side="left")

        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        self.plan_button = tk.Button(row, text="Check Plan", command=self._on_check_plan)
        self.plan_button.pack(side="left", padx=(0, 8))
        self.arm_button = tk.Button(row, text="Arm Sequence", command=self._on_arm, state="disabled")
        self.arm_button.pack(side="left", padx=(0, 8))
        self.fire_button = tk.Button(row, text="Fire Sequence", command=self._on_fire, state="disabled")
        self.fire_button.pack(side="left", padx=(0, 16))
        tk.Button(row, text="Save Profile...", command=self._on_save_profile).pack(side="left", padx=(0, 8))
        tk.Button(row, text="Load Profile...", command=self._on_load_profile).pack(side="left")

        tk.Label(
            frame, textvariable=self.plan_var, anchor="w", justify="left",
            font=("TkFixedFont", 8),
        ).pack(fill="x", pady=(2, 0))

    # ------------------------------------------------------------ gating
    def set_connected(self, connected: bool):
        self._connected = connected
        if not connected:
            self._servo_enabled = False
            self._armed = None
        self._refresh_buttons()

    def set_servo_enabled(self, enabled: bool):
        self._servo_enabled = enabled
        if not enabled:
            # Same rule as every other mode's Fire: it only ever comes back
            # through a fresh, successful Arm.
            self._armed = None
        self._refresh_buttons()

    def on_disable(self):
        """Disable pressed -- an armed sequence is invalidated."""
        self._armed = None
        self._refresh_buttons()

    def _refresh_buttons(self):
        can_arm = self._connected and self._servo_enabled and not self._busy
        self.arm_button.config(state="normal" if can_arm else "disabled")
        can_fire = can_arm and self._armed is not None
        self.fire_button.config(state="normal" if can_fire else "disabled")

    # ------------------------------------------------------------ profile
    def _float(self, var, name):
        try:
            return float(var.get())
        except ValueError:
            raise ValueError(f"{name} must be a number")

    def profile_from_fields(self) -> dict:
        """Builds the profile dict from the app's Current Profile / PID /
        Amp fields plus this panel's own. Raises ValueError naming the
        first bad field."""
        app = self.app
        f = self._float
        p = default_profile()
        p["name"] = self.name_var.get().strip()
        p["tip"] = self.tip_var.get().strip()
        p["notes"] = self.notes_var.get().strip()
        p["impact_velocity_m_s"] = f(app.current_profile_velocity_var, "Impact Velocity")
        m = p["motion"]
        m["start_mm"] = f(app.current_profile_start_var, "Start")
        m["stop_mm"] = f(app.current_profile_stop_var, "Stop")
        m["impact_position_mm"] = f(app.current_profile_position_var, "Impact Position")
        p["impact_depth_mm"] = round(m["stop_mm"] - m["impact_position_mm"], 4)
        m["peak_velocity_position_mm"] = f(app.current_profile_peak_velocity_position_var, "Peak Velocity Position")
        m["mass_kg"] = f(app.current_profile_mass_var, "Mass")
        m["kf_accel"] = f(app.current_profile_kf_accel_var, "Force Constant Accel")
        m["kf_decel"] = f(app.current_profile_kf_decel_var, "Force Constant Decel")
        m["ff_stretch"] = {
            "accel_rise": f(app.current_profile_ff_accel_rise_stretch_var, "Accel Rise Stretch"),
            "accel_fall": f(app.current_profile_ff_accel_fall_stretch_var, "Accel Fall Stretch"),
            "decel_rise": f(app.current_profile_ff_decel_rise_stretch_var, "Decel Rise Stretch"),
            "decel_fall": f(app.current_profile_ff_decel_fall_stretch_var, "Decel Fall Stretch"),
        }
        m["ff_advance_us"] = f(app.current_profile_ff_advance_us_var, "Feedforward Advance")
        m["pre_pause_ms"] = f(app.current_profile_pre_pause_ms_var, "Pre-Move Pause")
        m["post_pause_ms"] = f(app.current_profile_post_pause_ms_var, "Post-Move Pause")
        m["motion_pid"] = {
            "kp": f(app.motion_kp_var, "Motion KP"), "kd": f(app.motion_kd_var, "Motion KD"),
            "ki": f(app.motion_ki_var, "Motion KI"), "offset_v": f(app.motion_offset_var, "Motion Offset"),
        }
        m["rest_pid"] = {
            "kp": f(app.rest_kp_var, "Rest KP"), "kd": f(app.rest_kd_var, "Rest KD"),
            "ki": f(app.rest_ki_var, "Rest KI"), "offset_v": f(app.rest_offset_var, "Rest Offset"),
        }
        s = p["sequence"]
        s["condition"] = {
            "enabled": bool(self.condition_enabled_var.get()),
            "current_a": f(self.condition_current_var, "Condition current"),
            "duration_ms": f(self.condition_ms_var, "Condition duration"),
            "settle_ms": f(self.condition_settle_var, "Condition settle"),
            "position_mm": f(self.condition_position_var, "Condition position"),
        }
        s["dwell"] = {
            "duration_ms": f(self.dwell_ms_var, "Dwell"),
            "hold_current_a": f(self.hold_current_var, "Hold Current"),
            "hold_servo": bool(self.hold_servo_var.get()),
            "fast_tail_ms": f(self.fast_tail_var, "Fast Capture Tail"),
        }
        s["retract"] = {
            "kick_current_a": f(self.kick_current_var, "Retract Kick"),
            "kick_ms": f(self.kick_ms_var, "Kick duration"),
            "pid": {
                "kp": f(self.retract_kp_var, "Retract KP"), "kd": f(self.retract_kd_var, "Retract KD"),
                "ki": f(self.retract_ki_var, "Retract KI"), "offset_v": f(self.retract_offset_var, "Retract Offset"),
            },
            "speed_m_s": f(self.retract_speed_var, "Retract Speed"),
            "accel_m_s2": f(self.retract_accel_var, "Retract Accel"),
            "decel_m_s2": f(self.retract_decel_var, "Retract Decel"),
            "target_mm": f(self.retract_target_var, "Retract Target"),
        }
        s["recording"] = {"circular": bool(self.circular_var.get())}
        s["disable_at_end"] = bool(self.disable_at_end_var.get())
        p["amp"]["full_scale_current_a"] = f(app.amp_full_scale_current_var, "Amp Full-Scale Current")
        return p

    def apply_profile_to_fields(self, p: dict):
        """Inverse of profile_from_fields: fills the app's shared fields
        and this panel's from `p` (a complete profile)."""
        app = self.app
        m, s = p["motion"], p["sequence"]
        self.name_var.set(p.get("name") or "")
        self.tip_var.set(p.get("tip") or "")
        self.notes_var.set(p.get("notes") or "")
        app.current_profile_velocity_var.set(str(p["impact_velocity_m_s"]))
        app.current_profile_start_var.set(str(m["start_mm"]))
        app.current_profile_stop_var.set(str(m["stop_mm"]))
        app.current_profile_position_var.set(str(profile_impact_position_mm(p)))
        app.current_profile_peak_velocity_position_var.set(str(m["peak_velocity_position_mm"]))
        app.current_profile_mass_var.set(str(m["mass_kg"]))
        app.current_profile_kf_accel_var.set(str(m["kf_accel"]))
        app.current_profile_kf_decel_var.set(str(m["kf_decel"]))
        st = m["ff_stretch"]
        app.current_profile_ff_accel_rise_stretch_var.set(str(st["accel_rise"]))
        app.current_profile_ff_accel_fall_stretch_var.set(str(st["accel_fall"]))
        app.current_profile_ff_decel_rise_stretch_var.set(str(st["decel_rise"]))
        app.current_profile_ff_decel_fall_stretch_var.set(str(st["decel_fall"]))
        app.current_profile_ff_advance_us_var.set(str(m["ff_advance_us"]))
        app.current_profile_pre_pause_ms_var.set(str(int(round(float(m["pre_pause_ms"])))))
        app.current_profile_post_pause_ms_var.set(str(int(round(float(m["post_pause_ms"])))))
        for key, var in (("kp", app.motion_kp_var), ("kd", app.motion_kd_var),
                         ("ki", app.motion_ki_var), ("offset_v", app.motion_offset_var)):
            var.set(str(m["motion_pid"][key]))
        for key, var in (("kp", app.rest_kp_var), ("kd", app.rest_kd_var),
                         ("ki", app.rest_ki_var), ("offset_v", app.rest_offset_var)):
            var.set(str(m["rest_pid"][key]))
        c = s["condition"]
        self.condition_enabled_var.set(bool(c["enabled"]))
        self.condition_current_var.set(str(c["current_a"]))
        self.condition_ms_var.set(str(c["duration_ms"]))
        self.condition_settle_var.set(str(c["settle_ms"]))
        self.condition_position_var.set(str(c["position_mm"]))
        d = s["dwell"]
        self.dwell_ms_var.set(str(d["duration_ms"]))
        self.hold_current_var.set(str(d["hold_current_a"]))
        self.hold_servo_var.set(bool(d["hold_servo"]))
        self.fast_tail_var.set(str(d["fast_tail_ms"]))
        r = s["retract"]
        self.kick_current_var.set(str(r["kick_current_a"]))
        self.kick_ms_var.set(str(r["kick_ms"]))
        self.retract_kp_var.set(str(r["pid"]["kp"]))
        self.retract_kd_var.set(str(r["pid"]["kd"]))
        self.retract_ki_var.set(str(r["pid"]["ki"]))
        self.retract_offset_var.set(str(r["pid"]["offset_v"]))
        self.retract_speed_var.set(str(r["speed_m_s"]))
        self.retract_accel_var.set(str(r["accel_m_s2"]))
        self.retract_decel_var.set(str(r["decel_m_s2"]))
        self.retract_target_var.set(str(r["target_mm"]))
        self.circular_var.set(bool(s["recording"]["circular"]))
        self.disable_at_end_var.set(bool(s["disable_at_end"]))
        app.amp_full_scale_current_var.set(str(p["amp"]["full_scale_current_a"]))
        # The amp scale is a plain display/conversion setting -- apply it
        # the same way the Apply button does so plots match the profile.
        app._on_apply_amp_gain()

    def _compile_from_fields(self):
        profile = self.profile_from_fields()
        return compile_sequence(profile)

    def _on_check_plan(self):
        try:
            compiled = self._compile_from_fields()
        except ValueError as e:
            self.status_var.set(f"Plan check failed: {e}")
            return
        self._show_plan(compiled)
        self.status_var.set("Plan OK -- see the summary below the sequence buttons")

    def _show_plan(self, compiled):
        layout = compiled.layout
        lines = compiled.program_text.count("\n")
        slow_ms = layout.slow_interval_samples * layout.sample_period_s * 1000
        self.plan_var.set(
            f"Program {lines} lines | fast {layout.fast_samples} samples x4 "
            f"({layout.fast_samples * layout.sample_period_s * 1000:.1f}ms"
            f"{', rolling' if layout.fast_circular else ''}) | "
            f"slow {layout.slow_samples} samples x3 @ {slow_ms:.2f}ms "
            f"({layout.slow_samples * slow_ms / 1000:.2f}s) | FF {layout.ff_samples} | "
            f"{layout.total_elements}/3000 array elements | "
            f"peak v {compiled.peak_velocity_m_s:.3f}m/s | dwell {compiled.dwell_ms:g}ms"
        )

    def _on_save_profile(self):
        try:
            profile = self.profile_from_fields()
            compile_sequence(profile)  # same checks Arm would apply
        except ValueError as e:
            self.status_var.set(f"Save failed: {e}")
            return
        if not profile["name"]:
            self.status_var.set("Save failed: give the profile a name")
            return
        PROFILES_DIR.mkdir(exist_ok=True)
        path = filedialog.asksaveasfilename(
            title="Save impact profile", initialdir=str(PROFILES_DIR),
            initialfile=profile["name"] + PROFILE_SUFFIX,
            defaultextension=PROFILE_SUFFIX,
            filetypes=[("Impact profiles", "*" + PROFILE_SUFFIX), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            save_profile(profile, path)
        except (OSError, ValueError) as e:
            self.status_var.set(f"Save failed: {e}")
            return
        self.status_var.set(f"Saved profile {Path(path).name}")

    def _on_load_profile(self):
        PROFILES_DIR.mkdir(exist_ok=True)
        path = filedialog.askopenfilename(
            title="Load impact profile", initialdir=str(PROFILES_DIR),
            filetypes=[("Impact profiles", "*" + PROFILE_SUFFIX), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            profile = load_profile(path)
        except (OSError, ValueError) as e:
            self.status_var.set(f"Load failed: {e}")
            return
        self.apply_profile_to_fields(profile)
        self._armed = None
        self._refresh_buttons()
        self.status_var.set(f"Loaded profile {Path(path).name} -- re-Arm before firing")

    # ------------------------------------------------------------ arm/fire
    def _on_arm(self):
        try:
            compiled = self._compile_from_fields()
        except ValueError as e:
            self.status_var.set(f"Arm failed: {e}")
            return
        self._armed = None
        self._busy = True
        self._refresh_buttons()
        self.status_var.set("Arming sequence...")
        try:
            prepare_controller(self.controller, compiled)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: {e}")
            self._busy = False
            self._refresh_buttons()
            return

        def done():
            self._armed = compiled
            self._busy = False
            self._refresh_buttons()
            self._show_plan(compiled)
            self.status_var.set(
                f"Sequence armed: {compiled.profile['impact_velocity_m_s']}m/s at "
                f"{compiled.impact_position_mm:.2f}mm, dwell {compiled.dwell_ms:g}ms"
            )

        def failed(message):
            self._busy = False
            self._refresh_buttons()
            self.status_var.set(f"Arm failed: {message}")

        wait_for_motion_complete(self.controller, self.root.after, on_complete=done, on_error=failed)

    def _on_fire(self):
        compiled = self._armed
        if compiled is None:
            return
        self._busy = True
        self._refresh_buttons()
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: {e}")
            self._busy = False
            self._refresh_buttons()
            return
        self.status_var.set("Firing sequence...")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=lambda: self._on_fire_complete(compiled),
            on_error=self._on_fire_error,
        )

    def _on_fire_error(self, message):
        self._armed = None
        self._busy = False
        self._refresh_buttons()
        self.status_var.set(message)

    def _on_fire_complete(self, compiled):
        self._armed = None
        self._busy = False
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"
        try:
            recording = fetch_sequence_recording(self.controller, compiled)
        except gclib.GclibError as e:
            self.status_var.set(f"Sequence data upload failed: {e} [controller last error: {last_error}]")
            self._refresh_buttons()
            return
        try:
            saved, summary = self.present_recording(compiled, recording)
        except Exception as e:
            self.status_var.set(
                f"Sequence ran, but plotting/saving failed: {e} "
                f"[{len(recording.fast.position)} fast samples, controller last error: {last_error}]"
            )
        else:
            self.status_var.set(
                f"Sequence captured: {summary} -> {saved.name} [controller last error: {last_error}]"
            )
        self._refresh_buttons()

    def present_recording(self, compiled, recording):
        """Metrics, data files, the familiar impact figure for the fast
        segment and the whole-sequence overview. Returns (json path,
        one-line summary)."""
        from gui.app import CAPTURES_DIR  # late import: app imports this module

        position_mm = [Controller.counts_to_mm(float(p)) for p in recording.fast.position]
        metrics = impact_metrics(
            position_mm, recording.fast_times_s, recording.layout.sample_period_s,
            compiled.impact_position_mm, recording.dwell_start_s, recording.retract_start_s,
        )
        doc = capture_document(recording, compiled, metrics)
        stem = "sequence_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        saved = save_capture(doc, CAPTURES_DIR, stem)
        text = info_text(compiled)
        if recording.fast_wrapped:
            text += "\nFast segment: rolling buffer wrapped (start of pre-pause dropped)"
        self.app._show_capture(
            recording.fast, recording.layout.sample_period_s, text, feedforward_v=compiled.ff_plot_v,
        )
        figure = build_sequence_figure(doc, text, format_metrics(metrics))
        show_figure(self.root, figure, f"Impact sequence - {stem}", CAPTURES_DIR / f"{stem}.png")
        v = metrics.get("impact_velocity_m_s")
        summary = (
            f"impact v {v:.3f}m/s, " if v is not None else "impact position not reached, "
        ) + f"peak {metrics['peak_position_mm']:.2f}mm, {len(recording.fast.position)} fast + {len(recording.slow.position)} slow samples"
        return saved, summary


class ConditioningControls:
    """'Condition LVDT (mode 2)' / 'Restore mode 1' buttons: run the
    conditioning program with the pulse pointed into whichever hard
    stop the striker is nearest (read live), leaving the servo enabled
    under the app's Rest settings. Needs the servo enabled and the
    striker parked against a stop (Slow Retract/Extend first)."""

    def __init__(self, app, parent):
        self.app = app
        self.controller = app.controller
        self.root = app.root
        self.status_var = app.status_var
        self._busy = False
        row = tk.Frame(parent)
        row.pack(fill="x", pady=(4, 0))
        self.mode2_button = tk.Button(
            row, text=f"Condition LVDT (mode 2: {MODE2_PULSE_CURRENT_A:g}A x {MODE2_PULSE_MS:g}ms)",
            command=lambda: self._run(MODE2_PULSE_CURRENT_A, MODE2_PULSE_MS, 1), state="disabled",
        )
        self.mode2_button.pack(side="left", padx=(0, 8))
        self.mode1_button = tk.Button(
            row, text=f"Restore mode 1 ({MODE1_PULSE_REPEATS}x {MODE1_PULSE_CURRENT_A:g}A x {MODE1_PULSE_MS:g}ms)",
            command=lambda: self._run(MODE1_PULSE_CURRENT_A, MODE1_PULSE_MS, MODE1_PULSE_REPEATS),
            state="disabled",
        )
        self.mode1_button.pack(side="left", padx=(0, 8))
        tk.Label(
            row, text="(pulses into the nearest hard stop -- park against a stop first)", anchor="w",
        ).pack(side="left")

    def set_servo_enabled(self, enabled: bool):
        state = "normal" if enabled and not self._busy else "disabled"
        self.mode2_button.config(state=state)
        self.mode1_button.config(state=state)

    def _run(self, current_a, pulse_ms, repeats):
        app = self.app
        try:
            rest_kp = float(app.rest_kp_var.get())
            rest_kd = float(app.rest_kd_var.get())
            rest_ki = float(app.rest_ki_var.get())
            rest_offset_v = float(app.rest_offset_var.get())
        except ValueError:
            self.status_var.set("Rest KP/KD/KI/Offset must be numbers")
            return
        try:
            position_mm = Controller.counts_to_mm(self.controller.read_position())
        except gclib.GclibError as e:
            self.status_var.set(f"Condition failed: {e}")
            return
        # Positive OF pushes toward retract. Nearest stop decides the sign.
        toward_retract = position_mm < FULL_STROKE_MM / 2
        pulse_v = Controller.current_to_volts(current_a) * (1 if toward_retract else -1)
        pulse_v = round(pulse_v, 6) + 0.0
        if abs(pulse_v) > TORQUE_VOLTS_FULL_SCALE:
            self.status_var.set(
                f"Condition failed: {current_a}A exceeds the amp's full-scale current"
            )
            return
        text = build_conditioning_program(
            pulse_v=pulse_v, pulse_ms=pulse_ms, repeats=repeats,
            rest_kp=rest_kp, rest_kd=rest_kd, rest_ki=rest_ki, rest_offset_v=rest_offset_v,
        )
        self._busy = True
        self.set_servo_enabled(True)
        try:
            self.controller.download_program(text)
            self.controller.run_program("COND")
        except gclib.GclibError as e:
            self._busy = False
            self.status_var.set(f"Condition failed: {e}")
            return
        stop = "retract" if toward_retract else "extend"
        self.status_var.set(f"Conditioning: {current_a:g}A x {pulse_ms:g}ms into the {stop} stop...")

        def done():
            self._busy = False
            try:
                last_error = self.controller.read_last_error()
            except gclib.GclibError as e:
                last_error = f"(could not read: {e})"
            self.status_var.set(
                f"Conditioning pulse done ({stop} stop) [controller last error: {last_error}]"
            )

        def failed(message):
            self._busy = False
            self.status_var.set(message)

        wait_for_program_complete(self.controller, self.root.after, on_complete=done, on_error=failed)
