"""Operator-facing GUI for running calibrated impact profiles -- the
"user GUI" from the handoff notes, laid out after docs/images/
gui_sketch.jpg.

What it does, and only this:
  * Connect / Disconnect, a permanently-available Disable (abort).
  * Live position / current / servo readout.
  * Pick a tip, then one of the impact profiles developed for it in the
    testing GUI (profiles/*.json, see sequence/profile.py), and a dwell
    time within the profile's allowed range.
  * Go to Impact Position (slow move to where the tip will first touch,
    for positioning the animal), Retract, Extend.
  * Arm (download + condition + retract to Start) then Fire; every run
    is saved (JSON + CSV + PNGs) under the data directory and summarised
    in the Last Fire panel, with View Plot.
  * Calibrate Position: guided two-point LVDT calibration taken in
    sensor mode 2 (conditioning pulse into each hard stop first),
    persisted across sessions.
  * Settings (addresses, directories, amp scale, stroke) and Help.

Every motion/recording decision is made by sequence/impact_sequence.py
from the profile file -- nothing about the trajectory is editable here.
The servo is enabled automatically by the actions that need it (under
the selected profile's Rest gains), and Disable is the stop button.

This module is the only Tkinter user besides the testing GUI; all
gclib access stays in controller.controller.
"""

import datetime
import json
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from analysis.capture_analysis import build_capture_figure, build_sequence_figure, compute_velocity
from analysis.capture_io import capture_document, load_capture, save_capture
from analysis.sequence_metrics import format_metrics, impact_metrics
from controller.controller import (
    Controller,
    DEFAULT_ADDRESS,
    DEFAULT_AMP_FULL_SCALE_CURRENT_A,
    DEFAULT_REST_KD,
    DEFAULT_REST_KI,
    DEFAULT_REST_KP,
    DEFAULT_REST_OFFSET_V,
    DEFAULT_SAMPLE_PERIOD_US,
    FULL_STROKE_MM,
    SLOW_ACCEL_M_S2,
    SLOW_DECEL_M_S2,
    SLOW_RETRACT_MARGIN_MM,
    SLOW_SPEED_M_S,
    TRAVEL_MARGIN_MM,
)
from gui.figure_popup import show_figure
from program.conditioning_program import (
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
    list_profiles,
    profile_label,
)

APP_DIR = Path(__file__).resolve().parent.parent.parent
SETTINGS_DIR = APP_DIR / "settings"
SETTINGS_PATH = SETTINGS_DIR / "user_settings.json"
CALIBRATION_PATH = SETTINGS_DIR / "lvdt_calibration.json"
POLL_INTERVAL_MS = 200
# Same averaging the testing GUI's Set buttons use.
CALIBRATION_SAMPLE_COUNT = 20
CALIBRATION_SAMPLE_INTERVAL_MS = 50

HELP_TEXT = """PIG CCI IMPACTOR -- OPERATOR PROCEDURE

1. Connect (IP address in Settings). The servo is enabled automatically
   by any motion button; DISABLE at the top stops everything, always.
2. If the position readout looks wrong, run Calibrate Position: the
   striker is driven to each hard stop, the LVDT is conditioned with a
   10A pulse (sensor "mode 2", the state every impact leaves it in) and
   the reading is averaged. The calibration is saved and reloaded on
   the next start.
3. Choose the Tip, then the Impact Profile (velocity / depth) developed
   for it in the testing GUI, and the Dwell time.
4. Go to Impact Position moves the tip slowly to where it will first
   touch the brain for this profile's depth. Position the animal so the
   tip just touches the surface, then press Retract.
5. Arm: the sequence is downloaded, the sensor is conditioned, and the
   striker is retracted to the profile's Start. Fire runs the whole
   sequence (impact, dwell, retract) on the controller; the host only
   waits and reads back the data.
6. Last Fire shows the measured impact velocity (at the impact
   position), peak extension, rebound and dwell, and where the data
   was saved. View Plot opens the traces.

Profiles live in the profiles directory (Settings); each is one
tip/velocity/depth combination calibrated in the testing GUI.
"""


def load_settings() -> dict:
    settings = {
        "address": DEFAULT_ADDRESS,
        "profiles_dir": str(APP_DIR / "profiles"),
        "data_dir": str(APP_DIR / "data"),
        "amp_full_scale_current_a": DEFAULT_AMP_FULL_SCALE_CURRENT_A,
        "stroke_mm": round(FULL_STROKE_MM, 2),
    }
    try:
        with open(SETTINGS_PATH, "r", encoding="utf-8") as f:
            stored = json.load(f)
        if isinstance(stored, dict):
            settings.update(stored)
    except (OSError, ValueError):
        pass
    return settings


def save_settings(settings: dict) -> None:
    SETTINGS_DIR.mkdir(parents=True, exist_ok=True)
    with open(SETTINGS_PATH, "w", encoding="utf-8") as f:
        json.dump(settings, f, indent=2)


def load_calibration() -> bool:
    """Applies the saved two-point calibration, if any. True if applied."""
    try:
        with open(CALIBRATION_PATH, "r", encoding="utf-8") as f:
            doc = json.load(f)
        Controller.set_calibration_points([tuple(p) for p in doc["points"]])
        return True
    except (OSError, ValueError, KeyError, TypeError):
        return False


def save_calibration(points) -> None:
    SETTINGS_DIR.mkdir(parents=True, exist_ok=True)
    with open(CALIBRATION_PATH, "w", encoding="utf-8") as f:
        json.dump({
            "points": [list(p) for p in points],
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
        }, f, indent=2)


class UserApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Pig CCI Impactor")
        self.controller = Controller()
        self.settings = load_settings()
        Controller.set_amp_current_per_volt(float(self.settings["amp_full_scale_current_a"]))
        self._calibrated = load_calibration()
        self._poll_after_id = None
        self._busy = False
        self._servo_enabled = False
        self._armed = None
        self._profiles = []  # (path, profile) for the selected tip
        self._all_profiles = []
        self._last_doc = None
        self._last_compiled = None

        self.address_var = tk.StringVar(value=self.settings["address"])
        self.status_var = tk.StringVar(value="Disconnected")
        self.servo_var = tk.StringVar(value="Servo: —")
        self.position_var = tk.StringVar(value="—")
        self.current_var = tk.StringVar(value="—")
        self.tip_var = tk.StringVar(value="")
        self.profile_var = tk.StringVar(value="")
        self.profile_detail_var = tk.StringVar(value="")
        self.dwell_var = tk.StringVar(value="")
        self.last_tip_var = tk.StringVar(value="—")
        self.last_profile_var = tk.StringVar(value="—")
        self.last_velocity_var = tk.StringVar(value="—")
        self.last_depth_var = tk.StringVar(value="—")
        self.last_dwell_var = tk.StringVar(value="—")
        self.last_rebound_var = tk.StringVar(value="—")
        self.last_path_var = tk.StringVar(value="—")
        self.calibration_var = tk.StringVar(value="")

        self._build_widgets()
        self._refresh_profiles()
        self._refresh_calibration_text()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------ widgets
    def _build_widgets(self):
        tk.Button(
            self.root, text="DISABLE", command=self._on_disable, bg="red", fg="white",
            activebackground="darkred", activeforeground="white",
            font=("Arial", 16, "bold"), height=2,
        ).pack(padx=10, pady=(10, 0), fill="x")

        row = tk.Frame(self.root)
        row.pack(padx=10, pady=(8, 0), fill="x")
        tk.Label(row, text="IP Address:").pack(side="left")
        self.address_entry = tk.Entry(row, textvariable=self.address_var, width=16)
        self.address_entry.pack(side="left", padx=(4, 10))
        self.connect_button = tk.Button(row, text="Connect", command=self._on_connect)
        self.connect_button.pack(side="left")
        self.disconnect_button = tk.Button(row, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.disconnect_button.pack(side="left", padx=(4, 0))
        self.servo_label = tk.Label(row, textvariable=self.servo_var, width=12, anchor="w")
        self.servo_label.pack(side="left", padx=(16, 0))

        tk.Label(self.root, textvariable=self.status_var, anchor="w", wraplength=900, justify="left").pack(
            padx=10, pady=(4, 0), fill="x"
        )

        readout = tk.Frame(self.root)
        readout.pack(padx=10, pady=(4, 8), fill="x")
        tk.Label(readout, text="Position (mm):", width=14, anchor="w").pack(side="left")
        tk.Label(readout, textvariable=self.position_var, width=10, anchor="w", font=("Arial", 12, "bold")).pack(side="left")
        tk.Label(readout, text="Current (A):", width=12, anchor="w").pack(side="left")
        tk.Label(readout, textvariable=self.current_var, width=10, anchor="w", font=("Arial", 12, "bold")).pack(side="left")

        body = tk.Frame(self.root)
        body.pack(padx=10, fill="both", expand=True)
        left = tk.Frame(body)
        left.pack(side="left", fill="y", padx=(0, 20))
        right = tk.Frame(body)
        right.pack(side="left", fill="both", expand=True)

        button_opts = dict(width=24, height=2)
        self.arm_button = tk.Button(left, text="Arm", command=self._on_arm, state="disabled", **button_opts)
        self.arm_button.pack(pady=(0, 4))
        self.fire_button = tk.Button(
            left, text="FIRE", command=self._on_fire, state="disabled", bg="#ffcccc",
            activebackground="#ff9999", **button_opts,
        )
        self.fire_button.pack(pady=(0, 12))
        self.go_button = tk.Button(left, text="Go to Impact Position", command=self._on_go_to_impact, state="disabled", **button_opts)
        self.go_button.pack(pady=(0, 4))
        self.retract_button = tk.Button(left, text="Retract", command=self._on_retract, state="disabled", **button_opts)
        self.retract_button.pack(pady=(0, 4))
        self.extend_button = tk.Button(left, text="Extend", command=self._on_extend, state="disabled", **button_opts)
        self.extend_button.pack(pady=(0, 12))
        self.calibrate_button = tk.Button(left, text="Calibrate Position", command=self._on_calibrate, state="disabled", **button_opts)
        self.calibrate_button.pack(pady=(0, 4))
        tk.Label(left, textvariable=self.calibration_var, justify="left", anchor="w", font=("TkFixedFont", 8)).pack(fill="x")
        self.motion_buttons = (self.go_button, self.retract_button, self.extend_button, self.calibrate_button)

        select = tk.LabelFrame(right, text="Run selection")
        select.pack(fill="x")
        row = tk.Frame(select)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Tip:", width=14, anchor="w").pack(side="left")
        self.tip_combo = ttk.Combobox(row, textvariable=self.tip_var, state="readonly", width=20)
        self.tip_combo.pack(side="left")
        self.tip_combo.bind("<<ComboboxSelected>>", lambda e: self._on_tip_selected())
        tk.Button(row, text="Reload profiles", command=self._refresh_profiles).pack(side="left", padx=(8, 0))
        row = tk.Frame(select)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Impact profile:", width=14, anchor="w").pack(side="left")
        self.profile_combo = ttk.Combobox(row, textvariable=self.profile_var, state="readonly", width=48)
        self.profile_combo.pack(side="left")
        self.profile_combo.bind("<<ComboboxSelected>>", lambda e: self._on_profile_selected())
        row = tk.Frame(select)
        row.pack(fill="x", pady=2)
        tk.Label(row, text=f"Dwell time (ms) [{DWELL_MIN_MS:g}-{DWELL_MAX_MS:g}]:", anchor="w").pack(side="left")
        self.dwell_entry = tk.Entry(row, textvariable=self.dwell_var, width=8)
        self.dwell_entry.pack(side="left", padx=(4, 0))
        tk.Label(select, textvariable=self.profile_detail_var, justify="left", anchor="w", font=("TkFixedFont", 8)).pack(fill="x", pady=(2, 0))

        last = tk.LabelFrame(right, text="Last Fire")
        last.pack(fill="x", pady=(10, 0))
        for label, var in (
            ("Tip:", self.last_tip_var), ("Profile:", self.last_profile_var),
            ("Impact velocity:", self.last_velocity_var), ("Depth / peak:", self.last_depth_var),
            ("Dwell (measured):", self.last_dwell_var), ("Rebound:", self.last_rebound_var),
        ):
            row = tk.Frame(last)
            row.pack(fill="x")
            tk.Label(row, text=label, width=18, anchor="w").pack(side="left")
            tk.Label(row, textvariable=var, anchor="w").pack(side="left")
        row = tk.Frame(last)
        row.pack(fill="x", pady=(4, 2))
        self.view_plot_button = tk.Button(row, text="View Plot", command=self._on_view_plot, state="disabled")
        self.view_plot_button.pack(side="left")
        row = tk.Frame(last)
        row.pack(fill="x")
        tk.Label(row, text="Data file:", width=18, anchor="w").pack(side="left")
        tk.Label(row, textvariable=self.last_path_var, anchor="w", wraplength=520, justify="left").pack(side="left")

        bottom = tk.Frame(self.root)
        bottom.pack(padx=10, pady=10, fill="x")
        tk.Button(bottom, text="Settings", command=self._on_settings).pack(side="left")
        tk.Button(bottom, text="Help", command=self._on_help).pack(side="left", padx=(8, 0))

    # ------------------------------------------------------------ profiles
    def _refresh_profiles(self):
        entries = list_profiles(self.settings["profiles_dir"])
        bad = [p.name for p, v in entries if isinstance(v, Exception)]
        self._all_profiles = [(p, v) for p, v in entries if isinstance(v, dict)]
        tips = sorted({v.get("tip") or "(no tip)" for _, v in self._all_profiles})
        self.tip_combo["values"] = tips
        if self.tip_var.get() not in tips:
            self.tip_var.set(tips[0] if tips else "")
        self._on_tip_selected()
        if bad:
            self.status_var.set("Ignored unreadable profile(s): " + ", ".join(bad))
        elif not self._all_profiles:
            self.status_var.set(f"No profiles found in {self.settings['profiles_dir']}")

    def _on_tip_selected(self):
        tip = self.tip_var.get()
        self._profiles = [(p, v) for p, v in self._all_profiles if (v.get("tip") or "(no tip)") == tip]
        labels = [profile_label(v) for _, v in self._profiles]
        self.profile_combo["values"] = labels
        if self.profile_var.get() not in labels:
            self.profile_var.set(labels[0] if labels else "")
        self._on_profile_selected()

    def _on_profile_selected(self):
        self._armed = None
        profile = self.selected_profile()
        if profile is None:
            self.profile_detail_var.set("")
            self.dwell_var.set("")
        else:
            m = profile["motion"]
            self.profile_detail_var.set(
                f"velocity {profile['impact_velocity_m_s']} m/s, depth {profile['impact_depth_mm']} mm, "
                f"start/stop {m['start_mm']}/{m['stop_mm']} mm\n"
                f"hold {profile['sequence']['dwell']['hold_current_a']} A, "
                f"retract {profile['sequence']['retract']['speed_m_s']} m/s, "
                f"amp full scale {profile['amp']['full_scale_current_a']} A"
                + (f"\n{profile['notes']}" if profile.get("notes") else "")
            )
            self.dwell_var.set(f"{float(profile['sequence']['dwell']['duration_ms']):g}")
        self._refresh_buttons()

    def selected_profile(self):
        label = self.profile_var.get()
        for _, profile in self._profiles:
            if profile_label(profile) == label:
                return profile
        return None

    def _selected_dwell_ms(self):
        try:
            dwell = float(self.dwell_var.get())
        except ValueError:
            raise ValueError("Dwell time must be a number (ms)")
        if not (DWELL_MIN_MS <= dwell <= DWELL_MAX_MS):
            raise ValueError(f"Dwell time must be {DWELL_MIN_MS:g}-{DWELL_MAX_MS:g} ms")
        return dwell

    # ------------------------------------------------------------ gating
    def _refresh_buttons(self):
        connected = self.controller.is_connected
        idle = connected and not self._busy
        for button in self.motion_buttons:
            button.config(state="normal" if idle else "disabled")
        if self.selected_profile() is None:
            self.go_button.config(state="disabled")
        self.arm_button.config(state="normal" if idle and self.selected_profile() is not None else "disabled")
        self.fire_button.config(
            state="normal" if idle and self._armed is not None and self._servo_enabled else "disabled"
        )
        self.view_plot_button.config(state="normal" if self._last_doc is not None else "disabled")

    def _set_busy(self, busy: bool):
        self._busy = busy
        self._refresh_buttons()

    # ------------------------------------------------------------ connection
    def _on_connect(self):
        address = self.address_var.get().strip()
        try:
            self.controller.connect(address)
        except gclib.GclibError as e:
            self.status_var.set(f"Connect failed: {e}")
            return
        self.settings["address"] = address
        try:
            save_settings(self.settings)
        except OSError:
            pass
        self.status_var.set(f"Connected to {address}")
        self.connect_button.config(state="disabled")
        self.disconnect_button.config(state="normal")
        self.address_entry.config(state="disabled")
        self._armed = None
        self._refresh_buttons()
        self._poll()

    def _on_disconnect(self):
        self._stop_polling()
        error = None
        try:
            self.controller.abort()
        except gclib.GclibError as e:
            error = e
        self.controller.disconnect()
        self.status_var.set("Disconnected" if error is None else f"Disconnected (servo disable failed: {error})")
        self.connect_button.config(state="normal")
        self.disconnect_button.config(state="disabled")
        self.address_entry.config(state="normal")
        self.servo_var.set("Servo: —")
        self._servo_enabled = False
        self._armed = None
        self._busy = False
        self._refresh_buttons()

    def _on_disable(self):
        if not self.controller.is_connected:
            self.status_var.set("Not connected -- nothing to disable")
            return
        was_armed = self._armed is not None
        try:
            self.controller.abort()
        except gclib.GclibError as e:
            self.status_var.set(f"Disable failed: {e}")
            return
        self._armed = None
        self._busy = False
        self._refresh_buttons()
        self.status_var.set("DISABLED -- motion/program aborted" + (" (armed sequence cancelled)" if was_armed else ""))

    def _rest_gains(self):
        profile = self.selected_profile()
        if profile is not None:
            r = profile["motion"]["rest_pid"]
            return float(r["kp"]), float(r["kd"]), float(r["ki"]), float(r["offset_v"])
        return DEFAULT_REST_KP, DEFAULT_REST_KD, DEFAULT_REST_KI, DEFAULT_REST_OFFSET_V

    def _ensure_enabled(self) -> bool:
        """Rest gains/offset + Rest sample period, then SHA if the servo
        is off. False (with a status message) on failure."""
        kp, kd, ki, offset_v = self._rest_gains()
        try:
            self.controller.set_pid_gains(kp, kd, ki)
            self.controller.set_offset(offset_v)
            self.controller.set_sample_period(DEFAULT_SAMPLE_PERIOD_US)
            if not self.controller.read_servo_enabled():
                self.controller.enable()
        except gclib.GclibError as e:
            self.status_var.set(f"Enable failed: {e}")
            return False
        return True

    # ------------------------------------------------------------ slow moves
    def _slow_move(self, target_mm: float, label: str, on_complete=None):
        if not self._ensure_enabled():
            return
        self._set_busy(True)
        try:
            self.controller.set_motion_profile(
                Controller.meters_to_counts(SLOW_SPEED_M_S),
                Controller.meters_to_counts(SLOW_ACCEL_M_S2),
                Controller.meters_to_counts(SLOW_DECEL_M_S2),
            )
            self.controller.move_absolute(Controller.mm_to_counts(target_mm))
        except gclib.GclibError as e:
            self.status_var.set(f"{label} failed: {e}")
            self._set_busy(False)
            return
        self.status_var.set(f"{label}: moving to {target_mm:.2f} mm...")

        def done():
            self._set_busy(False)
            self.status_var.set(f"{label} complete ({target_mm:.2f} mm)")
            if on_complete is not None:
                on_complete()

        def failed(message):
            self._set_busy(False)
            self.status_var.set(f"{label} failed: {message}")

        wait_for_motion_complete(self.controller, self.root.after, on_complete=done, on_error=failed)

    def _on_go_to_impact(self):
        profile = self.selected_profile()
        if profile is None:
            self.status_var.set("Select a profile first")
            return
        try:
            compiled = compile_sequence(profile)
        except ValueError as e:
            self.status_var.set(f"Profile can't be run: {e}")
            return
        self._armed = None
        self._slow_move(compiled.impact_position_mm, "Go to impact position")

    def _on_retract(self):
        self._armed = None
        self._slow_move(SLOW_RETRACT_MARGIN_MM, "Retract")

    def _on_extend(self):
        self._armed = None
        self._slow_move(float(self.settings["stroke_mm"]) - TRAVEL_MARGIN_MM, "Extend")

    # ------------------------------------------------------------ arm / fire
    def _on_arm(self):
        profile = self.selected_profile()
        if profile is None:
            self.status_var.set("Select a profile first")
            return
        try:
            dwell_ms = self._selected_dwell_ms()
            compiled = compile_sequence(profile, dwell_ms=dwell_ms)
        except ValueError as e:
            self.status_var.set(f"Arm failed: {e}")
            return
        self._armed = None
        self._set_busy(True)
        self.status_var.set("Arming: downloading sequence, retracting to start...")
        try:
            prepare_controller(self.controller, compiled)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: {e}")
            self._set_busy(False)
            return

        def done():
            self._armed = compiled
            self._set_busy(False)
            self.status_var.set(
                f"ARMED: {profile.get('name')} -- {compiled.profile['impact_velocity_m_s']} m/s, "
                f"depth {compiled.profile['impact_depth_mm']} mm, dwell {dwell_ms:g} ms. Press FIRE."
            )

        def failed(message):
            self._set_busy(False)
            self.status_var.set(f"Arm failed: {message}")

        wait_for_motion_complete(self.controller, self.root.after, on_complete=done, on_error=failed)

    def _on_fire(self):
        compiled = self._armed
        if compiled is None:
            return
        self._set_busy(True)
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: {e}")
            self._set_busy(False)
            return
        self.status_var.set("FIRING -- sequence running on the controller...")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=lambda: self._on_fire_complete(compiled),
            on_error=self._on_fire_error,
        )

    def _on_fire_error(self, message):
        self._armed = None
        self._set_busy(False)
        self.status_var.set(message)

    def _on_fire_complete(self, compiled):
        self._armed = None
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"
        try:
            recording = fetch_sequence_recording(self.controller, compiled)
        except gclib.GclibError as e:
            self.status_var.set(f"Run finished but data upload failed: {e} [controller: {last_error}]")
            self._set_busy(False)
            return
        try:
            path = self._store_run(compiled, recording)
        except Exception as e:
            self.status_var.set(f"Run finished but saving failed: {e} [controller: {last_error}]")
        else:
            self.status_var.set(f"Run complete -- data saved to {path} [controller: {last_error}]")
        self._set_busy(False)

    def _store_run(self, compiled, recording) -> Path:
        position_mm = [Controller.counts_to_mm(float(p)) for p in recording.fast.position]
        metrics = impact_metrics(
            position_mm, recording.fast_times_s, recording.layout.sample_period_s,
            compiled.impact_position_mm, recording.dwell_start_s, recording.retract_start_s,
        )
        doc = capture_document(recording, compiled, metrics)
        name = (compiled.profile.get("name") or "run").replace(" ", "_")
        stem = f"{name}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        path = save_capture(doc, self.settings["data_dir"], stem)
        self._last_doc = doc
        self._last_compiled = compiled
        self._last_stem = stem
        profile = compiled.profile
        self.last_tip_var.set(profile.get("tip") or "—")
        self.last_profile_var.set(profile_label(profile))
        v = metrics.get("impact_velocity_m_s")
        self.last_velocity_var.set(
            f"{v:.3f} m/s (nominal {profile['impact_velocity_m_s']} m/s)" if v is not None
            else "impact position not reached"
        )
        peak = metrics.get("peak_position_mm")
        self.last_depth_var.set(
            f"nominal {profile['impact_depth_mm']} mm; peak position {peak:.2f} mm "
            f"({peak - compiled.impact_position_mm:.2f} mm past impact position)"
            if peak is not None else "—"
        )
        d = metrics.get("dwell_measured_s")
        self.last_dwell_var.set(f"{d * 1000:.1f} ms (set {compiled.dwell_ms:g} ms)" if d is not None else "—")
        r = metrics.get("rebound_mm")
        self.last_rebound_var.set(f"{r:.2f} mm" if r is not None else "—")
        self.last_path_var.set(str(path))
        # PNGs alongside the data, without opening windows (View Plot does).
        for figure, suffix in self._build_figures(doc, compiled):
            figure.savefig(Path(self.settings["data_dir"]) / f"{stem}_{suffix}.png")
        return path

    def _build_figures(self, doc, compiled):
        fast = doc["fast"]
        text = info_text(compiled)
        metrics_text = format_metrics(doc.get("metrics", {}))
        impact_figure = build_capture_figure(
            fast["position_mm"], fast["commanded_position_mm"], fast["position_error_mm"],
            fast["torque_v"], fast["current_a"],
            compute_velocity(fast["commanded_position_mm"], doc["layout"]["sample_period_s"]),
            doc["layout"]["sample_period_s"], text + "\n" + metrics_text,
            feedforward_v=(fast["feedforward_v"] + [0.0] * len(fast["position_mm"]))[: len(fast["position_mm"])],
        )
        sequence_figure = build_sequence_figure(doc, text, metrics_text)
        return [(impact_figure, "impact"), (sequence_figure, "sequence")]

    def _on_view_plot(self):
        if self._last_doc is None:
            return
        try:
            for figure, suffix in self._build_figures(self._last_doc, self._last_compiled):
                show_figure(self.root, figure, f"{suffix} - {self._last_stem}")
        except Exception as e:
            self.status_var.set(f"Plot failed: {e}")

    # ------------------------------------------------------------ calibration
    def _refresh_calibration_text(self):
        points = Controller.calibration_points()
        (c0, m0), (c1, m1) = points[0], points[-1]
        scale = abs((c1 - c0) / (m1 - m0))
        source = "saved calibration" if self._calibrated else "default calibration"
        self.calibration_var.set(
            f"{source}\n{m0:.2f} mm = {Controller.counts_to_volts(c0):.3f} V\n"
            f"{m1:.2f} mm = {Controller.counts_to_volts(c1):.3f} V\n{scale:.1f} counts/mm"
        )

    def _on_calibrate(self):
        stroke_mm = float(self.settings["stroke_mm"])
        if not messagebox.askokcancel(
            "Calibrate Position",
            "The striker will be driven to the retract stop, conditioned with a "
            f"{MODE2_PULSE_CURRENT_A:g}A pulse and read, then to the extend stop "
            f"({stroke_mm:g} mm) and read the same way.\n\nMake sure nothing is in the "
            "way of the full stroke. Continue?",
        ):
            return
        amps = Controller.current_to_volts(MODE2_PULSE_CURRENT_A)
        kp, kd, ki, offset_v = self._rest_gains()
        results = []

        def pulse(sign, then):
            text = build_conditioning_program(
                pulse_v=round(sign * amps, 6) + 0.0, pulse_ms=MODE2_PULSE_MS,
                rest_kp=kp, rest_kd=kd, rest_ki=ki, rest_offset_v=offset_v,
            )
            try:
                self.controller.download_program(text)
                self.controller.run_program("COND")
            except gclib.GclibError as e:
                fail(f"conditioning pulse failed: {e}")
                return
            self.status_var.set("Calibration: conditioning pulse...")
            wait_for_program_complete(self.controller, self.root.after, on_complete=then, on_error=fail)

        def sample(mm, then):
            readings = []

            def take():
                try:
                    readings.append(self.controller.read_position())
                except gclib.GclibError as e:
                    fail(f"read failed: {e}")
                    return
                if len(readings) < CALIBRATION_SAMPLE_COUNT:
                    self.root.after(CALIBRATION_SAMPLE_INTERVAL_MS, take)
                else:
                    results.append((sum(readings) / len(readings), mm))
                    then()

            self.status_var.set(f"Calibration: averaging reading at {mm:g} mm...")
            take()

        def fail(message):
            self._set_busy(False)
            self.status_var.set(f"Calibration failed: {message}")

        def finish():
            try:
                Controller.set_calibration_points(results)
                save_calibration(results)
            except (ValueError, OSError) as e:
                fail(str(e))
                return
            self._calibrated = True
            self._refresh_calibration_text()
            self._set_busy(False)
            self.status_var.set("Calibration complete and saved")

        # At each end: move to the stop, pulse into it, then re-command
        # the stop position (the striker can drift off the stop while
        # the loop is open during the pulse's settle) and average the
        # reading while the Rest gains hold it there -- the same holding
        # state the testing GUI's Set buttons read in.
        def at_extend_stop():
            pulse(-1, lambda: self._slow_move(
                stroke_mm, "Calibration (extend, re-seat)", lambda: sample(stroke_mm, finish),
            ))

        def at_retract_stop():
            pulse(+1, lambda: self._slow_move(
                0.0, "Calibration (retract, re-seat)",
                lambda: sample(0.0, lambda: self._slow_move(stroke_mm, "Calibration (extend)", at_extend_stop)),
            ))

        self._armed = None
        self._slow_move(0.0, "Calibration (retract)", at_retract_stop)

    # ------------------------------------------------------------ settings / help
    def _on_settings(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("Settings")
        vars_ = {
            "address": tk.StringVar(value=str(self.settings["address"])),
            "profiles_dir": tk.StringVar(value=str(self.settings["profiles_dir"])),
            "data_dir": tk.StringVar(value=str(self.settings["data_dir"])),
            "amp_full_scale_current_a": tk.StringVar(value=str(self.settings["amp_full_scale_current_a"])),
            "stroke_mm": tk.StringVar(value=str(self.settings["stroke_mm"])),
        }
        labels = {
            "address": "Controller IP address", "profiles_dir": "Profiles directory",
            "data_dir": "Data directory",
            "amp_full_scale_current_a": "Amp full-scale current (A) -- CME Analog Input Scale",
            "stroke_mm": "Full stroke (mm) -- extend stop position",
        }
        for key in vars_:
            row = tk.Frame(dialog)
            row.pack(fill="x", padx=10, pady=2)
            tk.Label(row, text=labels[key] + ":", width=48, anchor="w").pack(side="left")
            tk.Entry(row, textvariable=vars_[key], width=40).pack(side="left")

        def apply():
            try:
                amp = float(vars_["amp_full_scale_current_a"].get())
                stroke = float(vars_["stroke_mm"].get())
                if amp <= 0 or stroke <= 0:
                    raise ValueError
            except ValueError:
                self.status_var.set("Settings: amp current and stroke must be positive numbers")
                return
            self.settings.update({
                "address": vars_["address"].get().strip(),
                "profiles_dir": vars_["profiles_dir"].get().strip(),
                "data_dir": vars_["data_dir"].get().strip(),
                "amp_full_scale_current_a": amp,
                "stroke_mm": stroke,
            })
            Controller.set_amp_current_per_volt(amp)
            self.address_var.set(self.settings["address"])
            try:
                save_settings(self.settings)
            except OSError as e:
                self.status_var.set(f"Settings saved for this session only: {e}")
            else:
                self.status_var.set("Settings saved")
            self._refresh_profiles()
            dialog.destroy()

        row = tk.Frame(dialog)
        row.pack(fill="x", padx=10, pady=8)
        tk.Button(row, text="Save", command=apply).pack(side="left")
        tk.Button(row, text="Cancel", command=dialog.destroy).pack(side="left", padx=(8, 0))

    def _on_help(self):
        window = tk.Toplevel(self.root)
        window.title("Help")
        text = tk.Text(window, width=78, height=32, wrap="word")
        text.insert("1.0", HELP_TEXT)
        text.config(state="disabled")
        text.pack(padx=10, pady=10, fill="both", expand=True)

    # ------------------------------------------------------------ polling
    def _poll(self):
        try:
            position = self.controller.read_position()
            output_voltage = self.controller.read_output_voltage()
            servo_enabled = self.controller.read_servo_enabled()
        except gclib.GclibError as e:
            self.status_var.set(f"Read error: {e} -- reconnect to resume")
            self._poll_after_id = None
            return
        self.position_var.set(f"{Controller.counts_to_mm(position):.2f}")
        self.current_var.set(f"{Controller.volts_to_current(output_voltage):.2f}")
        self._servo_enabled = servo_enabled
        self.servo_var.set("Servo: ON" if servo_enabled else "Servo: OFF")
        self.servo_label.config(fg="red" if servo_enabled else "black")
        if not servo_enabled and self._armed is not None and not self._busy:
            # Same rule as the testing GUI: Fire only survives while the
            # servo stays on; a fresh Arm is needed otherwise.
            self._armed = None
        self._refresh_buttons()
        self._poll_after_id = self.root.after(POLL_INTERVAL_MS, self._poll)

    def _stop_polling(self):
        if self._poll_after_id is not None:
            self.root.after_cancel(self._poll_after_id)
            self._poll_after_id = None

    def _on_close(self):
        self._stop_polling()
        if self.controller.is_connected:
            try:
                self.controller.abort()
            except gclib.GclibError as e:
                print(f"Servo disable failed during shutdown: {e}")
            self.controller.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    UserApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
