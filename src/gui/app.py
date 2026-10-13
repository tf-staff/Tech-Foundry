"""Tkinter control panel: connect/disconnect, servo enable/disable,
go-to-position/speed, PID gains, impact capture/plot, and live readout.

project_spec_v1.txt, section 4, steps c-h. This is the only module that
imports Tkinter — all gclib interaction stays in controller.controller.
"""

import datetime
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from analysis.capture_analysis import (
    SLIDER_WINDOW_LENGTH_MIN,
    build_capture_figure,
    compute_velocity,
)
from controller.controller import (
    ACCEL_MAX_COUNTS_PER_S2,
    ACCEL_MIN_COUNTS_PER_S2,
    AMP_FULL_SCALE_CURRENT_MAX_A,
    AMP_FULL_SCALE_CURRENT_MIN_A,
    Controller,
    DEFAULT_ACCEL_M_S2,
    DEFAULT_ADDRESS,
    DEFAULT_AMP_FULL_SCALE_CURRENT_A,
    DEFAULT_DECEL_M_S2,
    DEFAULT_MOTION_FV,
    DEFAULT_MOTION_KD,
    DEFAULT_MOTION_KI,
    DEFAULT_MOTION_KP,
    DEFAULT_MOTION_OFFSET_V,
    DEFAULT_PVT_FA,
    DEFAULT_PVT_FV,
    DEFAULT_REST_KD,
    DEFAULT_REST_KI,
    DEFAULT_REST_KP,
    DEFAULT_REST_OFFSET_V,
    DEFAULT_SAMPLE_PERIOD_US,
    DEFAULT_SPEED_M_S,
    EXTEND_MM,
    EXTEND_VOLTS,
    FA_MAX,
    FA_MIN,
    FULL_STROKE_MM,
    FV_MAX,
    FV_MIN,
    OF_MAX_V,
    OF_MIN_V,
    RECORD_ARRAY_SIZE,
    RETRACT_MM,
    RETRACT_VOLTS,
    SLOW_ACCEL_M_S2,
    SLOW_DECEL_M_S2,
    SLOW_RETRACT_MARGIN_MM,
    SLOW_SPEED_M_S,
    SP_MAX_COUNTS_PER_S,
    SP_MIN_COUNTS_PER_S,
    TM_MAX_US,
    TM_MIN_US,
    TRAVEL_MARGIN_MM,
)
from gui.sequence_panel import ConditioningControls, SequencePanel
from program.contour_program import build_contour_program, compute_contour_increments
from program.current_profile_program import (
    AT_SAMPLES as CURRENT_PROFILE_AT_SAMPLES,
    FF_ARRAY_NAME as CURRENT_PROFILE_FF_ARRAY_NAME,
    TM_US_ACTUAL as CURRENT_PROFILE_TM_US_ACTUAL,
    build_current_profile_program,
)
from program.current_profile_trajectory import (
    compute_current_profile_curve,
    compute_feedforward_v,
    sample_position_mm,
)
from program.impact_program import build_impact_program
from program.pvt_program import build_pvt_program, compute_pvt_segments
from program.simple_open_loop_program import build_simple_open_loop_program
from recording.recording_sequence import (
    RECORD_POST_PAUSE_MS,
    RECORD_PRE_PAUSE_MS,
    RecordingSequence,
    wait_for_motion_complete,
    wait_for_program_complete,
)

SPEED_RANGE_M_S = (
    Controller.counts_to_meters(SP_MIN_COUNTS_PER_S),
    Controller.counts_to_meters(SP_MAX_COUNTS_PER_S),
)
ACCEL_RANGE_M_S2 = (
    Controller.counts_to_meters(ACCEL_MIN_COUNTS_PER_S2),
    Controller.counts_to_meters(ACCEL_MAX_COUNTS_PER_S2),
)
POLL_INTERVAL_MS = 200  # ~5 Hz

# LVDT calibration capture: each Set button averages this many samples at
# this spacing (~1s total) rather than grabbing one instantaneous reading.
# The signal carries roughly +/-0.2mm of noise, so a single sample is far
# too coarse to anchor a calibration point; averaging ~20 brings that down
# by about sqrt(20). Sampled via root.after rather than a blocking loop so
# the GUI stays responsive while it runs.
CALIBRATION_SAMPLE_COUNT = 20
CALIBRATION_SAMPLE_INTERVAL_MS = 50
# A Set press reads both the raw counts and the LVDT volts, which are two
# views of one 16-bit reading and so should agree through
# Controller.COUNTS_PER_VOLT. This is how far apart they may drift (in
# counts, ~0.05V) before the status line says so -- wide enough for ADC
# noise and the averaging residual, narrow enough to catch a wrong
# constant, which would silently skew every hand-entered voltage.
CALIBRATION_COUNTS_PER_VOLT_TOLERANCE = 150.0

CAPTURES_DIR = Path(__file__).resolve().parent.parent.parent / "captures"
EXAMPLE_DMC_DIR = Path(__file__).resolve().parent.parent.parent / "example_dmc"


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Voice Coil Bench Control")
        self.root.geometry(f"950x{int(self.root.winfo_screenheight() * 0.9)}")

        self.controller = Controller()
        self._poll_after_id = None

        self.address_var = tk.StringVar(value=DEFAULT_ADDRESS)
        self.status_var = tk.StringVar(value="Disconnected")
        self.position_counts_var = tk.StringVar(value="—")
        self.position_mm_var = tk.StringVar(value="—")
        self.lvdt_voltage_var = tk.StringVar(value="—")
        self.output_current_var = tk.StringVar(value="—")
        self.amp_full_scale_current_var = tk.StringVar(
            value=str(DEFAULT_AMP_FULL_SCALE_CURRENT_A)
        )
        self.servo_status_var = tk.StringVar(value="Servo: —")

        self.target_var = tk.StringVar(value="")
        self.speed_var = tk.StringVar(value=f"{DEFAULT_SPEED_M_S:.4f}")
        self.accel_var = tk.StringVar(value=f"{DEFAULT_ACCEL_M_S2:.4f}")
        self.decel_var = tk.StringVar(value=f"{DEFAULT_DECEL_M_S2:.4f}")
        self.rest_kp_var = tk.StringVar(value=str(DEFAULT_REST_KP))
        self.rest_kd_var = tk.StringVar(value=str(DEFAULT_REST_KD))
        self.rest_ki_var = tk.StringVar(value=str(DEFAULT_REST_KI))
        self.rest_offset_var = tk.StringVar(value=str(DEFAULT_REST_OFFSET_V))
        self.motion_kp_var = tk.StringVar(value=str(DEFAULT_MOTION_KP))
        self.motion_kd_var = tk.StringVar(value=str(DEFAULT_MOTION_KD))
        self.motion_ki_var = tk.StringVar(value=str(DEFAULT_MOTION_KI))
        self.motion_offset_var = tk.StringVar(value=str(DEFAULT_MOTION_OFFSET_V))
        self.motion_fv_var = tk.StringVar(value=str(DEFAULT_MOTION_FV))
        self.motion_sample_period_us_var = tk.StringVar(value=str(DEFAULT_SAMPLE_PERIOD_US))
        # Whatever Motion sample period was last successfully applied --
        # that's what a capture's data was actually sampled at, since
        # recordings apply it before start_recording() and only revert to
        # Rest's default after stop_recording() finishes retrieving the
        # data. _on_recording_complete reads this rather than the live TM
        # state, since by the time it runs the rate has already reverted.
        self._capture_sample_period_s = DEFAULT_SAMPLE_PERIOD_US / 1_000_000
        # Motion-info text shown on the plot -- snapshotted alongside its
        # matching sample period, same "what actually ran" reasoning.
        self._go_capture_info = ""

        self.pre_pause_ms_var = tk.StringVar(value=str(RECORD_PRE_PAUSE_MS))
        self.post_pause_ms_var = tk.StringVar(value=str(RECORD_POST_PAUSE_MS))

        self.program_path_var = tk.StringVar(value="")

        self.pre_impact_retract_voltage_var = tk.StringVar(value="")
        self.pre_impact_retract_wait_ms_var = tk.StringVar(value="")
        self.impact_accel_primary_voltage_var = tk.StringVar(value="")
        self.impact_accel_primary_wait_ms_var = tk.StringVar(value="")
        self.impact_accel_secondary_voltage_var = tk.StringVar(value="")
        self.impact_accel_secondary_wait_ms_var = tk.StringVar(value="")
        self.impact_decel_voltage_var = tk.StringVar(value="")
        self.impact_decel_wait_ms_var = tk.StringVar(value="")
        self.hold_voltage_var = tk.StringVar(value="")
        self.hold_wait_ms_var = tk.StringVar(value="")
        self.retract_accel_primary_voltage_var = tk.StringVar(value="")
        self.retract_accel_primary_wait_ms_var = tk.StringVar(value="")
        self.retract_accel_secondary_voltage_var = tk.StringVar(value="")
        self.retract_accel_secondary_wait_ms_var = tk.StringVar(value="")
        self.retract_decel_voltage_var = tk.StringVar(value="")
        self.retract_decel_wait_ms_var = tk.StringVar(value="")
        self.end_with_closed_loop_move_var = tk.BooleanVar(value=False)
        self.impact_start_var = tk.StringVar(value="0.00")
        # Snapshotted from the Motion sample period baked into the armed
        # program's text, not re-read live at Fire time -- the field could
        # change between Arm and Fire. Same reasoning as
        # _capture_sample_period_s above.
        self._armed_capture_sample_period_s = DEFAULT_SAMPLE_PERIOD_US / 1_000_000
        self._armed_capture_info = ""

        self.contour_velocity_var = tk.StringVar(value="")
        self.contour_position_var = tk.StringVar(value="")
        self.contour_start_var = tk.StringVar(value="0.00")
        self.contour_stop_var = tk.StringVar(value=f"{FULL_STROKE_MM:.2f}")
        # Same snapshot reasoning as _armed_capture_sample_period_s, kept
        # separate since Open Loop Motion and Contoured Motion can be armed
        # independently of each other.
        self._armed_contour_sample_period_s = DEFAULT_SAMPLE_PERIOD_US / 1_000_000
        self._armed_contour_info = ""

        self.pvt_velocity_var = tk.StringVar(value="")
        self.pvt_position_var = tk.StringVar(value="")
        self.pvt_start_var = tk.StringVar(value="0.00")
        self.pvt_stop_var = tk.StringVar(value=f"{FULL_STROKE_MM:.2f}")
        self.pvt_fv_var = tk.StringVar(value=str(DEFAULT_PVT_FV))
        self.pvt_fa_var = tk.StringVar(value=str(DEFAULT_PVT_FA))
        self.pvt_ac_var = tk.StringVar(value=f"{DEFAULT_ACCEL_M_S2:.4f}")
        self.pvt_dc_var = tk.StringVar(value=f"{DEFAULT_DECEL_M_S2:.4f}")
        # Same snapshot reasoning as _armed_contour_sample_period_s, kept
        # separate since all three motion modes can be armed independently.
        self._armed_pvt_sample_period_s = DEFAULT_SAMPLE_PERIOD_US / 1_000_000
        self._armed_pvt_info = ""

        self.current_profile_velocity_var = tk.StringVar(value="")
        self.current_profile_position_var = tk.StringVar(value="")
        # Where peak velocity occurs -- strictly between Start and Impact
        # Position. Peak velocity itself isn't a direct input; it's
        # solved for (current_profile_trajectory.compute_current_profile_
        # curve) so the single continuous peak->end deceleration passes
        # through the exact Impact Position/Velocity partway along --
        # letting the actuator decelerate over a longer span than plain
        # impact->end alone would allow, to avoid saturating current at
        # high impact velocities.
        self.current_profile_peak_velocity_position_var = tk.StringVar(value="")
        self.current_profile_start_var = tk.StringVar(value="0.00")
        self.current_profile_stop_var = tk.StringVar(value=f"{FULL_STROKE_MM:.2f}")
        self.current_profile_mass_var = tk.StringVar(value="")
        # Separate Kf per phase -- bench-observed: Volt Output didn't
        # track the single-Kf Feedforward curve equally well in both
        # directions, so the actuator's real force constant apparently
        # differs between accelerating (phase 1) and decelerating
        # (phase 2) current.
        self.current_profile_kf_accel_var = tk.StringVar(value="")
        self.current_profile_kf_decel_var = tk.StringVar(value="")
        # Independently time-stretch the Feedforward curve's 4 quarter-
        # segments (accel-phase rising/falling acceleration halves,
        # decel-phase rising/falling deceleration-magnitude halves) --
        # bench-motivated (actual velocity narrower/"pointier" than the
        # broad-topped commanded velocity) tuning knobs. 1.0 (default,
        # each) reproduces today's unstretched curve exactly; never
        # affects the physical CD/position move -- see
        # current_profile_trajectory.compute_feedforward_v.
        self.current_profile_ff_accel_rise_stretch_var = tk.StringVar(value="1.0")
        self.current_profile_ff_accel_fall_stretch_var = tk.StringVar(value="1.0")
        self.current_profile_ff_decel_rise_stretch_var = tk.StringVar(value="1.0")
        self.current_profile_ff_decel_fall_stretch_var = tk.StringVar(value="1.0")
        # Own Pre/Post-Move Pause fields, not the shared pre_pause_ms_var/
        # post_pause_ms_var other modes use -- this mode's recording
        # arrays share a tight controller-wide array-table budget with its
        # feedforward array (see current_profile_program.py), so a much
        # smaller default (2ms vs the other modes' 10ms) matters here
        # specifically, without affecting them.
        self.current_profile_pre_pause_ms_var = tk.StringVar(value="2")
        self.current_profile_post_pause_ms_var = tk.StringVar(value="2")
        # Pads the start of the Contour Mode move with stationary CD
        # segments so thread 1's OF-writing loop (which starts at the
        # same instant as the first CD line) has a head start before
        # real motion begins -- bench-observed fix for actual position
        # lagging commanded position at high impact velocities. 0 (the
        # default) adds no padding. See current_profile_program.py's
        # build_current_profile_program docstring (ff_advance_us).
        self.current_profile_ff_advance_us_var = tk.StringVar(value="0")
        # No Motion Sample Period field for this mode -- TM is fixed at
        # current_profile_program.TM_US (bench-confirmed, see the plan),
        # not user-configurable the way Contour/PVT's own Sample Period
        # is, so there's no per-arm sample period to snapshot here either.
        self._armed_current_profile_info = ""
        # The feedforward curve, resampled at TM_US (not the coarser
        # cadence actually downloaded/played back) so it lines up with
        # the recording's own sample grid for plotting -- see
        # _on_arm_current_profile and _show_capture's feedforward_v.
        self._armed_current_profile_ff_plot_v = []

        # Simple Open Loop: one open-loop current pulse, no position
        # control, no motion before or after. Default current 0 so a
        # stray Fire before anything is typed does nothing. Reuses the
        # shared Motion Sample Period and Pre/Post-Move Pause fields, so
        # it needs no sample-period var of its own.
        self.simple_ol_current_var = tk.StringVar(value="0")
        self.simple_ol_hold_ms_var = tk.StringVar(value="100")
        self._loaded_simple_ol_sample_period_s = DEFAULT_SAMPLE_PERIOD_US / 1_000_000
        self._loaded_simple_ol_info = ""

        # LVDT Calibration: up to 4 (position, captured reading) points
        # defining up to 3 piecewise-linear regions (Controller.
        # set_calibration_points). Point 1 is pinned at 0mm -- it defines
        # the origin -- and points 2-4 are optional, so 2/3/4 filled points
        # give 1/2/3 regions. No Galil software travel limits (BL/FL) are
        # used in this application at all, so there's no Min/Max field here
        # configuring a controller-side limit; range checks against
        # [0, FULL_STROKE_MM] happen client-side wherever a target position
        # is computed.
        # Points 1 and 4 default to the bench-measured ends of travel, so
        # the out-of-the-box 2-point calibration spans the full stroke and
        # the two middle rows are free for intermediate points without
        # having to retype an end.
        self.cal_position_vars = [
            tk.StringVar(value=f"{RETRACT_MM:.2f}"),
            tk.StringVar(value=""),
            tk.StringVar(value=""),
            tk.StringVar(value=f"{EXTEND_MM:.2f}"),
        ]
        self.cal_voltage_vars = [
            tk.StringVar(value=f"{RETRACT_VOLTS:.4f}"),
            tk.StringVar(value=""),
            tk.StringVar(value=""),
            tk.StringVar(value=f"{EXTEND_VOLTS:.4f}"),
        ]
        # Per-row tag to the right of each Set button: whether that row's
        # voltage was measured or typed, or the row is being skipped.
        self.cal_captured_vars = [tk.StringVar(value="") for _ in range(4)]
        # Counts per point, kept alongside the voltage entry above (None
        # until that row has a usable voltage). A Set press stores the
        # directly measured counts; a hand-typed voltage is converted with
        # Controller.volts_to_counts. _cal_volts tracks which voltage each
        # stored count came from, so an edited entry is recognized and
        # reconverted while a captured one keeps its measured counts.
        self._cal_counts = [None] * 4
        self._cal_volts = [None] * 4
        self._cal_sampling = False
        self.scale_var = tk.StringVar(value="")

        self.record_var = tk.BooleanVar(value=True)
        self._recording_sequence = RecordingSequence(
            self.controller, self.root.after, self.status_var.set,
            self._apply_motion_settings, self._apply_rest_settings,
            self._on_recording_complete,
        )

        self._build_widgets()
        # The default points are a complete 2-point calibration on their
        # own, so show their scales straight away rather than leaving the
        # section blank until something is connected or captured.
        self._refresh_calibration_display()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_widgets(self):
        # Labeled "Disable" (not "Abort") since a plain disable() alone
        # doesn't reliably stop a running move/program -- this button does
        # both (Controller.abort(): AB then MOA), so "Disable" is the
        # accurate, unambiguous label for what it actually does. Always
        # enabled, never gated on connection state. Parented directly to
        # self.root and packed *before* the scrollable container below,
        # so it stays fixed at the top of the window -- reachable as the
        # fastest possible abort -- regardless of scroll position.
        disable_button = tk.Button(
            self.root,
            text="Disable",
            command=self._on_disable,
            bg="red",
            fg="white",
            activebackground="darkred",
            activeforeground="white",
            font=("Arial", 14, "bold"),
            height=2,
        )
        disable_button.pack(padx=10, pady=(10, 0), fill="x")

        # Vertical scrollbar -- the window's natural content height
        # exceeds smaller laptop screens with no way to reach the bottom
        # controls otherwise. content is the real parent for every
        # top-level widget below (self.root itself stays reserved for
        # actual window-level operations: title/geometry/after/protocol/
        # the Disable button above/the capture-plot Toplevel/destroy).
        container = tk.Frame(self.root)
        container.pack(fill="both", expand=True)

        canvas = tk.Canvas(container, highlightthickness=0)
        scrollbar = tk.Scrollbar(container, orient="vertical", command=canvas.yview)
        content = tk.Frame(canvas)

        content.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas_window = canvas.create_window((0, 0), window=content, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Keep packed rows (fill="x") spanning the actual visible width.
        canvas.bind(
            "<Configure>", lambda e: canvas.itemconfig(canvas_window, width=e.width)
        )
        # Mouse-wheel scrolling (Windows delta is +/-120 per notch).
        canvas.bind_all(
            "<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta / 120), "units")
        )

        connection_frame = tk.Frame(content)
        connection_frame.pack(padx=10, pady=10, fill="x")

        tk.Label(connection_frame, text="IP:").pack(side="left")
        self.address_entry = tk.Entry(
            connection_frame, textvariable=self.address_var, width=16
        )
        self.address_entry.pack(side="left", padx=(4, 10))

        self.connect_button = tk.Button(
            connection_frame, text="Connect", command=self._on_connect
        )
        self.connect_button.pack(side="left")

        self.disconnect_button = tk.Button(
            connection_frame,
            text="Disconnect",
            command=self._on_disconnect,
            state="disabled",
        )
        self.disconnect_button.pack(side="left", padx=(4, 0))

        servo_frame = tk.Frame(content)
        servo_frame.pack(padx=10, fill="x")

        self.enable_button = tk.Button(
            servo_frame, text="Enable", command=self._on_enable, state="disabled"
        )
        self.enable_button.pack(side="left", padx=(0, 10))

        self.servo_status_label = tk.Label(
            servo_frame, textvariable=self.servo_status_var, anchor="w"
        )
        self.servo_status_label.pack(side="left")

        tk.Label(content, textvariable=self.status_var, anchor="w").pack(
            padx=10, fill="x"
        )

        readout_frame = tk.Frame(content)
        readout_frame.pack(padx=10, pady=10, fill="x")

        self._add_readout_row(readout_frame, "Position (counts):", self.position_counts_var)
        self._add_readout_row(readout_frame, "Position (mm):", self.position_mm_var)
        self._add_readout_row(readout_frame, "LVDT voltage (V):", self.lvdt_voltage_var)
        self._add_readout_row(readout_frame, "Output Current (A):", self.output_current_var)

        # Amp's own CME "Analog Input Scale" setting -- purely a display/
        # plotting unit-conversion value (Controller.volts_to_current),
        # not sent to the Galil controller at all, so it just needs an
        # explicit Apply rather than being baked into any downloaded
        # program.
        amp_gain_frame = tk.Frame(content)
        amp_gain_frame.pack(padx=10, pady=(0, 10), fill="x")
        tk.Label(
            amp_gain_frame,
            text=(
                f"Amp Full-Scale Current (A) "
                f"[{AMP_FULL_SCALE_CURRENT_MIN_A}-{AMP_FULL_SCALE_CURRENT_MAX_A}]:"
            ),
        ).pack(side="left")
        self.amp_full_scale_current_entry = tk.Entry(
            amp_gain_frame, textvariable=self.amp_full_scale_current_var, width=8
        )
        self.amp_full_scale_current_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            amp_gain_frame, text="Apply", command=self._on_apply_amp_gain
        ).pack(side="left")

        # No Galil software travel limits (BL/FL) are used in this
        # application -- this section instead records what the controller's
        # raw position means in mm at up to four points, which every other
        # mm field in the app is checked against client-side. Gated on
        # connection only, same as Download/Run -- no motion is involved,
        # just a position read.
        limits_frame = tk.LabelFrame(content, text="LVDT Calibration")
        limits_frame.pack(padx=10, pady=(0, 10), fill="x")

        # Point 1 is pinned at 0mm (readonly) since it defines the origin;
        # 2-4 are free, and a row with either field blank is skipped. Each
        # row's voltage can be typed directly or filled in by Set, which
        # averages ~1s of readings (see CALIBRATION_SAMPLE_COUNT) -- typed
        # and captured voltages are treated identically from there on, so a
        # calibration can be restored by hand without the rig connected.
        self.cal_set_buttons = []
        self.cal_position_entries = []
        self.cal_voltage_entries = []
        for index in range(4):
            row = tk.Frame(limits_frame)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=f"Point {index + 1} (mm):", width=14, anchor="w").pack(side="left")
            entry = tk.Entry(row, textvariable=self.cal_position_vars[index], width=8)
            if index == 0:
                entry.config(state="readonly")
            else:
                # Re-derive the map when a field is edited, so changing a
                # row after capturing it takes effect -- but only on
                # commit, not on every keystroke.
                self._bind_calibration_commit(entry)
            entry.pack(side="left", padx=(2, 8))
            self.cal_position_entries.append(entry)
            tk.Label(row, text="LVDT (V):", width=8, anchor="w").pack(side="left")
            voltage_entry = tk.Entry(
                row, textvariable=self.cal_voltage_vars[index], width=9
            )
            self._bind_calibration_commit(voltage_entry)
            voltage_entry.pack(side="left", padx=(2, 8))
            self.cal_voltage_entries.append(voltage_entry)
            button = tk.Button(
                row, text="Set", state="disabled",
                command=lambda i=index: self._on_set_calibration_point(i),
            )
            button.pack(side="left", padx=(0, 8))
            self.cal_set_buttons.append(button)
            tk.Label(
                row, textvariable=self.cal_captured_vars[index], anchor="w", width=16,
            ).pack(side="left")

        # One line per active region plus the overall linear scale -- with
        # more than one region the per-region numbers are the whole point,
        # since how much they differ from each other IS the nonlinearity.
        tk.Label(
            limits_frame, textvariable=self.scale_var, anchor="w",
            justify="left", font=("TkFixedFont", 8),
        ).pack(fill="x", pady=(4, 0))
        # LVDT mode-2/mode-1 conditioning pulses (handoff notes, section
        # 4) -- condition to mode 2 BEFORE capturing calibration points.
        self.conditioning_controls = ConditioningControls(self, limits_frame)

        motion_frame = tk.LabelFrame(content, text="Closed Loop Motion")
        motion_frame.pack(padx=10, pady=(0, 10), fill="x")

        target_row = tk.Frame(motion_frame)
        target_row.pack(fill="x", pady=2)
        tk.Label(target_row, text="Target (mm):", width=18, anchor="w").pack(side="left")
        self.target_entry = tk.Entry(target_row, textvariable=self.target_var, width=10)
        self.target_entry.pack(side="left")
        self.go_button = tk.Button(
            target_row, text="Go", command=self._on_go, state="disabled"
        )
        self.go_button.pack(side="left", padx=(10, 0))
        tk.Checkbutton(target_row, text="Record", variable=self.record_var).pack(
            side="left", padx=(10, 0)
        )
        # Always move to the full calibrated [0, FULL_STROKE_MM] range at a
        # fixed slow profile, leave PID gains exactly as-is, and never
        # record -- even if Record is checked. See _slow_move_to_limit.
        self.slow_retract_button = tk.Button(
            target_row, text="Slow Retract", command=self._on_slow_retract,
            state="disabled",
        )
        self.slow_retract_button.pack(side="left", padx=(10, 0))
        self.slow_extend_button = tk.Button(
            target_row, text="Slow Extend", command=self._on_slow_extend,
            state="disabled",
        )
        self.slow_extend_button.pack(side="left", padx=(4, 0))

        profile_row = tk.Frame(motion_frame)
        profile_row.pack(fill="x", pady=2)
        tk.Label(
            profile_row,
            text=f"Speed (m/s) [{SPEED_RANGE_M_S[0]:.3f}-{SPEED_RANGE_M_S[1]:.3f}]:",
        ).pack(side="left")
        self.speed_entry = tk.Entry(profile_row, textvariable=self.speed_var, width=8)
        self.speed_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            profile_row,
            text=f"Accel (m/s²) [{ACCEL_RANGE_M_S2[0]:.3f}-{ACCEL_RANGE_M_S2[1]:.3f}]:",
        ).pack(side="left")
        self.accel_entry = tk.Entry(profile_row, textvariable=self.accel_var, width=8)
        self.accel_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            profile_row,
            text=f"Decel (m/s²) [{ACCEL_RANGE_M_S2[0]:.3f}-{ACCEL_RANGE_M_S2[1]:.3f}]:",
        ).pack(side="left")
        self.decel_entry = tk.Entry(profile_row, textvariable=self.decel_var, width=8)
        self.decel_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            profile_row, text="Reset", command=self._on_reset_motion_profile
        ).pack(side="left")

        # Rest gains apply on every Enable click (before SHA) and again the
        # instant a move completes; Motion gains apply right before a move
        # begins. See _on_enable / _on_go / _apply_rest_pid_gains.
        rest_pid_frame = tk.LabelFrame(content, text="Rest PID")
        rest_pid_frame.pack(padx=10, pady=(0, 10), fill="x")

        rest_pid_row = tk.Frame(rest_pid_frame)
        rest_pid_row.pack(fill="x", pady=2)
        tk.Label(rest_pid_row, text="KP:").pack(side="left")
        self.rest_kp_entry = tk.Entry(rest_pid_row, textvariable=self.rest_kp_var, width=8)
        self.rest_kp_entry.pack(side="left", padx=(2, 8))
        tk.Label(rest_pid_row, text="KD:").pack(side="left")
        self.rest_kd_entry = tk.Entry(rest_pid_row, textvariable=self.rest_kd_var, width=8)
        self.rest_kd_entry.pack(side="left", padx=(2, 8))
        tk.Label(rest_pid_row, text="KI:").pack(side="left")
        self.rest_ki_entry = tk.Entry(rest_pid_row, textvariable=self.rest_ki_var, width=8)
        self.rest_ki_entry.pack(side="left", padx=(2, 8))
        tk.Label(rest_pid_row, text=f"Offset (V) [{OF_MIN_V}-{OF_MAX_V}]:").pack(side="left")
        self.rest_offset_entry = tk.Entry(
            rest_pid_row, textvariable=self.rest_offset_var, width=8
        )
        self.rest_offset_entry.pack(side="left", padx=(2, 8))
        tk.Button(rest_pid_row, text="Reset", command=self._on_reset_rest_pid_gains).pack(
            side="left"
        )

        motion_pid_frame = tk.LabelFrame(content, text="Motion PID")
        motion_pid_frame.pack(padx=10, pady=(0, 10), fill="x")

        motion_pid_row = tk.Frame(motion_pid_frame)
        motion_pid_row.pack(fill="x", pady=2)
        tk.Label(motion_pid_row, text="KP:").pack(side="left")
        self.motion_kp_entry = tk.Entry(motion_pid_row, textvariable=self.motion_kp_var, width=8)
        self.motion_kp_entry.pack(side="left", padx=(2, 8))
        tk.Label(motion_pid_row, text="KD:").pack(side="left")
        self.motion_kd_entry = tk.Entry(motion_pid_row, textvariable=self.motion_kd_var, width=8)
        self.motion_kd_entry.pack(side="left", padx=(2, 8))
        tk.Label(motion_pid_row, text="KI:").pack(side="left")
        self.motion_ki_entry = tk.Entry(motion_pid_row, textvariable=self.motion_ki_var, width=8)
        self.motion_ki_entry.pack(side="left", padx=(2, 8))
        tk.Label(motion_pid_row, text=f"Offset (V) [{OF_MIN_V}-{OF_MAX_V}]:").pack(side="left")
        self.motion_offset_entry = tk.Entry(
            motion_pid_row, textvariable=self.motion_offset_var, width=8
        )
        self.motion_offset_entry.pack(side="left", padx=(2, 8))
        tk.Label(motion_pid_row, text=f"FV [{FV_MIN}-{FV_MAX}]:").pack(side="left")
        self.motion_fv_entry = tk.Entry(motion_pid_row, textvariable=self.motion_fv_var, width=8)
        self.motion_fv_entry.pack(side="left", padx=(2, 8))
        tk.Button(motion_pid_row, text="Reset", command=self._on_reset_motion_pid_gains).pack(
            side="left"
        )

        motion_sample_period_row = tk.Frame(motion_pid_frame)
        motion_sample_period_row.pack(fill="x", pady=2)
        tk.Label(
            motion_sample_period_row,
            text=f"Sample Period (usec) [{TM_MIN_US}-{TM_MAX_US}]:",
        ).pack(side="left")
        self.motion_sample_period_entry = tk.Entry(
            motion_sample_period_row, textvariable=self.motion_sample_period_us_var, width=8,
        )
        self.motion_sample_period_entry.pack(side="left", padx=(2, 8))

        # Both pauses are counted as part of the recording -- see
        # RecordingSequence.start(). Only meaningful when Record is checked.
        recording_frame = tk.LabelFrame(content, text="Recording Settings")
        recording_frame.pack(padx=10, pady=(0, 10), fill="x")

        recording_row = tk.Frame(recording_frame)
        recording_row.pack(fill="x", pady=2)
        tk.Label(recording_row, text="Pre-Move Pause (ms):").pack(side="left")
        self.pre_pause_entry = tk.Entry(
            recording_row, textvariable=self.pre_pause_ms_var, width=8
        )
        self.pre_pause_entry.pack(side="left", padx=(2, 8))
        tk.Label(recording_row, text="Post-Move Pause (ms):").pack(side="left")
        self.post_pause_entry = tk.Entry(
            recording_row, textvariable=self.post_pause_ms_var, width=8
        )
        self.post_pause_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            recording_row, text="Reset", command=self._on_reset_recording_settings
        ).pack(side="left")

        # Timing-critical/impact moves belong in DMC programs on the
        # controller, not commanded from Python -- Download stores a .dmc's
        # text on the controller, Run triggers it (XQ). A downloaded
        # program manages its own SHA/MO, independent of the app's own
        # Enable/Rest-PID state machine, so these aren't gated on servo
        # state like Go/Enable are -- only on being connected. Disable
        # (always enabled) stops a running program early, same as any move.
        program_frame = tk.LabelFrame(content, text="Program")
        program_frame.pack(padx=10, pady=(0, 10), fill="x")

        program_file_row = tk.Frame(program_frame)
        program_file_row.pack(fill="x", pady=2)
        tk.Label(program_file_row, text="File:").pack(side="left")
        self.program_path_entry = tk.Entry(
            program_file_row, textvariable=self.program_path_var, width=40,
            state="readonly",
        )
        self.program_path_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            program_file_row, text="Browse...", command=self._on_browse_program
        ).pack(side="left")

        program_action_row = tk.Frame(program_frame)
        program_action_row.pack(fill="x", pady=2)
        self.download_program_button = tk.Button(
            program_action_row, text="Download", command=self._on_download_program,
            state="disabled",
        )
        self.download_program_button.pack(side="left", padx=(0, 8))
        self.run_program_button = tk.Button(
            program_action_row, text="Run", command=self._on_run_program,
            state="disabled",
        )
        self.run_program_button.pack(side="left")

        # Contour Mode position move (thread 0) running concurrently with
        # a feedforward current/voltage curve -- derived from a smooth,
        # jerk-limited trajectory's acceleration via FF(t)=m*a(t)/Kf --
        # played back into OF from a host-downloaded array (thread 1), so
        # the PID only has to trim a small residual instead of tracking
        # the whole profile through position error alone. No Motion FV or
        # Sample Period fields here (unlike Contour/PVT): this mode's own
        # feedforward loop replaces FV's role entirely, and TM is fixed at
        # the bench-confirmed current_profile_program.TM_US, not user-
        # configurable. Own Pre/Post-Move Pause fields (not the shared
        # ones) since this mode's array-table budget is tight -- see
        # current_profile_program.py. Listed first among the motion
        # modes -- the main motion tool in regular use.
        current_profile_frame = tk.LabelFrame(content, text="Current Profile Motion")
        current_profile_frame.pack(padx=10, pady=(0, 10), fill="x")

        current_profile_range_row = tk.Frame(current_profile_frame)
        current_profile_range_row.pack(fill="x", pady=2)
        tk.Label(current_profile_range_row, text="Start (mm):").pack(side="left")
        self.current_profile_start_entry = tk.Entry(
            current_profile_range_row, textvariable=self.current_profile_start_var, width=8
        )
        self.current_profile_start_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_range_row, text="Stop (mm):").pack(side="left")
        self.current_profile_stop_entry = tk.Entry(
            current_profile_range_row, textvariable=self.current_profile_stop_var, width=8
        )
        self.current_profile_stop_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            current_profile_range_row, text="Reset", command=self._on_reset_current_profile_range
        ).pack(side="left")

        current_profile_fields_row = tk.Frame(current_profile_frame)
        current_profile_fields_row.pack(fill="x", pady=2)
        tk.Label(current_profile_fields_row, text="Impact Velocity (m/s):").pack(side="left")
        self.current_profile_velocity_entry = tk.Entry(
            current_profile_fields_row, textvariable=self.current_profile_velocity_var, width=8
        )
        self.current_profile_velocity_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_fields_row, text="Impact Position (mm):").pack(side="left")
        self.current_profile_position_entry = tk.Entry(
            current_profile_fields_row, textvariable=self.current_profile_position_var, width=8
        )
        self.current_profile_position_entry.pack(side="left", padx=(2, 8))

        current_profile_peak_row = tk.Frame(current_profile_frame)
        current_profile_peak_row.pack(fill="x", pady=2)
        tk.Label(current_profile_peak_row, text="Peak Velocity Position (mm):").pack(side="left")
        self.current_profile_peak_velocity_position_entry = tk.Entry(
            current_profile_peak_row,
            textvariable=self.current_profile_peak_velocity_position_var,
            width=8,
        )
        self.current_profile_peak_velocity_position_entry.pack(side="left", padx=(2, 8))

        current_profile_ff_row = tk.Frame(current_profile_frame)
        current_profile_ff_row.pack(fill="x", pady=2)
        tk.Label(current_profile_ff_row, text="Mass (kg):").pack(side="left")
        self.current_profile_mass_entry = tk.Entry(
            current_profile_ff_row, textvariable=self.current_profile_mass_var, width=8
        )
        self.current_profile_mass_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_ff_row, text="Force Constant Accel (Kf):").pack(side="left")
        self.current_profile_kf_accel_entry = tk.Entry(
            current_profile_ff_row, textvariable=self.current_profile_kf_accel_var, width=8
        )
        self.current_profile_kf_accel_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_ff_row, text="Force Constant Decel (Kf):").pack(side="left")
        self.current_profile_kf_decel_entry = tk.Entry(
            current_profile_ff_row, textvariable=self.current_profile_kf_decel_var, width=8
        )
        self.current_profile_kf_decel_entry.pack(side="left", padx=(2, 8))

        # Feedforward curve reshaping -- independently stretches each of
        # the curve's 4 quarter-segments in time (see
        # current_profile_trajectory.compute_feedforward_v's own
        # accel_rise_stretch/accel_fall_stretch/decel_rise_stretch/
        # decel_fall_stretch docstring); never touches the physical CD
        # move.
        current_profile_stretch_row = tk.Frame(current_profile_frame)
        current_profile_stretch_row.pack(fill="x", pady=2)
        tk.Label(current_profile_stretch_row, text="Accel Rise Stretch:").pack(side="left")
        self.current_profile_ff_accel_rise_stretch_entry = tk.Entry(
            current_profile_stretch_row,
            textvariable=self.current_profile_ff_accel_rise_stretch_var, width=6,
        )
        self.current_profile_ff_accel_rise_stretch_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_stretch_row, text="Accel Fall Stretch:").pack(side="left")
        self.current_profile_ff_accel_fall_stretch_entry = tk.Entry(
            current_profile_stretch_row,
            textvariable=self.current_profile_ff_accel_fall_stretch_var, width=6,
        )
        self.current_profile_ff_accel_fall_stretch_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_stretch_row, text="Decel Rise Stretch:").pack(side="left")
        self.current_profile_ff_decel_rise_stretch_entry = tk.Entry(
            current_profile_stretch_row,
            textvariable=self.current_profile_ff_decel_rise_stretch_var, width=6,
        )
        self.current_profile_ff_decel_rise_stretch_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_stretch_row, text="Decel Fall Stretch:").pack(side="left")
        self.current_profile_ff_decel_fall_stretch_entry = tk.Entry(
            current_profile_stretch_row,
            textvariable=self.current_profile_ff_decel_fall_stretch_var, width=6,
        )
        self.current_profile_ff_decel_fall_stretch_entry.pack(side="left", padx=(2, 8))

        current_profile_pause_row = tk.Frame(current_profile_frame)
        current_profile_pause_row.pack(fill="x", pady=2)
        tk.Label(current_profile_pause_row, text="Pre-Move Pause (ms):").pack(side="left")
        self.current_profile_pre_pause_entry = tk.Entry(
            current_profile_pause_row, textvariable=self.current_profile_pre_pause_ms_var, width=8
        )
        self.current_profile_pre_pause_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_pause_row, text="Post-Move Pause (ms):").pack(side="left")
        self.current_profile_post_pause_entry = tk.Entry(
            current_profile_pause_row, textvariable=self.current_profile_post_pause_ms_var, width=8
        )
        self.current_profile_post_pause_entry.pack(side="left", padx=(2, 8))
        tk.Label(current_profile_pause_row, text="Feedforward Advance (us):").pack(side="left")
        self.current_profile_ff_advance_entry = tk.Entry(
            current_profile_pause_row, textvariable=self.current_profile_ff_advance_us_var, width=8
        )
        self.current_profile_ff_advance_entry.pack(side="left", padx=(2, 8))

        current_profile_action_row = tk.Frame(current_profile_frame)
        current_profile_action_row.pack(fill="x", pady=2)
        self.current_profile_arm_button = tk.Button(
            current_profile_action_row, text="Arm", command=self._on_arm_current_profile,
            state="disabled",
        )
        self.current_profile_arm_button.pack(side="left", padx=(0, 8))
        self.current_profile_fire_button = tk.Button(
            current_profile_action_row, text="Fire", command=self._on_fire_current_profile,
            state="disabled",
        )
        self.current_profile_fire_button.pack(side="left")

        # Full impact sequence built on the Current Profile fields above
        # (conditioning pulse, dwell, retract, two-segment recording) and
        # the profile-file hand-off to the user GUI -- see
        # gui/sequence_panel.py; shares this frame's fields by reference.
        self.sequence_panel = SequencePanel(self, content)

        # Closed-loop trajectory: Arm computes a constant-acceleration
        # contour (0 velocity at Start -> Impact Velocity at Impact Position
        # -> 0 velocity at Stop), using the Galil Contour Mode (CM/CD/DT),
        # and downloads it as a program -- same Arm/Fire pattern as Open
        # Loop Motion, but its own independent armed state and its own
        # Start/Stop range. Reuses Motion PID, Motion Sample Period, and
        # Pre/Post-Move Pause fields.
        contour_frame = tk.LabelFrame(content, text="Contoured Motion")
        contour_frame.pack(padx=10, pady=(0, 10), fill="x")

        contour_range_row = tk.Frame(contour_frame)
        contour_range_row.pack(fill="x", pady=2)
        tk.Label(contour_range_row, text="Start (mm):").pack(side="left")
        self.contour_start_entry = tk.Entry(
            contour_range_row, textvariable=self.contour_start_var, width=8
        )
        self.contour_start_entry.pack(side="left", padx=(2, 8))
        tk.Label(contour_range_row, text="Stop (mm):").pack(side="left")
        self.contour_stop_entry = tk.Entry(
            contour_range_row, textvariable=self.contour_stop_var, width=8
        )
        self.contour_stop_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            contour_range_row, text="Reset", command=self._on_reset_contour_range
        ).pack(side="left")

        contour_fields_row = tk.Frame(contour_frame)
        contour_fields_row.pack(fill="x", pady=2)
        tk.Label(contour_fields_row, text="Impact Velocity (m/s):").pack(side="left")
        self.contour_velocity_entry = tk.Entry(
            contour_fields_row, textvariable=self.contour_velocity_var, width=8
        )
        self.contour_velocity_entry.pack(side="left", padx=(2, 8))
        tk.Label(contour_fields_row, text="Impact Position (mm):").pack(side="left")
        self.contour_position_entry = tk.Entry(
            contour_fields_row, textvariable=self.contour_position_var, width=8
        )
        self.contour_position_entry.pack(side="left", padx=(2, 8))

        contour_action_row = tk.Frame(contour_frame)
        contour_action_row.pack(fill="x", pady=2)
        self.contour_arm_button = tk.Button(
            contour_action_row, text="Arm", command=self._on_arm_contour, state="disabled"
        )
        self.contour_arm_button.pack(side="left", padx=(0, 8))
        self.contour_fire_button = tk.Button(
            contour_action_row, text="Fire", command=self._on_fire_contour, state="disabled"
        )
        self.contour_fire_button.pack(side="left")

        # Same trajectory as Contoured Motion, but driven through the
        # Galil's PVT mode (PV/BT) instead of Contour Mode (CD/DT). PVT
        # motion itself ignores AC/DC/SP for shaping the trajectory --
        # its own AC/DC only feed the FA feedforward formula. Reuses
        # Motion PID (KP/KD/KI/Offset only -- FV is overridden by PVT's
        # own FV below), Motion Sample Period, and Pre/Post-Move Pause
        # fields, same as Contoured Motion.
        pvt_frame = tk.LabelFrame(content, text="PVT Motion")
        pvt_frame.pack(padx=10, pady=(0, 10), fill="x")

        pvt_range_row = tk.Frame(pvt_frame)
        pvt_range_row.pack(fill="x", pady=2)
        tk.Label(pvt_range_row, text="Start (mm):").pack(side="left")
        self.pvt_start_entry = tk.Entry(
            pvt_range_row, textvariable=self.pvt_start_var, width=8
        )
        self.pvt_start_entry.pack(side="left", padx=(2, 8))
        tk.Label(pvt_range_row, text="Stop (mm):").pack(side="left")
        self.pvt_stop_entry = tk.Entry(
            pvt_range_row, textvariable=self.pvt_stop_var, width=8
        )
        self.pvt_stop_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            pvt_range_row, text="Reset", command=self._on_reset_pvt_range
        ).pack(side="left")

        pvt_fields_row = tk.Frame(pvt_frame)
        pvt_fields_row.pack(fill="x", pady=2)
        tk.Label(pvt_fields_row, text="Impact Velocity (m/s):").pack(side="left")
        self.pvt_velocity_entry = tk.Entry(
            pvt_fields_row, textvariable=self.pvt_velocity_var, width=8
        )
        self.pvt_velocity_entry.pack(side="left", padx=(2, 8))
        tk.Label(pvt_fields_row, text="Impact Position (mm):").pack(side="left")
        self.pvt_position_entry = tk.Entry(
            pvt_fields_row, textvariable=self.pvt_position_var, width=8
        )
        self.pvt_position_entry.pack(side="left", padx=(2, 8))

        pvt_feedforward_row = tk.Frame(pvt_frame)
        pvt_feedforward_row.pack(fill="x", pady=2)
        tk.Label(pvt_feedforward_row, text=f"PVT FV [{FV_MIN}-{FV_MAX}]:").pack(side="left")
        self.pvt_fv_entry = tk.Entry(
            pvt_feedforward_row, textvariable=self.pvt_fv_var, width=8
        )
        self.pvt_fv_entry.pack(side="left", padx=(2, 8))
        tk.Label(pvt_feedforward_row, text=f"PVT FA [{FA_MIN}-{FA_MAX}]:").pack(side="left")
        self.pvt_fa_entry = tk.Entry(
            pvt_feedforward_row, textvariable=self.pvt_fa_var, width=8
        )
        self.pvt_fa_entry.pack(side="left", padx=(2, 8))

        pvt_profile_row = tk.Frame(pvt_frame)
        pvt_profile_row.pack(fill="x", pady=2)
        tk.Label(
            pvt_profile_row,
            text=f"PVT Accel (m/s²) [{ACCEL_RANGE_M_S2[0]:.3f}-{ACCEL_RANGE_M_S2[1]:.3f}]:",
        ).pack(side="left")
        self.pvt_ac_entry = tk.Entry(pvt_profile_row, textvariable=self.pvt_ac_var, width=8)
        self.pvt_ac_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            pvt_profile_row,
            text=f"PVT Decel (m/s²) [{ACCEL_RANGE_M_S2[0]:.3f}-{ACCEL_RANGE_M_S2[1]:.3f}]:",
        ).pack(side="left")
        self.pvt_dc_entry = tk.Entry(pvt_profile_row, textvariable=self.pvt_dc_var, width=8)
        self.pvt_dc_entry.pack(side="left", padx=(2, 8))

        pvt_action_row = tk.Frame(pvt_frame)
        pvt_action_row.pack(fill="x", pady=2)
        self.pvt_arm_button = tk.Button(
            pvt_action_row, text="Arm", command=self._on_arm_pvt, state="disabled"
        )
        self.pvt_arm_button.pack(side="left", padx=(0, 8))
        self.pvt_fire_button = tk.Button(
            pvt_action_row, text="Fire", command=self._on_fire_pvt, state="disabled"
        )
        self.pvt_fire_button.pack(side="left")

        # Arm moves to Min (slow-retract parameters) and downloads a
        # built-from-current-fields impact program (see
        # program.impact_program.build_impact_program); Fire runs it and
        # plots the result. No SHA in that program -- Disable is the
        # master abort here, so Arm is gated on the servo already being
        # enabled, and Fire is grayed out the instant Disable is pressed
        # and stays grayed out until a fresh Arm succeeds (see _poll).
        impact_frame = tk.LabelFrame(content, text="Open Loop Motion")
        impact_frame.pack(padx=10, pady=(0, 10), fill="x")

        impact_range_row = tk.Frame(impact_frame)
        impact_range_row.pack(fill="x", pady=2)
        tk.Label(impact_range_row, text="Start (mm):").pack(side="left")
        self.impact_start_entry = tk.Entry(
            impact_range_row, textvariable=self.impact_start_var, width=8
        )
        self.impact_start_entry.pack(side="left", padx=(2, 8))
        tk.Button(
            impact_range_row, text="Reset", command=self._on_reset_impact_range
        ).pack(side="left")

        impact_pre_retract_row = tk.Frame(impact_frame)
        impact_pre_retract_row.pack(fill="x", pady=2)
        tk.Label(
            impact_pre_retract_row, text="Pre Impact Retract Voltage (V) [>=0]:"
        ).pack(side="left")
        self.pre_impact_retract_voltage_entry = tk.Entry(
            impact_pre_retract_row, textvariable=self.pre_impact_retract_voltage_var, width=8
        )
        self.pre_impact_retract_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            impact_pre_retract_row, text="Pre Impact Retract Wait Time (ms) [0-1000]:"
        ).pack(side="left")
        self.pre_impact_retract_wait_entry = tk.Entry(
            impact_pre_retract_row, textvariable=self.pre_impact_retract_wait_ms_var, width=8
        )
        self.pre_impact_retract_wait_entry.pack(side="left", padx=(2, 8))

        impact_fields_row = tk.Frame(impact_frame)
        impact_fields_row.pack(fill="x", pady=2)
        tk.Label(
            impact_fields_row, text="Impact Accel Primary Voltage (V) [<=0]:"
        ).pack(side="left")
        self.impact_accel_primary_voltage_entry = tk.Entry(
            impact_fields_row, textvariable=self.impact_accel_primary_voltage_var, width=8
        )
        self.impact_accel_primary_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(impact_fields_row, text="Impact Accel Primary Wait Time (ms):").pack(
            side="left"
        )
        self.impact_accel_primary_wait_entry = tk.Entry(
            impact_fields_row, textvariable=self.impact_accel_primary_wait_ms_var, width=8
        )
        self.impact_accel_primary_wait_entry.pack(side="left", padx=(2, 8))

        impact_accel_secondary_row = tk.Frame(impact_frame)
        impact_accel_secondary_row.pack(fill="x", pady=2)
        tk.Label(
            impact_accel_secondary_row, text="Impact Accel Secondary Voltage (V):"
        ).pack(side="left")
        self.impact_accel_secondary_voltage_entry = tk.Entry(
            impact_accel_secondary_row,
            textvariable=self.impact_accel_secondary_voltage_var,
            width=8,
        )
        self.impact_accel_secondary_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            impact_accel_secondary_row,
            text="Impact Accel Secondary Wait Time (ms) [0-1000]:",
        ).pack(side="left")
        self.impact_accel_secondary_wait_entry = tk.Entry(
            impact_accel_secondary_row,
            textvariable=self.impact_accel_secondary_wait_ms_var,
            width=8,
        )
        self.impact_accel_secondary_wait_entry.pack(side="left", padx=(2, 8))

        impact_decel_row = tk.Frame(impact_frame)
        impact_decel_row.pack(fill="x", pady=2)
        tk.Label(impact_decel_row, text="Impact Decel Voltage (V) [>=0]:").pack(side="left")
        self.impact_decel_voltage_entry = tk.Entry(
            impact_decel_row, textvariable=self.impact_decel_voltage_var, width=8
        )
        self.impact_decel_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(impact_decel_row, text="Impact Decel Wait Time (ms):").pack(side="left")
        self.impact_decel_wait_entry = tk.Entry(
            impact_decel_row, textvariable=self.impact_decel_wait_ms_var, width=8
        )
        self.impact_decel_wait_entry.pack(side="left", padx=(2, 8))

        impact_hold_row = tk.Frame(impact_frame)
        impact_hold_row.pack(fill="x", pady=2)
        tk.Label(impact_hold_row, text="Hold Voltage (V) [<=0]:").pack(side="left")
        self.hold_voltage_entry = tk.Entry(
            impact_hold_row, textvariable=self.hold_voltage_var, width=8
        )
        self.hold_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(impact_hold_row, text="Hold Wait Time (ms) [0-1000]:").pack(side="left")
        self.hold_wait_entry = tk.Entry(
            impact_hold_row, textvariable=self.hold_wait_ms_var, width=8
        )
        self.hold_wait_entry.pack(side="left", padx=(2, 8))

        impact_retract_row = tk.Frame(impact_frame)
        impact_retract_row.pack(fill="x", pady=2)
        tk.Label(
            impact_retract_row, text="Retract Accel Primary Voltage (V) [>=0]:"
        ).pack(side="left")
        self.retract_accel_primary_voltage_entry = tk.Entry(
            impact_retract_row, textvariable=self.retract_accel_primary_voltage_var, width=8
        )
        self.retract_accel_primary_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            impact_retract_row, text="Retract Accel Primary Wait Time (ms) [0-1000]:"
        ).pack(side="left")
        self.retract_accel_primary_wait_entry = tk.Entry(
            impact_retract_row, textvariable=self.retract_accel_primary_wait_ms_var, width=8
        )
        self.retract_accel_primary_wait_entry.pack(side="left", padx=(2, 8))

        impact_retract_secondary_row = tk.Frame(impact_frame)
        impact_retract_secondary_row.pack(fill="x", pady=2)
        tk.Label(
            impact_retract_secondary_row,
            text="Retract Accel Secondary Voltage (V) [>=0]:",
        ).pack(side="left")
        self.retract_accel_secondary_voltage_entry = tk.Entry(
            impact_retract_secondary_row,
            textvariable=self.retract_accel_secondary_voltage_var,
            width=8,
        )
        self.retract_accel_secondary_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            impact_retract_secondary_row,
            text="Retract Accel Secondary Wait Time (ms) [0-1000]:",
        ).pack(side="left")
        self.retract_accel_secondary_wait_entry = tk.Entry(
            impact_retract_secondary_row,
            textvariable=self.retract_accel_secondary_wait_ms_var,
            width=8,
        )
        self.retract_accel_secondary_wait_entry.pack(side="left", padx=(2, 8))

        impact_retract_decel_row = tk.Frame(impact_frame)
        impact_retract_decel_row.pack(fill="x", pady=2)
        tk.Label(
            impact_retract_decel_row, text="Retract Decel Voltage (V) [<=0]:"
        ).pack(side="left")
        self.retract_decel_voltage_entry = tk.Entry(
            impact_retract_decel_row, textvariable=self.retract_decel_voltage_var, width=8
        )
        self.retract_decel_voltage_entry.pack(side="left", padx=(2, 8))
        tk.Label(
            impact_retract_decel_row, text="Retract Decel Wait Time (ms) [0-1000]:"
        ).pack(side="left")
        self.retract_decel_wait_entry = tk.Entry(
            impact_retract_decel_row, textvariable=self.retract_decel_wait_ms_var, width=8
        )
        self.retract_decel_wait_entry.pack(side="left", padx=(2, 8))

        impact_end_move_row = tk.Frame(impact_frame)
        impact_end_move_row.pack(fill="x", pady=2)
        tk.Checkbutton(
            impact_end_move_row,
            text="End with closed loop move to 2mm",
            variable=self.end_with_closed_loop_move_var,
        ).pack(side="left")

        impact_action_row = tk.Frame(impact_frame)
        impact_action_row.pack(fill="x", pady=2)
        self.arm_button = tk.Button(
            impact_action_row, text="Arm", command=self._on_arm, state="disabled"
        )
        self.arm_button.pack(side="left", padx=(0, 8))
        self.fire_button = tk.Button(
            impact_action_row, text="Fire", command=self._on_fire, state="disabled"
        )
        self.fire_button.pack(side="left")

        # A single open-loop current pulse with no position control and
        # no motion commanded before or after -- instrumentation for the
        # LVDT magnetization investigation (park the striker against a
        # stop by hand, apply a known current with position pinned, see
        # what the reading does). Load only downloads the program: unlike
        # every other mode's Arm it does not enable the servo and does
        # not move anywhere, since enabling or moving first would destroy
        # the magnetic state being measured. Reuses the shared Motion
        # Sample Period and Pre/Post-Move Pause fields; recording is
        # always on. See program.simple_open_loop_program.
        simple_ol_frame = tk.LabelFrame(content, text="Simple Open Loop")
        simple_ol_frame.pack(padx=10, pady=(0, 10), fill="x")

        simple_ol_fields_row = tk.Frame(simple_ol_frame)
        simple_ol_fields_row.pack(fill="x", pady=2)
        tk.Label(simple_ol_fields_row, text="Current (A) [<0 = extend]:").pack(side="left")
        self.simple_ol_current_entry = tk.Entry(
            simple_ol_fields_row, textvariable=self.simple_ol_current_var, width=8
        )
        self.simple_ol_current_entry.pack(side="left", padx=(2, 8))
        tk.Label(simple_ol_fields_row, text="Hold Delay (ms):").pack(side="left")
        self.simple_ol_hold_entry = tk.Entry(
            simple_ol_fields_row, textvariable=self.simple_ol_hold_ms_var, width=8
        )
        self.simple_ol_hold_entry.pack(side="left", padx=(2, 8))

        simple_ol_action_row = tk.Frame(simple_ol_frame)
        simple_ol_action_row.pack(fill="x", pady=2)
        self.simple_ol_load_button = tk.Button(
            simple_ol_action_row, text="Load", command=self._on_load_simple_open_loop,
            state="disabled",
        )
        self.simple_ol_load_button.pack(side="left", padx=(0, 8))
        self.simple_ol_fire_button = tk.Button(
            simple_ol_action_row, text="Fire", command=self._on_fire_simple_open_loop,
            state="disabled",
        )
        self.simple_ol_fire_button.pack(side="left")

    @staticmethod
    def _add_readout_row(parent, label_text, value_var):
        row = tk.Frame(parent)
        row.pack(fill="x", pady=2)
        tk.Label(row, text=label_text, width=18, anchor="w").pack(side="left")
        tk.Label(row, textvariable=value_var, anchor="w").pack(side="left")

    def _on_connect(self):
        address = self.address_var.get()
        try:
            self.controller.connect(address)
        except gclib.GclibError as e:
            self.status_var.set(f"Connect failed: {e}")
            return

        self.status_var.set(f"Connected to {address}")
        self.connect_button.config(state="disabled")
        self.disconnect_button.config(state="normal")
        self.address_entry.config(state="disabled")
        # Enable/Go stay disabled until the first poll tick confirms actual
        # servo state -- a fresh connection doesn't guarantee the servo
        # starts off (e.g. recovering from a crash that skipped the
        # auto-disable).
        self.enable_button.config(state="disabled")
        self.go_button.config(state="disabled")
        self.slow_retract_button.config(state="disabled")
        self.slow_extend_button.config(state="disabled")
        self.arm_button.config(state="disabled")
        self.contour_arm_button.config(state="disabled")
        self.pvt_arm_button.config(state="disabled")
        self.current_profile_arm_button.config(state="disabled")
        # Download/Run/Set LVDT Voltage aren't gated on servo state like
        # the above -- just on being connected -- so they go straight to
        # normal here. Simple Open Loop's Load joins them: that mode is
        # used entirely from a disabled servo, so gating it on servo
        # state would make it permanently unusable.
        self.download_program_button.config(state="normal")
        self.run_program_button.config(state="normal")
        for button in self.cal_set_buttons:
            button.config(state="normal")
        self.simple_ol_load_button.config(state="normal")
        # Fire only ever gets enabled by a successful Arm/Load, never here.
        self.fire_button.config(state="disabled")
        self.contour_fire_button.config(state="disabled")
        self.pvt_fire_button.config(state="disabled")
        self.current_profile_fire_button.config(state="disabled")
        self.simple_ol_fire_button.config(state="disabled")
        self.sequence_panel.set_connected(True)

        # connect()'s setup header just re-applied the defaults to the
        # controller -- reset these fields to match, so a value tuned in a
        # previous session doesn't misrepresent what's actually active now.
        # Settings only reach the controller again via Go, so this also
        # guards against silently pushing a stale leftover value on the
        # first Go click of a new session.
        self.target_var.set("")
        self._on_reset_motion_profile()
        self._on_reset_rest_pid_gains()
        self._on_reset_motion_pid_gains()
        self._on_reset_recording_settings()
        self._on_reset_impact_range()
        self._on_reset_contour_range()
        self._on_reset_pvt_range()
        self._on_reset_current_profile_range()
        self._refresh_calibration_display()

        self._poll()

    def _on_disconnect(self):
        self._stop_polling()
        disable_error = None
        try:
            self.controller.abort()
        except gclib.GclibError as e:
            disable_error = e
        self.controller.disconnect()
        if disable_error is not None:
            self.status_var.set(f"Disconnected (servo disable failed: {disable_error})")
        else:
            self.status_var.set("Disconnected")
        self.connect_button.config(state="normal")
        self.disconnect_button.config(state="disabled")
        self.address_entry.config(state="normal")
        self.enable_button.config(state="disabled")
        self.go_button.config(state="disabled")
        self.slow_retract_button.config(state="disabled")
        self.slow_extend_button.config(state="disabled")
        self.download_program_button.config(state="disabled")
        self.run_program_button.config(state="disabled")
        for button in self.cal_set_buttons:
            button.config(state="disabled")
        self.arm_button.config(state="disabled")
        self.fire_button.config(state="disabled")
        self.contour_arm_button.config(state="disabled")
        self.contour_fire_button.config(state="disabled")
        self.pvt_arm_button.config(state="disabled")
        self.pvt_fire_button.config(state="disabled")
        self.current_profile_arm_button.config(state="disabled")
        self.current_profile_fire_button.config(state="disabled")
        self.simple_ol_load_button.config(state="disabled")
        self.simple_ol_fire_button.config(state="disabled")
        self.sequence_panel.set_connected(False)
        self.conditioning_controls.set_servo_enabled(False)
        self.servo_status_var.set("Servo: —")

    def _on_disable(self):
        if not self.controller.is_connected:
            self.status_var.set("Not connected — nothing to disable")
            return
        # Captured before the abort -- a successfully armed Fire is about
        # to be invalidated by it (the downloaded program's assumptions,
        # e.g. the axis already sitting at the expected retract position,
        # no longer hold once Disable interrupts things).
        was_armed = (
            str(self.fire_button["state"]) == "normal"
            or str(self.contour_fire_button["state"]) == "normal"
            or str(self.pvt_fire_button["state"]) == "normal"
            or str(self.current_profile_fire_button["state"]) == "normal"
            or str(self.simple_ol_fire_button["state"]) == "normal"
            or str(self.sequence_panel.fire_button["state"]) == "normal"
        )
        try:
            self.controller.abort()
        except gclib.GclibError as e:
            self.status_var.set(f"Disable failed: {e}")
            return
        # Force this immediately rather than waiting for the next ~200ms
        # poll tick -- Fire must not be clickable for even one tick after a
        # Disable that aborted an armed sequence. The poll loop's own
        # servo-based gating (see _poll) keeps both grayed out afterward
        # regardless -- even across a bare re-Enable -- until a fresh Arm
        # succeeds; this just makes the *first* instant correct too.
        self.fire_button.config(state="disabled")
        self.contour_fire_button.config(state="disabled")
        self.pvt_fire_button.config(state="disabled")
        self.current_profile_fire_button.config(state="disabled")
        # Simple Open Loop's Fire is cleared here too -- Disable is the
        # explicit abort, so it should invalidate a loaded program like
        # any other mode. _poll deliberately does NOT re-clear it, since
        # this mode runs with the servo off by design.
        self.simple_ol_fire_button.config(state="disabled")
        self.sequence_panel.on_disable()
        if was_armed:
            self.status_var.set("Arm sequence aborted")

    def _on_enable(self):
        # Rest gains/sample period must be in place before SHA closes the
        # servo loop -- never enable with unknown/unapplied settings.
        if not self._apply_rest_settings():
            return
        try:
            self.controller.enable()
        except gclib.GclibError as e:
            self.status_var.set(f"Enable failed: {e}")

    def _on_go(self):
        # Parse and validate everything up front; only once it all checks
        # out do we start sending commands to the controller, so a bad
        # field never leaves settings partially applied.
        try:
            target_mm = float(self.target_var.get())
        except ValueError:
            self.status_var.set("Target must be a number (mm)")
            return

        if not (0 <= target_mm <= FULL_STROKE_MM):
            self.status_var.set(
                f"Target must satisfy 0 <= target <= {FULL_STROKE_MM}mm"
            )
            return

        try:
            speed_m_s = float(self.speed_var.get())
            accel_m_s2 = float(self.accel_var.get())
            decel_m_s2 = float(self.decel_var.get())
        except ValueError:
            self.status_var.set("Speed/Accel/Decel must be numbers (m/s, m/s²)")
            return

        # Just parse checks here -- motion gains/sample period aren't sent
        # yet. For a recording move, RecordingSequence applies them itself
        # (via _apply_motion_settings) before start_recording(), not here.
        try:
            motion_kp = float(self.motion_kp_var.get())
            motion_kd = float(self.motion_kd_var.get())
            motion_ki = float(self.motion_ki_var.get())
        except ValueError:
            self.status_var.set("Motion KP/KD/KI must be numbers")
            return

        try:
            motion_period_us = float(self.motion_sample_period_us_var.get())
        except ValueError:
            self.status_var.set("Motion sampling period must be a number (usec)")
            return
        if not (TM_MIN_US <= motion_period_us <= TM_MAX_US):
            self.status_var.set(
                f"Motion sampling period must be {TM_MIN_US}-{TM_MAX_US}usec"
            )
            return

        try:
            pre_pause_ms = int(self.pre_pause_ms_var.get())
            post_pause_ms = int(self.post_pause_ms_var.get())
        except ValueError:
            self.status_var.set("Pre/Post-Move Pause must be whole numbers (ms)")
            return
        if pre_pause_ms < 0 or post_pause_ms < 0:
            self.status_var.set("Pre/Post-Move Pause must not be negative")
            return

        if not self._apply_motion_profile(speed_m_s, accel_m_s2, decel_m_s2):
            return

        target_counts = Controller.mm_to_counts(target_mm)

        if self.record_var.get():
            self._go_capture_info = self._format_motion_info("Go", [
                ("Target", f"{target_mm:.2f}mm"),
                ("Speed/Accel/Decel", f"{speed_m_s}/{accel_m_s2}/{decel_m_s2} m/s, m/s²"),
                ("Sample Period", f"{motion_period_us}us"),
                ("Pre/Post Pause", f"{pre_pause_ms}/{post_pause_ms}ms"),
                ("Motion PID", f"KP={motion_kp} KD={motion_kd} KI={motion_ki}"),
                (
                    "Rest PID",
                    f"KP={self.rest_kp_var.get()} KD={self.rest_kd_var.get()} "
                    f"KI={self.rest_ki_var.get()} Offset={self.rest_offset_var.get()}V",
                ),
            ])
            # Prevent a re-entrant second capture; the regular 5 Hz poll's
            # servo-based gating re-enables this once the sequence is done
            # (or immediately corrects a failed setup within one poll tick).
            self.go_button.config(state="disabled")
            self._recording_sequence.start(target_counts, pre_pause_ms, post_pause_ms)
            return

        # No pre-move pause on a plain move, so "right before motion
        # begins" is here -- apply immediately, unlike the recording path.
        if not self._apply_motion_settings():
            return

        # Same re-entrancy guard as the recording branch above; the 5 Hz
        # poll's servo-based gating re-enables Go once the wait finishes.
        self.go_button.config(state="disabled")
        try:
            self.controller.move_absolute(target_counts)
        except gclib.GclibError as e:
            self.status_var.set(f"Move failed: {e}")
            return

        wait_for_motion_complete(
            self.controller, self.root.after,
            on_complete=self._apply_rest_settings,
            on_error=self.status_var.set,
        )

    def _on_slow_retract(self):
        self._slow_move_and_report(is_retract=True)

    def _on_slow_extend(self):
        self._slow_move_and_report(is_retract=False)

    def _slow_move_and_report(self, is_retract: bool):
        # Only the standalone Slow Retract/Extend buttons report completion
        # to the status bar this way -- _slow_move_to_limit is also called
        # from _on_arm/_on_arm_contour, which already run their own
        # wait_for_motion_complete to proceed to a program download, and
        # would race/overwrite each other if this wait ran there too.
        if not self._slow_move_to_limit(is_retract=is_retract):
            return
        label = "retract" if is_retract else "extend"
        wait_for_motion_complete(
            self.controller, self.root.after,
            on_complete=lambda: self.status_var.set(f"Slow {label} complete"),
            on_error=self.status_var.set,
        )

    def _slow_move_to_limit(
        self, is_retract: bool, min_mm: float = 0.0, max_mm: float = FULL_STROKE_MM,
        margin: float = None, target_mm: float = None,
    ) -> bool:
        # Moves to min_mm (retract) or max_mm (extend) -- the full
        # calibrated range by default (plain Slow Retract/Extend), a
        # mode-specific Start/Stop range when called from Contoured
        # Motion's Arm, or an explicit target_mm (Open Loop Motion's Arm,
        # which has no Stop field). Fixed slow profile; PID gains are left
        # exactly as-is -- no rest/motion switching -- and this never
        # records, even if Record is checked. Returns True once the move
        # is actually sent, False on any earlier validation/apply/GclibError
        # failure.
        if not self._apply_motion_profile(SLOW_SPEED_M_S, SLOW_ACCEL_M_S2, SLOW_DECEL_M_S2):
            return False

        # Retract stops SLOW_RETRACT_MARGIN_MM short of min_mm by default --
        # the voice coil can't reliably hold position right at the true
        # mechanical retract limit (see controller.py). Callers that want a
        # different margin (e.g. Contoured Motion's Arm, margin=0) pass it
        # explicitly, or bypass the min_mm/max_mm/margin calc entirely with
        # an explicit target_mm (e.g. Open Loop Motion's Arm).
        if target_mm is None:
            if margin is None:
                margin = SLOW_RETRACT_MARGIN_MM if is_retract else TRAVEL_MARGIN_MM
            target_mm = self._inset_mm(min_mm, max_mm, is_retract, margin)
        target_counts = Controller.mm_to_counts(target_mm)
        label = "retract" if is_retract else "extend"

        self.go_button.config(state="disabled")
        self.slow_retract_button.config(state="disabled")
        self.slow_extend_button.config(state="disabled")
        try:
            self.controller.move_absolute(target_counts)
        except gclib.GclibError as e:
            self.status_var.set(f"Slow {label} failed: {e}")
            return False
        self.status_var.set(f"Slow {label} in progress...")
        return True

    @staticmethod
    def _inset_mm(min_mm, max_mm, is_retract: bool, margin: float) -> float:
        # Halves the margin instead if min_mm/max_mm are too close together
        # for the full margin to fit both directions without crossing.
        margin = min(margin, (max_mm - min_mm) / 2)
        return min_mm + margin if is_retract else max_mm - margin

    def _on_browse_program(self):
        path = filedialog.askopenfilename(
            title="Select a DMC program",
            initialdir=str(EXAMPLE_DMC_DIR),
            filetypes=[("DMC programs", "*.dmc"), ("All files", "*.*")],
        )
        if path:
            self.program_path_var.set(path)

    def _on_download_program(self):
        path = self.program_path_var.get()
        if not path:
            self.status_var.set("No program file selected")
            return
        try:
            text = Path(path).read_text()
        except OSError as e:
            self.status_var.set(f"Read program file failed: {e}")
            return

        try:
            self.controller.download_program(text)
        except gclib.GclibError as e:
            self.status_var.set(f"Program download failed: {e}")
            return
        self.status_var.set(f"Downloaded {Path(path).name}")

    def _on_run_program(self):
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Run program failed: {e}")
            return
        self.status_var.set("Program running...")
        # Prevent a re-entrant second Run while this one is still being
        # watched; re-enabled by _on_program_wait_done below (Download/Run
        # aren't gated by the 5 Hz poll like Go is, so this needs doing
        # explicitly).
        self.run_program_button.config(state="disabled")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=self._on_program_wait_complete,
            on_error=self._on_program_wait_done,
        )

    def _on_program_wait_complete(self):
        # See _on_fire_wait_complete's comment -- "not running" alone
        # doesn't distinguish a clean EN from an early error halt.
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"
        self._on_program_wait_done(f"Program finished [controller last error: {last_error}]")

    def _on_program_wait_done(self, message):
        self.status_var.set(message)
        self.run_program_button.config(state="normal")

    def _on_arm(self):
        # Fully self-contained validation (not shared with _on_go's blocks
        # -- see the plan's reasoning: _on_go is bench-proven and
        # safety-relevant, not worth a refactor-for-reuse regression risk).
        try:
            start_mm = float(self.impact_start_var.get())
        except ValueError:
            self.status_var.set("Start must be a number (mm)")
            return
        if not (0 <= start_mm <= FULL_STROKE_MM):
            self.status_var.set(f"Start must satisfy 0 <= start <= {FULL_STROKE_MM}mm")
            return

        pre_impact_retract_raw = self.pre_impact_retract_voltage_var.get()
        try:
            pre_impact_retract_voltage_v = (
                float(pre_impact_retract_raw) if pre_impact_retract_raw.strip() else 0.0
            )
        except ValueError:
            self.status_var.set("Pre Impact Retract Voltage must be a number (V)")
            return
        if not (pre_impact_retract_voltage_v >= 0):
            self.status_var.set("Pre Impact Retract Voltage must be >= 0")
            return

        pre_impact_retract_wait_ms = 0
        if pre_impact_retract_voltage_v:
            try:
                pre_impact_retract_wait_ms = float(self.pre_impact_retract_wait_ms_var.get())
            except ValueError:
                self.status_var.set("Pre Impact Retract Wait Time must be a number (ms)")
                return
            if not (0 <= pre_impact_retract_wait_ms <= 1000):
                self.status_var.set("Pre Impact Retract Wait Time must be 0-1000ms")
                return

        try:
            speed_m_s = float(self.speed_var.get())
            accel_m_s2 = float(self.accel_var.get())
            decel_m_s2 = float(self.decel_var.get())
        except ValueError:
            self.status_var.set("Speed/Accel/Decel must be numbers (m/s, m/s²)")
            return

        try:
            motion_kp = float(self.motion_kp_var.get())
            motion_kd = float(self.motion_kd_var.get())
            motion_ki = float(self.motion_ki_var.get())
            motion_offset_v = float(self.motion_offset_var.get())
            motion_fv = float(self.motion_fv_var.get())
        except ValueError:
            self.status_var.set("Motion KP/KD/KI/Offset/FV must be numbers")
            return

        try:
            motion_period_us = float(self.motion_sample_period_us_var.get())
        except ValueError:
            self.status_var.set("Motion sampling period must be a number (usec)")
            return
        if not (TM_MIN_US <= motion_period_us <= TM_MAX_US):
            self.status_var.set(
                f"Motion sampling period must be {TM_MIN_US}-{TM_MAX_US}usec"
            )
            return

        try:
            pre_pause_ms = int(self.pre_pause_ms_var.get())
            post_pause_ms = int(self.post_pause_ms_var.get())
        except ValueError:
            self.status_var.set("Pre/Post-Move Pause must be whole numbers (ms)")
            return
        if pre_pause_ms < 0 or post_pause_ms < 0:
            self.status_var.set("Pre/Post-Move Pause must not be negative")
            return

        try:
            rest_kp = float(self.rest_kp_var.get())
            rest_kd = float(self.rest_kd_var.get())
            rest_ki = float(self.rest_ki_var.get())
            rest_offset_v = float(self.rest_offset_var.get())
        except ValueError:
            self.status_var.set("Rest KP/KD/KI/Offset must be numbers")
            return

        try:
            impact_accel_primary_voltage_v = float(
                self.impact_accel_primary_voltage_var.get()
            )
        except ValueError:
            self.status_var.set("Impact Accel Primary Voltage must be a number (V)")
            return
        if not (impact_accel_primary_voltage_v <= 0):
            self.status_var.set("Impact Accel Primary Voltage must be <= 0")
            return

        try:
            impact_accel_primary_wait_ms = float(self.impact_accel_primary_wait_ms_var.get())
        except ValueError:
            self.status_var.set("Impact Accel Primary Wait Time must be a number (ms)")
            return
        if impact_accel_primary_wait_ms < 0:
            self.status_var.set("Impact Accel Primary Wait Time must not be negative")
            return

        try:
            impact_accel_secondary_voltage_v = float(
                self.impact_accel_secondary_voltage_var.get()
            )
        except ValueError:
            self.status_var.set("Impact Accel Secondary Voltage must be a number (V)")
            return

        try:
            impact_accel_secondary_wait_ms = float(
                self.impact_accel_secondary_wait_ms_var.get()
            )
        except ValueError:
            self.status_var.set("Impact Accel Secondary Wait Time must be a number (ms)")
            return
        if not (0 <= impact_accel_secondary_wait_ms <= 1000):
            self.status_var.set("Impact Accel Secondary Wait Time must be 0-1000ms")
            return

        try:
            impact_decel_voltage_v = float(self.impact_decel_voltage_var.get())
        except ValueError:
            self.status_var.set("Impact Decel Voltage must be a number (V)")
            return
        if not (impact_decel_voltage_v >= 0):
            self.status_var.set("Impact Decel Voltage must be >= 0")
            return

        try:
            impact_decel_wait_ms = float(self.impact_decel_wait_ms_var.get())
        except ValueError:
            self.status_var.set("Impact Decel Wait Time must be a number (ms)")
            return
        if impact_decel_wait_ms < 0:
            self.status_var.set("Impact Decel Wait Time must not be negative")
            return

        try:
            hold_voltage_v = float(self.hold_voltage_var.get())
        except ValueError:
            self.status_var.set("Hold Voltage must be a number (V)")
            return
        if not (hold_voltage_v <= 0):
            self.status_var.set("Hold Voltage must be <= 0")
            return

        try:
            hold_wait_ms = float(self.hold_wait_ms_var.get())
        except ValueError:
            self.status_var.set("Hold Wait Time must be a number (ms)")
            return
        if not (0 <= hold_wait_ms <= 1000):
            self.status_var.set("Hold Wait Time must be 0-1000ms")
            return

        try:
            retract_accel_primary_voltage_v = float(
                self.retract_accel_primary_voltage_var.get()
            )
        except ValueError:
            self.status_var.set("Retract Accel Primary Voltage must be a number (V)")
            return
        if not (retract_accel_primary_voltage_v >= 0):
            self.status_var.set("Retract Accel Primary Voltage must be >= 0")
            return

        try:
            retract_accel_primary_wait_ms = float(
                self.retract_accel_primary_wait_ms_var.get()
            )
        except ValueError:
            self.status_var.set("Retract Accel Primary Wait Time must be a number (ms)")
            return
        if not (0 <= retract_accel_primary_wait_ms <= 1000):
            self.status_var.set("Retract Accel Primary Wait Time must be 0-1000ms")
            return

        try:
            retract_accel_secondary_voltage_v = float(
                self.retract_accel_secondary_voltage_var.get()
            )
        except ValueError:
            self.status_var.set("Retract Accel Secondary Voltage must be a number (V)")
            return
        if not (retract_accel_secondary_voltage_v >= 0):
            self.status_var.set("Retract Accel Secondary Voltage must be >= 0")
            return

        try:
            retract_accel_secondary_wait_ms = float(
                self.retract_accel_secondary_wait_ms_var.get()
            )
        except ValueError:
            self.status_var.set(
                "Retract Accel Secondary Wait Time must be a number (ms)"
            )
            return
        if not (0 <= retract_accel_secondary_wait_ms <= 1000):
            self.status_var.set("Retract Accel Secondary Wait Time must be 0-1000ms")
            return

        try:
            retract_decel_voltage_v = float(self.retract_decel_voltage_var.get())
        except ValueError:
            self.status_var.set("Retract Decel Voltage must be a number (V)")
            return
        if not (retract_decel_voltage_v <= 0):
            self.status_var.set("Retract Decel Voltage must be <= 0")
            return

        try:
            retract_decel_wait_ms = float(self.retract_decel_wait_ms_var.get())
        except ValueError:
            self.status_var.set("Retract Decel Wait Time must be a number (ms)")
            return
        if not (0 <= retract_decel_wait_ms <= 1000):
            self.status_var.set("Retract Decel Wait Time must be 0-1000ms")
            return

        # Invalidate any previously-armed program before attempting a new
        # one; re-entrancy guard on Arm itself while this attempt runs.
        self.fire_button.config(state="disabled")
        self.arm_button.config(state="disabled")

        self.status_var.set("Arming...")

        # Download before any motion is commanded this cycle -- a program
        # download disables the servo for ~300ms (Galil manual, Ch.7), and
        # with nothing being held yet, that disable has nothing to disturb.
        # Explicit MO/SH bracket it so the servo re-engages from a known,
        # torque-free state rather than however it happened to be left,
        # avoiding a snap once the retract (last step, below) settles into
        # its hold.
        try:
            self.controller.disable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: {e}")
            self.arm_button.config(state="normal")
            return

        program_text = build_impact_program(
            final_hold_counts=Controller.mm_to_counts(SLOW_RETRACT_MARGIN_MM),
            start_counts=Controller.mm_to_counts(start_mm),
            slow_speed_counts=Controller.meters_to_counts(SLOW_SPEED_M_S),
            slow_accel_counts=Controller.meters_to_counts(SLOW_ACCEL_M_S2),
            slow_decel_counts=Controller.meters_to_counts(SLOW_DECEL_M_S2),
            closed_loop_speed_counts=Controller.meters_to_counts(speed_m_s),
            closed_loop_accel_counts=Controller.meters_to_counts(accel_m_s2),
            closed_loop_decel_counts=Controller.meters_to_counts(decel_m_s2),
            motion_sample_period_us=motion_period_us,
            pre_impact_retract_voltage_v=pre_impact_retract_voltage_v,
            pre_impact_retract_wait_cycles=Controller.ms_to_cycles(
                pre_impact_retract_wait_ms, motion_period_us
            ),
            pre_pause_ms=pre_pause_ms,
            impact_accel_primary_voltage_v=impact_accel_primary_voltage_v,
            impact_accel_primary_wait_cycles=Controller.ms_to_cycles(
                impact_accel_primary_wait_ms, motion_period_us
            ),
            impact_accel_secondary_voltage_v=impact_accel_secondary_voltage_v,
            impact_accel_secondary_wait_cycles=Controller.ms_to_cycles(
                impact_accel_secondary_wait_ms, motion_period_us
            ),
            impact_decel_voltage_v=impact_decel_voltage_v,
            impact_decel_wait_cycles=Controller.ms_to_cycles(
                impact_decel_wait_ms, motion_period_us
            ),
            hold_voltage_v=hold_voltage_v,
            hold_wait_cycles=Controller.ms_to_cycles(hold_wait_ms, motion_period_us),
            retract_accel_primary_voltage_v=retract_accel_primary_voltage_v,
            retract_accel_primary_wait_cycles=Controller.ms_to_cycles(
                retract_accel_primary_wait_ms, motion_period_us
            ),
            retract_accel_secondary_voltage_v=retract_accel_secondary_voltage_v,
            retract_accel_secondary_wait_cycles=Controller.ms_to_cycles(
                retract_accel_secondary_wait_ms, motion_period_us
            ),
            retract_decel_voltage_v=retract_decel_voltage_v,
            retract_decel_wait_cycles=Controller.ms_to_cycles(
                retract_decel_wait_ms, motion_period_us
            ),
            end_with_closed_loop_move=self.end_with_closed_loop_move_var.get(),
            motion_kp=motion_kp,
            motion_kd=motion_kd,
            motion_ki=motion_ki,
            post_pause_ms=post_pause_ms,
            rest_kp=rest_kp,
            rest_kd=rest_kd,
            rest_ki=rest_ki,
            rest_offset_v=rest_offset_v,
        )

        try:
            self.controller.download_program(program_text)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: program download failed: {e}")
            self.arm_button.config(state="normal")
            return

        # Sizes the 4 shared record arrays to RECORD_ARRAY_SIZE before
        # Fire runs this program -- this program's own embedded DM lines
        # request the same fixed size, but relying on THOSE to grow the
        # arrays (if a prior Current Profile Arm left them smaller) was
        # bench-confirmed unreliable; sizing them here first (DA+DM if a
        # real grow is needed, handled by size_shared_record_arrays)
        # makes that embedded DM a harmless same-size redeclaration.
        try:
            self.controller.size_shared_record_arrays(RECORD_ARRAY_SIZE)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: sizing record array space failed: {e}")
            self.arm_button.config(state="normal")
            return

        # Re-enable -- Rest gains applied first, same "never SHA with
        # unknown/unapplied gains" principle as _on_enable.
        if not self._apply_rest_settings():
            self.arm_button.config(state="normal")
            return
        try:
            self.controller.enable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: enable failed: {e}")
            self.arm_button.config(state="normal")
            return

        if not self._slow_move_to_limit(
            is_retract=True, target_mm=max(SLOW_RETRACT_MARGIN_MM, start_mm)
        ):
            self.arm_button.config(state="normal")
            return

        def handle_retract_complete():
            self._armed_capture_sample_period_s = motion_period_us / 1_000_000
            self._armed_capture_info = self._format_motion_info("Open Loop Motion", [
                ("Start", f"{start_mm:.2f}mm"),
                (
                    "Pre Impact Retract",
                    f"{pre_impact_retract_voltage_v}V / {pre_impact_retract_wait_ms}ms",
                ),
                (
                    "Impact Accel Primary",
                    f"{impact_accel_primary_voltage_v}V / {impact_accel_primary_wait_ms}ms",
                ),
                (
                    "Impact Accel Secondary",
                    f"{impact_accel_secondary_voltage_v}V / {impact_accel_secondary_wait_ms}ms",
                ),
                ("Impact Decel", f"{impact_decel_voltage_v}V / {impact_decel_wait_ms}ms"),
                ("Hold", f"{hold_voltage_v}V / {hold_wait_ms}ms"),
                (
                    "Retract Accel Primary",
                    f"{retract_accel_primary_voltage_v}V / {retract_accel_primary_wait_ms}ms",
                ),
                (
                    "Retract Accel Secondary",
                    f"{retract_accel_secondary_voltage_v}V / {retract_accel_secondary_wait_ms}ms",
                ),
                ("Retract Decel", f"{retract_decel_voltage_v}V / {retract_decel_wait_ms}ms"),
                ("End w/ Closed-Loop Move", str(self.end_with_closed_loop_move_var.get())),
                (
                    "Closed-Loop Speed/Accel/Decel",
                    f"{speed_m_s}/{accel_m_s2}/{decel_m_s2} m/s, m/s²",
                ),
                ("Sample Period", f"{motion_period_us}us"),
                ("Pre/Post Pause", f"{pre_pause_ms}/{post_pause_ms}ms"),
                (
                    "Motion PID",
                    f"KP={motion_kp} KD={motion_kd} KI={motion_ki} "
                    f"Offset={motion_offset_v}V FV={motion_fv}",
                ),
                ("Rest PID", f"KP={rest_kp} KD={rest_kd} KI={rest_ki} Offset={rest_offset_v}V"),
            ])
            self.arm_button.config(state="normal")
            self.fire_button.config(state="normal")
            self.status_var.set(
                f"Armed: impact accel voltage={impact_accel_primary_voltage_v}V, "
                f"impact accel wait={impact_accel_primary_wait_ms}ms, "
                f"impact decel voltage={impact_decel_voltage_v}V, "
                f"impact decel wait={impact_decel_wait_ms}ms"
            )

        def handle_retract_error(message):
            self.status_var.set(f"Arm failed: {message}")
            self.arm_button.config(state="normal")

        wait_for_motion_complete(
            self.controller, self.root.after,
            on_complete=handle_retract_complete,
            on_error=handle_retract_error,
        )

    def _on_fire(self):
        self.arm_button.config(state="disabled")
        self.fire_button.config(state="disabled")
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: {e}")
            self.arm_button.config(state="normal")
            self.fire_button.config(state="normal")
            return

        self.status_var.set("Firing...")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=self._on_fire_wait_complete,
            on_error=self._on_fire_wait_error,
        )

    def _on_fire_wait_complete(self):
        # read_program_running() alone can't tell "reached EN cleanly"
        # apart from "halted partway on an error" -- both look like
        # "thread 0 not running". Surface the last error code/text either
        # way so a silent early-exit is diagnosable instead of just
        # showing up as an empty/short capture with no explanation.
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"

        try:
            recorded_data = self.controller.upload_recorded_data()
        except gclib.GclibError as e:
            self.status_var.set(f"Impact data upload failed: {e} [controller last error: {last_error}]")
            self.arm_button.config(state="normal")
            self.fire_button.config(state="normal")
            return

        # Never leave Arm/Fire stuck disabled and the status frozen over
        # an unexpected plotting failure -- bench-confirmed this can
        # otherwise look exactly like a hang.
        try:
            plotted = self._show_capture(
                recorded_data, self._armed_capture_sample_period_s, self._armed_capture_info
            )
        except Exception as e:
            self.status_var.set(
                f"Impact ran, but plotting failed: {e} "
                f"[{len(recorded_data.position)} samples, controller last error: {last_error}]"
            )
        else:
            status = (
                f"Impact captured [{len(recorded_data.position)} samples, "
                f"controller last error: {last_error}]"
            )
            if not plotted:
                status = f"{self.status_var.get()} [controller last error: {last_error}]"
            self.status_var.set(status)

        self.arm_button.config(state="normal")
        self.fire_button.config(state="normal")

    def _on_fire_wait_error(self, message):
        self.status_var.set(message)
        self.arm_button.config(state="normal")
        self.fire_button.config(state="normal")

    def _on_load_simple_open_loop(self):
        if not self.controller.is_connected:
            self.status_var.set("Not connected -- nothing to load")
            return

        try:
            current_a = float(self.simple_ol_current_var.get())
        except ValueError:
            self.status_var.set("Current must be a number (A)")
            return
        current_volts = Controller.current_to_volts(current_a)
        if not (OF_MIN_V <= current_volts <= OF_MAX_V):
            max_a = Controller.volts_to_current(OF_MAX_V)
            self.status_var.set(
                f"Current {current_a}A needs {current_volts:.2f}V, outside the "
                f"{OF_MIN_V}..{OF_MAX_V}V offset range -- at the current amp "
                f"full-scale setting the limit is +/-{max_a:.2f}A"
            )
            return

        try:
            hold_ms = float(self.simple_ol_hold_ms_var.get())
        except ValueError:
            self.status_var.set("Hold Delay must be a number (ms)")
            return
        if hold_ms < 0:
            self.status_var.set("Hold Delay must not be negative")
            return

        try:
            motion_period_us = float(self.motion_sample_period_us_var.get())
        except ValueError:
            self.status_var.set("Motion sampling period must be a number (usec)")
            return
        if not (TM_MIN_US <= motion_period_us <= TM_MAX_US):
            self.status_var.set(
                f"Motion sampling period must be {TM_MIN_US}-{TM_MAX_US}usec"
            )
            return

        try:
            pre_pause_ms = int(self.pre_pause_ms_var.get())
            post_pause_ms = int(self.post_pause_ms_var.get())
        except ValueError:
            self.status_var.set("Pre/Post-Move Pause must be whole numbers (ms)")
            return
        if pre_pause_ms < 0 or post_pause_ms < 0:
            self.status_var.set("Pre/Post-Move Pause must not be negative")
            return

        try:
            rest_kp = float(self.rest_kp_var.get())
            rest_kd = float(self.rest_kd_var.get())
            rest_ki = float(self.rest_ki_var.get())
        except ValueError:
            self.status_var.set("Rest KP/KD/KI must be numbers")
            return

        # Recording is a fixed RECORD_ARRAY_SIZE samples and RC 1 is
        # non-circular, so a window longer than that is silently cropped
        # rather than reported -- reject it up front instead (same
        # lesson as Current Profile's own feedforward-stretch crop).
        window_samples = round((pre_pause_ms + hold_ms + post_pause_ms) * 1000 / motion_period_us)
        if window_samples > RECORD_ARRAY_SIZE:
            self.status_var.set(
                f"Capture window needs {window_samples} samples, exceeds the "
                f"{RECORD_ARRAY_SIZE}-sample recording limit -- raise Motion "
                f"Sample Period or shorten Hold Delay/pauses"
            )
            return

        program_text = build_simple_open_loop_program(
            sample_period_us=motion_period_us,
            pre_pause_ms=pre_pause_ms,
            current_volts=current_volts,
            hold_wait_cycles=Controller.ms_to_cycles(hold_ms, motion_period_us),
            post_pause_ms=post_pause_ms,
            rest_kp=rest_kp,
            rest_kd=rest_kd,
            rest_ki=rest_ki,
        )

        # Invalidate any previously-loaded program before attempting a
        # new one; re-entrancy guard on Load itself while this runs.
        self.simple_ol_fire_button.config(state="disabled")
        self.simple_ol_load_button.config(state="disabled")
        self.status_var.set("Loading...")

        # Download ONLY -- no disable, no enable, no motion. That's the
        # whole point of this mode: whatever magnetic/mechanical state
        # the striker is in when you press Load is the state under test.
        try:
            self.controller.download_program(program_text)
        except gclib.GclibError as e:
            self.status_var.set(f"Load failed: program download failed: {e}")
            self.simple_ol_load_button.config(state="normal")
            return

        try:
            self.controller.size_shared_record_arrays(RECORD_ARRAY_SIZE)
        except gclib.GclibError as e:
            self.status_var.set(f"Load failed: sizing record array space failed: {e}")
            self.simple_ol_load_button.config(state="normal")
            return

        self._loaded_simple_ol_sample_period_s = motion_period_us / 1_000_000
        self._loaded_simple_ol_info = self._format_motion_info("Simple Open Loop", [
            ("Current", f"{current_a}A ({current_volts:.3f}V)"),
            ("Hold Delay", f"{hold_ms}ms"),
            ("Sample Period", f"{motion_period_us}us"),
            ("Pre/Post Pause", f"{pre_pause_ms}/{post_pause_ms}ms"),
            ("Rest PID", f"KP={rest_kp} KD={rest_kd} KI={rest_ki}"),
        ])
        self.simple_ol_load_button.config(state="normal")
        self.simple_ol_fire_button.config(state="normal")
        self.status_var.set(
            f"Loaded: {current_a}A for {hold_ms}ms -- servo stays off until Fire"
        )

    def _on_fire_simple_open_loop(self):
        # This program zeroes KP/KD/KI before enabling, so firing while
        # the servo is actively holding would drop the holding force and
        # let the striker fall under gravity on the vertical mount. The
        # intended state is servo off with the striker parked by hand.
        try:
            if self.controller.read_servo_enabled():
                self.status_var.set(
                    "Fire refused: disable the servo first -- this mode zeroes PID "
                    "gains, so firing while holding would drop the striker"
                )
                return
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: could not read servo state: {e}")
            return

        self.simple_ol_load_button.config(state="disabled")
        self.simple_ol_fire_button.config(state="disabled")
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: {e}")
            self.simple_ol_load_button.config(state="normal")
            self.simple_ol_fire_button.config(state="normal")
            return

        self.status_var.set("Firing...")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=self._on_fire_simple_open_loop_wait_complete,
            on_error=self._on_fire_simple_open_loop_wait_error,
        )

    def _on_fire_simple_open_loop_wait_complete(self):
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"

        try:
            recorded_data = self.controller.upload_recorded_data()
        except gclib.GclibError as e:
            self.status_var.set(
                f"Simple Open Loop data upload failed: {e} "
                f"[controller last error: {last_error}]"
            )
            self.simple_ol_load_button.config(state="normal")
            self.simple_ol_fire_button.config(state="normal")
            return

        try:
            plotted = self._show_capture(
                recorded_data, self._loaded_simple_ol_sample_period_s,
                self._loaded_simple_ol_info,
            )
        except Exception as e:
            self.status_var.set(
                f"Simple Open Loop ran, but plotting failed: {e} "
                f"[{len(recorded_data.position)} samples, controller last error: {last_error}]"
            )
        else:
            status = (
                f"Simple Open Loop captured [{len(recorded_data.position)} samples, "
                f"controller last error: {last_error}]"
            )
            if not plotted:
                status = f"{self.status_var.get()} [controller last error: {last_error}]"
            self.status_var.set(status)

        # Fire stays available -- the program is still resident and the
        # fire/read/fire cycle this mode exists for shouldn't need a
        # re-Load between pulses.
        self.simple_ol_load_button.config(state="normal")
        self.simple_ol_fire_button.config(state="normal")

    def _on_fire_simple_open_loop_wait_error(self, message):
        self.status_var.set(message)
        self.simple_ol_load_button.config(state="normal")
        self.simple_ol_fire_button.config(state="normal")

    def _on_arm_contour(self):
        # Fully self-contained validation, same style as _on_arm.
        try:
            min_mm = float(self.contour_start_var.get())
            max_mm = float(self.contour_stop_var.get())
        except ValueError:
            self.status_var.set("Start/Stop must be numbers (mm)")
            return
        if not (0 <= min_mm < max_mm <= FULL_STROKE_MM):
            self.status_var.set(
                f"Start/Stop must satisfy 0 <= start < stop <= {FULL_STROKE_MM}mm"
            )
            return

        try:
            motion_kp = float(self.motion_kp_var.get())
            motion_kd = float(self.motion_kd_var.get())
            motion_ki = float(self.motion_ki_var.get())
            motion_offset_v = float(self.motion_offset_var.get())
            motion_fv = float(self.motion_fv_var.get())
        except ValueError:
            self.status_var.set("Motion KP/KD/KI/Offset/FV must be numbers")
            return

        try:
            motion_period_us = float(self.motion_sample_period_us_var.get())
        except ValueError:
            self.status_var.set("Motion sampling period must be a number (usec)")
            return
        if not (TM_MIN_US <= motion_period_us <= TM_MAX_US):
            self.status_var.set(
                f"Motion sampling period must be {TM_MIN_US}-{TM_MAX_US}usec"
            )
            return

        try:
            pre_pause_ms = int(self.pre_pause_ms_var.get())
            post_pause_ms = int(self.post_pause_ms_var.get())
        except ValueError:
            self.status_var.set("Pre/Post-Move Pause must be whole numbers (ms)")
            return
        if pre_pause_ms < 0 or post_pause_ms < 0:
            self.status_var.set("Pre/Post-Move Pause must not be negative")
            return

        try:
            rest_kp = float(self.rest_kp_var.get())
            rest_kd = float(self.rest_kd_var.get())
            rest_ki = float(self.rest_ki_var.get())
            rest_offset_v = float(self.rest_offset_var.get())
        except ValueError:
            self.status_var.set("Rest KP/KD/KI/Offset must be numbers")
            return

        try:
            impact_velocity_m_s = float(self.contour_velocity_var.get())
        except ValueError:
            self.status_var.set("Impact Velocity must be a number (m/s)")
            return
        if impact_velocity_m_s <= 0:
            self.status_var.set("Impact Velocity must be positive")
            return

        try:
            impact_position_mm = float(self.contour_position_var.get())
        except ValueError:
            self.status_var.set("Impact Position must be a number (mm)")
            return

        # No padding -- Arm moves to exactly the user-specified Start; add
        # any buffer by editing the Start field itself.
        start_mm = self._inset_mm(min_mm, max_mm, True, 0)
        end_mm = self._inset_mm(min_mm, max_mm, False, TRAVEL_MARGIN_MM)
        if not (start_mm < impact_position_mm < end_mm):
            self.status_var.set(
                f"Impact Position {impact_position_mm}mm must be strictly "
                f"between the inset travel range [{start_mm:.2f}, {end_mm:.2f}]mm"
            )
            return

        try:
            increments = compute_contour_increments(
                start_mm, impact_position_mm, end_mm,
                impact_velocity_m_s, motion_period_us / 1_000_000,
            )
        except ValueError as e:
            self.status_var.set(f"Arm failed: {e}")
            return

        # Invalidate any previously-armed program before attempting a new
        # one; re-entrancy guard on Arm itself while this attempt runs.
        self.contour_fire_button.config(state="disabled")
        self.contour_arm_button.config(state="disabled")

        self.status_var.set("Arming...")

        # Download before any motion is commanded this cycle -- a program
        # download disables the servo for ~300ms (Galil manual, Ch.7), and
        # with nothing being held yet, that disable has nothing to disturb.
        # Explicit MO/SH bracket it so the servo re-engages from a known,
        # torque-free state rather than however it happened to be left,
        # avoiding a snap once the retract (last step, below) settles into
        # its hold.
        try:
            self.controller.disable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: {e}")
            self.contour_arm_button.config(state="normal")
            return

        program_text = build_contour_program(
            start_counts=Controller.mm_to_counts(start_mm),
            slow_speed_counts=Controller.meters_to_counts(SLOW_SPEED_M_S),
            slow_accel_counts=Controller.meters_to_counts(SLOW_ACCEL_M_S2),
            slow_decel_counts=Controller.meters_to_counts(SLOW_DECEL_M_S2),
            motion_sample_period_us=motion_period_us,
            pre_pause_ms=pre_pause_ms,
            motion_kp=motion_kp,
            motion_kd=motion_kd,
            motion_ki=motion_ki,
            motion_offset_v=motion_offset_v,
            motion_fv=motion_fv,
            increments=increments,
            post_pause_ms=post_pause_ms,
            rest_kp=rest_kp,
            rest_kd=rest_kd,
            rest_ki=rest_ki,
            rest_offset_v=rest_offset_v,
        )

        try:
            self.controller.download_program(program_text)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: program download failed: {e}")
            self.contour_arm_button.config(state="normal")
            return

        # Sizes the 4 shared record arrays to RECORD_ARRAY_SIZE before
        # Fire runs this program -- this program's own embedded DM lines
        # request the same fixed size, but relying on THOSE to grow the
        # arrays (if a prior Current Profile Arm left them smaller) was
        # bench-confirmed unreliable; sizing them here first (DA+DM if a
        # real grow is needed, handled by size_shared_record_arrays)
        # makes that embedded DM a harmless same-size redeclaration.
        try:
            self.controller.size_shared_record_arrays(RECORD_ARRAY_SIZE)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: sizing record array space failed: {e}")
            self.contour_arm_button.config(state="normal")
            return

        # Re-enable -- Rest gains applied first, same "never SHA with
        # unknown/unapplied gains" principle as _on_enable.
        if not self._apply_rest_settings():
            self.contour_arm_button.config(state="normal")
            return
        try:
            self.controller.enable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: enable failed: {e}")
            self.contour_arm_button.config(state="normal")
            return

        if not self._slow_move_to_limit(is_retract=True, min_mm=min_mm, max_mm=max_mm, margin=0):
            self.contour_arm_button.config(state="normal")
            return

        def handle_retract_complete():
            self._armed_contour_sample_period_s = motion_period_us / 1_000_000
            self._armed_contour_info = self._format_motion_info("Contoured Motion", [
                ("Start/Stop", f"{min_mm:.2f}/{max_mm:.2f}mm"),
                ("Impact Velocity", f"{impact_velocity_m_s}m/s"),
                ("Impact Position", f"{impact_position_mm}mm"),
                ("Inset Start/End", f"{start_mm:.2f}/{end_mm:.2f}mm"),
                ("Contour Points", str(len(increments))),
                ("Sample Period", f"{motion_period_us}us"),
                ("Pre/Post Pause", f"{pre_pause_ms}/{post_pause_ms}ms"),
                (
                    "Motion PID",
                    f"KP={motion_kp} KD={motion_kd} KI={motion_ki} "
                    f"Offset={motion_offset_v}V FV={motion_fv}",
                ),
                ("Rest PID", f"KP={rest_kp} KD={rest_kd} KI={rest_ki} Offset={rest_offset_v}V"),
            ])
            self.contour_arm_button.config(state="normal")
            self.contour_fire_button.config(state="normal")
            self.status_var.set(
                f"Armed: impact velocity={impact_velocity_m_s}m/s, "
                f"impact position={impact_position_mm}mm, {len(increments)} contour points"
            )

        def handle_retract_error(message):
            self.status_var.set(f"Arm failed: {message}")
            self.contour_arm_button.config(state="normal")

        wait_for_motion_complete(
            self.controller, self.root.after,
            on_complete=handle_retract_complete,
            on_error=handle_retract_error,
        )

    def _on_fire_contour(self):
        self.contour_arm_button.config(state="disabled")
        self.contour_fire_button.config(state="disabled")
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: {e}")
            self.contour_arm_button.config(state="normal")
            self.contour_fire_button.config(state="normal")
            return

        self.status_var.set("Firing...")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=self._on_fire_contour_wait_complete,
            on_error=self._on_fire_contour_wait_error,
        )

    def _on_fire_contour_wait_complete(self):
        # See _on_fire_wait_complete's comment -- "not running" alone
        # doesn't distinguish a clean EN from an early error halt.
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"

        try:
            recorded_data = self.controller.upload_recorded_data()
        except gclib.GclibError as e:
            self.status_var.set(f"Contour data upload failed: {e} [controller last error: {last_error}]")
            self.contour_arm_button.config(state="normal")
            self.contour_fire_button.config(state="normal")
            return

        try:
            plotted = self._show_capture(
                recorded_data, self._armed_contour_sample_period_s, self._armed_contour_info
            )
        except Exception as e:
            self.status_var.set(
                f"Contour ran, but plotting failed: {e} "
                f"[{len(recorded_data.position)} samples, controller last error: {last_error}]"
            )
        else:
            status = (
                f"Contour captured [{len(recorded_data.position)} samples, "
                f"controller last error: {last_error}]"
            )
            if not plotted:
                status = f"{self.status_var.get()} [controller last error: {last_error}]"
            self.status_var.set(status)

        self.contour_arm_button.config(state="normal")
        self.contour_fire_button.config(state="normal")

    def _on_fire_contour_wait_error(self, message):
        self.status_var.set(message)
        self.contour_arm_button.config(state="normal")
        self.contour_fire_button.config(state="normal")

    def _on_arm_current_profile(self):
        # Fully self-contained validation, same style as _on_arm_contour.
        try:
            min_mm = float(self.current_profile_start_var.get())
            max_mm = float(self.current_profile_stop_var.get())
        except ValueError:
            self.status_var.set("Start/Stop must be numbers (mm)")
            return
        if not (0 <= min_mm < max_mm <= FULL_STROKE_MM):
            self.status_var.set(
                f"Start/Stop must satisfy 0 <= start < stop <= {FULL_STROKE_MM}mm"
            )
            return

        # No Motion FV/Sample Period fields for this mode -- see the
        # frame's own comment.
        try:
            motion_kp = float(self.motion_kp_var.get())
            motion_kd = float(self.motion_kd_var.get())
            motion_ki = float(self.motion_ki_var.get())
            motion_offset_v = float(self.motion_offset_var.get())
        except ValueError:
            self.status_var.set("Motion KP/KD/KI/Offset must be numbers")
            return

        try:
            pre_pause_ms = int(self.current_profile_pre_pause_ms_var.get())
            post_pause_ms = int(self.current_profile_post_pause_ms_var.get())
        except ValueError:
            self.status_var.set("Pre/Post-Move Pause must be whole numbers (ms)")
            return
        if pre_pause_ms < 0 or post_pause_ms < 0:
            self.status_var.set("Pre/Post-Move Pause must not be negative")
            return

        try:
            ff_advance_us = float(self.current_profile_ff_advance_us_var.get())
        except ValueError:
            self.status_var.set("Feedforward Advance must be a number (us)")
            return
        if ff_advance_us < 0:
            self.status_var.set("Feedforward Advance must not be negative")
            return

        try:
            rest_kp = float(self.rest_kp_var.get())
            rest_kd = float(self.rest_kd_var.get())
            rest_ki = float(self.rest_ki_var.get())
            rest_offset_v = float(self.rest_offset_var.get())
        except ValueError:
            self.status_var.set("Rest KP/KD/KI/Offset must be numbers")
            return

        try:
            impact_velocity_m_s = float(self.current_profile_velocity_var.get())
        except ValueError:
            self.status_var.set("Impact Velocity must be a number (m/s)")
            return
        if impact_velocity_m_s <= 0:
            self.status_var.set("Impact Velocity must be positive")
            return

        try:
            impact_position_mm = float(self.current_profile_position_var.get())
        except ValueError:
            self.status_var.set("Impact Position must be a number (mm)")
            return

        try:
            peak_v_position_mm = float(self.current_profile_peak_velocity_position_var.get())
        except ValueError:
            self.status_var.set("Peak Velocity Position must be a number (mm)")
            return

        try:
            mass_kg = float(self.current_profile_mass_var.get())
        except ValueError:
            self.status_var.set("Mass must be a number (kg)")
            return
        if mass_kg <= 0:
            self.status_var.set("Mass must be positive")
            return

        try:
            kf_accel = float(self.current_profile_kf_accel_var.get())
            kf_decel = float(self.current_profile_kf_decel_var.get())
        except ValueError:
            self.status_var.set("Force Constant Accel/Decel (Kf) must be numbers")
            return
        if kf_accel == 0 or kf_decel == 0:
            self.status_var.set("Force Constant Accel/Decel (Kf) must not be zero")
            return

        try:
            ff_accel_rise_stretch = float(self.current_profile_ff_accel_rise_stretch_var.get())
            ff_accel_fall_stretch = float(self.current_profile_ff_accel_fall_stretch_var.get())
            ff_decel_rise_stretch = float(self.current_profile_ff_decel_rise_stretch_var.get())
            ff_decel_fall_stretch = float(self.current_profile_ff_decel_fall_stretch_var.get())
        except ValueError:
            self.status_var.set("Accel/Decel Rise/Fall Stretch must be numbers")
            return
        if min(
            ff_accel_rise_stretch, ff_accel_fall_stretch,
            ff_decel_rise_stretch, ff_decel_fall_stretch,
        ) <= 0:
            self.status_var.set("Accel/Decel Rise/Fall Stretch must be positive")
            return

        # No padding -- Arm moves to exactly the user-specified Start; add
        # any buffer by editing the Start field itself.
        start_mm = self._inset_mm(min_mm, max_mm, True, 0)
        end_mm = self._inset_mm(min_mm, max_mm, False, TRAVEL_MARGIN_MM)
        if not (start_mm < peak_v_position_mm < end_mm):
            self.status_var.set(
                f"Peak Velocity Position {peak_v_position_mm}mm must satisfy, within "
                f"the inset travel range: {start_mm:.2f} < peak < {end_mm:.2f}mm"
            )
            return
        if not (start_mm < impact_position_mm < end_mm):
            self.status_var.set(
                f"Impact Position {impact_position_mm}mm must satisfy, within the "
                f"inset travel range: {start_mm:.2f} < impact < {end_mm:.2f}mm"
            )
            return

        try:
            curve = compute_current_profile_curve(
                start_mm, peak_v_position_mm, impact_position_mm, end_mm, impact_velocity_m_s,
            )
        except ValueError as e:
            self.status_var.set(f"Arm failed: {e}")
            return

        # Position sampled at the Contour Mode CD grid (2*TM_US_ACTUAL),
        # feedforward sampled at the bench-confirmed OF-update cadence
        # (AT_SAMPLES*TM_US_ACTUAL) -- both drawn from the same curve,
        # independently resampled. Uses the bench-measured REAL sample
        # period (TM_US_ACTUAL, ~122us), not the nominal requested TM_US
        # (125us) -- see current_profile_program.py's own comment on
        # TM_US_ACTUAL for the bench measurement behind this. See also
        # current_profile_trajectory.py.
        cd_period_s = 2 * CURRENT_PROFILE_TM_US_ACTUAL / 1_000_000
        ff_period_s = CURRENT_PROFILE_AT_SAMPLES * CURRENT_PROFILE_TM_US_ACTUAL / 1_000_000
        positions_mm = sample_position_mm(curve, cd_period_s)
        positions_counts = [Controller.mm_to_counts(start_mm)] + [
            Controller.mm_to_counts(p) for p in positions_mm
        ]
        increments = [
            positions_counts[i] - positions_counts[i - 1]
            for i in range(1, len(positions_counts))
        ]
        ff_values = compute_feedforward_v(
            curve, ff_period_s, mass_kg, kf_accel, kf_decel,
            ff_accel_rise_stretch, ff_accel_fall_stretch, ff_decel_rise_stretch, ff_decel_fall_stretch,
        )
        # Separate, finer resample at TM_US_ACTUAL itself -- purely so the
        # feedforward curve lines up with the recording's own sample grid
        # for plotting later (_show_capture's feedforward_v), independent
        # of ff_values above (what's actually downloaded/played back). Same
        # stretch factors as above so the plot matches what's downloaded.
        ff_plot_v = compute_feedforward_v(
            curve, CURRENT_PROFILE_TM_US_ACTUAL / 1_000_000, mass_kg, kf_accel, kf_decel,
            ff_accel_rise_stretch, ff_accel_fall_stretch, ff_decel_rise_stretch, ff_decel_fall_stretch,
        )
        # Recording (RC 1) starts before pre_pause_ms's WT, so index 0 of
        # every *recorded* trace (Position, Cmd Position) is the pre-move
        # pause, before the curve's own t=0 (when CMA/the first CD point
        # actually fires) -- pad ff_plot_v with that many leading zero
        # samples so it shares the same true time origin as the recorded
        # traces once _show_capture pads/truncates it to length, rather
        # than appearing shifted pre_pause_ms too early.
        ff_plot_v = [0.0] * round(pre_pause_ms * 1000 / CURRENT_PROFILE_TM_US_ACTUAL) + ff_plot_v

        try:
            program_text, record_samples = build_current_profile_program(
                start_counts=Controller.mm_to_counts(start_mm),
                slow_speed_counts=Controller.meters_to_counts(SLOW_SPEED_M_S),
                slow_accel_counts=Controller.meters_to_counts(SLOW_ACCEL_M_S2),
                slow_decel_counts=Controller.meters_to_counts(SLOW_DECEL_M_S2),
                pre_pause_ms=pre_pause_ms,
                motion_kp=motion_kp,
                motion_kd=motion_kd,
                motion_ki=motion_ki,
                motion_offset_v=motion_offset_v,
                increments=increments,
                ff_values=ff_values,
                ff_advance_us=ff_advance_us,
                post_pause_ms=post_pause_ms,
                rest_kp=rest_kp,
                rest_kd=rest_kd,
                rest_ki=rest_ki,
                rest_offset_v=rest_offset_v,
            )
        except ValueError as e:
            self.status_var.set(f"Arm failed: {e}")
            return

        # Invalidate any previously-armed program before attempting a new
        # one; re-entrancy guard on Arm itself while this attempt runs.
        self.current_profile_fire_button.config(state="disabled")
        self.current_profile_arm_button.config(state="disabled")

        self.status_var.set("Arming...")

        try:
            self.controller.disable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: {e}")
            self.current_profile_arm_button.config(state="normal")
            return

        try:
            self.controller.download_program(program_text)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: program download failed: {e}")
            self.current_profile_arm_button.config(state="normal")
            return

        # Size the 4 shared record arrays to exactly what this program's
        # own DM lines will also request, right now at Arm time -- frees
        # array-table space for the feedforward array below (these 4
        # arrays would otherwise still be sized from whatever the last
        # program to actually RUN left them at, up to the full
        # 4x750=3000 budget by default) AND avoids relying on the new
        # program's own DM lines to grow them later at Fire time, which
        # was bench-confirmed unreliable on its own. See
        # Controller.size_shared_record_arrays.
        try:
            self.controller.size_shared_record_arrays(record_samples)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: sizing record array space failed: {e}")
            self.current_profile_arm_button.config(state="normal")
            return

        # Array after program, per current_profile_program.py's own QD
        # caveat -- bench-confirmed ordering (Phase 0's ff_loop_timing_
        # bench.py/concurrent_thread_bench.py).
        try:
            self.controller.download_array(CURRENT_PROFILE_FF_ARRAY_NAME, ff_values)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: feedforward array download failed: {e}")
            self.current_profile_arm_button.config(state="normal")
            return

        # Re-enable -- Rest gains applied first, same "never SHA with
        # unknown/unapplied gains" principle as _on_enable.
        if not self._apply_rest_settings():
            self.current_profile_arm_button.config(state="normal")
            return
        try:
            self.controller.enable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: enable failed: {e}")
            self.current_profile_arm_button.config(state="normal")
            return

        if not self._slow_move_to_limit(is_retract=True, min_mm=min_mm, max_mm=max_mm, margin=0):
            self.current_profile_arm_button.config(state="normal")
            return

        def handle_retract_complete():
            self._armed_current_profile_ff_plot_v = ff_plot_v
            self._armed_current_profile_info = self._format_motion_info("Current Profile Motion", [
                ("Start/Stop", f"{min_mm:.2f}/{max_mm:.2f}mm"),
                ("Impact Velocity", f"{impact_velocity_m_s}m/s"),
                ("Impact Position", f"{impact_position_mm}mm"),
                ("Peak Velocity Position", f"{peak_v_position_mm}mm"),
                ("Peak Velocity (solved)", f"{curve.phase2.v0:.4f}m/s"),
                ("Inset Start/End", f"{start_mm:.2f}/{end_mm:.2f}mm"),
                ("Contour Points", str(len(increments))),
                ("Feedforward Samples", str(len(ff_values))),
                ("Mass", f"{mass_kg}kg"),
                ("Force Constant Accel/Decel (Kf)", f"{kf_accel}/{kf_decel}"),
                (
                    "FF Stretch (accel rise/fall, decel rise/fall)",
                    f"{ff_accel_rise_stretch}/{ff_accel_fall_stretch}/"
                    f"{ff_decel_rise_stretch}/{ff_decel_fall_stretch}",
                ),
                ("Pre/Post Pause", f"{pre_pause_ms}/{post_pause_ms}ms"),
                ("Feedforward Advance", f"{ff_advance_us}us"),
                (
                    "Motion PID",
                    f"KP={motion_kp} KD={motion_kd} KI={motion_ki} Offset={motion_offset_v}V",
                ),
                ("Rest PID", f"KP={rest_kp} KD={rest_kd} KI={rest_ki} Offset={rest_offset_v}V"),
            ])
            self.current_profile_arm_button.config(state="normal")
            self.current_profile_fire_button.config(state="normal")
            self.status_var.set(
                f"Armed: impact velocity={impact_velocity_m_s}m/s, "
                f"impact position={impact_position_mm}mm, {len(increments)} contour points"
            )

        def handle_retract_error(message):
            self.status_var.set(f"Arm failed: {message}")
            self.current_profile_arm_button.config(state="normal")

        wait_for_motion_complete(
            self.controller, self.root.after,
            on_complete=handle_retract_complete,
            on_error=handle_retract_error,
        )

    def _on_fire_current_profile(self):
        self.current_profile_arm_button.config(state="disabled")
        self.current_profile_fire_button.config(state="disabled")
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: {e}")
            self.current_profile_arm_button.config(state="normal")
            self.current_profile_fire_button.config(state="normal")
            return

        self.status_var.set("Firing...")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=self._on_fire_current_profile_wait_complete,
            on_error=self._on_fire_current_profile_wait_error,
        )

    def _on_fire_current_profile_wait_complete(self):
        # See _on_fire_wait_complete's comment -- "not running" alone
        # doesn't distinguish a clean EN from an early error halt.
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"

        try:
            recorded_data = self.controller.upload_recorded_data()
        except gclib.GclibError as e:
            self.status_var.set(
                f"Current Profile data upload failed: {e} [controller last error: {last_error}]"
            )
            self.current_profile_arm_button.config(state="normal")
            self.current_profile_fire_button.config(state="normal")
            return

        try:
            plotted = self._show_capture(
                recorded_data, CURRENT_PROFILE_TM_US_ACTUAL / 1_000_000,
                self._armed_current_profile_info,
                feedforward_v=self._armed_current_profile_ff_plot_v,
            )
        except Exception as e:
            self.status_var.set(
                f"Current Profile ran, but plotting failed: {e} "
                f"[{len(recorded_data.position)} samples, controller last error: {last_error}]"
            )
        else:
            status = (
                f"Current Profile captured [{len(recorded_data.position)} samples, "
                f"controller last error: {last_error}]"
            )
            if not plotted:
                status = f"{self.status_var.get()} [controller last error: {last_error}]"
            self.status_var.set(status)

        self.current_profile_arm_button.config(state="normal")
        self.current_profile_fire_button.config(state="normal")

    def _on_fire_current_profile_wait_error(self, message):
        self.status_var.set(message)
        self.current_profile_arm_button.config(state="normal")
        self.current_profile_fire_button.config(state="normal")

    def _on_arm_pvt(self):
        # Fully self-contained validation, same style as _on_arm_contour.
        try:
            min_mm = float(self.pvt_start_var.get())
            max_mm = float(self.pvt_stop_var.get())
        except ValueError:
            self.status_var.set("Start/Stop must be numbers (mm)")
            return
        if not (0 <= min_mm < max_mm <= FULL_STROKE_MM):
            self.status_var.set(
                f"Start/Stop must satisfy 0 <= start < stop <= {FULL_STROKE_MM}mm"
            )
            return

        try:
            motion_kp = float(self.motion_kp_var.get())
            motion_kd = float(self.motion_kd_var.get())
            motion_ki = float(self.motion_ki_var.get())
            motion_offset_v = float(self.motion_offset_var.get())
            # PVT's own FV (below) overrides this for the move itself --
            # still parsed here since build_pvt_program needs it to reset
            # FV back to Motion PID's own value once the move completes.
            motion_fv = float(self.motion_fv_var.get())
        except ValueError:
            self.status_var.set("Motion KP/KD/KI/Offset/FV must be numbers")
            return

        try:
            motion_period_us = float(self.motion_sample_period_us_var.get())
        except ValueError:
            self.status_var.set("Motion sampling period must be a number (usec)")
            return
        if not (TM_MIN_US <= motion_period_us <= TM_MAX_US):
            self.status_var.set(
                f"Motion sampling period must be {TM_MIN_US}-{TM_MAX_US}usec"
            )
            return

        try:
            pre_pause_ms = int(self.pre_pause_ms_var.get())
            post_pause_ms = int(self.post_pause_ms_var.get())
        except ValueError:
            self.status_var.set("Pre/Post-Move Pause must be whole numbers (ms)")
            return
        if pre_pause_ms < 0 or post_pause_ms < 0:
            self.status_var.set("Pre/Post-Move Pause must not be negative")
            return

        try:
            rest_kp = float(self.rest_kp_var.get())
            rest_kd = float(self.rest_kd_var.get())
            rest_ki = float(self.rest_ki_var.get())
            rest_offset_v = float(self.rest_offset_var.get())
        except ValueError:
            self.status_var.set("Rest KP/KD/KI/Offset must be numbers")
            return

        try:
            impact_velocity_m_s = float(self.pvt_velocity_var.get())
        except ValueError:
            self.status_var.set("Impact Velocity must be a number (m/s)")
            return
        if impact_velocity_m_s <= 0:
            self.status_var.set("Impact Velocity must be positive")
            return

        try:
            impact_position_mm = float(self.pvt_position_var.get())
        except ValueError:
            self.status_var.set("Impact Position must be a number (mm)")
            return

        try:
            pvt_fv = float(self.pvt_fv_var.get())
        except ValueError:
            self.status_var.set("PVT FV must be a number")
            return
        if not (FV_MIN <= pvt_fv <= FV_MAX):
            self.status_var.set(f"PVT FV must be {FV_MIN}-{FV_MAX}")
            return

        try:
            pvt_fa = float(self.pvt_fa_var.get())
        except ValueError:
            self.status_var.set("PVT FA must be a number")
            return
        if not (FA_MIN <= pvt_fa <= FA_MAX):
            self.status_var.set(f"PVT FA must be {FA_MIN}-{FA_MAX}")
            return

        try:
            pvt_ac_m_s2 = float(self.pvt_ac_var.get())
            pvt_dc_m_s2 = float(self.pvt_dc_var.get())
        except ValueError:
            self.status_var.set("PVT Accel/Decel must be numbers (m/s²)")
            return
        if not (ACCEL_RANGE_M_S2[0] <= pvt_ac_m_s2 <= ACCEL_RANGE_M_S2[1]):
            self.status_var.set(
                f"PVT Accel must be {ACCEL_RANGE_M_S2[0]:.3f}-{ACCEL_RANGE_M_S2[1]:.3f}m/s²"
            )
            return
        if not (ACCEL_RANGE_M_S2[0] <= pvt_dc_m_s2 <= ACCEL_RANGE_M_S2[1]):
            self.status_var.set(
                f"PVT Decel must be {ACCEL_RANGE_M_S2[0]:.3f}-{ACCEL_RANGE_M_S2[1]:.3f}m/s²"
            )
            return

        # No padding -- Arm moves to exactly the user-specified Start; add
        # any buffer by editing the Start field itself.
        start_mm = self._inset_mm(min_mm, max_mm, True, 0)
        end_mm = self._inset_mm(min_mm, max_mm, False, TRAVEL_MARGIN_MM)
        if not (start_mm < impact_position_mm < end_mm):
            self.status_var.set(
                f"Impact Position {impact_position_mm}mm must be strictly "
                f"between the inset travel range [{start_mm:.2f}, {end_mm:.2f}]mm"
            )
            return

        try:
            segments = compute_pvt_segments(
                start_mm, impact_position_mm, end_mm,
                impact_velocity_m_s, motion_period_us / 1_000_000,
            )
        except ValueError as e:
            self.status_var.set(f"Arm failed: {e}")
            return

        # Invalidate any previously-armed program before attempting a new
        # one; re-entrancy guard on Arm itself while this attempt runs.
        self.pvt_fire_button.config(state="disabled")
        self.pvt_arm_button.config(state="disabled")

        self.status_var.set("Arming...")

        # Download before any motion is commanded this cycle -- a program
        # download disables the servo for ~300ms (Galil manual, Ch.7), and
        # with nothing being held yet, that disable has nothing to disturb.
        # Explicit MO/SH bracket it so the servo re-engages from a known,
        # torque-free state rather than however it happened to be left,
        # avoiding a snap once the retract (last step, below) settles into
        # its hold.
        try:
            self.controller.disable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: {e}")
            self.pvt_arm_button.config(state="normal")
            return

        program_text = build_pvt_program(
            start_counts=Controller.mm_to_counts(start_mm),
            slow_speed_counts=Controller.meters_to_counts(SLOW_SPEED_M_S),
            slow_accel_counts=Controller.meters_to_counts(SLOW_ACCEL_M_S2),
            slow_decel_counts=Controller.meters_to_counts(SLOW_DECEL_M_S2),
            motion_sample_period_us=motion_period_us,
            pre_pause_ms=pre_pause_ms,
            motion_kp=motion_kp,
            motion_kd=motion_kd,
            motion_ki=motion_ki,
            motion_offset_v=motion_offset_v,
            motion_fv=motion_fv,
            pvt_fv=pvt_fv,
            pvt_fa=pvt_fa,
            pvt_ac_counts=Controller.meters_to_counts(pvt_ac_m_s2),
            pvt_dc_counts=Controller.meters_to_counts(pvt_dc_m_s2),
            segments=segments,
            post_pause_ms=post_pause_ms,
            rest_kp=rest_kp,
            rest_kd=rest_kd,
            rest_ki=rest_ki,
            rest_offset_v=rest_offset_v,
        )

        try:
            self.controller.download_program(program_text)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: program download failed: {e}")
            self.pvt_arm_button.config(state="normal")
            return

        # Sizes the 4 shared record arrays to RECORD_ARRAY_SIZE before
        # Fire runs this program -- this program's own embedded DM lines
        # request the same fixed size, but relying on THOSE to grow the
        # arrays (if a prior Current Profile Arm left them smaller) was
        # bench-confirmed unreliable; sizing them here first (DA+DM if a
        # real grow is needed, handled by size_shared_record_arrays)
        # makes that embedded DM a harmless same-size redeclaration.
        try:
            self.controller.size_shared_record_arrays(RECORD_ARRAY_SIZE)
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: sizing record array space failed: {e}")
            self.pvt_arm_button.config(state="normal")
            return

        # Re-enable -- Rest gains applied first, same "never SHA with
        # unknown/unapplied gains" principle as _on_enable.
        if not self._apply_rest_settings():
            self.pvt_arm_button.config(state="normal")
            return
        try:
            self.controller.enable()
        except gclib.GclibError as e:
            self.status_var.set(f"Arm failed: enable failed: {e}")
            self.pvt_arm_button.config(state="normal")
            return

        if not self._slow_move_to_limit(is_retract=True, min_mm=min_mm, max_mm=max_mm, margin=0):
            self.pvt_arm_button.config(state="normal")
            return

        def handle_retract_complete():
            self._armed_pvt_sample_period_s = motion_period_us / 1_000_000
            self._armed_pvt_info = self._format_motion_info("PVT Motion", [
                ("Start/Stop", f"{min_mm:.2f}/{max_mm:.2f}mm"),
                ("Impact Velocity", f"{impact_velocity_m_s}m/s"),
                ("Impact Position", f"{impact_position_mm}mm"),
                ("Inset Start/End", f"{start_mm:.2f}/{end_mm:.2f}mm"),
                ("PVT Segments", str(len(segments))),
                ("PVT FV/FA", f"{pvt_fv}/{pvt_fa}"),
                ("PVT Accel/Decel", f"{pvt_ac_m_s2}/{pvt_dc_m_s2} m/s²"),
                ("Sample Period", f"{motion_period_us}us"),
                ("Pre/Post Pause", f"{pre_pause_ms}/{post_pause_ms}ms"),
                (
                    "Motion PID",
                    f"KP={motion_kp} KD={motion_kd} KI={motion_ki} "
                    f"Offset={motion_offset_v}V FV={motion_fv}",
                ),
                ("Rest PID", f"KP={rest_kp} KD={rest_kd} KI={rest_ki} Offset={rest_offset_v}V"),
            ])
            self.pvt_arm_button.config(state="normal")
            self.pvt_fire_button.config(state="normal")
            self.status_var.set(
                f"Armed: impact velocity={impact_velocity_m_s}m/s, "
                f"impact position={impact_position_mm}mm, {len(segments)} PVT segments"
            )

        def handle_retract_error(message):
            self.status_var.set(f"Arm failed: {message}")
            self.pvt_arm_button.config(state="normal")

        wait_for_motion_complete(
            self.controller, self.root.after,
            on_complete=handle_retract_complete,
            on_error=handle_retract_error,
        )

    def _on_fire_pvt(self):
        self.pvt_arm_button.config(state="disabled")
        self.pvt_fire_button.config(state="disabled")
        try:
            self.controller.run_program()
        except gclib.GclibError as e:
            self.status_var.set(f"Fire failed: {e}")
            self.pvt_arm_button.config(state="normal")
            self.pvt_fire_button.config(state="normal")
            return

        self.status_var.set("Firing...")
        wait_for_program_complete(
            self.controller, self.root.after,
            on_complete=self._on_fire_pvt_wait_complete,
            on_error=self._on_fire_pvt_wait_error,
        )

    def _on_fire_pvt_wait_complete(self):
        # See _on_fire_wait_complete's comment -- "not running" alone
        # doesn't distinguish a clean EN from an early error halt.
        try:
            last_error = self.controller.read_last_error()
        except gclib.GclibError as e:
            last_error = f"(could not read: {e})"

        try:
            recorded_data = self.controller.upload_recorded_data()
        except gclib.GclibError as e:
            self.status_var.set(f"PVT data upload failed: {e} [controller last error: {last_error}]")
            self.pvt_arm_button.config(state="normal")
            self.pvt_fire_button.config(state="normal")
            return

        try:
            plotted = self._show_capture(
                recorded_data, self._armed_pvt_sample_period_s, self._armed_pvt_info
            )
        except Exception as e:
            self.status_var.set(
                f"PVT ran, but plotting failed: {e} "
                f"[{len(recorded_data.position)} samples, controller last error: {last_error}]"
            )
        else:
            status = (
                f"PVT captured [{len(recorded_data.position)} samples, "
                f"controller last error: {last_error}]"
            )
            if not plotted:
                status = f"{self.status_var.get()} [controller last error: {last_error}]"
            self.status_var.set(status)

        self.pvt_arm_button.config(state="normal")
        self.pvt_fire_button.config(state="normal")

    def _on_fire_pvt_wait_error(self, message):
        self.status_var.set(message)
        self.pvt_arm_button.config(state="normal")
        self.pvt_fire_button.config(state="normal")

    def _on_recording_complete(self, recorded_data):
        try:
            self._show_capture(recorded_data, self._capture_sample_period_s, self._go_capture_info)
        except Exception as e:
            self.status_var.set(f"Capture failed to plot: {e}")

    @staticmethod
    def _format_motion_info(motion_type: str, pairs) -> str:
        return motion_type + "\n" + "\n".join(f"{label}: {value}" for label, value in pairs)

    def _show_capture(self, recorded_data, sample_period_s, info_text, feedforward_v=None) -> bool:
        # Never attempt to plot too few samples for a meaningful
        # savgol window -- bench-confirmed this can otherwise raise deep
        # inside scipy (window_length > sample count), which is easy to
        # trigger with a near-empty capture and, left uncaught, silently
        # kills whichever Tkinter callback is displaying the result.
        if len(recorded_data.position) < SLIDER_WINDOW_LENGTH_MIN:
            self.status_var.set(
                f"Only {len(recorded_data.position)} sample(s) recorded -- nothing to plot"
            )
            return False

        position_mm = [Controller.counts_to_mm(float(p)) for p in recorded_data.position]
        # The same recorded counts through the piecewise map, for the plot's
        # Position N/Velocity N traces. Omitted when only 2 points are
        # calibrated, where the piecewise map IS the straight line and the
        # N traces would just sit on top of the L ones.
        position_mm_nonlinear = None
        if Controller.calibration_region_count() > 1:
            position_mm_nonlinear = [
                Controller.counts_to_mm_piecewise(float(p)) for p in recorded_data.position
            ]
        # Commanded position stays on the linear map: the trajectory was
        # generated through mm_to_counts, which is linear by design (see
        # Controller._linear_span), so converting it back the same way is
        # the self-consistent choice.
        commanded_position_mm = [
            Controller.counts_to_mm(float(p)) for p in recorded_data.commanded_position
        ]
        position_error_mm = [
            Controller.counts_delta_to_mm(float(e)) for e in recorded_data.position_error
        ]
        torque_v = [Controller.torque_counts_to_volts(float(t)) for t in recorded_data.torque]
        current_a = [Controller.volts_to_current(v) for v in torque_v]
        commanded_velocity_m_s = compute_velocity(commanded_position_mm, sample_period_s)
        # Amp gain is read live here, not snapshotted at Arm/Go time -- it's
        # a post-hoc display setting, not baked into the downloaded
        # program, so whatever's active now is what actually produced
        # current_a above.
        full_scale_amps = Controller.get_amp_full_scale_current()
        info_text = f"{info_text}\nAmp Full-Scale Current: {full_scale_amps:.3f}A"
        if feedforward_v is not None:
            # Precomputed at Arm time (Current Profile mode only) from the
            # planned curve, so its length can differ slightly from
            # however many samples the controller actually recorded --
            # pad/truncate to match rather than handing build_capture_figure
            # a mismatched-length series.
            n = len(position_mm)
            feedforward_v = (feedforward_v + [0.0] * n)[:n]
        figure = build_capture_figure(
            position_mm, commanded_position_mm, position_error_mm, torque_v, current_a,
            commanded_velocity_m_s, sample_period_s, info_text, feedforward_v=feedforward_v,
            position_mm_nonlinear=position_mm_nonlinear,
            calibration_text=self._calibration_text(),
        )

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        CAPTURES_DIR.mkdir(exist_ok=True)
        filepath = CAPTURES_DIR / f"capture_{timestamp}.png"
        figure.savefig(filepath)

        popup = tk.Toplevel(self.root)
        popup.title(f"Impact capture - {timestamp}")
        canvas = FigureCanvasTkAgg(figure, master=popup)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(canvas, popup)
        toolbar.update()
        return True

    def _on_set_calibration_point(self, index):
        """Average ~1s of readings and store them as point `index`'s LVDT
        value. Non-blocking: samples are collected via root.after so the
        GUI (and the Disable button) stay responsive throughout."""
        if not self.controller.is_connected:
            self.status_var.set("Not connected -- nothing to calibrate")
            return
        if self._cal_sampling:
            return

        self._cal_sampling = True
        for button in self.cal_set_buttons:
            button.config(state="disabled")
        counts_samples, volts_samples = [], []

        def finish(message):
            self._cal_sampling = False
            for button in self.cal_set_buttons:
                button.config(state="normal" if self.controller.is_connected else "disabled")
            self.status_var.set(message)

        def sample():
            try:
                counts_samples.append(float(self.controller.read_position()))
                volts_samples.append(float(self.controller.read_lvdt_voltage()))
            except gclib.GclibError as e:
                finish(f"Set Point {index + 1} failed: {e}")
                return
            if len(counts_samples) < CALIBRATION_SAMPLE_COUNT:
                self.status_var.set(
                    f"Sampling point {index + 1}... "
                    f"{len(counts_samples)}/{CALIBRATION_SAMPLE_COUNT}"
                )
                self.root.after(CALIBRATION_SAMPLE_INTERVAL_MS, sample)
                return

            mean_counts = sum(counts_samples) / len(counts_samples)
            mean_volts = sum(volts_samples) / len(volts_samples)
            # Write the measurement into the same entry the user can type
            # into, so captured and hand-entered points are one thing from
            # here on, and keep the measured counts against the voltage it
            # came from (see _active_calibration_points).
            self.cal_voltage_vars[index].set(f"{mean_volts:+.4f}")
            self._cal_counts[index] = mean_counts
            # The entry text is rounded, so remember the rounded value --
            # that's what _counts_for_point will be comparing against when
            # it decides whether this row is still the measured one.
            self._cal_volts[index] = float(self.cal_voltage_vars[index].get())
            problem = self._rebuild_calibration()
            if problem:
                finish(f"Point {index + 1} captured, but {problem}")
                return
            message = (
                f"Point {index + 1} set: {mean_volts:+.4f}V "
                f"({mean_counts:.0f} counts), averaged over "
                f"{CALIBRATION_SAMPLE_COUNT} samples"
            )
            # The measured counts and the measured volts are two views of
            # one 16-bit reading, so they should agree through
            # COUNTS_PER_VOLT. If they don't, that constant is wrong and
            # every hand-typed voltage is being converted wrongly -- worth
            # saying out loud the moment it's observable rather than
            # letting it show up as a quietly skewed calibration.
            expected_counts = Controller.volts_to_counts(mean_volts)
            if abs(expected_counts - mean_counts) > CALIBRATION_COUNTS_PER_VOLT_TOLERANCE:
                message += (
                    f" -- WARNING: {mean_volts:+.4f}V converts to "
                    f"{expected_counts:.0f} counts, not {mean_counts:.0f}; "
                    f"hand-entered voltages will be off"
                )
            finish(message)

        self.status_var.set(f"Sampling point {index + 1}...")
        self.root.after(CALIBRATION_SAMPLE_INTERVAL_MS, sample)

    def _bind_calibration_commit(self, entry):
        entry.bind("<FocusOut>", lambda _e: self._rebuild_calibration())
        entry.bind("<Return>", lambda _e: self._rebuild_calibration())

    def _active_calibration_points(self):
        """(counts, mm) for every row that has BOTH a voltage and a
        position -- a row missing either is skipped, which is what lets 2, 3
        or 4 filled points define 1, 2 or 3 regions. Returns
        (points, problem_message).

        A row whose voltage still reads exactly what Set measured keeps the
        directly measured counts; a typed or edited voltage is converted
        through COUNTS_PER_VOLT instead. So a calibration can be entered
        entirely by hand, and capturing loses no precision to a round trip
        through volts."""
        points = []
        for index in range(4):
            raw_mm = self.cal_position_vars[index].get().strip()
            raw_volts = self.cal_voltage_vars[index].get().strip()
            if not raw_mm or not raw_volts:
                continue
            try:
                mm = float(raw_mm)
            except ValueError:
                return [], f"Point {index + 1} position must be a number (mm)"
            try:
                volts = float(raw_volts)
            except ValueError:
                return [], f"Point {index + 1} LVDT value must be a number (V)"
            points.append((self._counts_for_point(index, volts), mm))
        return points, None

    def _counts_for_point(self, index, volts):
        if self._cal_counts[index] is not None and self._cal_volts[index] == volts:
            return self._cal_counts[index]
        return Controller.volts_to_counts(volts)

    def _rebuild_calibration(self):
        """Re-derive the calibration from whichever rows are complete.
        Returns None on success, or a message describing why the points
        couldn't be applied -- the entered values are kept either way, so a
        half-entered set can be finished rather than restarted."""
        points, problem = self._active_calibration_points()
        if problem is None and len(points) >= 2:
            try:
                Controller.set_calibration_points(points)
            except ValueError as e:
                problem = str(e)
        elif problem is None:
            problem = (
                f"only {len(points)} complete point(s) -- need at least 2 "
                f"(a point needs both a position and an LVDT voltage)"
            )
        self._refresh_calibration_display()
        return problem

    def _on_apply_amp_gain(self):
        try:
            full_scale_amps = float(self.amp_full_scale_current_var.get())
        except ValueError:
            self.status_var.set("Amp Full-Scale Current must be a number (A)")
            return
        if not (AMP_FULL_SCALE_CURRENT_MIN_A <= full_scale_amps <= AMP_FULL_SCALE_CURRENT_MAX_A):
            self.status_var.set(
                f"Amp Full-Scale Current must be "
                f"{AMP_FULL_SCALE_CURRENT_MIN_A}-{AMP_FULL_SCALE_CURRENT_MAX_A}A"
            )
            return
        Controller.set_amp_current_per_volt(full_scale_amps)
        self.status_var.set(f"Amp Full-Scale Current set to {full_scale_amps}A")

    def _refresh_calibration_display(self):
        """Per-row 'measured'/'entered' tags plus the active map's scales.

        Each region gets its own line: with one region that's just the
        straight-line scale, and with more, how far those per-region
        numbers differ from each other IS the nonlinearity being measured,
        so they're worth reading side by side. The overall linear scale is
        kept on the last line because it -- not the piecewise map -- is
        what every commanded move is still built on."""
        for index in range(4):
            raw_volts = self.cal_voltage_vars[index].get().strip()
            measured = False
            if raw_volts and self._cal_counts[index] is not None:
                try:
                    measured = float(raw_volts) == self._cal_volts[index]
                except ValueError:
                    measured = False
            if not raw_volts or not self.cal_position_vars[index].get().strip():
                self.cal_captured_vars[index].set("(skipped)")
            elif measured:
                self.cal_captured_vars[index].set(f"measured {self._cal_counts[index]:.0f} cts")
            else:
                self.cal_captured_vars[index].set("entered")

        self.scale_var.set("\n".join(self._calibration_scale_lines()))

    @staticmethod
    def _calibration_scale_lines():
        """The scale table: one row per active region, then the straight
        line through the outermost points. With a single region the two
        rows are identical by definition; with more, how far the per-region
        rows differ from each other IS the nonlinearity being measured. The
        Linear row is always shown because it -- not the piecewise map --
        is what every commanded move is still built on."""
        rows = ["Region   range (mm)     cts/mm    mm/V"]
        for index, region in enumerate(Controller.calibration_regions()):
            mm_lo, mm_hi, counts_per_mm, mm_per_volt = region
            rows.append(
                f"{index + 1:^6}{mm_lo:6.2f}-{mm_hi:6.2f}  "
                f"{counts_per_mm:9.2f}  {mm_per_volt:6.3f}"
            )
        points = Controller.calibration_points()
        counts_per_mm = Controller.counts_per_meter() / 1000
        rows.append(
            f"Linear{points[0][1]:6.2f}-{points[-1][1]:6.2f}  {counts_per_mm:9.2f}  "
            f"{1.0 / Controller.counts_to_volts(counts_per_mm):6.3f}"
        )
        return rows

    def _calibration_text(self):
        """The calibration block rendered alongside each capture, so a saved
        plot carries the map that produced it rather than depending on
        whatever is configured whenever it's next looked at."""
        lines = ["LVDT Calibration"]
        for counts, mm in Controller.calibration_points():
            lines.append(
                f"{mm:8.2f}mm {Controller.counts_to_volts(counts):+8.4f}V {counts:10.0f}cts"
            )
        lines.append("")
        lines += self._calibration_scale_lines()
        return "\n".join(lines)

    def _apply_motion_profile(self, speed_m_s, accel_m_s2, decel_m_s2) -> bool:
        speed = Controller.meters_to_counts(speed_m_s)
        accel = Controller.meters_to_counts(accel_m_s2)
        decel = Controller.meters_to_counts(decel_m_s2)
        try:
            self.controller.set_motion_profile(speed, accel, decel)
        except gclib.GclibError as e:
            self.status_var.set(f"Apply speed settings failed: {e}")
            return False
        return True

    def _apply_pid_gains(self, kp, kd, ki, offset_v) -> bool:
        try:
            self.controller.set_pid_gains(kp, kd, ki)
            self.controller.set_offset(offset_v)
        except gclib.GclibError as e:
            self.status_var.set(f"Apply PID gains failed: {e}")
            return False
        return True

    def _apply_rest_pid_gains(self) -> bool:
        try:
            rest_kp = float(self.rest_kp_var.get())
            rest_kd = float(self.rest_kd_var.get())
            rest_ki = float(self.rest_ki_var.get())
            rest_offset_v = float(self.rest_offset_var.get())
        except ValueError:
            self.status_var.set("Rest KP/KD/KI/Offset must be numbers")
            return False
        return self._apply_pid_gains(rest_kp, rest_kd, rest_ki, rest_offset_v)

    def _apply_motion_pid_gains(self) -> bool:
        try:
            motion_kp = float(self.motion_kp_var.get())
            motion_kd = float(self.motion_kd_var.get())
            motion_ki = float(self.motion_ki_var.get())
            motion_offset_v = float(self.motion_offset_var.get())
            motion_fv = float(self.motion_fv_var.get())
        except ValueError:
            self.status_var.set("Motion KP/KD/KI/Offset/FV must be numbers")
            return False
        if not self._apply_pid_gains(motion_kp, motion_kd, motion_ki, motion_offset_v):
            return False
        try:
            self.controller.set_velocity_feedforward(motion_fv)
        except gclib.GclibError as e:
            self.status_var.set(f"Apply PID gains failed: {e}")
            return False
        return True

    def _apply_sample_period(self, microseconds) -> bool:
        try:
            self.controller.set_sample_period(microseconds)
        except gclib.GclibError as e:
            self.status_var.set(f"Apply sample period failed: {e}")
            return False
        return True

    def _apply_motion_sample_period(self) -> bool:
        try:
            motion_period_us = float(self.motion_sample_period_us_var.get())
        except ValueError:
            self.status_var.set("Motion sampling period must be a number (usec)")
            return False
        if not (TM_MIN_US <= motion_period_us <= TM_MAX_US):
            self.status_var.set(
                f"Motion sampling period must be {TM_MIN_US}-{TM_MAX_US}usec"
            )
            return False
        if not self._apply_sample_period(motion_period_us):
            return False
        self._capture_sample_period_s = motion_period_us / 1_000_000
        return True

    def _apply_rest_sample_period(self) -> bool:
        return self._apply_sample_period(DEFAULT_SAMPLE_PERIOD_US)

    def _apply_motion_settings(self) -> bool:
        if not self._apply_motion_pid_gains():
            return False
        return self._apply_motion_sample_period()

    def _apply_rest_settings(self) -> bool:
        if not self._apply_rest_pid_gains():
            return False
        return self._apply_rest_sample_period()

    def _on_reset_impact_range(self):
        self.impact_start_var.set("0.00")

    def _on_reset_contour_range(self):
        self.contour_start_var.set("0.00")
        self.contour_stop_var.set(f"{FULL_STROKE_MM:.2f}")

    def _on_reset_pvt_range(self):
        self.pvt_start_var.set("0.00")
        self.pvt_stop_var.set(f"{FULL_STROKE_MM:.2f}")

    def _on_reset_current_profile_range(self):
        self.current_profile_start_var.set("0.00")
        self.current_profile_stop_var.set(f"{FULL_STROKE_MM:.2f}")

    def _on_reset_motion_profile(self):
        self.speed_var.set(f"{DEFAULT_SPEED_M_S:.4f}")
        self.accel_var.set(f"{DEFAULT_ACCEL_M_S2:.4f}")
        self.decel_var.set(f"{DEFAULT_DECEL_M_S2:.4f}")

    def _on_reset_rest_pid_gains(self):
        self.rest_kp_var.set(str(DEFAULT_REST_KP))
        self.rest_kd_var.set(str(DEFAULT_REST_KD))
        self.rest_ki_var.set(str(DEFAULT_REST_KI))
        self.rest_offset_var.set(str(DEFAULT_REST_OFFSET_V))

    def _on_reset_motion_pid_gains(self):
        self.motion_kp_var.set(str(DEFAULT_MOTION_KP))
        self.motion_kd_var.set(str(DEFAULT_MOTION_KD))
        self.motion_ki_var.set(str(DEFAULT_MOTION_KI))
        self.motion_offset_var.set(str(DEFAULT_MOTION_OFFSET_V))
        self.motion_fv_var.set(str(DEFAULT_MOTION_FV))
        self.motion_sample_period_us_var.set(str(DEFAULT_SAMPLE_PERIOD_US))

    def _on_reset_recording_settings(self):
        self.pre_pause_ms_var.set(str(RECORD_PRE_PAUSE_MS))
        self.post_pause_ms_var.set(str(RECORD_POST_PAUSE_MS))

    def _poll(self):
        try:
            position = self.controller.read_position()
            lvdt_voltage = self.controller.read_lvdt_voltage()
            output_voltage = self.controller.read_output_voltage()
            servo_enabled = self.controller.read_servo_enabled()
        except gclib.GclibError as e:
            self.status_var.set(f"Read error: {e} — reconnect to resume")
            self._poll_after_id = None
            return

        self.position_counts_var.set(str(position))
        # Both calibrations side by side while more than one region is
        # configured, so the two maps can be watched diverging live; a
        # single value when they'd be identical anyway.
        linear_mm = Controller.counts_to_mm(position)
        if Controller.calibration_region_count() > 1:
            nonlinear_mm = Controller.counts_to_mm_piecewise(position)
            self.position_mm_var.set(f"{linear_mm:.2f} / {nonlinear_mm:.2f}  (L/N)")
        else:
            self.position_mm_var.set(f"{linear_mm:.2f}")
        self.lvdt_voltage_var.set(f"{lvdt_voltage:.3f}")
        self.output_current_var.set(f"{Controller.volts_to_current(output_voltage):.3f}")
        if servo_enabled:
            self.servo_status_var.set("Servo: ON")
            self.servo_status_label.config(fg="red")
        else:
            self.servo_status_var.set("Servo: OFF")
            self.servo_status_label.config(fg="black")
        self.go_button.config(state="normal" if servo_enabled else "disabled")
        self.slow_retract_button.config(state="normal" if servo_enabled else "disabled")
        self.slow_extend_button.config(state="normal" if servo_enabled else "disabled")
        self.arm_button.config(state="normal" if servo_enabled else "disabled")
        self.contour_arm_button.config(state="normal" if servo_enabled else "disabled")
        self.pvt_arm_button.config(state="normal" if servo_enabled else "disabled")
        self.current_profile_arm_button.config(state="normal" if servo_enabled else "disabled")
        self.sequence_panel.set_servo_enabled(servo_enabled)
        self.conditioning_controls.set_servo_enabled(servo_enabled)
        # Fire is only ever force-disabled here, never force-enabled --
        # only a successful Arm does that. This makes Disable the master
        # abort for an armed impact: Fire grays out the instant the servo
        # drops and stays grayed out (even across a bare re-Enable) until
        # a fresh Arm succeeds. Same for Contoured Motion's/PVT's/Current
        # Profile's Fire.
        # Simple Open Loop is deliberately absent from both the Arm-style
        # gating above and the Fire gating below: its Load/Fire pair is
        # used entirely from a DISABLED servo (the program enables itself
        # and disables again), so servo-state gating would leave both
        # buttons permanently grayed out. Load is connection-gated in
        # _on_connect/_on_disconnect instead, and its Fire is cleared by
        # an explicit Disable press (see _on_disable).
        if not servo_enabled:
            self.fire_button.config(state="disabled")
            self.contour_fire_button.config(state="disabled")
            self.pvt_fire_button.config(state="disabled")
            self.current_profile_fire_button.config(state="disabled")
        # Disable intentionally has no such gating -- it's the ABORT
        # equivalent and must always be clickable regardless of believed
        # servo state.
        self.enable_button.config(state="disabled" if servo_enabled else "normal")

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
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
