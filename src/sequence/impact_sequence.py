"""Turns an impact profile (sequence/profile.py) into everything needed
to run it, and drives the controller-side Arm/Fire/upload steps -- the
one code path BOTH GUIs use, so what the testing GUI tunes is exactly
what the user GUI runs.

    compiled = compile_sequence(profile, dwell_ms=...)   # pure
    prepare_controller(controller, compiled)             # Arm, part 1
    <caller waits for the retract to Start to finish>    # Arm, part 2
    controller.run_program()                             # Fire
    <caller waits for the program to finish>
    recording = fetch_sequence_recording(controller, compiled)

compile_sequence dispatches on profile["motion"]["mode"] through
COMPILERS, so a different motion program (if the testing GUI grows one)
plugs in as a new compiler returning a CompiledSequence, with no GUI
changes. The GUI layers only ever see CompiledSequence.

The conversions here deliberately use the PROFILE's amp full-scale
current (not whatever the app's Amp Full-Scale field currently says)
for every amps->volts step, because the profile's feedforward/hold/
pulse values were developed against that setting. Position conversions
use the live LVDT calibration (Controller class state), which is the
rig's, not the profile's.
"""

import copy
from typing import NamedTuple

import gclib

from controller.controller import (
    Controller,
    DEFAULT_SAMPLE_PERIOD_US,
    OF_MAX_V,
    OF_MIN_V,
    SLOW_ACCEL_M_S2,
    SLOW_DECEL_M_S2,
    SLOW_SPEED_M_S,
    TORQUE_VOLTS_FULL_SCALE,
)
from program.current_profile_program import (
    AT_SAMPLES,
    FF_ARRAY_NAME,
    TM_US_ACTUAL,
)
from program.current_profile_trajectory import (
    compute_current_profile_curve,
    compute_feedforward_v,
    sample_position_mm,
)
from program.sequence_program import RecordLayout, build_sequence_program
from recording.sequence_recording import SequenceRecording, assemble_sequence_recording
from sequence.profile import merge_defaults, profile_impact_position_mm, validate_profile


class CompiledSequence(NamedTuple):
    profile: dict  # the profile as compiled (defaults filled, dwell applied)
    program_text: str
    layout: RecordLayout
    ff_values: list  # what gets downloaded as FF_ARRAY_NAME
    ff_plot_v: list  # feedforward resampled on the fast recording grid
    increments: list
    start_mm: float
    end_mm: float
    impact_position_mm: float
    peak_velocity_m_s: float
    dwell_ms: float
    info_lines: list  # (label, value) pairs for the capture figure


def amps_to_volts(amps: float, full_scale_a: float) -> float:
    """OF volts for `amps` at an amp configured to `full_scale_a` per
    10V (CME Analog Input Scale)."""
    return amps * TORQUE_VOLTS_FULL_SCALE / full_scale_a


def _checked_volts(amps, full_scale_a, name):
    volts = amps_to_volts(amps, full_scale_a)
    if not (OF_MIN_V <= volts <= OF_MAX_V):
        max_a = OF_MAX_V * full_scale_a / TORQUE_VOLTS_FULL_SCALE
        raise ValueError(
            f"{name} {amps}A needs {volts:.2f}V, outside the OF range -- at "
            f"{full_scale_a}A full scale the limit is +/-{max_a:.2f}A"
        )
    # 6 decimals is far below OF's own ~0.3mV resolution and keeps the
    # program text short (80-char line limit).
    return round(volts, 6) + 0.0


def compile_current_profile_sequence(profile: dict, dwell_ms=None) -> CompiledSequence:
    """Builds the Current-Profile-based sequence program for `profile`.
    `dwell_ms` overrides the profile's dwell (the operator-selected
    value in the user GUI). Raises ValueError for anything that can't
    be run."""
    p = merge_defaults(profile)
    errors = validate_profile(p)
    if errors:
        raise ValueError("; ".join(errors))
    m, s = p["motion"], p["sequence"]
    if dwell_ms is not None:
        s["dwell"]["duration_ms"] = float(dwell_ms)
    dwell_ms = float(s["dwell"]["duration_ms"])
    full_scale_a = float(p["amp"]["full_scale_current_a"])

    start_mm = float(m["start_mm"])
    end_mm = float(m["stop_mm"])
    impact_mm = profile_impact_position_mm(p)
    peak_mm = float(m["peak_velocity_position_mm"])
    velocity = float(p["impact_velocity_m_s"])
    curve = compute_current_profile_curve(start_mm, peak_mm, impact_mm, end_mm, velocity)

    cd_period_s = 2 * TM_US_ACTUAL / 1_000_000
    ff_period_s = AT_SAMPLES * TM_US_ACTUAL / 1_000_000
    positions_counts = [Controller.mm_to_counts(start_mm)] + [
        Controller.mm_to_counts(x) for x in sample_position_mm(curve, cd_period_s)
    ]
    increments = [positions_counts[i] - positions_counts[i - 1] for i in range(1, len(positions_counts))]
    stretch = m["ff_stretch"]
    stretch_args = (
        float(stretch["accel_rise"]), float(stretch["accel_fall"]),
        float(stretch["decel_rise"]), float(stretch["decel_fall"]),
    )
    mass, kf_a, kf_d = float(m["mass_kg"]), float(m["kf_accel"]), float(m["kf_decel"])
    ff_values = compute_feedforward_v(curve, ff_period_s, mass, kf_a, kf_d, *stretch_args)
    pre_pause_ms = int(round(float(m["pre_pause_ms"])))
    post_pause_ms = int(round(float(m["post_pause_ms"])))
    ff_plot_v = compute_feedforward_v(
        curve, TM_US_ACTUAL / 1_000_000, mass, kf_a, kf_d, *stretch_args,
    )
    ff_plot_v = [0.0] * round(pre_pause_ms * 1000 / TM_US_ACTUAL) + ff_plot_v

    cond = s["condition"]
    conditioning = bool(cond["enabled"]) and float(cond["duration_ms"]) > 0
    # Positive OF = toward retract: the conditioning pulse pushes INTO the
    # retract stop (profile current entered as a positive magnitude).
    condition_v = _checked_volts(float(cond["current_a"]), full_scale_a, "Condition pulse") if conditioning else 0.0
    dwell = s["dwell"]
    # Hold current is entered as "pushing into the extend stop" -- negative OF.
    hold_v = -_checked_volts(float(dwell["hold_current_a"]), full_scale_a, "Hold current")
    retract = s["retract"]
    # Kick is entered as "toward retract" -- positive OF.
    kick_v = _checked_volts(float(retract["kick_current_a"]), full_scale_a, "Retract kick") if float(retract["kick_ms"]) > 0 else 0.0
    rpid = retract["pid"]
    mpid, rest = m["motion_pid"], m["rest_pid"]

    program_text, layout = build_sequence_program(
        start_counts=Controller.mm_to_counts(start_mm),
        slow_speed_counts=Controller.meters_to_counts(SLOW_SPEED_M_S),
        slow_accel_counts=Controller.meters_to_counts(SLOW_ACCEL_M_S2),
        slow_decel_counts=Controller.meters_to_counts(SLOW_DECEL_M_S2),
        condition_counts=Controller.mm_to_counts(float(cond["position_mm"])),
        condition_pulse_v=condition_v if conditioning else 0.0,
        condition_pulse_ms=float(cond["duration_ms"]) if conditioning else 0.0,
        condition_settle_ms=float(cond["settle_ms"]),
        pre_pause_ms=pre_pause_ms,
        motion_kp=float(mpid["kp"]), motion_kd=float(mpid["kd"]), motion_ki=float(mpid["ki"]),
        motion_offset_v=float(mpid["offset_v"]),
        increments=increments,
        ff_values=ff_values,
        ff_advance_us=float(m["ff_advance_us"]),
        dwell_ms=dwell_ms,
        hold_v=hold_v,
        hold_servo=bool(dwell["hold_servo"]),
        fast_tail_ms=float(dwell["fast_tail_ms"]),
        retract_kick_v=kick_v,
        retract_kick_ms=float(retract["kick_ms"]),
        retract_kp=float(rpid["kp"]), retract_kd=float(rpid["kd"]), retract_ki=float(rpid["ki"]),
        retract_offset_v=float(rpid["offset_v"]),
        retract_speed_counts=Controller.meters_to_counts(float(retract["speed_m_s"])),
        retract_accel_counts=Controller.meters_to_counts(float(retract["accel_m_s2"])),
        retract_decel_counts=Controller.meters_to_counts(float(retract["decel_m_s2"])),
        retract_target_counts=Controller.mm_to_counts(float(retract["target_mm"])),
        post_pause_ms=post_pause_ms,
        rest_kp=float(rest["kp"]), rest_kd=float(rest["kd"]), rest_ki=float(rest["ki"]),
        rest_offset_v=float(rest["offset_v"]),
        fast_circular=bool(s["recording"]["circular"]),
        disable_at_end=bool(s["disable_at_end"]),
    )

    slow_ms = layout.slow_interval_samples * TM_US_ACTUAL / 1000
    info_lines = [
        ("Profile", f"{p.get('name') or 'unnamed'} ({p.get('tip') or '?'})"),
        ("Impact Velocity", f"{velocity}m/s"),
        ("Impact Position/Depth", f"{impact_mm:.2f}mm / {float(p['impact_depth_mm']):.2f}mm"),
        ("Start/Stop", f"{start_mm:.2f}/{end_mm:.2f}mm"),
        ("Peak Velocity Position", f"{peak_mm}mm"),
        ("Peak Velocity (solved)", f"{curve.phase2.v0:.4f}m/s"),
        ("Contour Points / FF Samples", f"{len(increments)} / {len(ff_values)}"),
        ("Mass / Kf accel / Kf decel", f"{mass}kg / {kf_a} / {kf_d}"),
        ("FF Stretch (ar/af/dr/df)", "/".join(f"{x}" for x in stretch_args)),
        ("FF Advance / Pre / Post", f"{float(m['ff_advance_us'])}us / {pre_pause_ms} / {post_pause_ms}ms"),
        ("Condition", (
            f"{float(cond['current_a'])}A x {float(cond['duration_ms'])}ms @ {float(cond['position_mm'])}mm"
            if conditioning else "off"
        )),
        ("Dwell / Hold", (
            f"{dwell_ms:.1f}ms @ {float(dwell['hold_current_a'])}A "
            f"({'servo on' if dwell['hold_servo'] else 'open loop'}), fast tail {float(dwell['fast_tail_ms'])}ms"
        )),
        ("Retract", (
            f"{float(retract['speed_m_s'])}m/s {float(retract['accel_m_s2'])}/{float(retract['decel_m_s2'])}m/s2 "
            f"to {float(retract['target_mm'])}mm"
            + (f", kick {float(retract['kick_current_a'])}A x {float(retract['kick_ms'])}ms" if kick_v else "")
        )),
        ("Recording", (
            f"fast {layout.fast_samples}x4 @ {TM_US_ACTUAL}us"
            f"{' (rolling)' if layout.fast_circular else ''}, "
            f"slow {layout.slow_samples}x3 @ {slow_ms:.2f}ms, "
            f"{layout.total_elements}/3000 elements"
        )),
        ("Motion PID", f"KP={mpid['kp']} KD={mpid['kd']} KI={mpid['ki']} Offset={mpid['offset_v']}V"),
        ("Retract PID", f"KP={rpid['kp']} KD={rpid['kd']} KI={rpid['ki']} Offset={rpid['offset_v']}V"),
        ("Rest PID", f"KP={rest['kp']} KD={rest['kd']} KI={rest['ki']} Offset={rest['offset_v']}V"),
        ("Amp Full-Scale Current", f"{full_scale_a}A"),
    ]
    return CompiledSequence(
        profile=p, program_text=program_text, layout=layout, ff_values=ff_values,
        ff_plot_v=ff_plot_v, increments=increments, start_mm=start_mm, end_mm=end_mm,
        impact_position_mm=impact_mm, peak_velocity_m_s=curve.phase2.v0, dwell_ms=dwell_ms,
        info_lines=info_lines,
    )


COMPILERS = {
    "current_profile_sequence": compile_current_profile_sequence,
}


def compile_sequence(profile: dict, dwell_ms=None) -> CompiledSequence:
    """Compiles `profile` with the compiler its motion mode names."""
    mode = (profile.get("motion") or {}).get("mode", "current_profile_sequence")
    try:
        compiler = COMPILERS[mode]
    except KeyError:
        raise ValueError(f"Unknown motion mode {mode!r}; known: {sorted(COMPILERS)}")
    return compiler(profile, dwell_ms=dwell_ms)


def info_text(compiled: CompiledSequence, title="Impact Sequence") -> str:
    return title + "\n" + "\n".join(f"{label}: {value}" for label, value in compiled.info_lines)


def prepare_controller(controller: Controller, compiled: CompiledSequence) -> None:
    """Controller-side Arm: servo off -> program download -> array table
    reset + all recording arrays sized (Controller.size_arrays) ->
    feedforward array download (after the program, per the QD caveat)
    -> Rest gains/offset + Rest sample period -> servo on -> slow move
    to Start. Returns once the move has been commanded; the caller
    waits for it (recording.recording_sequence.wait_for_motion_complete).

    Raises gclib.GclibError on any controller failure (the servo is
    left disabled in that case where possible)."""
    rest = compiled.profile["motion"]["rest_pid"]
    controller.disable()
    controller.download_program(compiled.program_text)
    controller.size_arrays(compiled.layout.array_sizes)
    controller.download_array(FF_ARRAY_NAME, compiled.ff_values)
    controller.set_pid_gains(float(rest["kp"]), float(rest["kd"]), float(rest["ki"]))
    controller.set_offset(float(rest["offset_v"]))
    controller.set_sample_period(DEFAULT_SAMPLE_PERIOD_US)
    controller.enable()
    controller.set_motion_profile(
        Controller.meters_to_counts(SLOW_SPEED_M_S),
        Controller.meters_to_counts(SLOW_ACCEL_M_S2),
        Controller.meters_to_counts(SLOW_DECEL_M_S2),
    )
    controller.move_absolute(Controller.mm_to_counts(compiled.start_mm))


def fetch_sequence_recording(controller: Controller, compiled: CompiledSequence) -> SequenceRecording:
    """Uploads and assembles both recording segments after the program
    has ended (run_program + wait_for_program_complete)."""
    return assemble_sequence_recording(controller, compiled.layout)


def profile_copy(profile: dict) -> dict:
    return copy.deepcopy(profile)
