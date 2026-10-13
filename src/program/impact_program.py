"""Builds the DMC program text for an open-loop-voltage impact test.

Pure text generation -- no Tkinter, no gclib -- so this stays testable
independent of both the GUI and the controller connection.
"""

from controller.controller import (
    DEFAULT_SAMPLE_PERIOD_US,
    RECORD_ARRAY_NAME,
    RECORD_ARRAY_SIZE,
    RECORD_CMD_ARRAY_NAME,
    RECORD_ERR_ARRAY_NAME,
    RECORD_TRQ_ARRAY_NAME,
)


def build_impact_program(
    *,
    final_hold_counts,
    start_counts,
    slow_speed_counts,
    slow_accel_counts,
    slow_decel_counts,
    closed_loop_speed_counts,
    closed_loop_accel_counts,
    closed_loop_decel_counts,
    motion_sample_period_us,
    pre_impact_retract_voltage_v,
    pre_impact_retract_wait_cycles,
    pre_pause_ms,
    impact_accel_primary_voltage_v,
    impact_accel_primary_wait_cycles,
    impact_accel_secondary_voltage_v,
    impact_accel_secondary_wait_cycles,
    impact_decel_voltage_v,
    impact_decel_wait_cycles,
    hold_voltage_v,
    hold_wait_cycles,
    retract_accel_primary_voltage_v,
    retract_accel_primary_wait_cycles,
    retract_accel_secondary_voltage_v,
    retract_accel_secondary_wait_cycles,
    retract_decel_voltage_v,
    retract_decel_wait_cycles,
    end_with_closed_loop_move,
    motion_kp,
    motion_kd,
    motion_ki,
    post_pause_ms,
    rest_kp,
    rest_kd,
    rest_ki,
    rest_offset_v,
):
    """Pre-impact retract (open-loop voltage if pre_impact_retract_voltage_v
    is nonzero, otherwise a closed-loop move to start_counts) -> set
    Motion sample period -> set up recording -> [zero PID -> open-loop
    retract line, if that path was taken] -> start recording -> pre-move
    pause -> [zero PID, if the closed-loop retract path was taken] ->
    open-loop voltage drive (accel primary, then accel secondary) ->
    open-loop decel voltage -> open-loop hold voltage -> open-loop
    post-impact retract (accel primary, accel secondary, then decel
    voltage/wait) -> [if end_with_closed_loop_move: closed-loop move to
    final_hold_counts under Motion gains; otherwise: zero-distance sync
    move to resync the stale reference position] -> post-move pause ->
    stop recording -> switch to Rest gains (no new PA -- target stays
    wherever the prior step left it) -> end.

    All wait times except pre_pause_ms/post_pause_ms are already
    converted to servo-cycle counts by the caller (Controller.ms_to_cycles)
    and sent as "WT n,1" for 1-cycle resolution, instead of the default
    2ms-resolution "WT n". Pre/Post-Move Pause stay plain millisecond
    waits -- shared with Contoured Motion, intentionally untouched.

    The closed-loop pre-impact retract runs before Motion sample
    period/recording are set up -- it relies on Rest gains already being
    active (set by Arm's _apply_rest_settings()) to actually close the
    loop, and isn't itself captured in the recording. PID stays at those
    Rest gains -- holding position -- through the pre-move pause, and is
    only zeroed right before the open-loop drive begins. This mirrors how
    the redundant retract worked before the open-loop retract was added.

    The post-impact retract is unconditional (always both phases) -- PID
    is already 0 by that point in the sequence either way, so there's no
    open/closed-loop choice to make there.

    When end_with_closed_loop_move is False, PID is still 0 at the sync
    move -- it's a zero-distance PA (to _TPA, wherever the axis actually
    is), so it doesn't need nonzero gains, and it exists solely to prevent
    a stale-_RPA snap once Rest gains go nonzero right after.

    No auto-disable at the end -- the servo is left enabled under Rest
    gains, actively holding in place, since disabling would defeat the
    point of switching to a holding gain set. Disable is the primary
    abort path for this program, by design.
    """
    lines = ["#MAIN"]
    if not pre_impact_retract_voltage_v:
        lines += [
            f"SP {slow_speed_counts}",
            f"AC {slow_accel_counts}",
            f"DC {slow_decel_counts}",
            f"PA {start_counts}",
            "BGA",
            "AMA",
        ]
    lines += [
        # Sample period for capture.
        f"TM {motion_sample_period_us}",
        # Start recording -- 4 simultaneous channels, same pattern as
        # Controller.start_recording().
        f"DM {RECORD_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"DM {RECORD_CMD_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"DM {RECORD_ERR_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"DM {RECORD_TRQ_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"RA {RECORD_ARRAY_NAME}[],{RECORD_CMD_ARRAY_NAME}[],"
        f"{RECORD_ERR_ARRAY_NAME}[],{RECORD_TRQ_ARRAY_NAME}[]",
        "RD _TPA,_RPA,_TEA,_TTA",
    ]
    if pre_impact_retract_voltage_v:
        lines += [
            # Zero PID before the open-loop retract.
            "KD 0",
            "KP 0",
            "KI 0",
            f"OF {pre_impact_retract_voltage_v}",
            f"WT {pre_impact_retract_wait_cycles},1",
        ]
    lines += [
        # Start recording.
        "RC 1",
        f"WT {pre_pause_ms}",
    ]
    if not pre_impact_retract_voltage_v:
        # Closed-loop retract held position (Rest gains) through the
        # pre-pause -- only zero PID now, right before the open-loop drive.
        lines += ["KD 0", "KP 0", "KI 0"]
    lines += [
        # Open-loop voltage impact (accel primary, then accel secondary).
        f"OF {impact_accel_primary_voltage_v}",
        f"WT {impact_accel_primary_wait_cycles},1",
        f"OF {impact_accel_secondary_voltage_v}",
        f"WT {impact_accel_secondary_wait_cycles},1",
        # Open-loop decel voltage.
        f"OF {impact_decel_voltage_v}",
        f"WT {impact_decel_wait_cycles},1",
        # Open-loop hold voltage.
        f"OF {hold_voltage_v}",
        f"WT {hold_wait_cycles},1",
        # Post-impact retract
        f"OF {retract_accel_primary_voltage_v}",
        f"WT {retract_accel_primary_wait_cycles},1",
        f"OF {retract_accel_secondary_voltage_v}",
        f"WT {retract_accel_secondary_wait_cycles},1",
        f"OF {retract_decel_voltage_v}",
        f"WT {retract_decel_wait_cycles},1",
        # Return to closed loop -- OF 0 first so no open-loop bias is
        # left riding on the command line once gains go nonzero below.
        "OF 0",
    ]
    if end_with_closed_loop_move:
        lines += [
            # Closed-loop move to final_hold_counts under Motion gains --
            # PA before the gains go nonzero, so the servo reactivates
            # already pointed there instead of a stale prior target.
            f"PA {final_hold_counts}",
            f"KD {motion_kd}",
            f"KP {motion_kp}",
            f"KI {motion_ki}",
            f"SP {closed_loop_speed_counts}",
            f"AC {closed_loop_accel_counts}",
            f"DC {closed_loop_decel_counts}",
            "BGA",
            "AMA",
        ]
    else:
        # No closed-loop move -- PID is still 0, so this is a
        # zero-distance sync to wherever the axis actually is, purely to
        # prevent a stale-_RPA snap once Rest gains go nonzero below.
        lines += ["p = _TPA", "PA p", "BGA", "AMA"]
    lines += [
        f"WT {post_pause_ms}",
        # Stop recording.
        "RC 0",
        # Restore Rest gains -- target stays wherever the prior step left
        # it (no new PA), so the axis keeps actively holding there.
        f"KD {rest_kd}",
        f"KP {rest_kp}",
        f"KI {rest_ki}",
        f"OF {rest_offset_v}",
        f"TM {DEFAULT_SAMPLE_PERIOD_US}",
        "EN",
    ]
    return "\n".join(lines) + "\n"
