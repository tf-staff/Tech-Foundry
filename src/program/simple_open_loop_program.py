"""Builds the DMC program text for Simple Open Loop mode -- a single
open-loop current pulse of a given magnitude and duration, recorded, with
no position control and no motion commanded before or after.

Built as instrumentation for the LVDT magnetization investigation (see
the plan file): park the striker against a hard stop by hand, apply a
known current with position physically pinned, and watch what the
position reading does. Open Loop Motion mode can't serve that purpose --
its Arm enables the servo and slow-retracts to a start position first,
which moves the striker off the stop and passes a conditioning current
through the coil, destroying the magnetic state under test.

Two deliberate departures from impact_program.py, both required by that
experiment:
  * This program issues its own "SHA" -- the GUI's Load step only
    downloads, so nothing is enabled until Fire actually runs.
  * It ends DISABLED ("MOA") at zero current, rather than holding under
    Rest gains. Leaving the servo enabled and holding at the end would
    apply exactly the steady conditioning current that resets the
    magnetic state being measured.

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


def build_simple_open_loop_program(
    *,
    sample_period_us,
    pre_pause_ms,
    current_volts,
    hold_wait_cycles,
    post_pause_ms,
    rest_kp,
    rest_kd,
    rest_ki,
):
    """Set sample period -> set up recording -> zero PID/offset -> enable
    -> start recording -> pre-move pause -> apply `current_volts` for
    `hold_wait_cycles` servo cycles -> back to 0 -> post-move pause ->
    stop recording -> disable -> restore Rest gains -> end.

    `current_volts` is the OF command voltage, already converted from the
    user's requested amps by the caller (Controller.current_to_volts) and
    already range-checked against OF_MIN_V/OF_MAX_V. Negative drives
    toward extension, matching impact_program.py's own convention.

    `hold_wait_cycles` is already converted from milliseconds by the
    caller (Controller.ms_to_cycles) and sent as "WT n,1" for 1-cycle
    resolution, same as impact_program.py's own wait fields.

    PID gains and OF are zeroed BEFORE "SHA" on purpose: enabling with a
    live gain set would both fight the hand-placed striker position and
    pass a conditioning current through the coil before the measurement
    starts. Zeroed first, the enable itself is silent -- no current, no
    motion.

    Rest gains are restored after "MOA" (motor already off, so this only
    leaves the controller in a sane state for a later GUI Enable, which
    applies its own Rest settings anyway). OF is deliberately left at 0
    rather than rest_offset_v, so nothing biases the command line while
    the axis sits disabled being read by hand.
    """
    return "\n".join([
        "#MAIN",
        # Sample period for capture.
        f"TM {sample_period_us}",
        # Start recording -- 4 simultaneous channels, same pattern as
        # Controller.start_recording().
        f"DM {RECORD_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"DM {RECORD_CMD_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"DM {RECORD_ERR_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"DM {RECORD_TRQ_ARRAY_NAME}[{RECORD_ARRAY_SIZE}]",
        f"RA {RECORD_ARRAY_NAME}[],{RECORD_CMD_ARRAY_NAME}[],"
        f"{RECORD_ERR_ARRAY_NAME}[],{RECORD_TRQ_ARRAY_NAME}[]",
        "RD _TPA,_RPA,_TEA,_TTA",
        # Zero everything before enabling -- see the docstring.
        "KD 0",
        "KP 0",
        "KI 0",
        "OF 0",
        "SHA",
        "RC 1",
        f"WT {pre_pause_ms}",
        # The measurement itself.
        f"OF {current_volts}",
        f"WT {hold_wait_cycles},1",
        "OF 0",
        f"WT {post_pause_ms}",
        "RC 0",
        # Leave the axis disabled at zero current, free to be pushed by
        # hand for the post-pulse reading.
        "MOA",
        f"KD {rest_kd}",
        f"KP {rest_kp}",
        f"KI {rest_ki}",
        f"TM {DEFAULT_SAMPLE_PERIOD_US}",
        "EN",
    ]) + "\n"
