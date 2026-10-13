"""Builds the DMC program for a standalone LVDT conditioning pulse --
the "set the sensor to mode 2 deliberately" step from the handoff notes
(section 4): with the striker resting against a hard stop and the
servo enabled under Rest gains, zero the position loop, drive a known
current INTO that stop for a short time (10A / 20ms is the validated
mode-2 recipe; 3x 2A / 20ms restores mode 1), release, resync the
position reference and put the Rest gains back -- leaving the servo
ENABLED and holding, which is the state the LVDT must then be
calibrated/read in (the amp's PWM ripple is part of the mode-1/mode-2
measurement; a disabled amp reads the unrelated "mode 0" span).

The pulse direction must point into the stop the striker is already
against, otherwise 10A simply launches it. Callers pick the sign from
the live position (nearest end of travel) -- see
Controller.current_to_volts and the sign convention in
impact_program.py (positive OF = toward retract).

Pure text generation -- no Tkinter, no gclib.
"""

from program.sequence_program import conditioning_lines

# Validated recipe (handoff notes, section 4).
MODE2_PULSE_CURRENT_A = 10.0
MODE2_PULSE_MS = 20.0
MODE1_PULSE_CURRENT_A = 2.0
MODE1_PULSE_MS = 20.0
MODE1_PULSE_REPEATS = 3
CONDITION_SETTLE_MS = 50.0


def build_conditioning_program(
    *, pulse_v, pulse_ms, repeats=1, settle_ms=CONDITION_SETTLE_MS,
    rest_kp, rest_kd, rest_ki, rest_offset_v,
):
    """`pulse_v`: signed OF volts (positive = toward retract, into the
    retract stop). `repeats`: how many pulse/settle cycles (3 for the
    mode-1 recipe). Ends with Rest gains/offset active, servo still
    enabled, no recording."""
    if pulse_ms <= 0:
        raise ValueError("Pulse duration must be positive")
    if repeats < 1:
        raise ValueError("Repeats must be at least 1")
    lines = ["#COND"]
    for _ in range(repeats):
        lines += conditioning_lines(
            pulse_v=pulse_v, pulse_ms=pulse_ms, settle_ms=settle_ms,
            rest_kp=rest_kp, rest_kd=rest_kd, rest_ki=rest_ki, rest_offset_v=rest_offset_v,
        )
    lines.append("EN")
    return "\n".join(lines) + "\n"
