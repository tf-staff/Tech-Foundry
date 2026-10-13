"""Builds the DMC program text for a closed-loop contoured motion move --
constant-acceleration ramp up to an impact velocity at an impact position,
then constant-acceleration ramp down to 0 velocity at a max position, using
the Galil DMC-30000's Contour Mode (CM/CD/DT).

Pure text/data generation -- no Tkinter, no gclib -- so this stays testable
independent of both the GUI and the controller connection.
"""

from controller.controller import (
    Controller,
    DEFAULT_SAMPLE_PERIOD_US,
    POST_TEST_DISABLE_DELAY_MS,
    RECORD_ARRAY_NAME,
    RECORD_ARRAY_SIZE,
    RECORD_CMD_ARRAY_NAME,
    RECORD_ERR_ARRAY_NAME,
    RECORD_TRQ_ARRAY_NAME,
)

# Galil Contour Mode (DMC-30000 manual, ch.6, "Contour Mode"): DT n sets the
# segment time interval to 2**n sample periods (TM). n=1 -> 2x TM, matching
# the requested "points every sample period x2".
CONTOUR_DT_EXPONENT = 1

# The contour buffer holds up to 511 segments (_CM reports free space, 511
# max). Capped comfortably under that so Arm can validate client-side
# instead of risking an oversized download.
CONTOUR_MAX_SEGMENTS = 500

# CD position increments are limited to +/-32,000 counts per segment
# (DMC-30000 manual, Contour Mode command summary).
CONTOUR_MAX_SEGMENT_COUNTS = 32000


def compute_contour_increments(
    start_mm, impact_mm, end_mm, impact_velocity_m_s, sample_period_s,
):
    """Two-phase constant-acceleration position trajectory: 0 velocity at
    start_mm, ramping to impact_velocity_m_s exactly at impact_mm, then
    ramping back to 0 velocity exactly at end_mm. Sampled every
    2*sample_period_s (DT=2**CONTOUR_DT_EXPONENT), returned as a list of
    integer position increments (counts), one per CD line -- the first
    sample needs no increment, since it's wherever the axis already is
    after the redundant retract to start_mm.

    Raises ValueError if impact_velocity_m_s isn't positive, impact_mm
    isn't strictly between start_mm and end_mm, the computed contour needs
    more than CONTOUR_MAX_SEGMENTS points, or any single increment exceeds
    +/-CONTOUR_MAX_SEGMENT_COUNTS.
    """
    if impact_velocity_m_s <= 0:
        raise ValueError("Impact velocity must be positive")
    if not (start_mm < impact_mm < end_mm):
        raise ValueError("Impact position must be strictly between start and end")

    dt = 2 * sample_period_s

    d1_m = (impact_mm - start_mm) / 1000
    accel1 = impact_velocity_m_s**2 / (2 * d1_m)
    t1 = impact_velocity_m_s / accel1
    n1 = max(1, round(t1 / dt))

    d2_m = (end_mm - impact_mm) / 1000
    accel2 = impact_velocity_m_s**2 / (2 * d2_m)
    t2 = impact_velocity_m_s / accel2
    n2 = max(1, round(t2 / dt))

    positions_mm = []
    for i in range(n1 + 1):
        t = i * dt if i < n1 else t1
        positions_mm.append(start_mm + 0.5 * accel1 * t**2 * 1000)
    for i in range(1, n2 + 1):
        t = i * dt if i < n2 else t2
        positions_mm.append(
            impact_mm + (impact_velocity_m_s * t - 0.5 * accel2 * t**2) * 1000
        )

    positions_counts = [Controller.mm_to_counts(p) for p in positions_mm]
    increments = [
        positions_counts[i] - positions_counts[i - 1]
        for i in range(1, len(positions_counts))
    ]

    if len(increments) > CONTOUR_MAX_SEGMENTS:
        raise ValueError(
            f"Contour requires {len(increments)} points, exceeds the "
            f"{CONTOUR_MAX_SEGMENTS}-point buffer limit -- increase impact "
            f"velocity or reduce the Start/Stop span"
        )
    for inc in increments:
        if abs(inc) > CONTOUR_MAX_SEGMENT_COUNTS:
            raise ValueError(
                f"Contour segment increment {inc} counts exceeds "
                f"+/-{CONTOUR_MAX_SEGMENT_COUNTS} -- reduce impact velocity"
            )

    return increments


def build_contour_program(
    *,
    start_counts,
    slow_speed_counts,
    slow_accel_counts,
    slow_decel_counts,
    motion_sample_period_us,
    pre_pause_ms,
    motion_kp,
    motion_kd,
    motion_ki,
    motion_offset_v,
    motion_fv,
    increments,
    post_pause_ms,
    rest_kp,
    rest_kd,
    rest_ki,
    rest_offset_v,
):
    """Redundant slow retract to start_counts -> set Motion sample period ->
    start recording -> pre-move pause -> apply Motion PID gains/offset/FV ->
    Contour Mode move through `increments` -> post-move pause -> stop
    recording -> reset PID/offset/sample period to Rest values -> settle ->
    disable -> end.

    Motion PID gains are applied (not left at Rest) before CMA -- Contour
    Mode still runs the normal closed-loop position servo, CD only changes
    how the position command is generated.
    """
    lines = [
        "#MAIN",
        # Redundant slow retract.
        f"SP {slow_speed_counts}",
        f"AC {slow_accel_counts}",
        f"DC {slow_decel_counts}",
        f"PA {start_counts}",
        "BGA",
        "AMA",
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
        "RC 1",
        f"WT {pre_pause_ms}",
        f"KD {motion_kd}",
        f"KP {motion_kp}",
        f"KI {motion_ki}",
        f"OF {motion_offset_v}",
        f"FV {motion_fv}",
        "CMA",
        f"DT {CONTOUR_DT_EXPONENT}",
    ]
    lines += [f"CD {inc}" for inc in increments]
    lines += [
        "CD 0=0",
        "#WAIT",
        "JP#WAIT,_CM<>511",
        f"WT {post_pause_ms}",
        # Stop recording.
        "RC 0",
        # Reset to rest state.
        f"KD {rest_kd}",
        f"KP {rest_kp}",
        f"KI {rest_ki}",
        f"OF {rest_offset_v}",
        f"TM {DEFAULT_SAMPLE_PERIOD_US}",
        # Settle, then auto-disable rather than leave the servo enabled
        # indefinitely after the test completes.
        f"WT {POST_TEST_DISABLE_DELAY_MS}",
        "OF 0",
        "MOA",
        "EN",
    ]
    return "\n".join(lines) + "\n"
