"""Builds the DMC program text for a closed-loop PVT (Position-Velocity-
Time) motion move -- same two-phase constant-acceleration trajectory as
Contour Mode (ramp up to an impact velocity at an impact position, then
ramp down to 0 velocity at a max position), using the Galil DMC-30000's
PVT mode (PV/BT) instead of Contour Mode (CD/DT).

Pure text/data generation -- no Tkinter, no gclib -- so this stays testable
independent of both the GUI and the controller connection.
"""

from controller.controller import (
    Controller,
    DEFAULT_ACCEL_COUNTS_PER_S2,
    DEFAULT_DECEL_COUNTS_PER_S2,
    DEFAULT_SAMPLE_PERIOD_US,
    POST_TEST_DISABLE_DELAY_MS,
    RECORD_ARRAY_NAME,
    RECORD_ARRAY_SIZE,
    RECORD_CMD_ARRAY_NAME,
    RECORD_ERR_ARRAY_NAME,
    RECORD_TRQ_ARRAY_NAME,
)

# The PVT buffer holds "127" segments per the DMC-30000 manual's PVT Mode
# chapter (128 per the firmware command reference's _PV table) -- 126
# stays safely under either reading, minus 1 reserved for the PVA=0,0,0
# terminator segment.
PVT_MAX_SEGMENTS = 126

# PV command's "t" (sample count) argument range: 0 to 2,048 (firmware
# command reference), and must be even -- position/velocity/acceleration
# are "calculated every other sample" during a PVT segment.
PVT_MAX_SEGMENT_SAMPLES = 2048


def compute_pvt_segments(
    start_mm, impact_mm, end_mm, impact_velocity_m_s, sample_period_s,
    max_segments=PVT_MAX_SEGMENTS,
):
    """Same two-phase constant-acceleration trajectory as
    compute_contour_increments (contour_program.py): 0 velocity at
    start_mm, ramping to ~impact_velocity_m_s at ~impact_mm, then
    ramping back to ~0 velocity at ~end_mm. Segment duration (in TM
    samples) is auto-fit to the smallest even value that still keeps the
    total segment count within max_segments -- finer segments wherever
    the buffer allows it.

    Every segment in a phase shares the exact same duration, and
    position/velocity are evaluated at consistent multiples of that
    duration under one constant acceleration per phase -- deliberately
    NOT bit-exact at the impact/end waypoints (each lands within half a
    segment's duration of the ideal target, same kind of rounding
    Contour Mode already accepts for its own segment count). This is a
    correctness requirement, not just simplicity: unlike Contour Mode's
    CD (position-only, linearly interpolated -- tolerates a slightly
    "off" last segment gracefully), PVT segments declare both position
    AND velocity, interpolated with a cubic. Giving a segment a duration
    that doesn't match its position/velocity delta forces the cubic to
    overshoot/reverse mid-segment to still hit both targets exactly at
    the declared time -- bench-confirmed to produce wild, non-monotonic
    velocity oscillation. Keeping every segment internally consistent
    (duration matches its own constant-acceleration delta) avoids that
    entirely.

    Returns a list of (position_delta_counts, velocity_counts_per_s,
    duration_samples) tuples, one per PVA=p,v,t segment.

    Raises ValueError if impact_velocity_m_s isn't positive, impact_mm
    isn't strictly between start_mm and end_mm, or the move needs more
    than max_segments even at the coarsest PVT_MAX_SEGMENT_SAMPLES-sample
    segment length.
    """
    if impact_velocity_m_s <= 0:
        raise ValueError("Impact velocity must be positive")
    if not (start_mm < impact_mm < end_mm):
        raise ValueError("Impact position must be strictly between start and end")

    d1_m = (impact_mm - start_mm) / 1000
    accel1 = impact_velocity_m_s**2 / (2 * d1_m)
    t1 = impact_velocity_m_s / accel1

    d2_m = (end_mm - impact_mm) / 1000
    accel2 = impact_velocity_m_s**2 / (2 * d2_m)
    t2 = impact_velocity_m_s / accel2

    t_samples = 2
    while True:
        dt = t_samples * sample_period_s
        n1 = max(1, round(t1 / dt))
        n2 = max(1, round(t2 / dt))
        if n1 + n2 <= max_segments:
            break
        t_samples += 2
        if t_samples > PVT_MAX_SEGMENT_SAMPLES:
            raise ValueError(
                f"PVT move needs more than {max_segments} segments even at the "
                f"coarsest {PVT_MAX_SEGMENT_SAMPLES}-sample segment length -- "
                f"reduce impact velocity or the Start/Stop span"
            )

    # n1/n2 round independently, so they don't always preserve t1:t2 --
    # worst when one phase rounds to only a few segments. Re-derive the
    # boundary velocity that's exactly achievable with n1/n2 segments (each
    # dt long), so both phases are continuous with each other and with 0
    # velocity at both ends: distance per phase is 0.5*v*T, so a single
    # v = 2*(d1+d2)/(T1+T2) covers the full span exactly. Sacrifices some
    # precision on the impact position to kill the velocity/position
    # discontinuity that otherwise showed up at the phase boundary.
    t1_actual = n1 * dt
    t2_actual = n2 * dt
    v = 2 * (d1_m + d2_m) / (t1_actual + t2_actual)
    accel1 = v / t1_actual
    accel2 = v / t2_actual

    positions_mm = [start_mm]
    velocities_m_s = [0.0]
    for i in range(1, n1 + 1):
        t = i * dt
        positions_mm.append(start_mm + 0.5 * accel1 * t**2 * 1000)
        velocities_m_s.append(accel1 * t)
    # Anchored to phase 1's actual last position, not the nominal
    # impact_mm, so phase 2 continues exactly where phase 1 left off (still
    # lands exactly on end_mm, since the two phase distances sum to
    # end_mm - start_mm by construction of `v` above).
    phase2_start_mm = positions_mm[-1]
    for i in range(1, n2 + 1):
        t = i * dt
        positions_mm.append(phase2_start_mm + (v * t - 0.5 * accel2 * t**2) * 1000)
        # Clamped at 0 -- floating-point round-off could otherwise land
        # the final velocity just below zero.
        velocities_m_s.append(max(0.0, v - accel2 * t))

    positions_counts = [Controller.mm_to_counts(p) for p in positions_mm]
    velocities_counts_per_s = [Controller.meters_to_counts(v) for v in velocities_m_s]

    return [
        (
            positions_counts[i] - positions_counts[i - 1],
            velocities_counts_per_s[i],
            t_samples,
        )
        for i in range(1, len(positions_counts))
    ]


def build_pvt_program(
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
    pvt_fv,
    pvt_fa,
    pvt_ac_counts,
    pvt_dc_counts,
    segments,
    post_pause_ms,
    rest_kp,
    rest_kd,
    rest_ki,
    rest_offset_v,
):
    """Redundant slow retract to start_counts -> set Motion sample period ->
    start recording -> pre-move pause -> apply Motion PID gains/offset,
    then PVT's own FV/FA/AC/DC (FV overrides Motion PID's own FV for this
    move; FA/AC/DC only feed the FA feedforward formula -- PVT motion
    itself ignores AC/DC/SP for shaping the trajectory) -> PVT move
    through `segments` -> post-move pause -> stop recording -> reset
    FV/FA/AC/DC to their original values (Motion PID's own FV, FA off,
    the app's baseline AC/DC) -> reset PID/offset/sample period to Rest
    values -> settle -> disable -> end.
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
        f"FV {pvt_fv}",
        f"FA {pvt_fa}",
        f"AC {pvt_ac_counts}",
        f"DC {pvt_dc_counts}",
    ]
    lines += [f"PVA={p},{v},{t}" for p, v, t in segments]
    lines += [
        "PVA=0,0,0",
        "BTA",
        "#WAIT",
        # "<" not "<>" -- robust either way to whether the PVA=0,0,0
        # terminator segment itself increments _BTA (undocumented).
        f"JP#WAIT,_BTA<{len(segments)}",
        f"WT {post_pause_ms}",
        # Stop recording.
        "RC 0",
        # Reset PVT's overrides back to their "original" values.
        f"FV {motion_fv}",
        "FA 0",
        f"AC {DEFAULT_ACCEL_COUNTS_PER_S2}",
        f"DC {DEFAULT_DECEL_COUNTS_PER_S2}",
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
