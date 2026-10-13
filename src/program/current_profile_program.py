"""Builds the DMC program text for Current Profile mode -- a Contour
Mode position move (thread 0) running concurrently with a feedforward
current/voltage curve played back into OF (Offset) from a host-
downloaded array (thread 1), so the feedforward term and the position
PID's own small residual trim sum automatically at the amplifier input.

Fixed, bench-confirmed timing (see the plan file "Current Profile Motion
Mode", Phase 0) -- TM=125us (fast servo; unlike Contour/PVT's own user-
configurable Sample Period, this mode's whole array-read + OF-write loop
was specifically validated, including concurrently with an active
Contour move and real PID gains, at this exact rate, so it is not a
user-facing field here), CD points spaced at DT=1 (2*TM=250us, standard
Contour Mode -- reuses contour_program.py's own CONTOUR_DT_EXPONENT so
the two stay in sync), the OF-loop paced at AT -4,1 (4*TM=500us) -- the
confirmed real achievable per-update cadence for the array-read+OF-write
loop.

Pure text/data generation -- no Tkinter, no gclib -- so this stays
testable independent of both the GUI and the controller connection.
Trajectory math (the quintic position/acceleration curve, and the
FF(t)=m*a(t)/Kf conversion) lives in current_profile_trajectory.py, not
here -- this module only turns already-sampled position increments and
feedforward values into DMC program text, mirroring how
contour_program.py's build_contour_program takes pre-computed
increments rather than computing them itself.
"""

from controller.controller import (
    ARRAY_TABLE_MAX_ELEMENTS,
    Controller,
    DEFAULT_ACCEL_COUNTS_PER_S2,
    DEFAULT_DECEL_COUNTS_PER_S2,
    DEFAULT_SAMPLE_PERIOD_US,
    POST_TEST_DISABLE_DELAY_MS,
    RECORD_ARRAY_NAME,
    RECORD_CMD_ARRAY_NAME,
    RECORD_ERR_ARRAY_NAME,
    RECORD_TRQ_ARRAY_NAME,
)
from program.contour_program import (
    CONTOUR_DT_EXPONENT,
    CONTOUR_MAX_SEGMENT_COUNTS,
    CONTOUR_MAX_SEGMENTS,
)

TM_US = 125  # nominal/requested value -- this is what "TM {TM_US}" actually sends
AT_SAMPLES = 4  # 4 samples/iteration, bench-confirmed floor -- see TM_US_ACTUAL for real cadence

# Bench-confirmed (src/headless/tm_period_bench.py, both a recording-
# based ground truth and an independent AT-based cross-check, landing
# within 1% of each other): requesting TM=125us actually achieves
# ~122us in practice -- close to the Galil firmware reference's own
# documented TM=1000->976us discrepancy (976/1000 = -2.4%; 125*(1-0.024)
# = 122.0 exactly). The controller is still sent the literal "TM 125"
# command (TM_US above) -- that request is what empirically produces
# ~122us -- but all of THIS mode's own timing math that assumes a
# sample period (trajectory sampling rates, recording-sample-count
# estimates) uses this real value instead, not the nominal TM_US.
TM_US_ACTUAL = 122

# The name Controller.download_array() dimensions and downloads the
# feedforward array under -- must already be resident (via a separate
# download_array() call, program download first per its own QD caveat)
# by the time #MAIN reaches XQ #LOOP,1.
FF_ARRAY_NAME = "ffProf"

# Headroom above the exact predicted recording-sample count. Kept tight
# (not the 20%+ margins that would be free elsewhere) because the
# array-table budget is genuinely close for a move anywhere near the
# top of this mode's ~60-80ms target range once the feedforward array is
# added on top of 4-channel recording -- see the array-budget check
# below. 5% roughly matches the ratio deviations actually observed on
# the Phase 0 bench (typically within a couple percent of predicted for
# a passing configuration), rather than an arbitrary round number.
# Bench-found while implementing this file: pre_pause_ms/post_pause_ms
# materially affect how much of the budget recording alone consumes,
# since the recording window spans them too -- keep both small for
# Current Profile mode specifically, and treat a near-80ms move as
# running close to this budget's edge.
RECORD_SIZE_MARGIN = 1.05


def build_current_profile_program(
    *,
    start_counts,
    slow_speed_counts,
    slow_accel_counts,
    slow_decel_counts,
    pre_pause_ms,
    motion_kp,
    motion_kd,
    motion_ki,
    motion_offset_v,
    increments,
    ff_values,
    ff_advance_us=0,
    post_pause_ms,
    rest_kp,
    rest_kd,
    rest_ki,
    rest_offset_v,
):
    """Redundant slow retract to start_counts -> set TM=125us -> start
    recording (sized to this specific move, not the shared default -- see
    array-budget check below) -> pre-move pause -> apply Motion PID
    gains/offset (FV always 0 -- this mode's feedforward is the OF loop,
    not the FV mechanism Contour/PVT use) -> Contour Mode move through
    `increments` (optionally preceded by stationary padding, see
    `ff_advance_us`), launching
    the OF-writing thread (#LOOP) just before the first CD point ->
    wait for the Contour move to complete, then wait for thread 1 to
    finish too (so TM/PID don't change while its AT pacing is still
    using them) -> post-move pause -> stop recording -> reset PID/offset
    to Rest values -> settle -> disable -> end.

    `increments`: CD position increments (counts), from
    current_profile_trajectory.sample_position_mm sampled at
    2*TM_US*1e-6 seconds, converted to counts/increments the same way
    contour_program.compute_contour_increments does internally.
    `ff_values`: feedforward volts (one per #LOOP iteration), from
    current_profile_trajectory.compute_feedforward_v fed by
    sample_acceleration_m_s2 sampled at AT_SAMPLES*TM_US*1e-6 seconds --
    already clamped to OF's range, downloaded separately via
    Controller.download_array(FF_ARRAY_NAME, ff_values) before this
    program runs (not embedded in the program text -- see FF_ARRAY_NAME).
    `ff_advance_us`: how long to hold physical position still (via
    leading zero-increment CD segments) before `increments`' real motion
    starts, while thread 1's OF-writing loop still starts at its usual
    moment (paired with the first CD line, now one of the padding
    segments) -- since the FF curve's acceleration ramps up from 0 at
    its own t=0, this gives the feedforward current a head start before
    real motion begins, addressing a bench-observed actual-position lag
    behind commanded position at high impact velocities. 0 (default)
    adds no padding.

    Raises ValueError if `increments` (plus any ff_advance_us padding)
    needs more than CONTOUR_MAX_SEGMENTS points or any single increment
    exceeds +/-CONTOUR_MAX_SEGMENT_COUNTS (same limits
    compute_contour_increments itself enforces), or if this move's
    recording arrays plus the feedforward array would exceed the
    controller's total array-table budget.

    Returns (program_text, record_samples) -- record_samples (the exact
    size embedded in this program's own DM lines for the 4 shared
    recording arrays below) is also returned so the caller can size
    those arrays itself at Arm time via Controller.size_shared_record_
    arrays(), reliably, before Fire runs this program -- relying on
    this program's own DM lines to grow them (from whatever a PRIOR
    Arm/Fire left them at) was bench-confirmed unreliable on its own
    (silently stayed at the smaller prior size, "only 1 sample(s)
    recorded"), the same in-place-array-growth quirk
    Controller.download_array's own shrink-then-grow works around.
    """
    ff_advance_segments = round(ff_advance_us / (2 * TM_US_ACTUAL))
    padded_increments = [0] * ff_advance_segments + list(increments)

    if len(padded_increments) > CONTOUR_MAX_SEGMENTS:
        raise ValueError(
            f"Current Profile move requires {len(padded_increments)} points "
            f"({len(increments)} motion + {ff_advance_segments} Feedforward Advance "
            f"padding), exceeds the {CONTOUR_MAX_SEGMENTS}-point Contour buffer limit "
            f"-- increase impact velocity, reduce the Start/Stop span, or reduce "
            f"Feedforward Advance"
        )
    for inc in increments:
        if abs(inc) > CONTOUR_MAX_SEGMENT_COUNTS:
            raise ValueError(
                f"Current Profile segment increment {inc} counts exceeds "
                f"+/-{CONTOUR_MAX_SEGMENT_COUNTS} -- reduce impact velocity"
            )

    # Recording starts before pre_pause_ms and stops after post_pause_ms
    # (RC 1 precedes CMA, RC 0 follows the post-move wait) -- the
    # recording window is pre_pause_ms + however long the ACTIVE part of
    # the move takes (both threads -- see active_samples below) +
    # post_pause_ms. Recording captures once per REAL TM tick (see
    # TM_US_ACTUAL); CD points are spaced at 2*TM, so thread 0's move
    # covers roughly 2*len(padded_increments) TM ticks.
    pause_samples = round((pre_pause_ms + post_pause_ms) * 1000 / TM_US_ACTUAL)
    move_samples = len(padded_increments) * 2
    # Thread 1's own OF-playback loop runs concurrently and isn't
    # necessarily the same length as thread 0's move -- each of its
    # AT_SAMPLES iterations pace AT_SAMPLES TM ticks apart (see AT
    # -{AT_SAMPLES},1 below), so its own real duration is
    # len(ff_values)*AT_SAMPLES TM ticks. The teardown correctly waits
    # for BOTH threads (#WAIT then #WAIT2) before RC 0 stops recording,
    # so real time already accounts for whichever finishes last -- but
    # RC 1 is non-circular (fixed-size, stops once its DM'd array fills,
    # not once thread 1 actually finishes), so record_samples itself
    # must also cover whichever thread runs longer, or a long Feedforward
    # stretch (accel/decel rise/fall > 1.0) silently crops the tail of
    # every recorded trace -- bench-observed as feedforward current
    # "getting cropped past the end of the recording."
    ff_samples = len(ff_values) * AT_SAMPLES
    active_samples = max(move_samples, ff_samples)
    record_samples = max(1, round((pause_samples + active_samples) * RECORD_SIZE_MARGIN))
    total_array_elements = 4 * record_samples + len(ff_values)
    if total_array_elements > ARRAY_TABLE_MAX_ELEMENTS:
        raise ValueError(
            f"Current Profile move needs {total_array_elements} array-table "
            f"elements (4x{record_samples} recording + {len(ff_values)} "
            f"feedforward), exceeds the {ARRAY_TABLE_MAX_ELEMENTS}-element "
            f"controller ceiling -- reduce the move's span/duration"
        )

    lines = [
        "#MAIN",
        # Redundant slow retract.
        f"SP {slow_speed_counts}",
        f"AC {slow_accel_counts}",
        f"DC {slow_decel_counts}",
        f"PA {start_counts}",
        "BGA",
        "AMA",
        f"TM {TM_US}",
        # Start recording -- 4 simultaneous channels, sized to this
        # specific move (not the shared RECORD_ARRAY_SIZE default) so
        # there's room left in the array table for the feedforward array.
        f"DM {RECORD_ARRAY_NAME}[{record_samples}]",
        f"DM {RECORD_CMD_ARRAY_NAME}[{record_samples}]",
        f"DM {RECORD_ERR_ARRAY_NAME}[{record_samples}]",
        f"DM {RECORD_TRQ_ARRAY_NAME}[{record_samples}]",
        f"RA {RECORD_ARRAY_NAME}[],{RECORD_CMD_ARRAY_NAME}[],"
        f"{RECORD_ERR_ARRAY_NAME}[],{RECORD_TRQ_ARRAY_NAME}[]",
        "RD _TPA,_RPA,_TEA,_TTA",
        "RC 1",
        f"WT {pre_pause_ms}",
        f"KD {motion_kd}",
        f"KP {motion_kp}",
        f"KI {motion_ki}",
        f"OF {motion_offset_v}",
        # Always 0 -- this mode's feedforward is the OF loop (thread 1),
        # not Contour/PVT's own FV mechanism.
        "FV 0",
        "CMA",
        f"DT {CONTOUR_DT_EXPONENT}",
        # Thread 1 owns OF exclusively from here on -- thread 0 never
        # writes it again until teardown, after confirming thread 1 has
        # stopped.
        "XQ #LOOP,1",
    ]
    lines += [f"CD {inc}" for inc in padded_increments]
    lines += [
        "CD 0=0",
        "#WAIT",
        "JP#WAIT,_CM<>511",
        # Wait for thread 1 too -- it's sized to finish at roughly the
        # same time as the Contour move, but this guarantees TM/PID
        # aren't touched below while its AT pacing might still be using
        # them.
        "#WAIT2",
        "JP#WAIT2,_XQ1<>-1",
        f"WT {post_pause_ms}",
        "RC 0",
        # Reset to rest state.
        f"KD {rest_kd}",
        f"KP {rest_kp}",
        f"KI {rest_ki}",
        # f"OF {rest_offset_v}",
        "OF 0",
        "MOA",
        f"TM {DEFAULT_SAMPLE_PERIOD_US}",
        f"AC {DEFAULT_ACCEL_COUNTS_PER_S2}",
        f"DC {DEFAULT_DECEL_COUNTS_PER_S2}",
        # Settle, then auto-disable rather than leave the servo enabled
        # indefinitely after the test completes.
        f"WT {POST_TEST_DISABLE_DELAY_MS}",
        "EN",
        "#LOOP",
        "AT0",
        "i=0",
        "#L",
        f"OF {FF_ARRAY_NAME}[i]",
        "i=i+1",
        f"AT -{AT_SAMPLES},1",
        f"JP#L,i<{len(ff_values)}",
        "OF 0",
        "EN",
    ]
    return "\n".join(lines) + "\n", record_samples
