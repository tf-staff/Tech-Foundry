"""Builds the DMC program text for a complete impact SEQUENCE:

    redundant slow retract -> [LVDT conditioning pulse into the retract
    stop] -> move to Start -> fast recording on -> pre-move pause ->
    Current Profile impact (Contour Mode position move on thread 0 +
    feedforward OF playback on thread 1, exactly as
    current_profile_program.py) -> DWELL at full extension under a hold
    current -> switch to slow recording -> [open-loop retract kick] ->
    closed-loop retract to a target -> post-move pause -> recording off
    -> Rest settings -> settle -> [disable] -> end.

The impact section is byte-for-byte the same shape as
current_profile_program.build_current_profile_program (same TM, same
CD/DT spacing, same #LOOP thread, same wait-for-both-threads teardown
ordering), so anything tuned in Current Profile mode carries over
unchanged. What this module adds is everything around it, plus a
recording scheme that fits a multi-second sequence into the
controller's 3000-element array table:

RECORDING -- two segments, one array table.
  * FAST segment: the 4 shared recording arrays (position, commanded
    position, following error, output command), one sample per servo
    tick (~122us), covering pre-pause + impact move + `fast_tail_ms` of
    the dwell (the rebound off the hard stop). By default this segment
    is a ROLLING (circular) buffer -- "RC 1,-n" -- so if the move runs
    longer than predicted the OLDEST samples (the pre-pause) are
    overwritten rather than the impact/rebound being silently cropped
    off the end, which is what the non-circular mode does (bench-
    observed, see current_profile_program.RECORD_SIZE_MARGIN). The
    program stamps TIME at each recording boundary and stores _RD after
    stopping so the host can unwrap the ring (see
    recording.sequence_recording).
  * SLOW segment: 3 arrays (position, commanded position, output
    command) at a coarser "RC k" interval (k servo samples, k<=254),
    covering the rest of the dwell, the retract and the post-pause.
    These arrays are sized to use whatever array space the fast
    segment and the feedforward array leave over, and k is chosen as
    the smallest interval that still covers the predicted slow-segment
    duration -- so a 1s dwell + retract is typically captured at a few
    ms per sample.
  Both segments live in the same program, so no host timing is
  involved anywhere; all arrays are dimensioned up front (never DM/DA
  while recording, per the RC reference) and the caller must size them
  at Arm time via Controller.size_arrays (same in-place-growth quirk
  current_profile_program.py documents).

DWELL TIMING -- the dwell is timed with AT (At Time) against a
reference set the instant both impact threads have finished: "AT0"
then "AT n,1" (n servo samples from the reference) just after the
recording switch, so the switch's own execution time doesn't add to
the dwell. Resolution is 2 samples (~244us) per the AT reference.

SIGN CONVENTIONS (all voltages are OF values, see impact_program.py):
negative OF drives toward EXTEND (increasing position), positive OF
toward RETRACT. Callers convert amps to volts (Controller.current_to_
volts) and apply the sign; this module takes signed volts.

Pure text/data generation -- no Tkinter, no gclib.
"""

import math
from typing import NamedTuple

from controller.controller import (
    ARRAY_TABLE_MAX_ELEMENTS,
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
from program.current_profile_program import (
    AT_SAMPLES,
    FF_ARRAY_NAME,
    RECORD_SIZE_MARGIN,
    TM_US,
    TM_US_ACTUAL,
)

# Slow-segment arrays (names <= 8 chars, first char alphabetic -- DM
# reference). Distinct from the 4 shared fast arrays.
SLOW_POS_ARRAY_NAME = "posSlo"
SLOW_CMD_ARRAY_NAME = "cmdSlo"
SLOW_TRQ_ARRAY_NAME = "trqSlo"
SLOW_ARRAY_NAMES = (SLOW_POS_ARRAY_NAME, SLOW_CMD_ARRAY_NAME, SLOW_TRQ_ARRAY_NAME)
FAST_ARRAY_NAMES = (
    RECORD_ARRAY_NAME, RECORD_CMD_ARRAY_NAME, RECORD_ERR_ARRAY_NAME, RECORD_TRQ_ARRAY_NAME,
)

# RC n0 range (firmware reference): 0..254 servo samples between records.
RC_MAX_INTERVAL_SAMPLES = 254
# WT n,1 / AT n,1: n0 minimum 2, resolution 2 (firmware reference).
WT_MIN_SAMPLES = 2
# Margin on the predicted slow-segment duration when choosing the RC
# interval -- the retract's real duration depends on how the closed
# loop actually settles, which a trapezoid estimate can't see.
SLOW_SEGMENT_MARGIN = 1.25
SLOW_SEGMENT_EXTRA_MS = 50.0

# Sentinel written into the last element of every FAST array before
# recording starts. Position/torque counts are 16-bit (|x| <= 32768), so
# this value can only survive if recording never reached that element
# -- which is how the host tells "filled/wrapped" from "stopped early"
# in circular mode (see recording.sequence_recording.unwrap_circular).
CIRCULAR_SENTINEL = -100000

# Controller variables (<= 8 chars) the program stores for the host.
VAR_FAST_NEXT = "nfast"  # _RD after the fast segment stopped
VAR_SLOW_NEXT = "nslow"  # _RD after the slow segment stopped
VAR_FAST_START = "tf0"  # TIME when fast recording started
VAR_FAST_STOP = "tf1"  # TIME when fast recording stopped
VAR_DWELL_START = "tdw0"  # TIME when both impact threads had finished
VAR_SLOW_START = "ts0"  # TIME when slow recording started
VAR_RETRACT_START = "trt0"  # TIME when the closed-loop retract was begun
VAR_SLOW_STOP = "ts1"  # TIME when slow recording stopped
SEQUENCE_VARIABLES = (
    VAR_FAST_NEXT, VAR_SLOW_NEXT, VAR_FAST_START, VAR_FAST_STOP,
    VAR_DWELL_START, VAR_SLOW_START, VAR_RETRACT_START, VAR_SLOW_STOP,
)


class RecordLayout(NamedTuple):
    """How the sequence program laid out the array table -- everything
    the host needs to size the arrays at Arm time (Controller.size_
    arrays) and to upload/unwrap/timestamp the capture afterwards."""

    sample_period_s: float  # real servo period (TM_US_ACTUAL)
    fast_samples: int  # per fast array (4 arrays)
    fast_circular: bool
    slow_samples: int  # per slow array (3 arrays)
    slow_interval_samples: int  # RC n0 for the slow segment
    ff_samples: int

    @property
    def array_sizes(self) -> dict:
        sizes = {name: self.fast_samples for name in FAST_ARRAY_NAMES}
        sizes.update({name: self.slow_samples for name in SLOW_ARRAY_NAMES})
        return sizes

    @property
    def total_elements(self) -> int:
        return 4 * self.fast_samples + 3 * self.slow_samples + self.ff_samples


def ms_to_samples(ms: float) -> int:
    """Milliseconds -> servo samples at the bench-measured real period."""
    return max(0, round(ms * 1000 / TM_US_ACTUAL))


def _wait_ms(ms: float) -> int:
    # WT n (milliseconds): minimum 2, resolution 2 -- rounded UP.
    return max(WT_MIN_SAMPLES, int(math.ceil(ms / 2) * 2))


def _wait_samples(n: int) -> int:
    # WT/AT n,1: minimum 2, resolution 2 -- round UP to the next even
    # count so a requested duration is never shortened.
    n = max(WT_MIN_SAMPLES, int(math.ceil(n / 2) * 2))
    return n


def predict_trapezoid_move_s(distance_counts, speed_counts, accel_counts, decel_counts) -> float:
    """Duration of a PA move under SP/AC/DC -- trapezoidal, or triangular
    when the distance is too short to reach SP. Used only to size the
    slow recording segment."""
    d = abs(float(distance_counts))
    v, a, dc = float(speed_counts), float(accel_counts), float(decel_counts)
    if d == 0 or v <= 0 or a <= 0 or dc <= 0:
        return 0.0
    ramp_distance = v * v / (2 * a) + v * v / (2 * dc)
    if ramp_distance >= d:
        v_peak = math.sqrt(2 * d * a * dc / (a + dc))
        return v_peak / a + v_peak / dc
    return v / a + v / dc + (d - ramp_distance) / v


def plan_record_layout(
    *,
    pre_pause_ms, move_samples, ff_samples, fast_tail_ms,
    dwell_ms, retract_s, post_pause_ms, fast_circular=True,
    extra_slow_ms=0.0,
) -> RecordLayout:
    """Splits the array table between the fast and slow segments.

    Fast: pre-pause + the longer of the two impact threads + fast tail,
    with RECORD_SIZE_MARGIN headroom. Slow: everything left after the
    fast arrays and the feedforward array, at the smallest RC interval
    that still covers the predicted remaining dwell + retract +
    post-pause (+ margin). Raises ValueError if the fast segment alone
    doesn't fit, or the slow segment would need an RC interval beyond
    the controller's 254-sample maximum."""
    fast_tail_ms = min(fast_tail_ms, dwell_ms)
    fast_samples = max(1, round(
        (ms_to_samples(pre_pause_ms) + move_samples + ms_to_samples(fast_tail_ms))
        * RECORD_SIZE_MARGIN
    ))
    remaining = ARRAY_TABLE_MAX_ELEMENTS - 4 * fast_samples - ff_samples
    slow_samples = remaining // 3
    if slow_samples < 1:
        raise ValueError(
            f"Fast capture needs 4x{fast_samples} + {ff_samples} feedforward = "
            f"{4 * fast_samples + ff_samples} array elements, leaving no room in the "
            f"{ARRAY_TABLE_MAX_ELEMENTS}-element table for the dwell/retract "
            f"recording -- shorten Fast Capture Tail or the move"
        )
    slow_duration_ms = (
        (dwell_ms - fast_tail_ms) + retract_s * 1000 + post_pause_ms + extra_slow_ms
    ) * SLOW_SEGMENT_MARGIN + SLOW_SEGMENT_EXTRA_MS
    slow_duration_samples = ms_to_samples(slow_duration_ms)
    slow_interval = max(1, int(math.ceil(slow_duration_samples / slow_samples)))
    if slow_interval > RC_MAX_INTERVAL_SAMPLES:
        raise ValueError(
            f"Dwell/retract recording would need one sample every {slow_interval} "
            f"servo samples to fit {slow_duration_ms:.0f}ms into {slow_samples} "
            f"elements, beyond the controller's {RC_MAX_INTERVAL_SAMPLES} maximum "
            f"-- shorten Dwell/retract or Fast Capture Tail"
        )
    return RecordLayout(
        sample_period_s=TM_US_ACTUAL / 1_000_000,
        fast_samples=fast_samples,
        fast_circular=fast_circular,
        slow_samples=slow_samples,
        slow_interval_samples=slow_interval,
        ff_samples=ff_samples,
    )


def conditioning_lines(
    *, pulse_v, pulse_ms, settle_ms, rest_kp, rest_kd, rest_ki, rest_offset_v,
) -> list:
    """Program fragment: zero the position loop, apply `pulse_v` for
    `pulse_ms` (plain millisecond WT -- meant to run at the Rest sample
    period, where WT's own TM scaling applies), release, settle for
    `settle_ms`, then resync the position
    reference to wherever the axis actually is BEFORE the Rest gains go
    back on (the "p=_TPA; PA p" sync impact_program.py uses to avoid a
    stale-_RPA snap), and settle again.

    Assumes the axis is already resting against a hard stop in the
    direction of the pulse (see conditioning_program.py) -- a 10A pulse
    on a free axis is a ~1m/s impulse, not a conditioning step."""
    return [
        "KD 0",
        "KP 0",
        "KI 0",
        f"OF {pulse_v}",
        f"WT {_wait_ms(pulse_ms)}",
        "OF 0",
        f"WT {_wait_ms(settle_ms)}",
        "p=_TPA",
        "PA p",
        "BGA",
        "AMA",
        f"KD {rest_kd}",
        f"KP {rest_kp}",
        f"KI {rest_ki}",
        f"OF {rest_offset_v}",
        f"WT {_wait_ms(settle_ms)}",
    ]


def build_sequence_program(
    *,
    start_counts,
    slow_speed_counts,
    slow_accel_counts,
    slow_decel_counts,
    condition_counts=None,
    condition_pulse_v=0.0,
    condition_pulse_ms=0.0,
    condition_settle_ms=50.0,
    pre_pause_ms,
    motion_kp,
    motion_kd,
    motion_ki,
    motion_offset_v,
    increments,
    ff_values,
    ff_advance_us=0,
    dwell_ms,
    hold_v,
    hold_servo=False,
    fast_tail_ms,
    retract_kick_v=0.0,
    retract_kick_ms=0.0,
    retract_kp,
    retract_kd,
    retract_ki,
    retract_offset_v,
    retract_speed_counts,
    retract_accel_counts,
    retract_decel_counts,
    retract_target_counts,
    post_pause_ms,
    rest_kp,
    rest_kd,
    rest_ki,
    rest_offset_v,
    fast_circular=True,
    disable_at_end=True,
):
    """Returns (program_text, RecordLayout). See the module docstring
    for the sequence and the recording scheme.

    `increments`/`ff_values`/`ff_advance_us`: exactly as
    current_profile_program.build_current_profile_program.
    `condition_counts`/`condition_pulse_v`/`condition_pulse_ms`: where
    to park (a PA target that is a hard stop -- normally the full-
    retract limit) and what to pulse there; pulse_ms <= 0 or pulse_v
    == 0 skips conditioning entirely.
    `dwell_ms`: hold at full extension for this long (AT-timed from the
    instant both impact threads finished). `hold_v`: OF during the
    dwell (negative pushes into the extend stop). `hold_servo`: keep
    the Motion PID active during the dwell (commanded position stays
    at the last CD point) instead of zeroing it.
    `fast_tail_ms`: how much of the dwell stays in the fast recording.
    `retract_kick_v`/`retract_kick_ms`: optional open-loop pulse
    (positive = toward retract) before the closed-loop retract.
    `retract_*`: PID + SP/AC/DC + target for the closed-loop retract.

    Raises ValueError on contour-buffer or array-table violations, same
    as build_current_profile_program."""
    if dwell_ms < 0 or fast_tail_ms < 0:
        raise ValueError("Dwell and Fast Capture Tail must not be negative")
    fast_tail_ms = min(fast_tail_ms, dwell_ms)

    ff_advance_segments = round(ff_advance_us / (2 * TM_US_ACTUAL))
    padded_increments = [0] * ff_advance_segments + list(increments)
    if len(padded_increments) > CONTOUR_MAX_SEGMENTS:
        raise ValueError(
            f"Impact move requires {len(padded_increments)} points "
            f"({len(increments)} motion + {ff_advance_segments} Feedforward Advance "
            f"padding), exceeds the {CONTOUR_MAX_SEGMENTS}-point Contour buffer limit "
            f"-- increase impact velocity, reduce the Start/Stop span, or reduce "
            f"Feedforward Advance"
        )
    for inc in increments:
        if abs(inc) > CONTOUR_MAX_SEGMENT_COUNTS:
            raise ValueError(
                f"Impact segment increment {inc} counts exceeds "
                f"+/-{CONTOUR_MAX_SEGMENT_COUNTS} -- reduce impact velocity"
            )

    move_samples = max(len(padded_increments) * 2, len(ff_values) * AT_SAMPLES)
    # The retract distance is from wherever the dwell ended (near the
    # extend stop, i.e. the end of the commanded move) back to the
    # target -- approximate with the whole commanded span.
    retract_distance = abs(sum(increments)) + abs(start_counts - retract_target_counts)
    retract_s = predict_trapezoid_move_s(
        retract_distance, retract_speed_counts, retract_accel_counts, retract_decel_counts,
    )
    layout = plan_record_layout(
        pre_pause_ms=pre_pause_ms, move_samples=move_samples, ff_samples=len(ff_values),
        fast_tail_ms=fast_tail_ms, dwell_ms=dwell_ms, retract_s=retract_s,
        post_pause_ms=post_pause_ms, fast_circular=fast_circular,
        extra_slow_ms=retract_kick_ms,
    )
    if layout.total_elements > ARRAY_TABLE_MAX_ELEMENTS:
        raise ValueError(
            f"Sequence needs {layout.total_elements} array-table elements, exceeds "
            f"the {ARRAY_TABLE_MAX_ELEMENTS}-element controller ceiling"
        )

    nf, ns = layout.fast_samples, layout.slow_samples
    dwell_samples = _wait_samples(ms_to_samples(dwell_ms))
    tail_samples = _wait_samples(ms_to_samples(fast_tail_ms))
    conditioning = condition_pulse_ms > 0 and condition_pulse_v != 0

    lines = ["#MAIN", f"SP {slow_speed_counts}", f"AC {slow_accel_counts}", f"DC {slow_decel_counts}"]
    if conditioning:
        park = start_counts if condition_counts is None else condition_counts
        lines += [
            # Park against the conditioning stop (slow, Rest gains).
            f"PA {park}",
            "BGA",
            "AMA",
            # Conditioning happens at the Rest sample period, like the
            # standalone conditioning program -- same current, same
            # duration, no dependence on TM.
        ]
        lines += conditioning_lines(
            pulse_v=condition_pulse_v,
            pulse_ms=condition_pulse_ms,
            settle_ms=condition_settle_ms,
            rest_kp=rest_kp, rest_kd=rest_kd, rest_ki=rest_ki, rest_offset_v=rest_offset_v,
        )
    lines += [
        # Redundant slow move to Start (Rest gains, Rest TM).
        f"PA {start_counts}",
        "BGA",
        "AMA",
        f"TM {TM_US}",
        # Every recording array dimensioned up front -- never DM while
        # recording. Same sizes the caller already applied at Arm time.
        f"DM {RECORD_ARRAY_NAME}[{nf}]",
        f"DM {RECORD_CMD_ARRAY_NAME}[{nf}]",
        f"DM {RECORD_ERR_ARRAY_NAME}[{nf}]",
        f"DM {RECORD_TRQ_ARRAY_NAME}[{nf}]",
        f"DM {SLOW_POS_ARRAY_NAME}[{ns}]",
        f"DM {SLOW_CMD_ARRAY_NAME}[{ns}]",
        f"DM {SLOW_TRQ_ARRAY_NAME}[{ns}]",
    ]
    if fast_circular:
        # Wrap detection for the ring buffer -- see CIRCULAR_SENTINEL.
        lines += [f"{name}[{nf - 1}]={CIRCULAR_SENTINEL}" for name in FAST_ARRAY_NAMES]
    lines += [
        f"RA {RECORD_ARRAY_NAME}[],{RECORD_CMD_ARRAY_NAME}[],"
        f"{RECORD_ERR_ARRAY_NAME}[],{RECORD_TRQ_ARRAY_NAME}[]",
        "RD _TPA,_RPA,_TEA,_TTA",
        f"{VAR_FAST_START}=TIME",
        f"RC 1,-{nf}" if fast_circular else "RC 1",
        f"WT {pre_pause_ms}",
        f"KD {motion_kd}",
        f"KP {motion_kp}",
        f"KI {motion_ki}",
        f"OF {motion_offset_v}",
        "FV 0",
        "CMA",
        f"DT {CONTOUR_DT_EXPONENT}",
        # Thread 1 owns OF from here until it ends (see
        # current_profile_program.py).
        "XQ #LOOP,1",
    ]
    lines += [f"CD {inc}" for inc in padded_increments]
    lines += [
        "CD 0=0",
        "#WAIT",
        "JP#WAIT,_CM<>511",
        "#WAIT2",
        "JP#WAIT2,_XQ1<>-1",
        # ---- Dwell: both threads done, the tip is at/against the stop.
        "AT0",
        f"{VAR_DWELL_START}=TIME",
    ]
    if not hold_servo:
        lines += ["KD 0", "KP 0", "KI 0"]
    lines += [
        f"OF {hold_v}",
        # First part of the dwell stays in the fast recording (rebound).
        f"WT {tail_samples},1",
        "RC 0",
        f"{VAR_FAST_NEXT}=_RD",
        f"{VAR_FAST_STOP}=TIME",
        # Switch to the slow recording for the rest of the sequence.
        f"RA {SLOW_POS_ARRAY_NAME}[],{SLOW_CMD_ARRAY_NAME}[],{SLOW_TRQ_ARRAY_NAME}[]",
        "RD _TPA,_RPA,_TTA",
        f"{VAR_SLOW_START}=TIME",
        f"RC {layout.slow_interval_samples}",
        # Dwell end, timed from the AT0 reference above so the recording
        # switch's own execution time doesn't stretch the dwell.
        f"AT {dwell_samples},1",
        # ---- Retract.
        "OF 0",
    ]
    if retract_kick_ms > 0 and retract_kick_v != 0:
        lines += [
            f"OF {retract_kick_v}",
            f"WT {_wait_samples(ms_to_samples(retract_kick_ms))},1",
            "OF 0",
        ]
    if not hold_servo:
        # Resync the reference to the actual position before gains go
        # nonzero -- PID is still 0 here, so this is a zero-distance
        # move that only prevents a stale-_RPA snap.
        lines += ["p=_TPA", "PA p", "BGA", "AMA"]
    lines += [
        f"KD {retract_kd}",
        f"KP {retract_kp}",
        f"KI {retract_ki}",
        f"OF {retract_offset_v}",
        f"SP {retract_speed_counts}",
        f"AC {retract_accel_counts}",
        f"DC {retract_decel_counts}",
        f"{VAR_RETRACT_START}=TIME",
        f"PA {retract_target_counts}",
        "BGA",
        "AMA",
        f"WT {post_pause_ms}",
        "RC 0",
        f"{VAR_SLOW_NEXT}=_RD",
        f"{VAR_SLOW_STOP}=TIME",
        # Reset to rest state.
        f"KD {rest_kd}",
        f"KP {rest_kp}",
        f"KI {rest_ki}",
        f"OF {rest_offset_v}",
        f"TM {DEFAULT_SAMPLE_PERIOD_US}",
        f"AC {DEFAULT_ACCEL_COUNTS_PER_S2}",
        f"DC {DEFAULT_DECEL_COUNTS_PER_S2}",
        f"WT {POST_TEST_DISABLE_DELAY_MS}",
    ]
    if disable_at_end:
        lines += ["OF 0", "MOA"]
    lines += [
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
    return "\n".join(lines) + "\n", layout
