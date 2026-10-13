"""Assembles a sequence capture (program/sequence_program.py's two
recording segments) from the raw arrays and TIME stamps the controller
holds after the program ends: unwraps the fast ring buffer, trims the
slow segment to what was actually recorded, and puts both segments on
one time axis.

Pure data handling -- no gclib here. The only controller access is a
tiny duck-typed interface (upload_array / read_variable), so this is
testable with a dict-backed fake.
"""

from typing import NamedTuple

from controller.controller import RecordedData
from program.sequence_program import (
    CIRCULAR_SENTINEL,
    FAST_ARRAY_NAMES,
    RecordLayout,
    SLOW_ARRAY_NAMES,
    VAR_DWELL_START,
    VAR_FAST_NEXT,
    VAR_FAST_START,
    VAR_FAST_STOP,
    VAR_RETRACT_START,
    VAR_SLOW_NEXT,
    VAR_SLOW_START,
    VAR_SLOW_STOP,
)


class SlowRecordedData(NamedTuple):
    """Raw counts from the slow segment: position (_TPA), commanded
    position (_RPA), output command (_TTA) -- same representations as
    RecordedData's fields of the same name."""
    position: list
    commanded_position: list
    torque: list


class SequenceRecording(NamedTuple):
    fast: RecordedData
    slow: SlowRecordedData
    layout: RecordLayout
    # Times in seconds relative to the start of the fast recording.
    fast_times_s: list
    slow_times_s: list
    dwell_start_s: float
    retract_start_s: float
    fast_wrapped: bool
    # Raw controller TIME stamps/pointers, for the saved data file.
    markers: dict


def unwrap_circular(values: list, next_index: int, filled: bool) -> list:
    """Puts a ring-buffer array into chronological order. `next_index`
    is _RD after RC 0 (where the NEXT sample would have gone); `filled`
    says whether the ring ever reached its last element (the sentinel
    test). Not filled -> only [0, next_index) holds data. Filled ->
    [next_index, end) is the oldest data, [0, next_index) the newest."""
    n = len(values)
    next_index = max(0, min(int(next_index), n))
    if not filled:
        return list(values[:next_index])
    return list(values[next_index:]) + list(values[:next_index])


def assemble_sequence_recording(controller, layout: RecordLayout) -> SequenceRecording:
    """Reads everything the finished sequence program left behind.
    `controller` needs upload_array(name, count) and
    read_variable(name)."""
    markers = {
        name: controller.read_variable(name)
        for name in (
            VAR_FAST_NEXT, VAR_SLOW_NEXT, VAR_FAST_START, VAR_FAST_STOP,
            VAR_DWELL_START, VAR_SLOW_START, VAR_RETRACT_START, VAR_SLOW_STOP,
        )
    }
    nf = layout.fast_samples
    fast_next = int(markers[VAR_FAST_NEXT])

    if layout.fast_circular:
        raw = {name: controller.upload_array(name, nf) for name in FAST_ARRAY_NAMES}
        # The sentinel sits in the last element of every fast array; the
        # ring counts as filled only once every one of them has been
        # overwritten (they're recorded in lockstep, so this is the
        # conservative reading of a disagreement).
        filled = all(
            len(raw[name]) == nf and float(raw[name][-1]) != CIRCULAR_SENTINEL
            for name in FAST_ARRAY_NAMES
        )
        fast_lists = [unwrap_circular(raw[name], fast_next, filled) for name in FAST_ARRAY_NAMES]
        wrapped = filled and fast_next != 0
    else:
        count = max(0, min(fast_next, nf))
        fast_lists = [controller.upload_array(name, count) for name in FAST_ARRAY_NAMES]
        wrapped = False
    fast = RecordedData(*fast_lists)

    slow_count = max(0, min(int(markers[VAR_SLOW_NEXT]), layout.slow_samples))
    slow = SlowRecordedData(*[controller.upload_array(name, slow_count) for name in SLOW_ARRAY_NAMES])

    dt = layout.sample_period_s
    t0 = markers[VAR_FAST_START]
    # The fast segment's samples end at the fast-stop stamp; in circular
    # mode the earliest retained sample may be later than t0.
    n_fast = len(fast.position)
    fast_end = markers[VAR_FAST_STOP] - t0
    fast_start = max(0.0, fast_end - n_fast) if layout.fast_circular else 0.0
    fast_times_s = [(fast_start + i) * dt for i in range(n_fast)]
    slow_offset = markers[VAR_SLOW_START] - t0
    k = layout.slow_interval_samples
    slow_times_s = [(slow_offset + i * k) * dt for i in range(len(slow.position))]

    return SequenceRecording(
        fast=fast,
        slow=slow,
        layout=layout,
        fast_times_s=fast_times_s,
        slow_times_s=slow_times_s,
        dwell_start_s=(markers[VAR_DWELL_START] - t0) * dt,
        retract_start_s=(markers[VAR_RETRACT_START] - t0) * dt,
        fast_wrapped=wrapped,
        markers=markers,
    )
