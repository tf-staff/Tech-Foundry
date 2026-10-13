from program.sequence_program import (
    CIRCULAR_SENTINEL, FAST_ARRAY_NAMES, RecordLayout, SLOW_ARRAY_NAMES,
)
from recording.sequence_recording import assemble_sequence_recording, unwrap_circular


class DictController:
    def __init__(self, arrays, variables):
        self.arrays, self.variables = arrays, variables

    def upload_array(self, name, count):
        return list(self.arrays[name][:count])

    def read_variable(self, name):
        return float(self.variables[name])


def _layout(nf=10, ns=6, circular=True):
    return RecordLayout(
        sample_period_s=122e-6, fast_samples=nf, fast_circular=circular,
        slow_samples=ns, slow_interval_samples=8, ff_samples=5,
    )


def test_unwrap_circular():
    ring = [5, 6, 7, 1, 2, 3, 4]
    assert unwrap_circular(ring, 3, True) == [1, 2, 3, 4, 5, 6, 7]
    assert unwrap_circular(ring, 3, False) == [5, 6, 7]
    assert unwrap_circular(ring, 0, True) == ring
    assert unwrap_circular(ring, 99, False) == ring


def _variables(nfast, nslow):
    return {
        "nfast": nfast, "nslow": nslow, "tf0": 1000, "tf1": 1000 + 25, "tdw0": 1000 + 20,
        "ts0": 1000 + 27, "trt0": 1000 + 200, "ts1": 1000 + 260,
    }


def test_assemble_wrapped_fast_segment():
    nf = 10
    # 25 samples written into a 10-slot ring: pointer ends at 5, oldest at 5.
    arrays = {}
    for name in FAST_ARRAY_NAMES:
        arrays[name] = [20 + i for i in range(5)] + [15 + i for i in range(5)]
    for name in SLOW_ARRAY_NAMES:
        arrays[name] = [100 + i for i in range(6)]
    rec = assemble_sequence_recording(DictController(arrays, _variables(5, 4)), _layout(nf))
    assert rec.fast.position == [15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
    assert rec.fast_wrapped
    assert len(rec.slow.position) == 4
    # Fast samples end at tf1 (25 samples after tf0) -> start at sample 15.
    assert abs(rec.fast_times_s[0] - 15 * 122e-6) < 1e-12
    assert abs(rec.slow_times_s[1] - (27 + 8) * 122e-6) < 1e-12
    assert abs(rec.dwell_start_s - 20 * 122e-6) < 1e-12


def test_assemble_unfilled_fast_segment():
    nf = 10
    arrays = {name: [1, 2, 3, 0, 0, 0, 0, 0, 0, CIRCULAR_SENTINEL] for name in FAST_ARRAY_NAMES}
    for name in SLOW_ARRAY_NAMES:
        arrays[name] = [0] * 6
    rec = assemble_sequence_recording(DictController(arrays, _variables(3, 0)), _layout(nf))
    assert rec.fast.position == [1, 2, 3]
    assert not rec.fast_wrapped
    assert rec.slow.position == []


def test_assemble_non_circular():
    arrays = {name: list(range(10)) for name in FAST_ARRAY_NAMES}
    for name in SLOW_ARRAY_NAMES:
        arrays[name] = [0] * 6
    rec = assemble_sequence_recording(
        DictController(arrays, _variables(7, 2)), _layout(10, circular=False),
    )
    assert rec.fast.position == list(range(7))
    assert rec.fast_times_s[0] == 0.0
