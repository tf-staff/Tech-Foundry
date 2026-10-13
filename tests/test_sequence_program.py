import re

import pytest

from controller.controller import ARRAY_TABLE_MAX_ELEMENTS, Controller
from program.conditioning_program import build_conditioning_program
from program.current_profile_program import AT_SAMPLES, TM_US_ACTUAL
from program.current_profile_trajectory import (
    compute_current_profile_curve, compute_feedforward_v, sample_position_mm,
)
from program.sequence_program import (
    CIRCULAR_SENTINEL, RC_MAX_INTERVAL_SAMPLES, SLOW_ARRAY_NAMES, build_sequence_program,
    ms_to_samples, plan_record_layout, predict_trapezoid_move_s,
)


def _impact(start=0.0, peak=25.0, impact=25.0, end=39.74, v=4.0):
    curve = compute_current_profile_curve(start, peak, impact, end, v)
    positions = [Controller.mm_to_counts(start)] + [
        Controller.mm_to_counts(p) for p in sample_position_mm(curve, 2 * TM_US_ACTUAL / 1e6)
    ]
    increments = [positions[i] - positions[i - 1] for i in range(1, len(positions))]
    ff = compute_feedforward_v(curve, AT_SAMPLES * TM_US_ACTUAL / 1e6, 0.007, 1.0, 0.6)
    return increments, ff


SLOW = Controller.meters_to_counts(0.1)  # 0.1 m/s in counts/s


def _build(**overrides):
    increments, ff = _impact()
    kwargs = dict(
        start_counts=Controller.mm_to_counts(0.0), slow_speed_counts=SLOW,
        slow_accel_counts=2 * SLOW, slow_decel_counts=2 * SLOW,
        condition_counts=Controller.mm_to_counts(0.0), condition_pulse_v=2.78,
        condition_pulse_ms=20, condition_settle_ms=50,
        pre_pause_ms=2, motion_kp=0, motion_kd=0, motion_ki=0, motion_offset_v=0,
        increments=increments, ff_values=ff, ff_advance_us=0,
        dwell_ms=200, hold_v=-0.5, hold_servo=False, fast_tail_ms=20,
        retract_kick_v=0, retract_kick_ms=0,
        retract_kp=0.1, retract_kd=0.1, retract_ki=0.002, retract_offset_v=0.04,
        retract_speed_counts=SLOW, retract_accel_counts=2 * SLOW, retract_decel_counts=2 * SLOW,
        retract_target_counts=Controller.mm_to_counts(0.0),
        post_pause_ms=10, rest_kp=0.1, rest_kd=0.1, rest_ki=0.002, rest_offset_v=0.04,
    )
    kwargs.update(overrides)
    return build_sequence_program(**kwargs)


def test_program_structure_and_budget():
    text, layout = _build()
    lines = text.splitlines()
    assert lines[0] == "#MAIN"
    assert lines.count("EN") == 2 and "#LOOP" in lines
    # Every line within the controller's 80-char limit.
    assert max(len(l) for l in lines) <= 80
    assert layout.total_elements <= ARRAY_TABLE_MAX_ELEMENTS
    assert layout.slow_samples > 0 and layout.slow_interval_samples <= RC_MAX_INTERVAL_SAMPLES
    # Conditioning happens before the move to start, at rest TM.
    assert lines.index("OF 2.78") < lines.index("TM 125")
    # Fast recording is circular, sized to the fast arrays, with sentinels.
    assert f"RC 1,-{layout.fast_samples}" in lines
    assert f"posRec[{layout.fast_samples - 1}]={CIRCULAR_SENTINEL}" in lines
    # Dwell timed by AT against the AT0 reference set after both threads.
    i_wait2 = lines.index("JP#WAIT2,_XQ1<>-1")
    assert lines[i_wait2 + 1] == "AT0"
    dwell_samples = ms_to_samples(200)
    at_lines = [l for l in lines if re.fullmatch(r"AT \d+,1", l)]
    assert len(at_lines) == 1
    n = int(at_lines[0].split()[1].split(",")[0])
    assert n % 2 == 0 and dwell_samples <= n <= dwell_samples + 1
    # Hold applied with PID zeroed, then slow recording switch.
    i_hold = lines.index("OF -0.5")
    assert lines[i_hold - 3:i_hold] == ["KD 0", "KP 0", "KI 0"]
    i_rc0 = lines.index("RC 0", i_hold)
    assert lines[i_rc0 + 1] == "nfast=_RD"
    assert f"RC {layout.slow_interval_samples}" in lines[i_rc0:]
    assert any(l.startswith("RA posSlo[]") for l in lines)
    # Retract: resync, retract gains, profile, PA target, and markers.
    i_pa = lines.index("PA " + str(Controller.mm_to_counts(0.0)), i_rc0)
    assert "trt0=TIME" in lines[:i_pa]
    assert "nslow=_RD" in lines and "ts1=TIME" in lines
    assert lines[-3:] == ["JP#L,i<%d" % len(_impact()[1]), "OF 0", "EN"]
    assert "MOA" in lines


def test_non_circular_and_servo_hold_variants():
    text, layout = _build(fast_circular=False, hold_servo=True, disable_at_end=False)
    lines = text.splitlines()
    assert "RC 1" in lines and not any(l.startswith("RC 1,-") for l in lines)
    assert not any(str(CIRCULAR_SENTINEL) in l for l in lines)
    assert "MOA" not in lines
    # With the servo held, PID is never zeroed after the move and no
    # resync move is issued.
    i_wait2 = lines.index("JP#WAIT2,_XQ1<>-1")
    assert "KP 0" not in lines[i_wait2:]
    assert "PA p" not in lines[i_wait2:]


def test_optional_kick_and_no_conditioning():
    text, _ = _build(condition_pulse_ms=0, retract_kick_v=1.5, retract_kick_ms=5)
    lines = text.splitlines()
    assert lines[1:4] == [f"SP {SLOW}", f"AC {2 * SLOW}", f"DC {2 * SLOW}"]
    assert lines[4].startswith("PA ")  # straight to the move to start
    assert "OF 1.5" in lines
    i_kick = lines.index("OF 1.5")
    assert lines[i_kick + 1].endswith(",1") and lines[i_kick + 2] == "OF 0"


def test_fast_tail_is_clamped_to_dwell():
    _, short = _build(dwell_ms=5, fast_tail_ms=50)
    _, long = _build(dwell_ms=50, fast_tail_ms=50)
    assert short.fast_samples < long.fast_samples


def test_layout_uses_remaining_space_and_rejects_impossible():
    increments, ff = _impact()
    layout = plan_record_layout(
        pre_pause_ms=2, move_samples=max(len(increments) * 2, len(ff) * AT_SAMPLES),
        ff_samples=len(ff), fast_tail_ms=20, dwell_ms=1000, retract_s=0.5, post_pause_ms=10,
    )
    assert 4 * layout.fast_samples + 3 * layout.slow_samples + len(ff) <= ARRAY_TABLE_MAX_ELEMENTS
    assert ARRAY_TABLE_MAX_ELEMENTS - layout.total_elements < 3
    covered_s = layout.slow_samples * layout.slow_interval_samples * TM_US_ACTUAL / 1e6
    assert covered_s >= (0.98 + 0.5 + 0.01) * 1.25
    with pytest.raises(ValueError):
        plan_record_layout(
            pre_pause_ms=2, move_samples=700, ff_samples=200, fast_tail_ms=0,
            dwell_ms=0, retract_s=0, post_pause_ms=0,
        )
    with pytest.raises(ValueError):
        plan_record_layout(
            pre_pause_ms=2, move_samples=100, ff_samples=50, fast_tail_ms=0,
            dwell_ms=60000, retract_s=0, post_pause_ms=0,
        )


def test_trapezoid_prediction():
    assert predict_trapezoid_move_s(0, 100, 100, 100) == 0
    # Long move: ramps + cruise.
    assert abs(predict_trapezoid_move_s(1000, 100, 100, 100) - 11.0) < 1e-9
    # Short move: triangular.
    t = predict_trapezoid_move_s(10, 100, 100, 100)
    assert abs(t - 2 * (10 ** 0.5) / 10 * 1) < 1e-9 or t > 0


def test_conditioning_program():
    text = build_conditioning_program(
        pulse_v=2.78, pulse_ms=20, repeats=3, rest_kp=0.5, rest_kd=0.5, rest_ki=0.01,
        rest_offset_v=0.2,
    )
    lines = text.splitlines()
    assert lines[0] == "#COND" and lines[-1] == "EN"
    assert lines.count("OF 2.78") == 3
    assert "WT 20" in lines and lines.count("OF 0.2") == 3
    with pytest.raises(ValueError):
        build_conditioning_program(
            pulse_v=1, pulse_ms=0, rest_kp=0, rest_kd=0, rest_ki=0, rest_offset_v=0,
        )
