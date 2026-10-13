"""Smoke tests for the pre-existing pure modules, so later changes can't
silently break the Current Profile pipeline the sequence work builds on."""

from controller.controller import ARRAY_TABLE_MAX_ELEMENTS, Controller
from program.current_profile_program import (
    AT_SAMPLES, TM_US_ACTUAL, build_current_profile_program,
)
from program.current_profile_trajectory import (
    compute_current_profile_curve, compute_feedforward_v, sample_position_mm,
)


def _increments(curve, start_mm):
    cd_period_s = 2 * TM_US_ACTUAL / 1e6
    positions = [Controller.mm_to_counts(start_mm)] + [
        Controller.mm_to_counts(p) for p in sample_position_mm(curve, cd_period_s)
    ]
    return [positions[i] - positions[i - 1] for i in range(1, len(positions))]


def test_curve_hits_impact_velocity_at_impact_position():
    curve = compute_current_profile_curve(0.0, 27.74, 27.74, 39.74, 4.0)
    assert abs(curve.phase2.v0 - 4.0) < 1e-6


def test_program_builds_and_fits_array_budget():
    curve = compute_current_profile_curve(0.0, 25.0, 25.0, 39.74, 4.0)
    increments = _increments(curve, 0.0)
    ff = compute_feedforward_v(curve, AT_SAMPLES * TM_US_ACTUAL / 1e6, 0.007, 1.0, 0.6)
    text, record_samples = build_current_profile_program(
        start_counts=Controller.mm_to_counts(0.0), slow_speed_counts=100,
        slow_accel_counts=100, slow_decel_counts=100, pre_pause_ms=2,
        motion_kp=0, motion_kd=0, motion_ki=0, motion_offset_v=0,
        increments=increments, ff_values=ff, post_pause_ms=2,
        rest_kp=0.1, rest_kd=0.1, rest_ki=0.002, rest_offset_v=0.04,
    )
    assert text.startswith("#MAIN\n")
    assert "XQ #LOOP,1" in text
    assert 4 * record_samples + len(ff) <= ARRAY_TABLE_MAX_ELEMENTS
