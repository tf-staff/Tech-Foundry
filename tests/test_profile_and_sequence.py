import json

import pytest

from analysis.capture_io import capture_document, load_capture, save_capture
from analysis.sequence_metrics import format_metrics, impact_metrics
from controller.controller import Controller
from program.current_profile_program import FF_ARRAY_NAME
from program.sequence_program import CIRCULAR_SENTINEL, FAST_ARRAY_NAMES, SLOW_ARRAY_NAMES
from sequence import impact_sequence
from sequence.profile import (
    DWELL_MAX_MS, default_profile, list_profiles, load_profile, merge_defaults,
    profile_label, save_profile, validate_profile,
)


def example_profile():
    p = default_profile()
    p.update({
        "name": "19mm_4ms_14mm", "tip": "19 mm", "impact_velocity_m_s": 4.0,
        "impact_depth_mm": 14.0,
    })
    p["motion"].update({
        "stop_mm": 39.74, "impact_position_mm": 25.74, "peak_velocity_position_mm": 25.74,
        "mass_kg": 0.007, "kf_accel": 1.0, "kf_decel": 0.6,
    })
    p["sequence"]["dwell"]["hold_current_a"] = 2.0
    p["amp"]["full_scale_current_a"] = 36.0
    return p


def test_default_profile_is_incomplete_and_example_is_valid():
    assert validate_profile(default_profile())
    assert validate_profile(example_profile()) == []


def test_validation_catches_ordering_and_missing():
    p = example_profile()
    p["motion"]["impact_position_mm"] = 45.0
    errors = validate_profile(p)
    assert any("Impact position" in e for e in errors)
    p = example_profile()
    del p["motion"]["mass_kg"]
    assert any("Mass" in e for e in validate_profile(p))


def test_merge_defaults_fills_missing_sections():
    p = merge_defaults({"name": "x", "sequence": {"dwell": {"duration_ms": 300}}})
    assert p["sequence"]["dwell"]["duration_ms"] == 300
    assert p["sequence"]["dwell"]["fast_tail_ms"] == default_profile()["sequence"]["dwell"]["fast_tail_ms"]
    assert p["motion"]["mode"] == "current_profile_sequence"


def test_save_load_roundtrip(tmp_path):
    path = tmp_path / "p.json"
    written = save_profile(example_profile(), path)
    assert written["created"] and written["calibration"]["points"]
    loaded = load_profile(path)
    assert loaded["motion"]["kf_decel"] == 0.6
    assert "19mm_4ms_14mm" in profile_label(loaded)
    (tmp_path / "bad.json").write_text("{}")
    (tmp_path / "junk.json").write_text("not json")
    listed = list_profiles(tmp_path)
    assert [p.name for p, _ in listed] == ["bad.json", "junk.json", "p.json"]
    assert isinstance(listed[0][1], ValueError) and isinstance(listed[1][1], ValueError)
    assert isinstance(listed[2][1], dict)
    with pytest.raises(ValueError):
        save_profile(default_profile(), tmp_path / "x.json")


def test_compile_sequence_and_dwell_override():
    compiled = impact_sequence.compile_sequence(example_profile(), dwell_ms=DWELL_MAX_MS)
    assert compiled.dwell_ms == DWELL_MAX_MS
    assert compiled.program_text.startswith("#MAIN")
    assert compiled.layout.total_elements <= 3000
    assert abs(compiled.peak_velocity_m_s - 4.0) < 1e-6
    assert compiled.impact_position_mm == 25.74
    # 2A at 36A full scale -> 0.556V, pushing into the extend stop (negative).
    assert "OF -0.555556" in compiled.program_text
    # 10A conditioning pulse -> +2.78V.
    assert "OF 2.777778" in compiled.program_text
    assert impact_sequence.info_text(compiled).startswith("Impact Sequence\nProfile:")
    with pytest.raises(ValueError):
        impact_sequence.compile_sequence({"motion": {"mode": "nope"}})


def test_compile_rejects_out_of_range_current():
    p = example_profile()
    p["sequence"]["dwell"]["hold_current_a"] = 50.0
    with pytest.raises(ValueError, match="Hold current"):
        impact_sequence.compile_sequence(p)


class _StubController(Controller):
    pass


def test_prepare_controller_order_and_arrays():
    compiled = impact_sequence.compile_sequence(example_profile())
    controller = _StubController()
    controller.connect("1.2.3.4")
    g = controller._g
    g.commands.clear()
    impact_sequence.prepare_controller(controller, compiled)
    cmds = g.commands
    assert g.programs[-1] == compiled.program_text
    assert cmds.index("MOA") < cmds.index("DA *[0]") < cmds.index("SHA")
    # Every recording array + the FF array dimensioned after the reset.
    dm = [c for c in cmds[cmds.index("DA *[0]"):] if c.startswith("DM ")]
    names = [c.split()[1].split("[")[0] for c in dm]
    assert set(names) == set(FAST_ARRAY_NAMES) | set(SLOW_ARRAY_NAMES) | {FF_ARRAY_NAME}
    assert names.index(FF_ARRAY_NAME) == len(names) - 1
    assert g.arrays[FF_ARRAY_NAME] == [round(v, 6) + 0.0 for v in compiled.ff_values]
    assert cmds[-1] == "BGA" and cmds[-2] == f"PA {Controller.mm_to_counts(0.0)}"


def test_fetch_recording_through_fake_controller():
    compiled = impact_sequence.compile_sequence(example_profile())
    controller = _StubController()
    controller.connect("1.2.3.4")
    g = controller._g
    nf, ns = compiled.layout.fast_samples, compiled.layout.slow_samples
    for name in FAST_ARRAY_NAMES:
        g.arrays[name] = [0] * nf
    for name in SLOW_ARRAY_NAMES:
        g.arrays[name] = [0] * ns
    g.arrays["posRec"][-1] = CIRCULAR_SENTINEL
    g.variables.update({
        "nfast": 50, "nslow": 10, "tf0": 0, "tf1": 50, "tdw0": 40, "ts0": 52, "trt0": 300, "ts1": 400,
    })
    rec = impact_sequence.fetch_sequence_recording(controller, compiled)
    assert len(rec.fast.position) == 50 and len(rec.slow.position) == 10
    assert not rec.fast_wrapped


def test_metrics_and_capture_file(tmp_path):
    compiled = impact_sequence.compile_sequence(example_profile())
    dt = compiled.layout.sample_period_s
    # Synthetic fast segment: ramp 0->39.7mm over 30ms then hold with a bounce.
    n_ramp = round(0.030 / dt)
    pos = [39.7 * (i / n_ramp) for i in range(n_ramp)] + [39.7, 39.2, 39.5, 39.7] * 20
    times = [i * dt for i in range(len(pos))]
    metrics = impact_metrics(pos, times, dt, 25.74, dwell_start_s=0.03, retract_start_s=0.23)
    assert metrics["impact_index"] is not None
    assert abs(metrics["impact_velocity_m_s"] - 39.7e-3 / 0.030) < 0.1
    assert abs(metrics["peak_position_mm"] - 39.7) < 1e-9
    assert abs(metrics["rebound_mm"] - 0.5) < 1e-9
    assert abs(metrics["dwell_measured_s"] - 0.2) < 1e-12
    assert "Impact velocity" in format_metrics(metrics)

    from controller.controller import RecordedData
    from recording.sequence_recording import SequenceRecording, SlowRecordedData
    counts = [Controller.mm_to_counts(p) for p in pos]
    fast = RecordedData(counts, counts, [0] * len(counts), [100] * len(counts))
    slow = SlowRecordedData(counts[:5], counts[:5], [50] * 5)
    rec = SequenceRecording(
        fast=fast, slow=slow, layout=compiled.layout, fast_times_s=times,
        slow_times_s=[0.05 + i * 0.002 for i in range(5)], dwell_start_s=0.03,
        retract_start_s=0.23, fast_wrapped=False, markers={"tf0": 0.0},
    )
    doc = capture_document(rec, compiled, metrics)
    path = save_capture(doc, tmp_path, stem="run1")
    assert path.name == "run1.json" and (tmp_path / "run1.csv").exists()
    loaded = load_capture(path)
    assert loaded["profile"]["name"] == "19mm_4ms_14mm"
    assert len(loaded["fast"]["position_mm"]) == len(pos)
    rows = (tmp_path / "run1.csv").read_text().splitlines()
    assert rows[0].startswith("segment,time_s") and len(rows) == 1 + len(pos) + 5
    from analysis.capture_analysis import build_sequence_figure
    figure = build_sequence_figure(loaded, "info", format_metrics(metrics))
    assert figure.axes
