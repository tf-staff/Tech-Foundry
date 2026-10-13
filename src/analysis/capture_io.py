"""Saving/loading sequence captures as files -- the "Data filepath" in
the user GUI's Last Fire panel, and the record a calibration run in the
testing GUI leaves behind.

Two files per run, same stem:
  <stem>.json  everything: the compiled profile, record layout, raw
               counts for both segments, timestamps/markers, converted
               traces, metrics, calibration. Loadable back into a
               figure (load_capture).
  <stem>.csv   the converted traces only, one row per sample, both
               segments concatenated with a `segment` column -- for
               spreadsheets.

Pure file I/O -- no Tkinter, no gclib.
"""

import csv
import datetime
import json
from pathlib import Path

from controller.controller import Controller


def convert_segment(position, commanded, torque, error=None) -> dict:
    """Raw counts -> mm / mm / V / A (/ mm) through the live
    calibration + amp scale, the same conversions app.py's
    _show_capture applies."""
    torque_v = [Controller.torque_counts_to_volts(float(t)) for t in torque]
    out = {
        "position_mm": [Controller.counts_to_mm(float(p)) for p in position],
        "commanded_position_mm": [Controller.counts_to_mm(float(p)) for p in commanded],
        "torque_v": torque_v,
        "current_a": [Controller.volts_to_current(v) for v in torque_v],
    }
    if error is not None:
        out["position_error_mm"] = [Controller.counts_delta_to_mm(float(e)) for e in error]
    return out


def capture_document(recording, compiled, metrics=None, extra=None) -> dict:
    """The JSON-able record of one run. `recording` is a
    recording.sequence_recording.SequenceRecording, `compiled` a
    sequence.impact_sequence.CompiledSequence."""
    fast = convert_segment(
        recording.fast.position, recording.fast.commanded_position,
        recording.fast.torque, recording.fast.position_error,
    )
    slow = convert_segment(recording.slow.position, recording.slow.commanded_position, recording.slow.torque)
    layout = recording.layout
    doc = {
        "format": "cci-impact-capture",
        "version": 1,
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        "profile": compiled.profile,
        "dwell_ms": compiled.dwell_ms,
        "impact_position_mm": compiled.impact_position_mm,
        "layout": dict(layout._asdict()),
        "markers": recording.markers,
        "fast_wrapped": recording.fast_wrapped,
        "dwell_start_s": recording.dwell_start_s,
        "retract_start_s": recording.retract_start_s,
        "fast": {
            "times_s": list(recording.fast_times_s),
            "raw": {
                "position": list(recording.fast.position),
                "commanded_position": list(recording.fast.commanded_position),
                "position_error": list(recording.fast.position_error),
                "torque": list(recording.fast.torque),
            },
            **fast,
            "feedforward_v": list(compiled.ff_plot_v[: len(recording.fast.position)]),
        },
        "slow": {
            "times_s": list(recording.slow_times_s),
            "raw": {
                "position": list(recording.slow.position),
                "commanded_position": list(recording.slow.commanded_position),
                "torque": list(recording.slow.torque),
            },
            **slow,
        },
        "metrics": metrics or {},
        "calibration_points": [list(p) for p in Controller.calibration_points()],
        "amp_full_scale_current_a": Controller.get_amp_full_scale_current(),
    }
    if extra:
        doc.update(extra)
    return doc


def save_capture(doc: dict, directory, stem=None) -> Path:
    """Writes <stem>.json and <stem>.csv into `directory`; returns the
    JSON path. Default stem: sequence_<timestamp>."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if stem is None:
        stem = "sequence_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = directory / f"{stem}.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
    with open(directory / f"{stem}.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "segment", "time_s", "position_mm", "commanded_position_mm",
            "position_error_mm", "torque_v", "current_a", "feedforward_v",
        ])
        fast = doc["fast"]
        for i, t in enumerate(fast["times_s"]):
            writer.writerow([
                "fast", f"{t:.6f}", fast["position_mm"][i], fast["commanded_position_mm"][i],
                fast["position_error_mm"][i], fast["torque_v"][i], fast["current_a"][i],
                fast["feedforward_v"][i] if i < len(fast["feedforward_v"]) else "",
            ])
        slow = doc["slow"]
        for i, t in enumerate(slow["times_s"]):
            writer.writerow([
                "slow", f"{t:.6f}", slow["position_mm"][i], slow["commanded_position_mm"][i],
                "", slow["torque_v"][i], slow["current_a"][i], "",
            ])
    return json_path


def load_capture(path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        doc = json.load(f)
    if doc.get("format") != "cci-impact-capture":
        raise ValueError(f"{path}: not a capture file")
    return doc
