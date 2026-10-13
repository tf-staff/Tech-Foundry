"""Drives both GUIs end to end against the stub gclib and a headless
fake tkinter: connect, poll, arm, fire, read back, save, plot."""

import json

import pytest

import fake_tk

fake_tk.install()

import tkinter as tk  # noqa: E402  (the fake)

from controller.controller import Controller  # noqa: E402
from program.sequence_program import FAST_ARRAY_NAMES, SLOW_ARRAY_NAMES  # noqa: E402
from sequence.profile import save_profile  # noqa: E402
from test_profile_and_sequence import example_profile  # noqa: E402


def _prime_controller_for_run(g, compiled, n_fast=60, n_slow=12):
    """Makes the stub controller look like a sequence that ran: the
    program has ended, arrays hold a plausible ramp, markers are set."""
    g.responses["MG _XQ0"] = "-1"
    g.responses["MG _MOA"] = "0"  # servo on
    g.responses["MG _BGA"] = "0"  # motion complete
    g.responses["TP"] = str(Controller.mm_to_counts(0.0))
    g.responses["MG @AN[1]"] = "8.0"
    g.responses["MG _TTA"] = "0.1"
    nf, ns = compiled.layout.fast_samples, compiled.layout.slow_samples
    ramp = [Controller.mm_to_counts(39.7 * i / n_fast) for i in range(n_fast)]
    for name in FAST_ARRAY_NAMES:
        g.arrays[name] = (ramp + [0] * nf)[:nf]
        g.arrays[name][-1] = -100000
    for name in SLOW_ARRAY_NAMES:
        g.arrays[name] = ([ramp[-1]] * n_slow + [0] * ns)[:ns]
    g.variables.update({
        "nfast": n_fast, "nslow": n_slow, "tf0": 0, "tf1": n_fast, "tdw0": n_fast - 10,
        "ts0": n_fast + 2, "trt0": n_fast + 1700, "ts1": n_fast + 2500,
    })


@pytest.fixture(autouse=True)
def clean_scheduler():
    fake_tk.SCHEDULER.pending.clear()
    yield
    fake_tk.SCHEDULER.pending.clear()


def test_testing_gui_sequence_arm_fire(tmp_path, monkeypatch):
    import gui.app as app_module
    from gui import sequence_panel

    monkeypatch.setattr(app_module, "CAPTURES_DIR", tmp_path / "captures")
    monkeypatch.setattr(sequence_panel, "PROFILES_DIR", tmp_path / "profiles")
    root = tk.Tk()
    app = app_module.App(root)
    panel = app.sequence_panel
    g = app.controller._g
    g.responses["MG _MOA"] = "0"
    g.responses["MG _XQ0"] = "-1"
    g.responses["TP"] = str(Controller.mm_to_counts(0.0))

    app._on_connect()
    assert fake_tk.SCHEDULER.run_pending(rounds=1) is False  # poll re-queues itself
    assert panel.arm_button["state"] == "normal" and panel.fire_button["state"] == "disabled"
    assert app.conditioning_controls.mode2_button["state"] == "normal"

    # Fill the Current Profile fields from the example profile.
    panel.apply_profile_to_fields(example_profile())
    assert app.current_profile_velocity_var.get() == "4.0"
    panel._on_check_plan()
    assert "array elements" in panel.plan_var.get()

    # Save / load round trip through the (stubbed) file dialogs.
    path = tmp_path / "profiles" / "p.json"
    monkeypatch.setattr(tk.filedialog, "asksaveasfilename", lambda **kw: str(path))
    panel._on_save_profile()
    assert path.exists()
    app.current_profile_mass_var.set("0.5")
    monkeypatch.setattr(tk.filedialog, "askopenfilename", lambda **kw: str(path))
    panel._on_load_profile()
    assert app.current_profile_mass_var.get() == "0.007"

    panel._on_arm()
    assert panel._busy
    assert fake_tk.SCHEDULER.run_pending(rounds=3) is False
    assert panel._armed is not None
    assert panel.fire_button["state"] == "normal"
    assert g.programs[-1] == panel._armed.program_text
    assert "ffProf" in g.arrays

    _prime_controller_for_run(g, panel._armed)
    panel._on_fire()
    assert "XQ #MAIN" in g.commands
    fake_tk.SCHEDULER.run_pending(rounds=3)
    assert "Sequence captured" in app.status_var.get(), app.status_var.get()
    saved = list((tmp_path / "captures").glob("sequence_*.json"))
    assert len(saved) == 1
    doc = json.loads(saved[0].read_text())
    assert len(doc["fast"]["position_mm"]) == 60 and len(doc["slow"]["position_mm"]) == 12
    assert (tmp_path / "captures").glob("*.png")
    assert panel.fire_button["state"] == "disabled"

    # Disable invalidates an armed sequence.
    panel._on_arm()
    fake_tk.SCHEDULER.run_pending(rounds=3)
    assert panel._armed is not None
    app._on_disable()
    assert panel._armed is None and panel.fire_button["state"] == "disabled"

    # Conditioning button: pulse into the retract stop (position ~0).
    app.conditioning_controls._run(10.0, 20.0, 1)
    assert g.programs[-1].startswith("#COND")
    assert "XQ #COND" in g.commands
    fake_tk.SCHEDULER.run_pending(rounds=3)
    assert "Conditioning pulse done" in app.status_var.get()
    app._on_disconnect()


def test_user_gui_full_run(tmp_path, monkeypatch):
    import gui.user_app as user_app

    monkeypatch.setattr(user_app, "SETTINGS_DIR", tmp_path / "settings")
    monkeypatch.setattr(user_app, "SETTINGS_PATH", tmp_path / "settings" / "user_settings.json")
    monkeypatch.setattr(user_app, "CALIBRATION_PATH", tmp_path / "settings" / "lvdt_calibration.json")
    profiles_dir = tmp_path / "profiles"
    data_dir = tmp_path / "data"
    save_profile(example_profile(), profiles_dir / "19mm_4ms_14mm.json")
    second = example_profile()
    second.update({"name": "19mm_4ms_10mm", "impact_depth_mm": 10.0})
    second["motion"]["impact_position_mm"] = 29.74
    second["motion"]["peak_velocity_position_mm"] = 29.74
    save_profile(second, profiles_dir / "19mm_4ms_10mm.json")
    (profiles_dir / "broken.json").write_text("{}")
    user_app.save_settings({
        "address": "10.0.0.5", "profiles_dir": str(profiles_dir), "data_dir": str(data_dir),
        "amp_full_scale_current_a": 36.0, "stroke_mm": 39.85,
    })

    root = tk.Tk()
    app = user_app.UserApp(root)
    assert app.tip_combo["values"] == ["19 mm"]
    assert len(app.profile_combo["values"]) == 2
    assert "broken.json" in app.status_var.get()
    assert app.address_var.get() == "10.0.0.5"
    assert Controller.get_amp_full_scale_current() == 36.0

    g = app.controller._g
    g.responses["MG _MOA"] = "1"  # servo off initially
    g.responses["MG _XQ0"] = "-1"
    g.responses["TP"] = str(Controller.mm_to_counts(0.0))
    app._on_connect()
    fake_tk.SCHEDULER.run_pending(rounds=1)
    assert app.arm_button["state"] == "normal" and app.fire_button["state"] == "disabled"
    assert app.servo_var.get() == "Servo: OFF"

    # Pick the 14 mm profile explicitly (profiles list alphabetically).
    from sequence.profile import profile_label
    app.profile_var.set(profile_label(example_profile()))
    app._on_profile_selected()
    assert app.selected_profile()["impact_depth_mm"] == 14.0

    # Go to impact position auto-enables and moves to stop - depth.
    app._on_go_to_impact()
    assert "SHA" in g.commands
    g.responses["MG _MOA"] = "0"
    assert g.commands[-1] == "BGA" and g.commands[-2] == f"PA {Controller.mm_to_counts(25.74)}"
    fake_tk.SCHEDULER.run_pending(rounds=3)
    assert "complete" in app.status_var.get()

    # Dwell validation.
    app.dwell_var.set("5000")
    app._on_arm()
    assert "Dwell time must be" in app.status_var.get()
    app.dwell_var.set("350")
    app._on_arm()
    fake_tk.SCHEDULER.run_pending(rounds=3)
    assert app._armed is not None and app._armed.dwell_ms == 350
    assert app.fire_button["state"] == "normal"

    _prime_controller_for_run(g, app._armed)
    app._on_fire()
    fake_tk.SCHEDULER.run_pending(rounds=3)
    assert "Run complete" in app.status_var.get(), app.status_var.get()
    assert app.last_velocity_var.get().endswith("(nominal 4.0 m/s)")
    assert "350" in app.last_dwell_var.get()
    files = sorted(p.name for p in data_dir.iterdir())
    assert any(f.endswith("_impact.png") for f in files) and any(f.endswith("_sequence.png") for f in files)
    assert any(f.endswith(".csv") for f in files) and any(f.endswith(".json") for f in files)
    assert app.view_plot_button["state"] == "normal"
    app._on_view_plot()

    # Calibration walks retract -> pulse -> average -> extend -> pulse -> average.
    g.responses["TP"] = lambda: str(Controller.mm_to_counts(0.0)) if "PA " + str(Controller.mm_to_counts(39.85)) not in g.commands else str(Controller.mm_to_counts(39.85))
    app._on_calibrate()
    assert fake_tk.SCHEDULER.run_pending(rounds=200) is False  # poll loop still alive
    assert app.status_var.get() == "Calibration complete and saved", app.status_var.get()
    cal = json.loads((tmp_path / "settings" / "lvdt_calibration.json").read_text())
    assert len(cal["points"]) == 2 and cal["points"][1][1] == 39.85
    assert sum(1 for p in g.programs if p.startswith("#COND")) == 2

    # Settings dialog round trip.
    app._on_settings()
    assert app.status_var.get() == "Calibration complete and saved"
    app._on_help()
    app._on_disable()
    app._on_disconnect()
    assert app.status_var.get().startswith("Disconnected")
