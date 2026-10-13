# Pig CCI Device Controller

Host software for the voice-coil-driven pig CCI (controlled cortical impact)
instrument: a Galil DMC-31011 motion controller driving a Copley Xenus
amplifier in current mode, with an LVDT as the only position sensor.

Two applications share one code base:

| Launcher | Purpose |
| --- | --- |
| `python run_testing_gui.py` | Development/testing GUI (`src/gui/app.py`). Every motion mode, every tunable, plus the **Impact Sequence** panel that develops a full run and saves it as a profile file. |
| `python run_user_gui.py` | Operator GUI (`src/gui/user_app.py`). Picks a tip and a saved profile, sets the dwell, runs the sequence, records the result. |

The hand-off between them is a JSON **profile** (`profiles/*.json`, schema in
`src/sequence/profile.py`), one per tip / velocity / depth combination.
Both GUIs compile a profile into the same controller program through
`src/sequence/impact_sequence.py`.

Start with `docs/HANDOFF_SUMMARY.md` (what was done in October 2026 and what
is still unverified on hardware) and `docs/reference/project_handoff_notes.txt`
(the project background).

## Layout

```
src/controller/   gclib wrapper, unit conversions, LVDT calibration state
src/program/      pure DMC program builders (one per motion mode) + trajectory math
                  sequence_program.py   full impact sequence + two-segment recording
                  conditioning_program.py  LVDT mode-2 / mode-1 pulse
src/recording/    host-side waiting helpers, sequence capture assembly/unwrap
src/sequence/     profile files, profile -> program compiler, Arm/Fetch steps
src/analysis/     capture figures, sequence metrics, capture JSON/CSV files
src/gui/          app.py (testing GUI), sequence_panel.py, user_app.py, figure_popup.py
src/headless/     bench scripts used to establish the controller timing facts
profiles/         saved impact profiles (an example is included)
tests/            pytest suite; runs without hardware or a display
docs/             handoff summary, reference notes, GUI sketch, capture images
```

Runtime output: `captures/` (testing GUI), `data/` and `settings/` (user GUI).
All three are git-ignored.

## Requirements

Python 3.10+, `numpy`, `scipy`, `matplotlib`, Tkinter, and Galil's `gclib`
Python binding on the bench machine (`pip install -r requirements.txt` covers
everything but gclib and Tkinter).

## Tests

```
pip install pytest
MPLBACKEND=Agg python -m pytest -q
```

`tests/conftest.py` stubs `gclib`; `tests/fake_tk.py` stands in for Tkinter so
both GUIs are driven end to end (connect, arm, fire, read back, save) with no
controller and no display.
