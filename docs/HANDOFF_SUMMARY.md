# Handoff summary -- October 2026 work

Companion to `docs/reference/project_handoff_notes.txt`. This file records
what was built against the three deliverables in those notes, how it works,
and -- most importantly -- what has **not** been verified on hardware. None of
this work has run on the bench: there was no controller available, so every
controller-facing claim below is "per the Galil firmware reference" or "per
the existing bench-confirmed code", not "observed".

## 1. Testing GUI: pre-retract, conditioning, dwell, retract, full recording

### What was added

A new **Impact Sequence** panel (`src/gui/sequence_panel.py`) sits directly
under the Current Profile Motion section and reuses its fields by reference.
The existing Current Profile Arm/Fire is untouched. The panel's own fields:

| Group | Fields | Default |
| --- | --- | --- |
| LVDT condition pulse | enabled, current (A, + = into retract stop), duration, settle, position | on, 10 A, 20 ms, 50 ms, 0 mm |
| Dwell | duration (100-1000 ms), hold current (A, + = into extend stop), keep position loop, fast capture tail | 200 ms, 0 A, off, 20 ms |
| Retract | optional open-loop kick (A, ms), retract PID + offset, speed/accel/decel, target | no kick, Rest gains, 0.1 m/s 0.2 m/s², 2 mm |
| Recording | rolling (circular) fast capture, disable servo at end | on, on |

Buttons: **Check Plan** (compiles and prints the program size / recording
layout without touching the controller), **Arm Sequence**, **Fire Sequence**,
**Save Profile...**, **Load Profile...**.

Two buttons were also added to the LVDT Calibration section: **Condition LVDT
(mode 2)** and **Restore mode 1**. Each pulses into whichever hard stop the
striker is nearest (sign chosen from the live position) with the position loop
zeroed, then resyncs and restores the Rest gains, leaving the servo enabled so
the calibration points can be captured in mode 2 straight away.

### The sequence program (`src/program/sequence_program.py`)

One downloaded program, thread 0, with the Current Profile impact exactly as
before (thread 1 plays the feedforward array into OF). In order:

1. Slow move to the conditioning position (default 0 mm, the retract stop),
   PID zeroed, pulse `OF` for the pulse duration, release, settle, resync
   (`p=_TPA; PA p`), Rest gains back, settle. All at the Rest sample period.
2. Slow move to Start, `TM 125`, dimension all seven recording arrays, write
   a sentinel into the last element of each fast array, start the fast
   recording (`RC 1,-n`, circular).
3. Pre-pause, Motion gains, `CMA`/`DT 1`, launch `#LOOP` on thread 1, the CD
   points, wait for the contour and for thread 1 (unchanged).
4. **Dwell**: `AT0` (time reference), PID zeroed unless "keep position loop",
   `OF hold`, wait the fast tail, stop the fast recording and store `_RD` and
   `TIME`, switch `RA`/`RD` to the three slow arrays, `RC k`, then
   `AT n,1` -- the dwell ends n samples after the reference regardless of how
   long the recording switch took. Resolution 2 samples (~244 µs).
5. **Retract**: `OF 0`, optional kick, resync (if the loop was open), retract
   gains, `SP/AC/DC`, `PA target`, `BGA`, `AMA`, post-pause, stop the slow
   recording, store `_RD`/`TIME`, Rest settings, `TM 1000`, settle, `MOA`.

The program stamps `TIME` (servo-sample clock) at every recording boundary,
at the dwell start and at the retract start into controller variables that
the host reads back (`MG tf0` etc.) to put both segments on one time axis and
to measure the dwell actually delivered.

### Recording within the 3000-element array table

* **Fast segment** -- the four existing arrays, one sample per servo tick
  (122 µs), sized to pre-pause + impact move + fast tail (+5%). It is a
  **rolling buffer** (`RC 1,-n`): if the move runs long, the pre-pause is
  overwritten instead of the rebound being cropped. The host detects whether
  the ring filled by the sentinel in the last element and unwraps using the
  stored `_RD` (`src/recording/sequence_recording.py`).
* **Slow segment** -- three arrays (`_TPA`, `_RPA`, `_TTA`; following error
  is derivable) at `RC k` servo samples, sized to *all remaining* array
  space, with k the smallest interval covering the predicted rest of the
  dwell + retract + post-pause (+25% and 50 ms). For the example profile with
  a 1 s dwell that is 451 samples at 5.4 ms; with a 200 ms dwell, ~1.8 ms.
* All arrays are dimensioned at Arm time with a full `DA *[0]` reset
  (`Controller.size_arrays`), then the feedforward array is downloaded, so the
  program's own `DM` lines are same-size redeclarations -- the pattern the
  notes say is the only reliable one.

Every run writes `captures/sequence_<ts>.json` (everything, loadable) and
`.csv` (converted traces, both segments), plus the usual impact PNG and a new
whole-sequence PNG with the dwell/retract boundaries marked.

### Motion strategy choices (the open questions in the notes)

* **Pre-retract / conditioning**: the sequence conditions the sensor to mode 2
  deliberately, by pulsing into the retract stop before every run, as the
  notes recommend. The pulse happens *before* the move to Start so Start can
  be anywhere.
* **Hold at full extension**: default is open loop -- PID zeroed and a
  constant hold current pressing into the extend stop for the dwell; the hard
  stop defines depth, the current defines the holding force. The alternative
  (keep the Motion PID with the commanded position at the last CD point, hold
  current as a bias) is one checkbox.
* **Post-retract**: an ordinary closed-loop `PA` move under its own gains and
  speed profile, after an optional open-loop kick. It runs at `TM 125` (not
  the Rest 1000 µs), so Rest gains behave differently than the Slow Retract
  button -- the Retract PID fields exist for that reason.

## 2. User GUI (`src/gui/user_app.py`, `python run_user_gui.py`)

Follows `docs/images/gui_sketch.jpg`: big DISABLE on top; IP/Connect/
Disconnect; status; position and current readout; left column Arm, FIRE,
Go to Impact Position, Retract, Extend, Calibrate Position; right column Tip
/ Impact profile / Dwell time selection and the Last Fire panel (tip,
profile, measured impact velocity, peak position vs nominal depth, measured
dwell, rebound, View Plot, data file path); Settings and Help at the bottom.

* Profiles are read from the profiles directory (Settings) and grouped by tip.
* The servo is enabled automatically by the motion buttons under the selected
  profile's Rest gains; DISABLE aborts everything.
* "Go to Impact Position" moves slowly to Stop − depth so the tip can be
  positioned on the brain surface.
* Arm compiles the selected profile with the chosen dwell and runs the same
  `prepare_controller` as the testing GUI; Fire runs the program, uploads,
  computes metrics, and saves JSON + CSV + two PNGs under the data directory
  as `<profile>_<timestamp>.*`.
* Calibrate Position: retract stop → 10 A pulse into it → re-seat → average
  20 readings → 0 mm; extend stop (stroke from Settings) → pulse → re-seat →
  average → stroke mm; saved to `settings/lvdt_calibration.json` and reloaded
  at start-up. Only a two-point calibration, as the notes decided.
* Settings persist in `settings/user_settings.json` (IP, directories, amp
  full-scale current, stroke).

## 3. Integration

`src/sequence/profile.py` defines the file format (JSON, every key has a
default, validation with readable messages). `src/sequence/impact_sequence.py`
compiles a profile into a `CompiledSequence` (program text, feedforward array,
record layout, plot overlay, info text) and drives the controller steps
(`prepare_controller`, `fetch_sequence_recording`). Both GUIs call only
these. The compiler is looked up by `profile["motion"]["mode"]` in
`COMPILERS`; a different motion program plugs in as a new compiler function
returning a `CompiledSequence`, with no GUI changes on either side. The
testing GUI's panel is a separate module so the main `app.py` diff is a
dozen lines of wiring (import, two constructor calls, button gating).

Amps→volts conversions inside a compiled profile use the *profile's* amp
full-scale current, not the app's current field, so a profile runs the way it
was developed even if the display setting differs.

## Tests

`MPLBACKEND=Agg python -m pytest -q` -- 24 tests: program builders, record
layout planning, ring unwrap, profile round trip, compile + controller command
ordering against the stub gclib, metrics and capture files, and both GUIs
driven end to end through a fake Tkinter (`tests/fake_tk.py`). No hardware or
display needed.

## Things I was not sure about (please check these on the bench first)

1. **Circular recording (`RC 1,-n`)** has never been used in this project.
   The firmware reference documents it, but the unwrap relies on two
   assumptions: `_RD` is still valid after `RC 0`, and the sentinel in the last
   element is overwritten only once the ring has filled. If anything looks
   rotated or truncated, untick "Rolling (circular) fast capture" -- the
   non-circular path is the proven one and is one flag away everywhere.
2. **Switching `RA`/`RD`/`RC` mid-program** to the slow arrays after `RC 0`
   is also new. The reference forbids `DM`/`DA` while recording (we don't) but
   says nothing about re-issuing `RA`/`RD` after a stop; I assume it works.
3. **`AT0` / `AT n,1` in thread 0 after thread 1 has used AT.** I assumed the
   AT reference is per thread or at least safe once thread 1 has ended. If the
   dwell comes out wrong, replace the `AT n,1` with a `WT n,1` (the switch
   overhead, ~0.5 ms, would then add to the dwell).
4. **Seven user arrays plus the feedforward array (eight total).** The manual
   says the record mode takes "up to eight arrays"; I found no documented
   limit on the number of user arrays, only the 3000-element total.
5. **`TIME` as a sample counter at `TM 125`.** The reference says it counts
   servo updates, so the time axes use 122 µs per count (the bench-measured
   period). If the stamps disagree with the recorded sample counts, that
   assumption is wrong.
6. **Conditioning pulse while the loop is open.** During the pulse and its
   settle the position loop is off; on the vertical mount gravity may pull
   the striker off the retract stop. The sequence then re-commands Start, and
   the calibration re-seats before reading, so it should not matter, but the
   notes' null-shift effect (current flowing shifts the reading) means the
   calibration is still taken under the Rest holding current, as before.
7. **Hold current default is 0 A** in `default_profile()` (the example profile
   uses 2 A). I had no basis for a value; it is the knob the notes call "the
   hold strategy at full extension".
8. **Retract at `TM 125`.** The retract runs at the impact sample period, so
   Rest gains act differently than during the Slow Retract button's move at
   1000 µs. If it is sluggish/oscillatory, tune the Retract PID fields; the
   alternative is to switch `TM` back to 1000 before the retract, which would
   change the slow recording interval math.
9. **Example profile** (`profiles/example_19mm_4ms_14mm.json`) copies the
   trajectory numbers visible in the September captures (mass 0.007, Kf
   1.0/0.6, stretch 2.8/0.4/0.4/0.4, advance 3000 µs, Motion PID 0, amp 36 A).
   Everything else in it is a default. Treat it as a template, not a
   calibration.
10. **Slow-segment duration prediction** uses a trapezoid estimate of the
    retract move plus margins. If the retract settles slowly the end of the
    slow segment is cropped (non-circular); the whole-sequence plot will show
    it. The `SLOW_SEGMENT_MARGIN`/`SLOW_SEGMENT_EXTRA_MS` constants are the
    knobs.
11. The user GUI could not be exercised with a real display here (no Tkinter
    in this environment); it runs end to end against the fake Tkinter in the
    tests, which checks logic and widget wiring but not layout. Expect
    cosmetic fixes.

## Not done / ideas

* A real current measurement (amp current monitor into analog input 2) is
  still the highest-value instrumentation change; the slow segment's third
  channel could carry it instead of `_RPA`.
* Host-side DR data-record streaming would remove the array-table limit for
  the slow part entirely, at the cost of host timing; not attempted.
* The other testing-GUI modes (open loop, contour, PVT, simple open loop) are
  untouched and still share program memory with the sequence: arming one
  mode then firing another runs the wrong program, as before.
