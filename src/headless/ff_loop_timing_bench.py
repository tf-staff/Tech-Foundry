"""Phase 0 bench validation for Current Profile mode (see the plan file
"Current Profile Motion Mode"): confirms whether an onboard interpreted
DMC program can read one element of a host-downloaded array and write it
into OF (Offset) at a fixed real-time cadence, for the whole duration of
a move, without falling behind -- before any of the rest of that
architecture gets built on top of it. Not part of the app; run manually
against real hardware.

Structured as a staged battery, not one combined test -- earlier combined
runs failed for reasons that had nothing to do with the actual question
(a corrupted array element, then a timing quirk), and each stage below
adds exactly one new piece so the first stage that fails localizes the
fault instead of producing another ambiguous pass/fail:
  0. Array round-trip -- download the FF array, read it straight back,
     compare. Isolates whether download_array() itself works at all,
     independent of any DMC program logic.
  1. Bare loop -- i=i+1/JP, no AT/array/OF. Baseline per-iteration
     interpreter speed and a sanity check on this script's own timing
     methodology (compare against the manual's own ~82-84us/iteration
     benchmark).
  2. AT-throttled loop -- adds AT pacing, still no array/OF. Isolates
     whether AT+TM behave as expected in isolation.
  3. AT + array read -- adds "value=ffTest[i]", still no OF write.
     Isolates whether array access inside the timed loop is the issue.
  4. AT + array read + OF write -- the real target.
Runs stages 0-4 at the coarsest TM candidate first and stops at the
first failure. Only once all 5 pass does it sweep to finer TM candidates
(stage 4 only, since that's the one this mode actually needs).

BENCH-CONFIRMED FINDING (kept here since it shapes every program this
script builds): the DMC-30010's AT command's n0 argument (firmware
command reference, Arguments table) lists a Resolution of 2 -- only even
sample counts are a valid step. "AT -1,1" (an odd n0, requesting a
1-sample wait) was confirmed on the bench to silently behave as a
2-sample wait: three independent findings converged on this --
(a) two runs at different TM (500us and 1000us) both landed at
ratio~1.96 against a 1-sample-per-iteration prediction, ruling out a
TM/AT coupling bug (if AT were ignoring TM entirely, only one of the two
would have been off); (b) requesting "AT -2,1" explicitly and predicting
2 samples/iteration instead landed at ratio~0.985, matching almost
exactly. So every AT-paced program here requests 2 samples per
iteration by default (see at_samples below) -- the real achievable
per-update granularity for this whole architecture is 2*TM, not 1*TM.

Reuses Controller.download_array (controller.py) -- new host->controller
array-write primitive, the DMC QD command via gclib's GArrayDownload --
and Controller.run_program()/read_program_running() (the same "MG _XQ0"
thread-status polling already used for Contour/PVT program completion),
plus a tight host-side poll (not the GUI's 100ms schedule() cadence) so
the measured wall-clock duration is precise enough to compare against
the predicted duration.

SAFETY: per Controller.disable()'s own comment, OF is a persistent bias
on the command output that keeps driving real current through the
amplifier regardless of servo enable state -- it is NOT inert just
because the axis isn't mid-move. This script:
  - calls enable() (Galil "SH", Servo Here) right before each run, which
    holds the axis closed-loop at wherever it physically already is (no
    separate move-to-position needed) -- so an OF bias gets fought by
    the position PID instead of driving free motion,
  - uses a small-amplitude synthetic feedforward array (+/-0.5V sine),
    not a real high-current impact profile, and stages 0-3 don't even
    touch OF at all,
  - prompts for an explicit keypress before each run so you can confirm
    the actuator is clear and it's safe to proceed.
Still: watch the actuator during each run and be ready to hit the
physical E-stop/power if anything moves unexpectedly.
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import Controller, DEFAULT_ADDRESS

ARRAY_NAME = "ffTest"

# Coarse to fine -- only used for stage 4's final sweep, once stages 0-3
# have already passed once at the coarsest of these. TM=500us was already
# separately confirmed comfortably passing (ratio~0.98) before OF's body
# was optimized to a single "OF arr[i]" line (see build_full_program) --
# skipped here to save bench time, in favor of directly bisecting the
# crossover between 250us (previously marginal) and 125us (previously a
# clear fail) now that the loop body is faster.
TM_US_CANDIDATES = [156.25]

# Order-of-magnitude sample count for a real ~60-80ms move at these
# rates (400 samples = 200ms at 500us, 100ms at 250us, 50ms at 125us --
# at the *nominal* TM; the real per-update cadence is 2*TM, see above).
N_SAMPLES = 400

# Small and gentle on purpose -- see SAFETY note above.
FF_AMPLITUDE_V = 0.5

# Bench-confirmed (see module docstring): AT's n0 resolution is 2, so
# every AT-paced program below requests this many samples per iteration.
AT_SAMPLES_PER_ITERATION = 2

# A ratio outside this band means the loop measurably drifted from the
# predicted duration -- not a hard physical limit, just a reasonable
# first threshold to flag for a closer look.
PASS_RATIO_LOW = 0.95
PASS_RATIO_HIGH = 1.10

POLL_INTERVAL_S = 0.001


def build_ff_array(n_samples: int, amplitude_v: float) -> list:
    return [amplitude_v * math.sin(2 * math.pi * i / n_samples) for i in range(n_samples)]


def build_bare_loop_program(tm_us: int, n_samples: int) -> str:
    # No AT, no array, no OF -- just raw interpreter loop speed.
    return f"""#MAIN
TM {tm_us}
i=0
#LOOP
i=i+1
JP#LOOP,i<{n_samples}
EN
"""


def build_at_loop_program(tm_us: int, n_samples: int, at_samples: int = AT_SAMPLES_PER_ITERATION) -> str:
    # AT0 sets a time reference once. Per the firmware command reference's
    # own worked example ("AT -100,1;' wait 100 samples from last time
    # reference (AT0)"), the reference only auto-advances to "now" when n0
    # is NEGATIVE (Remarks: "n0 < 0 ... resets the reference time when the
    # trippoint is complete to the current time"). A positive "AT n,1"
    # would check against the SAME original AT0 reference every time,
    # meaning only the very first iteration actually waits -- so this
    # always uses the negative form. at_samples defaults to the
    # bench-confirmed value of 2 (see module docstring: AT's n0 has a
    # documented Resolution of 2, and requesting 1 was confirmed to
    # silently behave as 2 anyway).
    return f"""#MAIN
TM {tm_us}
AT0
i=0
#LOOP
i=i+1
AT -{at_samples},1
JP#LOOP,i<{n_samples}
EN
"""


def build_array_read_program(
    tm_us: int, n_samples: int, array_name: str, at_samples: int = AT_SAMPLES_PER_ITERATION,
) -> str:
    return f"""#MAIN
TM {tm_us}
AT0
i=0
#LOOP
value={array_name}[i]
i=i+1
AT -{at_samples},1
JP#LOOP,i<{n_samples}
EN
"""


def build_full_program(
    tm_us: int, n_samples: int, array_name: str, at_samples: int = AT_SAMPLES_PER_ITERATION,
) -> str:
    return f"""#MAIN
TM {tm_us}
AT0
i=0
#LOOP
OF {array_name}[i]
i=i+1
AT -{at_samples},1
JP#LOOP,i<{n_samples}
OF 0
EN
"""


def build_full_loop_no_at_program(tm_us: int, n_samples: int, array_name: str) -> str:
    # Same loop body as build_full_program (array read + OF write) but
    # with no AT pacing at all -- the missing baseline analogous to Stage
    # 1's bare loop, this time for the FULL body. Needed to tell apart
    # "the loop body itself can't execute within 250us" (a hard ceiling)
    # from "something about AT specifically breaks down at that cadence"
    # (another quirk like the resolution=2 one) -- Stage 4's AT-paced
    # result alone can't distinguish those two.
    return f"""#MAIN
TM {tm_us}
i=0
#LOOP
OF {array_name}[i]
i=i+1
JP#LOOP,i<{n_samples}
OF 0
EN
"""


def array_round_trip_check(controller: Controller, array_name: str, values: list) -> bool:
    print(f"\n--- Stage 0: array round-trip ({len(values)} elements) ---")
    controller.download_array(array_name, values)
    download_error = controller.read_last_error()
    print(f"  TC 1 after download: {download_error.strip()!r}")

    readback = controller._g.GArrayUpload(array_name, 0, len(values) - 1)
    readback = [float(v) for v in readback]
    mismatches = [
        (i, sent, got)
        for i, (sent, got) in enumerate(zip(values, readback))
        if abs(sent - got) > 1e-4
    ]

    if len(readback) != len(values):
        print(f"  FAIL: sent {len(values)} elements, read back {len(readback)}")
        return False
    if mismatches:
        print(f"  FAIL: {len(mismatches)} mismatched elements, first: {mismatches[0]}")
        return False
    if not download_error.strip().startswith("0"):
        print(f"  FAIL: TC 1 reported an error after download")
        return False

    print("  PASS: array downloaded and read back identical")
    return True


def run_timed_stage(
    controller: Controller, label: str, tm_us: int, n_samples: int, program_text: str,
    array_name: str = None, ff_array: list = None, throttled: bool = True,
    samples_per_iteration: int = AT_SAMPLES_PER_ITERATION,
) -> bool:
    # throttled=False (stage 1 only): no AT pacing exists in this program,
    # so there's nothing to compare against a predicted duration -- report
    # raw per-iteration speed instead (informational, against the manual's
    # own ~82-84us/iteration reference benchmark) and pass/fail purely on
    # whether it ran cleanly.
    predicted_s = n_samples * samples_per_iteration * tm_us / 1_000_000
    predicted_label = f"{predicted_s * 1000:.0f}ms predicted" if throttled else "untimed baseline"
    print(f"\n--- {label} (TM={tm_us}us, N={n_samples}, {predicted_label}) ---")
    input("Confirm the actuator is clear, then press Enter...")

    controller.enable()
    # Program download first, array download second (when this stage
    # uses an array) -- every existing program builder in this codebase
    # (contour_program.py/pvt_program.py) issues its DM lines from
    # *inside* the downloaded program text rather than as a separate
    # host command beforehand, which is suspicious precedent for "a full
    # program download clears/reallocates the array table." Downloading
    # the array only after the program is already resident sidesteps
    # that risk either way.
    controller.download_program(program_text)
    if array_name is not None:
        controller.download_array(array_name, ff_array)
    predownload_error = controller.read_last_error()

    t0 = time.perf_counter()
    controller.run_program()
    while controller.read_program_running():
        time.sleep(POLL_INTERVAL_S)
    elapsed_s = time.perf_counter() - t0

    last_error = controller.read_last_error()
    controller.set_offset(0.0)
    controller.disable()

    clean = last_error.strip().startswith("0")

    if throttled:
        ratio = elapsed_s / predicted_s if predicted_s else float("nan")
        kept_pace = PASS_RATIO_LOW <= ratio <= PASS_RATIO_HIGH
        print(f"  predicted={predicted_s * 1000:.1f}ms actual={elapsed_s * 1000:.1f}ms ratio={ratio:.3f}")
    else:
        kept_pace = True
        per_iter_us = elapsed_s * 1_000_000 / n_samples if n_samples else float("nan")
        print(f"  actual={elapsed_s * 1000:.1f}ms ({per_iter_us:.1f}us/iteration, no AT pacing)")

    print(f"  TC 1 before run (post-download): {predownload_error.strip()!r}")
    print(f"  TC 1 after run: {last_error.strip()!r}")
    if kept_pace and clean:
        print(f"  PASS: {label}")
    else:
        reason = "ratio outside pass band" if not kept_pace else "program did not end cleanly"
        print(f"  FAIL: {label} ({reason})")

    return kept_pace and clean


def main():
    ff_array = build_ff_array(N_SAMPLES, FF_AMPLITUDE_V)
    coarsest_tm = TM_US_CANDIDATES[0]

    controller = Controller()
    try:
        controller.connect(DEFAULT_ADDRESS)
    except gclib.GclibError as e:
        print(f"Failed to connect to controller at {DEFAULT_ADDRESS}: {e}")
        return 1

    print(f"Connected to {DEFAULT_ADDRESS}")

    results = []
    try:
        if not array_round_trip_check(controller, ARRAY_NAME, ff_array):
            print("\nStopping -- array download/readback itself is broken.")
            return 1

        # Stage 1b: the full loop body (array read + OF write), same as
        # Stage 4, but with NO AT pacing at all, run at TM=125us
        # specifically (not coarsest_tm) -- the missing baseline that
        # tells apart "the loop body itself can't execute within 250us"
        # from "something about AT breaks down at that cadence specifically"
        # (Stage 4 alone, AT-paced, can't distinguish those). Informational
        # only (throttled=False) -- always passes on a clean run regardless
        # of how slow it turns out to be.
        run_timed_stage(
            controller, "Stage 1b: full loop (array + OF), no AT, TM=125us",
            125, N_SAMPLES, build_full_loop_no_at_program(125, N_SAMPLES, ARRAY_NAME),
            array_name=ARRAY_NAME, ff_array=ff_array, throttled=False,
        )

        stages = [
            ("Stage 1: bare loop", build_bare_loop_program(coarsest_tm, N_SAMPLES), None, False),
            ("Stage 2: AT-throttled loop", build_at_loop_program(coarsest_tm, N_SAMPLES), None, True),
            (
                "Stage 3: AT + array read",
                build_array_read_program(coarsest_tm, N_SAMPLES, ARRAY_NAME),
                ARRAY_NAME,
                True,
            ),
            (
                "Stage 4: AT + array read + OF write",
                build_full_program(coarsest_tm, N_SAMPLES, ARRAY_NAME),
                ARRAY_NAME,
                True,
            ),
        ]
        for label, program_text, array_name, throttled in stages:
            passed = run_timed_stage(
                controller, label, coarsest_tm, N_SAMPLES, program_text,
                array_name=array_name, ff_array=ff_array if array_name else None,
                throttled=throttled,
            )
            if not passed:
                print(f"\nStopping -- {label} failed, no point testing further stages.")
                return 1

        print(f"\nAll stages passed at TM={coarsest_tm}us (2*TM={coarsest_tm * 2}us actual cadence) "
              f"-- sweeping stage 4 to finer TM.")
        for tm_us in TM_US_CANDIDATES[1:]:
            passed = run_timed_stage(
                controller, f"Stage 4 @ TM={tm_us}us (2*TM={tm_us * 2}us cadence)", tm_us, N_SAMPLES,
                build_full_program(tm_us, N_SAMPLES, ARRAY_NAME),
                array_name=ARRAY_NAME, ff_array=ff_array,
            )
            results.append((tm_us, passed))
            if not passed:
                print(f"\nStopping -- TM={tm_us}us failed, no point testing faster.")
                break
    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")
    except gclib.GclibError as e:
        print(f"\nController communication error: {e}")
        return 1
    finally:
        try:
            controller.set_offset(0.0)
            controller.disable()
        except gclib.GclibError:
            pass
        controller.disconnect()

    print("\nSummary:")
    print(f"  TM={coarsest_tm}us (2*TM={coarsest_tm * 2}us cadence): all stages PASS")
    for tm_us, passed in results:
        print(f"  TM={tm_us}us (2*TM={tm_us * 2}us cadence): {'PASS' if passed else 'FAIL'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
