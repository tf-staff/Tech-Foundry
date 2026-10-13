"""Standalone bench measurement of TM's REAL achieved sample period vs.
the nominal/requested value. The firmware command reference documents
exactly one example ("TM 1000 will actually set an update rate of 976
microseconds... off by 2.4%") but gives no formula for other TM values,
and this project's Current Profile mode runs at TM=125us -- the
documented minimum -- where the same (or any) discrepancy is
unconfirmed. Not part of the app; run manually against real hardware.

Two independent measurements, each repeated N_REPEATS times for spread/
confidence rather than trusting a single sample:

STAGE A -- recording-based ground truth. RC 1 captures exactly one
sample per REAL TM tick (this project's own long-established, bench-
confirmed understanding of the Data Record feature -- see controller.py's
own comments). So real_TM_period = host-measured elapsed time /
recorded sample count. Deliberately uses NEITHER WT NOR AT to define the
measurement window -- the firmware reference lists both as commands
"automatically scaled to adjust for changes in sample time," so using
either would risk circularity (measuring their own scaling instead of
TM's true period independently). Instead the window is a bare i=i+1/JP
counting loop of a fixed iteration count -- unrelated to TM/AT pacing --
bracketed by a host time.perf_counter() stopwatch.

Each repeat runs this TWICE: once with a near-instant (1-iteration)
"calibration" loop, once with the real STAGE_A_ITERATIONS-iteration
loop, and takes the DIFFERENCE in both elapsed time and recorded count
between the two. elapsed_s otherwise also includes fixed overhead that
never shows up in recorded_count at all -- the host<->controller
round-trip to issue XQ, the TM/DM/RA/RD/RC1 setup lines executing before
recording even starts, and up to ~1ms of host polling-detection lag
after the program actually finishes -- which would otherwise inflate
the computed period, and is only a small fraction of a long window but
was bench-confirmed significant at Stage A's necessarily-short window
(capped by the recording array's own size). The calibration run shares
that same fixed overhead almost exactly (same setup lines, negligible
loop difference), so subtracting it out isolates just the extra
time/samples the longer loop actually added.

STAGE B -- AT-based cross-check, the same "AT -2,1" bare-loop pattern
bench-confirmed in ff_loop_timing_bench.py, timed the same stopwatch
way. The firmware reference claims AT is "automatically scaled" to
track TM's *real* period, not the nominal one -- Stage B checks whether
that claim actually holds at TM=125us by comparing against Stage A's
independent ground truth.

SAFETY: this script never enables the servo and issues no PA/BG/OF/SH --
only DM/RA/RD/RC (position recording) and plain counting loops. No
motion, no current, nothing to watch on the bench.
"""

import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import Controller, DEFAULT_ADDRESS, RECORD_ARRAY_NAME

TM_US = 125

# Stage A: bare-loop iteration count for the recording window. First
# version of this script sized this from the bare loop's own previously
# bench-confirmed ~99-100us/iteration pace (ff_loop_timing_bench.py
# Stage 1) -- but that baseline was measured with NO recording running
# concurrently, and RC 1 is itself active interrupt/CPU load throughout
# this loop (the same "concurrent load slows a simultaneously-running
# loop" effect bench-confirmed elsewhere in this project). The first
# run's recorded_count came back at exactly RECORD_SAMPLES (2900) every
# time -- the array saturated and stopped capturing before the loop's
# own 3400th iteration, meaning elapsed_s (bracketing the WHOLE loop)
# included dead time after recording had already stopped, inflating the
# computed period. Shrunk with real margin below -- see the saturation
# check in stage_a_recording_ground_truth, which now confirms
# recorded_count < RECORD_SAMPLES (not saturated) rather than assuming it.
STAGE_A_ITERATIONS = 500
RECORD_SAMPLES = 2900  # single channel -- the full controller-wide budget

# Stage B: AT -2,1 (250us nominal cadence), no recording/array-size
# ceiling to worry about -- a longer window for better precision
# (predicted ~5s at the nominal rate).
STAGE_B_ITERATIONS = 20000
AT_SAMPLES = 2

N_REPEATS = 5
POLL_INTERVAL_S = 0.001


def thread_running(controller: Controller) -> bool:
    return int(float(controller._g.GCommand("MG _XQ0"))) != -1


def run_program_and_time(controller: Controller, program_text: str) -> float:
    controller.download_program(program_text)
    t0 = time.perf_counter()
    controller.run_program()
    while thread_running(controller):
        time.sleep(POLL_INTERVAL_S)
    return time.perf_counter() - t0


def build_stage_a_program(n_iterations: int) -> str:
    # n_iterations is a parameter (not just STAGE_A_ITERATIONS) so the
    # same program shape can also build a "calibration" run (n=1) --
    # same TM/DM/RA/RD/RC1...RC0/EN overhead, negligible loop time --
    # used to measure and subtract that fixed overhead from the real
    # run's elapsed_s. See stage_a_recording_ground_truth.
    return f"""#MAIN
TM {TM_US}
DM {RECORD_ARRAY_NAME}[{RECORD_SAMPLES}]
RA {RECORD_ARRAY_NAME}[]
RD _TPA
RC 1
i=0
#LOOP
i=i+1
JP#LOOP,i<{n_iterations}
RC 0
EN
"""


def build_stage_b_program() -> str:
    return f"""#MAIN
TM {TM_US}
AT0
i=0
#LOOP
i=i+1
AT -{AT_SAMPLES},1
JP#LOOP,i<{STAGE_B_ITERATIONS}
EN
"""


def _run_stage_a_once(controller: Controller, n_iterations: int) -> tuple:
    program = build_stage_a_program(n_iterations)
    elapsed_s = run_program_and_time(controller, program)
    recorded_count = int(float(controller._g.GCommand("MG _RD")))
    return elapsed_s, recorded_count


def stage_a_recording_ground_truth(controller: Controller) -> list:
    periods_us = []
    for run in range(N_REPEATS):
        # Calibration run: identical program shape (same TM/DM/RA/RD/RC1
        # ... RC0/EN overhead), but the loop runs once -- measures the
        # fixed host<->controller round-trip + DM/array-setup + polling-
        # detection overhead that elapsed_s would otherwise silently
        # include on top of the real recording window (this is what you
        # flagged -- confirmed real, not just theoretical, see below).
        elapsed_calib_s, recorded_calib = _run_stage_a_once(controller, 1)
        elapsed_main_s, recorded_main = _run_stage_a_once(controller, STAGE_A_ITERATIONS)

        if recorded_main <= 0:
            raise RuntimeError("No samples recorded -- can't measure")
        # If the array filled (non-circular RC 1 stops capturing once
        # full), recording stopped some time before the loop's own
        # STAGE_A_ITERATIONS-th iteration -- elapsed_s (bracketing the
        # WHOLE loop) would then include dead time after recording
        # already stopped, inflating period_us below. Confirm we're
        # nowhere near that rather than assume it -- see STAGE_A_ITERATIONS'
        # own comment for how this bit a first version of this script.
        if recorded_main >= RECORD_SAMPLES:
            raise RuntimeError(
                f"recorded_count ({recorded_main}) reached RECORD_SAMPLES "
                f"({RECORD_SAMPLES}) -- the array likely saturated before the "
                f"loop finished, so elapsed_s no longer brackets just the "
                f"recording window. Reduce STAGE_A_ITERATIONS and re-run."
            )

        # Differential: subtract the calibration run's fixed overhead
        # from both the elapsed time and the (usually 0-2 sample) count
        # it captured during its own brief active window, isolating just
        # the extra time/samples the longer loop actually added.
        delta_elapsed_s = elapsed_main_s - elapsed_calib_s
        delta_count = recorded_main - recorded_calib
        if delta_count <= 0 or delta_elapsed_s <= 0:
            raise RuntimeError(
                f"Calibration run wasn't shorter than the main run "
                f"(elapsed {elapsed_calib_s * 1000:.2f}ms/{recorded_calib} samples vs "
                f"{elapsed_main_s * 1000:.2f}ms/{recorded_main} samples) -- "
                f"increase STAGE_A_ITERATIONS."
            )
        period_us = delta_elapsed_s * 1_000_000 / delta_count
        periods_us.append(period_us)
        print(
            f"  run {run + 1}/{N_REPEATS}: calib={elapsed_calib_s * 1000:.2f}ms/"
            f"{recorded_calib} samples, main={elapsed_main_s * 1000:.2f}ms/"
            f"{recorded_main} samples -> delta={delta_elapsed_s * 1000:.2f}ms/"
            f"{delta_count} samples -> {period_us:.3f}us/sample"
        )
    return periods_us


def stage_b_at_cross_check(controller: Controller) -> list:
    program = build_stage_b_program()
    periods_us = []
    for run in range(N_REPEATS):
        elapsed_s = run_program_and_time(controller, program)
        period_us = elapsed_s * 1_000_000 / (STAGE_B_ITERATIONS * AT_SAMPLES)
        periods_us.append(period_us)
        print(
            f"  run {run + 1}/{N_REPEATS}: {STAGE_B_ITERATIONS} iterations in "
            f"{elapsed_s * 1000:.2f}ms -> {period_us:.3f}us/sample implied"
        )
    return periods_us


def report(label: str, periods_us: list) -> float:
    mean_us = statistics.mean(periods_us)
    spread_us = max(periods_us) - min(periods_us)
    pct_off = (mean_us / TM_US - 1) * 100
    print(
        f"{label}: mean={mean_us:.3f}us, spread={spread_us:.3f}us, "
        f"nominal={TM_US}us ({pct_off:+.2f}%)\n"
    )
    return mean_us


def main():
    controller = Controller()
    try:
        controller.connect(DEFAULT_ADDRESS)
    except gclib.GclibError as e:
        print(f"Failed to connect to controller at {DEFAULT_ADDRESS}: {e}")
        return 1

    print(f"Connected to {DEFAULT_ADDRESS}")
    print("No motion, no current -- purely measuring TM's real sample period.\n")

    try:
        print(f"--- Stage A: recording-based ground truth ({N_REPEATS} runs) ---")
        periods_a = stage_a_recording_ground_truth(controller)
        mean_a = report("Stage A", periods_a)

        print(f"--- Stage B: AT -{AT_SAMPLES},1 cross-check ({N_REPEATS} runs) ---")
        periods_b = stage_b_at_cross_check(controller)
        mean_b = report("Stage B", periods_b)
    except gclib.GclibError as e:
        print(f"\nController communication error: {e}")
        return 1
    finally:
        controller.disconnect()

    diff_us = abs(mean_a - mean_b)
    print(
        f"Summary: ground truth={mean_a:.3f}us, AT cross-check={mean_b:.3f}us "
        f"(difference {diff_us:.3f}us, {diff_us / mean_a * 100:.2f}% of ground truth)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
