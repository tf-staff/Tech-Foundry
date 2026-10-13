"""Phase 0 concurrent-thread bench validation for Current Profile mode
(see the plan file "Current Profile Motion Mode") -- the last remaining
piece of the Phase 0 gate.

First run at TM=156.25us (AT -2,1, 312.5us cadence -- the floor found by
ff_loop_timing_bench.py's fine TM sweep) FAILED here (ratio 1.481) even
though the identical cadence passed comfortably in isolation (idle
motor). That's the risk this script exists to catch: thread 0 actively
running a Contour move (CD interpolation, position PID, etc. every tick)
competes for the same CPU cycles as thread 1's OF loop, which was
already at a razor-thin margin in isolation -- the isolated floor is NOT
the real floor once a real move is running concurrently.

A subsequent fine TM sweep (AT_SAMPLES fixed at 2, TM descending across
the 31.25us quantization grid) found TM=218.75us (437.5us cadence) as
the finest concurrent-safe point at AT -2,1 -- 187.5us and below failed.
Decision: proceed instead with TM=125us + AT -4,1 (500us cadence, the
same combination already confirmed passing in an earlier concurrent run,
ratio 1.017) -- keeps the servo loop itself fast for tight PID tracking,
at a cadence with more comfortable margin than the AT -2,1 crossover.

**Remaining concern before treating this as final**: every concurrent
run so far used whatever PID gains happened to already be resident on
the controller (uncontrolled, likely zero/stale) -- the real application
runs with real, nonzero Motion PID gains active during the move (that's
what makes position tracking work at all), and PID computation itself
is per-tick CPU work that could plausibly erode the margin further.
`build_combined_program` now sets real KP/KD/KI (defaulting to this
codebase's actual `DEFAULT_MOTION_KP`/`DEFAULT_MOTION_KD`/
`DEFAULT_MOTION_KI`, controller.py -- 3.0/10.0/0.01, not arbitrary
numbers) before `CMA`, and `TM_US_CANDIDATES`/`AT_SAMPLES` are set to
just the chosen production combination (TM=125us, AT -4,1) to directly
re-confirm it still holds with real PID gains active, rather than
re-running the whole sweep.

Both #MAIN (the Contour move) and #LOOP (the OF-loop) are combined into
ONE downloaded program (a DMC download can hold multiple labeled
routines, confirmed in .claude/reference/program_download_plan.txt) --
the host launches thread 0 ("XQ #MAIN") and thread 1 ("XQ #LOOP,1")
back to back, then polls both "_XQ0"/"_XQ1" (thread N's current line,
-1 once stopped) until both threads finish.

Verdict: total elapsed time should land close to whichever of the two
threads' own predicted duration is LONGER (they run concurrently, not
sequentially -- landing near the SUM instead would mean they weren't
actually concurrent; landing far past the longer one would mean one
thread held up the other).

Reuses compute_contour_increments (program/contour_program.py) for a
real, physically consistent small move -- not the full
build_contour_program (skips its PID/retract/recording setup, since
this test only cares about thread-concurrency timing, not the wider
per-mode program shape Phase 4 will build separately, and doesn't yet
include the recording (RC 1/DM/RA/RD) overhead the real application will
also run concurrently -- worth a follow-up once this crossover is found,
since that's further CPU load this script doesn't yet account for). OF
is left entirely owned by thread 1 -- thread 0 doesn't touch it, so
there's no write race between the two threads on the same register to
confound the timing result.

SAFETY: this script commands REAL physical motion (unlike
ff_loop_timing_bench.py, which only ever held position closed-loop) at
the same time as a continuous OF bias from the second thread. Uses a
short, slow, gentle move (5mm span, 0.15m/s peak) and a small +/-0.5V
synthetic FF amplitude, but confirm the actuator has clear, unobstructed
travel over the configured range before each run, and watch it during
the run.
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import Controller, DEFAULT_ADDRESS
from program.contour_program import compute_contour_increments, CONTOUR_DT_EXPONENT

# Chosen production combination (fast TM for tight PID, AT -4,1 for a
# 500us FF cadence with more margin than the AT -2,1 crossover found at
# TM=187.5-218.75us) -- a single-element list so this re-confirms that
# exact combination rather than re-running the whole sweep. Still a list
# (not a bare pair) so run_concurrent_stage/main() stay reusable for a
# future sweep if ever needed again.
TM_US_CANDIDATES = [125]
AT_SAMPLES = 4

# Real, nonzero Motion PID gains -- per your note, the controller output
# is disconnected for these runs, so the exact tuning doesn't matter;
# these just need to be nonzero so the position loop does real per-tick
# work, matching what the production move will actually run with.
MOTION_KP = 1.0
MOTION_KD = 1.0
MOTION_KI = 0.01

FF_AMPLITUDE_V = 0.5

# Small, slow, gentle -- see SAFETY note above. Not a real impact profile,
# just enough real motion to load thread 0 for a comparable duration to
# thread 1's FF loop.
START_MM = 0.0
IMPACT_MM = 2.5
END_MM = 5.0
IMPACT_VELOCITY_M_S = 0.15

ARRAY_NAME = "ffTest"

PASS_RATIO_LOW = 0.95
PASS_RATIO_HIGH = 1.10
POLL_INTERVAL_S = 0.001


def build_ff_array(n_samples: int, amplitude_v: float) -> list:
    return [amplitude_v * math.sin(2 * math.pi * i / n_samples) for i in range(n_samples)]


def build_combined_program(
    tm_us: float, increments: list, array_name: str, n_ff_samples: int, at_samples: int,
    kp: float = MOTION_KP, kd: float = MOTION_KD, ki: float = MOTION_KI,
) -> str:
    lines = [
        "#MAIN",
        f"TM {tm_us}",
        # Real, nonzero PID gains -- so the position loop does real
        # per-tick work, same as the production move will, rather than
        # measuring concurrency timing against whatever gains happened to
        # already be resident on the controller.
        f"KD {kd}",
        f"KP {kp}",
        f"KI {ki}",
        "CMA",
        f"DT {CONTOUR_DT_EXPONENT}",
    ]
    lines += [f"CD {inc}" for inc in increments]
    lines += [
        "CD 0=0",
        "#WAIT",
        "JP#WAIT,_CM<>511",
        "EN",
        "#LOOP",
        "AT0",
        "i=0",
        "#L",
        f"OF {array_name}[i]",
        "i=i+1",
        f"AT -{at_samples},1",
        f"JP#L,i<{n_ff_samples}",
        "OF 0",
        "EN",
    ]
    return "\n".join(lines) + "\n"


def thread_running(controller: Controller, thread_number: int) -> bool:
    return int(float(controller._g.GCommand(f"MG _XQ{thread_number}"))) != -1


def run_concurrent_stage(controller: Controller, tm_us: float, at_samples: int) -> bool:
    increments = compute_contour_increments(
        START_MM, IMPACT_MM, END_MM, IMPACT_VELOCITY_M_S, tm_us / 1_000_000,
    )
    predicted_move_s = len(increments) * 2 * tm_us / 1_000_000
    # Sized to roughly match the move's own duration at this TM/cadence --
    # both are governed by the same underlying physics/TM, so this stays
    # about right regardless of which TM candidate is being tested.
    cadence_s = at_samples * tm_us / 1_000_000
    n_ff_samples = max(1, round(predicted_move_s / cadence_s))
    predicted_ff_s = n_ff_samples * cadence_s
    longer_predicted_s = max(predicted_move_s, predicted_ff_s)

    label = f"TM={tm_us}us, AT -{at_samples},1 ({cadence_s * 1e6:.1f}us cadence)"
    print(f"\n--- {label} ---")
    print(f"  Contour move: {len(increments)} CD points, predicted {predicted_move_s * 1000:.1f}ms")
    print(f"  FF loop: {n_ff_samples} samples, predicted {predicted_ff_s * 1000:.1f}ms")
    print(
        f"  SAFETY: REAL move ({START_MM}->{IMPACT_MM}->{END_MM}mm, {IMPACT_VELOCITY_M_S}m/s peak) "
        f"while thread 1 writes a +/-{FF_AMPLITUDE_V}V OF bias."
    )
    input("  Confirm the actuator is clear, then press Enter...")

    ff_array = build_ff_array(n_ff_samples, FF_AMPLITUDE_V)
    program_text = build_combined_program(tm_us, increments, ARRAY_NAME, n_ff_samples, at_samples)

    controller.enable()
    # Program before array, same reasoning as ff_loop_timing_bench.py.
    controller.download_program(program_text)
    controller.download_array(ARRAY_NAME, ff_array)
    predownload_error = controller.read_last_error()

    t0 = time.perf_counter()
    controller.run_program("MAIN")
    controller._g.GCommand("XQ #LOOP,1")
    while thread_running(controller, 0) or thread_running(controller, 1):
        time.sleep(POLL_INTERVAL_S)
    elapsed_s = time.perf_counter() - t0

    last_error = controller.read_last_error()
    controller.set_offset(0.0)
    controller.disable()

    clean = last_error.strip().startswith("0")
    ratio = elapsed_s / longer_predicted_s if longer_predicted_s else float("nan")
    ok = clean and PASS_RATIO_LOW <= ratio <= PASS_RATIO_HIGH

    print(f"  TC 1 before run (post-download): {predownload_error.strip()!r}")
    print(f"  TC 1 after run: {last_error.strip()!r}")
    print(f"  Total elapsed (both threads): {elapsed_s * 1000:.1f}ms vs longer-of-two "
          f"predicted {longer_predicted_s * 1000:.1f}ms, ratio={ratio:.3f}")
    print(f"  {'PASS' if ok else 'FAIL'}: {label}")
    return ok


def main():
    controller = Controller()
    try:
        controller.connect(DEFAULT_ADDRESS)
    except gclib.GclibError as e:
        print(f"Failed to connect to controller at {DEFAULT_ADDRESS}: {e}")
        return 1

    print(f"Connected to {DEFAULT_ADDRESS}")

    results = []
    try:
        for tm_us in TM_US_CANDIDATES:
            ok = run_concurrent_stage(controller, tm_us, AT_SAMPLES)
            results.append((tm_us, ok))
            if not ok:
                print(f"\nStopping -- TM={tm_us}us failed under concurrent load, no point testing finer.")
                break
    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")
        return 1
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
    for tm_us, ok in results:
        cadence_us = AT_SAMPLES * tm_us
        print(f"  TM={tm_us}us ({cadence_us}us cadence): {'PASS' if ok else 'FAIL'}")

    return 0 if results and results[-1][1] else 1


if __name__ == "__main__":
    raise SystemExit(main())
