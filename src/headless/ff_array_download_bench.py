"""Standalone diagnostic for the Current Profile "feedforward array
download failed: ... (TC 1: 6 Number out of range)" bug -- bench-
confirmed reproducible with Start/Stop 0/39.74mm, Peak Velocity
Position=Impact Position=25mm, Impact Velocity=4m/s, Mass=0.001kg, Kf
accel/decel=1 (all 4 FF stretch=1.0), and STILL reproduces after both a
full controller restart and an app restart, and after fixing the one
concrete anomaly found so far (2 of the 41 feedforward values were
literal negative zero, which rendered as the string "-0.0" -- fixed via
Controller.download_array's now round(v, 6) + 0.0, but the failure
persisted). Not part of the app; run manually against real hardware to
narrow this down further than static analysis alone can.

Strategy: reproduce the exact failure against "ffProf" (the real array
name the app uses) first, then repeat against a throwaway scratch array
name to check whether the failure is specific to "ffProf" itself (e.g.
some interaction with a previously-downloaded program's own "OF
ffProf[i]" line) or reproduces regardless of array name. If it
reproduces on the scratch name too, binary-search the 41 values to
isolate the smallest failing sub-range/single value, since the
"negative zero" fix ruled out the one anomaly found by eye but there
could be another value-level issue the same way.

SAFETY: array-table operations only (DM/GArrayDownload via
Controller.download_array) -- no motion, no current, nothing to watch
on the bench.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import Controller, DEFAULT_ADDRESS
from program.current_profile_program import AT_SAMPLES, FF_ARRAY_NAME, TM_US_ACTUAL
from program.current_profile_trajectory import compute_current_profile_curve, compute_feedforward_v

SCRATCH_ARRAY_NAME = "diagArr"

# Exact repro parameters (.claude/reference/image3.png).
START_MM = 0.00
STOP_MM = 39.74
PEAK_V_POSITION_MM = 25.0
IMPACT_MM = 25.0
IMPACT_VELOCITY_M_S = 4.0
MASS_KG = 0.001
KF_ACCEL = 1.0
KF_DECEL = 1.0


def real_repro_values() -> list:
    curve = compute_current_profile_curve(
        START_MM, PEAK_V_POSITION_MM, IMPACT_MM, STOP_MM, IMPACT_VELOCITY_M_S,
    )
    ff_period_s = AT_SAMPLES * TM_US_ACTUAL / 1_000_000
    return compute_feedforward_v(
        curve, ff_period_s, MASS_KG, KF_ACCEL, KF_DECEL, 1.0, 1.0, 1.0, 1.0,
    )


def try_download(controller: Controller, array_name: str, values: list, label: str) -> bool:
    try:
        controller.download_array(array_name, values)
        print(f"  OK   {label} ({len(values)} values)")
        return True
    except gclib.GclibError as e:
        print(f"  FAIL {label} ({len(values)} values): {e}")
        return False


def bisect_failing_values(controller: Controller, values: list):
    # Narrows down to the smallest failing sub-range via binary split,
    # using SCRATCH_ARRAY_NAME so these probing downloads don't touch
    # the real ffProf array. Tests BOTH halves each round (not just the
    # one that "must" contain the bad value) since a single-bad-value
    # assumption might be wrong -- if neither half alone fails, this is
    # a length/combination effect, not one specific value, and we say so.
    lo, hi = 0, len(values)
    print(f"Bisecting {len(values)} values on {SCRATCH_ARRAY_NAME!r}...")
    while hi - lo > 1:
        mid = (lo + hi) // 2
        left_ok = try_download(controller, SCRATCH_ARRAY_NAME, values[lo:mid], f"indices [{lo}:{mid})")
        right_ok = try_download(controller, SCRATCH_ARRAY_NAME, values[mid:hi], f"indices [{mid}:{hi})")
        if not left_ok and right_ok:
            hi = mid
        elif left_ok and not right_ok:
            lo = mid
        elif not left_ok and not right_ok:
            print("  Both halves fail independently -- multiple bad values, or a "
                  "length-independent property both halves share. Stopping bisection "
                  "here; inspect both halves' values directly below.")
            return None
        else:
            print("  Neither half alone reproduces the failure -- not a single bad "
                  "value; likely a length or combination effect, not isolable this way.")
            return None
    print(f"Isolated to index {lo}")
    return lo


def main():
    controller = Controller()
    try:
        controller.connect(DEFAULT_ADDRESS)
    except gclib.GclibError as e:
        print(f"Failed to connect to controller at {DEFAULT_ADDRESS}: {e}")
        return 1

    print(f"Connected to {DEFAULT_ADDRESS}")
    print("Array-table operations only -- no motion, no current.\n")

    try:
        print("--- Sanity check: trivial synthetic array (scratch name) ---")
        try_download(controller, SCRATCH_ARRAY_NAME, [0.1, 0.2, 0.3], "synthetic [0.1, 0.2, 0.3]")

        values = real_repro_values()
        print(f"\nComputed {len(values)} real repro feedforward values.")
        print(f"  values: {values}")

        print(f"\n--- Real repro: full array to {FF_ARRAY_NAME!r} (matches the app) ---")
        ff_ok = try_download(controller, FF_ARRAY_NAME, values, f"full array -> {FF_ARRAY_NAME!r}")

        print(f"\n--- Same array, but to a throwaway scratch name {SCRATCH_ARRAY_NAME!r} ---")
        scratch_ok = try_download(controller, SCRATCH_ARRAY_NAME, values, f"full array -> {SCRATCH_ARRAY_NAME!r}")

        if ff_ok and scratch_ok:
            print("\nBoth succeeded -- could not reproduce this run. If the app still "
                  "fails with identical parameters, something about the app's own "
                  "call sequence (not just these values) differs -- worth comparing "
                  "against build_current_profile_program's array-sizing/ordering.")
        elif not ff_ok and scratch_ok:
            print(f"\n{FF_ARRAY_NAME!r} failed but a throwaway array with the SAME "
                  f"values succeeded -- points at something specific to the "
                  f"{FF_ARRAY_NAME!r} name/history in this session (e.g. a "
                  f"previously-downloaded program's own reference to it), not the "
                  f"values themselves.")
        elif not scratch_ok:
            print(f"\nReproduced on a throwaway array too -- this is about the VALUES "
                  f"themselves (or their length), not anything specific to "
                  f"{FF_ARRAY_NAME!r}. Bisecting to isolate which...")
            idx = bisect_failing_values(controller, values)
            if idx is not None:
                v = values[idx]
                print(f"\nCulprit value at index {idx}: raw={v!r}")
                print(f"  round(v, 6) + 0.0 = {round(v, 6) + 0.0!r}")
                print(f"  str() as sent      = {str(round(v, 6) + 0.0)!r}")
    except gclib.GclibError as e:
        print(f"\nController communication error: {e}")
        return 1
    finally:
        controller.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
