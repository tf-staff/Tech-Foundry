"""Tests whether "DA name[0]" actually reclaims array-table space for
reuse, or just marks an array inactive while the underlying memory
stays permanently consumed -- a candidate explanation for a new "TC 66
Array space full" error (Current Profile mode, downloading ffProf)
that appeared after a Current Profile -> Contour -> Current Profile
Arm sequence, even though the LOGICAL total at that point (record
arrays back down to ~205 each + a fresh 41-element ffProf) should have
left well over 2000 elements of headroom by simple addition.

Two things tested against real hardware:

1. REPRO -- replays the exact 3-step sequence that failed (using
   direct Controller calls, no motion/GUI): size the 4 record arrays +
   download a small ffProf (mimics Current Profile Arm #1) -> free
   ffProf and grow the 4 record arrays to 750 via DA+DM (mimics
   Contour Arm) -> shrink the 4 record arrays back down and download
   ffProf again (mimics Current Profile Arm #2, the one that failed).
   Reports the state and any failure (with TC 1) at each step.

2. ALTERNATIVE STRATEGY -- if the repro fails, tries the same overall
   sequence but using "DA *[0]" (deallocate every array at once) at the
   start of each resize round instead of per-array "DA name[0]", to see
   whether a full-table reset avoids whatever fragmentation/leak the
   per-array approach hits.

SAFETY: array-table operations only (DM/DA/GArrayDownload via
GCommand/Controller methods) -- no motion, no current, nothing to
watch on the bench.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import (
    Controller, DEFAULT_ADDRESS, RECORD_ARRAY_NAME, RECORD_CMD_ARRAY_NAME,
    RECORD_ERR_ARRAY_NAME, RECORD_TRQ_ARRAY_NAME, RECORD_ARRAY_SIZE,
)

FF_ARRAY_NAME = "ffProf"
CP_RECORD_SAMPLES = 205  # matches the real repro's own record_samples
FF_VALUES = [0.01 * i for i in range(1, 42)]  # 41 values, matches the real repro

RECORD_NAMES = (RECORD_ARRAY_NAME, RECORD_CMD_ARRAY_NAME, RECORD_ERR_ARRAY_NAME, RECORD_TRQ_ARRAY_NAME)


def report_state(controller: Controller, label: str) -> None:
    print(f"  [{label}] tracked sizes: {controller._array_sizes}, "
          f"tracked total: {sum(controller._array_sizes.values())}")


def step(controller: Controller, label: str, fn) -> bool:
    try:
        fn()
        print(f"  OK   {label}")
        return True
    except gclib.GclibError as e:
        print(f"  FAIL {label}: {e}")
        return False


def run_repro(controller: Controller) -> bool:
    print("--- Step 1: Current Profile Arm #1 (record arrays -> 205 each, ffProf fresh) ---")
    ok = step(controller, "size_shared_record_arrays(205)",
               lambda: controller.size_shared_record_arrays(CP_RECORD_SAMPLES))
    ok = ok and step(controller, "download_array(ffProf, 41 values)",
                       lambda: controller.download_array(FF_ARRAY_NAME, FF_VALUES))
    report_state(controller, "after step 1")
    if not ok:
        return False

    print("--- Step 2: Contour Arm (frees ffProf, record arrays -> 750 each) ---")
    ok = step(controller, "size_shared_record_arrays(750)",
               lambda: controller.size_shared_record_arrays(RECORD_ARRAY_SIZE))
    report_state(controller, "after step 2")
    if not ok:
        return False

    print("--- Step 3: Current Profile Arm #2 (record arrays -> 205 each, ffProf fresh again) ---")
    ok = step(controller, "size_shared_record_arrays(205)",
               lambda: controller.size_shared_record_arrays(CP_RECORD_SAMPLES))
    ok = ok and step(controller, "download_array(ffProf, 41 values)",
                       lambda: controller.download_array(FF_ARRAY_NAME, FF_VALUES))
    report_state(controller, "after step 3")
    return ok


def run_full_reset_alternative(controller: Controller) -> bool:
    # Same 3 logical steps, but using "DA *[0]" (deallocate everything)
    # before each resize round instead of the per-array DA inside
    # size_shared_record_arrays/_size_array.
    def full_reset_and_size(record_size, ff_values=None):
        try:
            controller._g.GCommand("DA *[0]")
        except gclib.GclibError as e:
            raise gclib.GclibError(f"'DA *[0]' failed: {e} (TC 1: {controller.read_last_error()})") from e
        controller._array_sizes.clear()
        controller.size_shared_record_arrays(record_size)
        if ff_values is not None:
            controller.download_array(FF_ARRAY_NAME, ff_values)

    print("--- Alt Step 1: full reset + Current Profile sizing ---")
    ok = step(controller, "DA *[0] + size(205) + download ffProf",
               lambda: full_reset_and_size(CP_RECORD_SAMPLES, FF_VALUES))
    report_state(controller, "after alt step 1")
    if not ok:
        return False

    print("--- Alt Step 2: full reset + Contour sizing ---")
    ok = step(controller, "DA *[0] + size(750)",
               lambda: full_reset_and_size(RECORD_ARRAY_SIZE))
    report_state(controller, "after alt step 2")
    if not ok:
        return False

    print("--- Alt Step 3: full reset + Current Profile sizing again ---")
    ok = step(controller, "DA *[0] + size(205) + download ffProf",
               lambda: full_reset_and_size(CP_RECORD_SAMPLES, FF_VALUES))
    report_state(controller, "after alt step 3")
    return ok


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
        print("=== Reproducing the real Current Profile -> Contour -> Current Profile sequence ===")
        repro_ok = run_repro(controller)

        if repro_ok:
            print("\nDid NOT reproduce this run -- the sequence succeeded. If the app still "
                  "fails, something else about timing/order may differ; try again or compare "
                  "against the app's exact call sequence.")
        else:
            print("\nReproduced the failure. Trying the 'DA *[0]' full-reset alternative "
                  "strategy instead of per-array DA...")
            alt_ok = run_full_reset_alternative(controller)
            if alt_ok:
                print("\n'DA *[0]' full-reset alternative WORKS where per-array DA did not.")
            else:
                print("\n'DA *[0]' full-reset alternative ALSO fails -- this isn't about "
                      "per-array vs. full-table DA specifically.")
    except gclib.GclibError as e:
        print(f"\nController communication error: {e}")
        return 1
    finally:
        controller.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
