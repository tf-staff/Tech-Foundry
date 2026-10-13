"""Tests whether Galil's "DA" command can reliably grow an array that's
already dimensioned to a smaller size mid-session. Syntax confirmed
from the Galil DMC command reference examples: "name[0]" deallocates a
specific array (the "[0]" suffix distinguishes an array from a plain
variable, which DA deallocates as just "name" with no brackets);
"*[0]" deallocates all arrays; "*,*[0]" deallocates all variables and
all arrays.

Context: ff_array_download_bench.py + Controller._size_array
bench-confirmed that growing an ALREADY-DIMENSIONED array (regardless
of a plain single DM or a shrink-then-grow sequence) fails outright on
this controller firmware ("Number out of range"), while a genuinely
fresh DM (array never dimensioned this session) always works. The
current workaround (_size_array) just refuses to grow and asks for a
controller restart instead. If DA reliably frees an array's space
first, a subsequent DM to a larger size might then count as "fresh"
(the array no longer exists) and succeed -- letting Current Profile
mode change parameters that grow the feedforward array without
requiring a restart.

Tests 2 candidate syntaxes, each against its OWN freshly-small scratch
array (so one candidate's outcome can't contaminate the next):
  A: "DA name[0]"  (deallocate one specific array -- the documented
                    per-array form)
  B: "DA *[0]"     (deallocate all arrays -- a blunter fallback; fine
                    here since this script owns no other array-table
                    state that matters)

SAFETY: array-table operations only (DM/GArrayDownload/DA via GCommand)
-- no motion, no current, nothing to watch on the bench.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import Controller, DEFAULT_ADDRESS

SMALL_VALUES = [0.1, 0.2, 0.3]
LARGE_VALUES = [float(i) / 10 for i in range(1, 21)]  # 20 values, forces real growth


def make_small(controller: Controller, name: str) -> bool:
    try:
        controller.download_array(name, SMALL_VALUES)
        print(f"  setup: {name!r} created at {len(SMALL_VALUES)} elements")
        return True
    except gclib.GclibError as e:
        print(f"  setup FAILED for {name!r}: {e}")
        return False


def try_da_then_grow(controller: Controller, name: str, da_cmd: str) -> bool:
    try:
        response = controller._g.GCommand(da_cmd)
        print(f"  {da_cmd!r} -> {response!r}")
    except gclib.GclibError as e:
        print(f"  {da_cmd!r} FAILED: {e} (TC 1: {controller.read_last_error()})")
        return False
    # DA should make the array "not exist" again -- clear our own
    # tracking too so _size_array treats the next request as fresh
    # (this mirrors what Controller itself would need to do if DA is
    # adopted for real).
    controller._array_sizes.pop(name, None)
    try:
        controller.download_array(name, LARGE_VALUES)
        print(f"  DM {name}[{len(LARGE_VALUES)}] + GArrayDownload -> OK after {da_cmd!r}")
        return True
    except gclib.GclibError as e:
        print(f"  Grow after {da_cmd!r} FAILED: {e}")
        return False


def main():
    controller = Controller()
    try:
        controller.connect(DEFAULT_ADDRESS)
    except gclib.GclibError as e:
        print(f"Failed to connect to controller at {DEFAULT_ADDRESS}: {e}")
        return 1

    print(f"Connected to {DEFAULT_ADDRESS}")
    print("Array-table operations only -- no motion, no current.\n")

    results = {}
    try:
        for label, name, da_cmd in [
            ("A: DA name[0]", "diagArrA", "DA diagArrA[0]"),
            ("B: DA *[0]", "diagArrB", "DA *[0]"),
        ]:
            print(f"--- Candidate {label} ---")
            if not make_small(controller, name):
                results[label] = "setup failed"
                continue
            ok = try_da_then_grow(controller, name, da_cmd)
            results[label] = "WORKS" if ok else "does not work"
            print()
    except gclib.GclibError as e:
        print(f"\nController communication error: {e}")
        return 1
    finally:
        controller.disconnect()

    print("--- Summary ---")
    for label, result in results.items():
        print(f"  {label}: {result}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
