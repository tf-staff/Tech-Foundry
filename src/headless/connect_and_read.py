"""Headless connect-and-read smoke test for the Galil DMC-31011.

Confirms the controller interface layer (src/controller/controller.py)
connects and reads correctly before any GUI is built (project_spec_v1.txt,
section 4, step b).

Read-only: does not send any setup/config or motion commands, and assumes
the controller already has LVDT feedback (AF1) enabled from prior bench
validation. Run with the controller powered on and reachable, then Ctrl+C
to stop.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import Controller, DEFAULT_ADDRESS

POLL_INTERVAL_S = 0.2  # ~5 Hz


def main():
    controller = Controller()

    try:
        controller.connect(DEFAULT_ADDRESS)
    except gclib.GclibError as e:
        print(f"Failed to connect to controller at {DEFAULT_ADDRESS}: {e}")
        return 1

    print(f"Connected to {DEFAULT_ADDRESS}")
    print("Reading position (counts) and LVDT voltage. Ctrl+C to stop.\n")

    try:
        while True:
            position = controller.read_position()
            lvdt_voltage = controller.read_lvdt_voltage()
            print(f"position={position}  lvdt_voltage={lvdt_voltage}")
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("\nStopping.")
    except gclib.GclibError as e:
        print(f"\nController communication error: {e}")
        return 1
    finally:
        controller.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
