"""Minimal Tkinter control panel: connect/disconnect + live readout.

project_spec_v1.txt, section 4, step c. This is the only module that
imports Tkinter — all gclib interaction stays in controller.controller.
"""

import sys
import tkinter as tk
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gclib

from controller.controller import Controller, DEFAULT_ADDRESS

POLL_INTERVAL_MS = 200  # ~5 Hz


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Voice Coil Bench Control")

        self.controller = Controller()
        self._poll_after_id = None

        self.address_var = tk.StringVar(value=DEFAULT_ADDRESS)
        self.status_var = tk.StringVar(value="Disconnected")
        self.position_counts_var = tk.StringVar(value="—")
        self.position_mm_var = tk.StringVar(value="—")
        self.lvdt_voltage_var = tk.StringVar(value="—")

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_widgets(self):
        connection_frame = tk.Frame(self.root)
        connection_frame.pack(padx=10, pady=10, fill="x")

        tk.Label(connection_frame, text="IP:").pack(side="left")
        self.address_entry = tk.Entry(
            connection_frame, textvariable=self.address_var, width=16
        )
        self.address_entry.pack(side="left", padx=(4, 10))

        self.connect_button = tk.Button(
            connection_frame, text="Connect", command=self._on_connect
        )
        self.connect_button.pack(side="left")

        self.disconnect_button = tk.Button(
            connection_frame,
            text="Disconnect",
            command=self._on_disconnect,
            state="disabled",
        )
        self.disconnect_button.pack(side="left", padx=(4, 0))

        tk.Label(self.root, textvariable=self.status_var, anchor="w").pack(
            padx=10, fill="x"
        )

        readout_frame = tk.Frame(self.root)
        readout_frame.pack(padx=10, pady=10, fill="x")

        self._add_readout_row(readout_frame, "Position (counts):", self.position_counts_var)
        self._add_readout_row(readout_frame, "Position (mm):", self.position_mm_var)
        self._add_readout_row(readout_frame, "LVDT voltage (V):", self.lvdt_voltage_var)

    @staticmethod
    def _add_readout_row(parent, label_text, value_var):
        row = tk.Frame(parent)
        row.pack(fill="x", pady=2)
        tk.Label(row, text=label_text, width=18, anchor="w").pack(side="left")
        tk.Label(row, textvariable=value_var, anchor="w").pack(side="left")

    def _on_connect(self):
        address = self.address_var.get()
        try:
            self.controller.connect(address)
        except gclib.GclibError as e:
            self.status_var.set(f"Connect failed: {e}")
            return

        self.status_var.set(f"Connected to {address}")
        self.connect_button.config(state="disabled")
        self.disconnect_button.config(state="normal")
        self.address_entry.config(state="disabled")
        self._poll()

    def _on_disconnect(self):
        self._stop_polling()
        self.controller.disconnect()
        self.status_var.set("Disconnected")
        self.connect_button.config(state="normal")
        self.disconnect_button.config(state="disabled")
        self.address_entry.config(state="normal")

    def _poll(self):
        try:
            position = self.controller.read_position()
            lvdt_voltage = self.controller.read_lvdt_voltage()
        except gclib.GclibError as e:
            self.status_var.set(f"Read error: {e} — reconnect to resume")
            self._poll_after_id = None
            return

        self.position_counts_var.set(str(position))
        self.position_mm_var.set(f"{Controller.counts_to_mm(position):.2f}")
        self.lvdt_voltage_var.set(f"{lvdt_voltage:.3f}")

        self._poll_after_id = self.root.after(POLL_INTERVAL_MS, self._poll)

    def _stop_polling(self):
        if self._poll_after_id is not None:
            self.root.after_cancel(self._poll_after_id)
            self._poll_after_id = None

    def _on_close(self):
        self._stop_polling()
        if self.controller.is_connected:
            self.controller.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
