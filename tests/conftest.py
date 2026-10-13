"""Shared test fixtures.

gclib (Galil's Python binding) is only available on the bench machine, so
a stub module is injected before anything under src/ is imported. The
stub records every command sent so program/array handling can be
asserted without hardware.
"""

import sys
import types
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


class GclibError(Exception):
    pass


class FakeGalil:
    """Minimal stand-in for gclib.py(): records commands, answers the
    handful of queries the code under test makes, and holds downloaded
    arrays/programs so tests can inspect them."""

    def __init__(self):
        self.timeout = 5000
        self.commands = []
        self.programs = []
        self.arrays = {}
        self.responses = {}
        self.variables = {}

    def GOpen(self, address):
        self.address = address

    def GClose(self):
        pass

    def GCommand(self, command):
        self.commands.append(command)
        if command in self.responses:
            value = self.responses[command]
            return value() if callable(value) else value
        if command.startswith("MG ") and command[3:] in self.variables:
            return str(self.variables[command[3:]])
        if command.startswith("TC"):
            return "0"
        return "0"

    def GProgramDownload(self, text, preprocessor=""):
        self.programs.append(text)

    def GArrayDownload(self, name, first, last, values):
        self.arrays[name] = list(values)

    def GArrayUpload(self, name, first, last):
        return list(self.arrays.get(name, [0.0] * (last + 1)))[first:last + 1]


def _install_gclib_stub():
    module = types.ModuleType("gclib")
    module.GclibError = GclibError
    module.py = FakeGalil
    sys.modules.setdefault("gclib", module)


_install_gclib_stub()
