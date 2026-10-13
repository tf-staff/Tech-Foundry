"""Launches the operator-facing user GUI (src/gui/user_app.py)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from gui.user_app import main  # noqa: E402

if __name__ == "__main__":
    main()
