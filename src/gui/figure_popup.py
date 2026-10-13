"""Shows a matplotlib Figure in its own Tkinter window and saves it as a
PNG -- shared by the testing GUI and the user GUI."""

import tkinter as tk
from pathlib import Path

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


def show_figure(root, figure, title: str, save_path=None) -> tk.Toplevel:
    """Embeds `figure` in a new Toplevel (with the standard navigation
    toolbar). If `save_path` is given the figure is written there first
    (parent directories created)."""
    if save_path is not None:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(save_path)
    popup = tk.Toplevel(root)
    popup.title(title)
    canvas = FigureCanvasTkAgg(figure, master=popup)
    canvas.draw()
    canvas.get_tk_widget().pack(fill="both", expand=True)
    toolbar = NavigationToolbar2Tk(canvas, popup)
    toolbar.update()
    return popup
