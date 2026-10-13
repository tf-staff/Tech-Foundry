"""A headless stand-in for tkinter (and matplotlib's Tk backend), just
enough to instantiate both GUIs and drive their callbacks in tests on a
machine with no display/tkinter. Widgets record their options; `after`
goes to a scheduler the test drains explicitly."""

import sys
import types


class Scheduler:
    def __init__(self):
        self.pending = []
        self._next_id = 0

    def after(self, delay_ms, callback=None, *args):
        self._next_id += 1
        if callback is not None:
            self.pending.append((self._next_id, delay_ms, callback, args))
        return self._next_id

    def after_cancel(self, after_id):
        self.pending = [p for p in self.pending if p[0] != after_id]

    def run_pending(self, rounds=50):
        """Runs every queued callback, repeatedly, until the queue is
        empty or `rounds` passes have been made (callbacks may re-queue
        themselves -- polling loops do)."""
        for _ in range(rounds):
            if not self.pending:
                return True
            batch, self.pending = self.pending, []
            for _, _, callback, args in batch:
                callback(*args)
        return not self.pending


SCHEDULER = Scheduler()


class Variable:
    def __init__(self, master=None, value=None, name=None):
        self._value = value

    def get(self):
        return self._value

    def set(self, value):
        self._value = value


class StringVar(Variable):
    def __init__(self, master=None, value="", name=None):
        super().__init__(master, "" if value is None else value)


class BooleanVar(Variable):
    def __init__(self, master=None, value=False, name=None):
        super().__init__(master, bool(value))

    def get(self):
        return bool(self._value)


class IntVar(Variable):
    pass


class DoubleVar(Variable):
    pass


class Widget:
    def __init__(self, master=None, **options):
        self.master = master
        self.options = dict(options)
        self.children = []
        self.bindings = {}
        if master is not None and hasattr(master, "children"):
            master.children.append(self)

    def pack(self, **kw):
        return self

    def grid(self, **kw):
        return self

    def place(self, **kw):
        return self

    def pack_forget(self):
        pass

    def config(self, **options):
        self.options.update(options)

    configure = config

    def cget(self, key):
        return self.options.get(key)

    def __getitem__(self, key):
        return self.options.get(key)

    def __setitem__(self, key, value):
        self.options[key] = value

    def bind(self, event, callback=None, add=None):
        self.bindings[event] = callback

    def bind_all(self, event, callback=None, add=None):
        self.bindings[event] = callback

    def after(self, delay_ms, callback=None, *args):
        return SCHEDULER.after(delay_ms, callback, *args)

    def after_cancel(self, after_id):
        SCHEDULER.after_cancel(after_id)

    def destroy(self):
        self.destroyed = True

    def winfo_screenheight(self):
        return 1080

    def winfo_width(self):
        return 900

    def update(self):
        pass

    def update_idletasks(self):
        pass

    def invoke(self):
        command = self.options.get("command")
        if command is not None and self.options.get("state", "normal") != "disabled":
            return command()


class Tk(Widget):
    def __init__(self):
        super().__init__(None)
        self.protocols = {}

    def title(self, text=None):
        self._title = text

    def geometry(self, spec=None):
        self._geometry = spec

    def protocol(self, name, callback):
        self.protocols[name] = callback

    def mainloop(self):
        pass


class Toplevel(Tk):
    def __init__(self, master=None, **options):
        Widget.__init__(self, master, **options)
        self.protocols = {}


class Frame(Widget):
    pass


class LabelFrame(Widget):
    pass


class Label(Widget):
    pass


class Button(Widget):
    pass


class Checkbutton(Widget):
    pass


class Entry(Widget):
    pass


class Text(Widget):
    def __init__(self, master=None, **options):
        super().__init__(master, **options)
        self.contents = ""

    def insert(self, index, text):
        self.contents += text

    def delete(self, *args):
        self.contents = ""


class Canvas(Widget):
    def create_window(self, *args, **kw):
        return 1

    def itemconfig(self, *args, **kw):
        pass

    def bbox(self, *args):
        return (0, 0, 100, 100)

    def yview(self, *args):
        pass

    def yview_scroll(self, *args):
        pass


class Scrollbar(Widget):
    def set(self, *args):
        pass


class Combobox(Widget):
    pass


filedialog = types.SimpleNamespace(
    askopenfilename=lambda **kw: "",
    asksaveasfilename=lambda **kw: "",
)
messagebox = types.SimpleNamespace(
    askokcancel=lambda *a, **kw: True,
    showinfo=lambda *a, **kw: None,
    showerror=lambda *a, **kw: None,
)
ttk = types.SimpleNamespace(Combobox=Combobox)


def install():
    """Puts the fake tkinter + Tk matplotlib backend into sys.modules,
    replacing a real tkinter if one is importable -- the smoke tests must
    not need a display even on the bench machine. Idempotent."""
    if getattr(sys.modules.get("tkinter"), "_fake", False):
        return
    tk = types.ModuleType("tkinter")
    tk._fake = True
    for name, value in globals().items():
        if isinstance(value, type) and issubclass(value, (Widget, Variable)):
            setattr(tk, name, value)
    tk.filedialog = filedialog
    tk.messagebox = messagebox
    tk.ttk = ttk
    tk.END = "end"
    sys.modules["tkinter"] = tk
    for sub, obj in (("filedialog", filedialog), ("messagebox", messagebox), ("ttk", ttk)):
        module = types.ModuleType(f"tkinter.{sub}")
        for key, value in vars(obj).items():
            setattr(module, key, value)
        sys.modules[f"tkinter.{sub}"] = module

    class FigureCanvasTkAgg:
        def __init__(self, figure, master=None):
            self.figure = figure

        def draw(self):
            pass

        def get_tk_widget(self):
            return Widget()

    class NavigationToolbar2Tk:
        def __init__(self, canvas, window):
            pass

        def update(self):
            pass

    backend = types.ModuleType("matplotlib.backends.backend_tkagg")
    backend.FigureCanvasTkAgg = FigureCanvasTkAgg
    backend.NavigationToolbar2Tk = NavigationToolbar2Tk
    sys.modules["matplotlib.backends.backend_tkagg"] = backend
