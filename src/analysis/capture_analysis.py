"""Impact-capture analysis: velocity derivation and plotting.

Pure data transform + matplotlib Figure construction -- no Tkinter here, so
this stays usable independent of the GUI. The interactive sliders/checkboxes
are matplotlib's own (backend-agnostic) widgets, not Tkinter, so they don't
compromise that separation.
"""

from matplotlib.figure import Figure
from matplotlib.widgets import CheckButtons, Slider
from scipy.signal import savgol_filter

# Velocity is a Savitzky-Golay pass (deriv=1) directly on position. A
# first-pass starting point, meant for interactive (slider) tuning against
# already-captured data.
SAVGOL_VEL_WINDOW_LENGTH = 10  # ~20ms at the 2ms sample period
SAVGOL_VEL_POLYORDER = 2

# Slider ranges. window_length's slider is stepped from an odd valmin so it
# can only ever land on odd values (scipy's requirement); 301 (~600ms) is a
# generous-but-usable static ceiling above the defaults above, clamped
# further to the capture's actual sample count.
SLIDER_WINDOW_LENGTH_MIN = 3
SLIDER_WINDOW_LENGTH_MAX = 301
SLIDER_POLYORDER_MIN = 1
SLIDER_POLYORDER_MAX = 5


def _clamp_window_length(window_length, n_samples):
    window_length = min(window_length, n_samples)
    if window_length % 2 == 0:
        window_length -= 1
    return max(window_length, 1)


def compute_velocity(
    mm_positions, sample_period_s,
    window_length=SAVGOL_VEL_WINDOW_LENGTH, polyorder=SAVGOL_VEL_POLYORDER,
):
    """Velocity (m/s), same length as mm_positions."""
    window_length = _clamp_window_length(window_length, len(mm_positions))
    polyorder = min(polyorder, window_length - 1)
    positions_m = [p / 1000 for p in mm_positions]
    return list(savgol_filter(
        positions_m, window_length=window_length, polyorder=polyorder,
        deriv=1, delta=sample_period_s,
    ))


def build_capture_figure(
    position_mm, commanded_position_mm, position_error_mm, torque_v, current_a,
    commanded_velocity_m_s, sample_period_s, info_text, feedforward_v=None,
    position_mm_nonlinear=None, calibration_text="",
):
    """Single set of axes with every field overlaid. Position L, Velocity L
    (Savitzky-Golay-derived from position), volt output/torque command, and
    coil current each get their own auto-scaled Y-axis, so those keep their
    real calibrated values regardless of the others' magnitude.

    position_mm_nonlinear is the same recorded counts put through the
    piecewise LVDT calibration instead of the straight line through the
    outermost points -- pass it to get "Position N"/"Velocity N" traces
    plotted against their L counterparts' axes, so the two calibrations are
    directly comparable on one capture. None (the default, and what you get
    with only 2 calibration points, where N would be identical to L) omits
    both N fields entirely.

    Commanded position and position error share Position L's axis/scale --
    directly comparable to actual position in the same mm units. Commanded
    velocity shares Velocity L's. feedforward_v (Current Profile mode's
    host-known OF curve, resampled to this capture's own sample_period_s --
    optional, None for every other mode) shares Volt Output's axis, so it's
    directly comparable to how much of the actual output it accounts for.

    Checkboxes toggle each field's line (and, for the fields with their own
    axis, that axis) on/off; the sliders tune the velocity Savitzky-Golay
    parameters, updating both velocity traces together. info_text (motion
    type + the parameters that produced this capture) renders below the
    plot; calibration_text renders to its left, under the plot's own left
    edge. Caller handles embedding/saving/displaying the figure."""
    times_s = [i * sample_period_s for i in range(len(position_mm))]
    velocity_m_s = compute_velocity(position_mm, sample_period_s)
    if feedforward_v is None:
        feedforward_v = [0.0] * len(position_mm)
    show_nonlinear = position_mm_nonlinear is not None
    velocity_n_m_s = (
        compute_velocity(position_mm_nonlinear, sample_period_s) if show_nonlinear else None
    )

    # (axis label, checkbox label, values, color, default-checked,
    # axis_mode). axis_mode "own" gets a dedicated Y-axis (Position L's is
    # the standard left spine; the others are offset twinx axes); any other
    # value is the CHECKBOX LABEL of an earlier "own" field, and plots
    # directly on that field's axis instead of getting one. Referencing the
    # owner by label rather than by index keeps this correct when the two
    # optional N fields are omitted and every later index shifts.
    #
    # Position/Velocity L and N default to checked -- comparing the two
    # calibrations is the point of having both. The rest start unchecked so
    # the plot isn't cluttered by default; they're a click away.
    fields = [
        ("Position L (mm)", "Position L", position_mm, "tab:blue", True, "own"),
    ]
    if show_nonlinear:
        fields.append(
            ("Position N (mm)", "Position N", position_mm_nonlinear,
             "tab:green", True, "Position L")
        )
    fields.append(("Velocity L (m/s)", "Velocity L", velocity_m_s, "tab:red", True, "own"))
    if show_nonlinear:
        fields.append(
            ("Velocity N (m/s)", "Velocity N", velocity_n_m_s,
             "tab:gray", True, "Velocity L")
        )
    fields += [
        ("Commanded Position (mm)", "Cmd Position", commanded_position_mm,
         "tab:purple", False, "Position L"),
        ("Position Error (mm)", "Pos Error", position_error_mm,
         "tab:orange", False, "Position L"),
        ("Volt Output (V)", "Volt Output", torque_v, "tab:brown", False, "own"),
        ("Current (A)", "Current", current_a, "tab:cyan", False, "own"),
        ("Commanded Velocity (m/s)", "Cmd Velocity", commanded_velocity_m_s,
         "tab:olive", False, "Velocity L"),
        ("Feedforward (V)", "Feedforward", feedforward_v, "tab:pink", False, "Volt Output"),
    ]
    field_index = {f[1]: i for i, f in enumerate(fields)}

    figure = Figure(figsize=(13, 8))
    left, right = 0.07, 0.60
    figure.subplots_adjust(left=left, right=right, top=0.93, bottom=0.40)
    # To the right of the window/polyorder sliders (which span x=[0.30,
    # 0.58]) -- anchored just under the plot's bottom edge to use the full
    # available height down to y=0, not just the sliders' own height band.
    figure.text(0.62, 0.39, info_text, fontsize=9, family="monospace", va="top", ha="left")
    # LVDT calibration block, under the plot's own left edge -- left of the
    # velocity sliders (which span x=[0.45, 0.73]), in the space the
    # removed acceleration sliders used to occupy. The sliders sit further
    # right than they used to specifically to clear this block: their
    # labels are drawn to the LEFT of their own axes, so at x=0.30 they
    # collided with the widest line of the scale table.
    if calibration_text:
        figure.text(
            0.02, 0.30, calibration_text, fontsize=8, family="monospace",
            va="top", ha="left",
        )
    main_axes_width = right - left
    # Space consecutive offset (twinx) spines this far apart in FIGURE-
    # relative terms; converted below to the axes-relative units
    # set_position(("axes", ...)) expects, since axes-relative offsets that
    # look small can still run the spine off the edge of the figure if
    # main_axes_width is narrow (bench-found: a fixed axes-relative step
    # pushed later spines past x=1.0 in figure coordinates, overlapping the
    # checkbox panel entirely).
    spine_step = 0.065 / main_axes_width

    main_axes = figure.add_subplot(111)
    main_axes.set_xlabel("Time (s)")
    main_axes.set_title("Impact capture")

    lines = []
    axes_list = []
    owns_axis = []
    twinx_count = 0
    for i, (axis_label, _, values, color, default_checked, axis_mode) in enumerate(fields):
        if axis_mode == "own":
            axes = main_axes if i == 0 else main_axes.twinx()
            if i != 0:
                axes.spines["right"].set_position(("axes", 1 + spine_step * twinx_count))
                twinx_count += 1
        else:
            axes = axes_list[field_index[axis_mode]]
        (line,) = axes.plot(times_s, values, color=color, label=axis_label)
        if axis_mode == "own":
            axes.set_ylabel(axis_label, color=color)
            axes.tick_params(axis="y", colors=color)
        if not default_checked:
            line.set_visible(False)
            if axis_mode == "own":
                axes.get_yaxis().set_visible(False)
                axes.spines["left" if i == 0 else "right"].set_visible(False)
        lines.append(line)
        axes_list.append(axes)
        owns_axis.append(axis_mode == "own")

    # Toolbar hover-coordinate readout: matplotlib's default format_coord
    # for overlaid twin axes reads the cursor's Y pixel back through each
    # axis's own scale (_AxesBase.format_coord) -- accurate only if the
    # cursor is exactly on a line. Replace it with each visible field's
    # actual sampled value at the cursor's time instead, snapping to the
    # nearest real sample (all fields share the same times_s grid) so the
    # displayed values are exact data points, not a pixel-scale readback.
    # Hidden (unchecked) fields are skipped -- lines[i].get_visible() is
    # checked live, so this stays correct as checkboxes are toggled.
    n_samples = len(times_s)

    def format_coord(x, _y):
        if x is None or n_samples == 0:
            return ""
        index = max(0, min(round(x / sample_period_s), n_samples - 1))
        parts = [f"t={times_s[index]:.4f}s"]
        for i, (_, checkbox_label, _, _, _, _) in enumerate(fields):
            if lines[i].get_visible():
                parts.append(f"{checkbox_label}={lines[i].get_ydata()[index]:.4g}")
        return "   ".join(parts)

    seen_axes_ids = set()
    for axes in axes_list:
        if id(axes) not in seen_axes_ids:
            seen_axes_ids.add(id(axes))
            axes.format_coord = format_coord

    # Checkboxes to show/hide each field -- toggles the line and, if the
    # field owns a dedicated axis, that axis's ticks/label/spine too
    # (shared fields don't touch Position's axis when toggled). Colored to
    # match each line, so the checkbox panel doubles as a legend. Sized up
    # from the small default (bench feedback: default check/frame markers
    # were hard to see/click).
    check_axes = figure.add_axes([0.82, 0.45, 0.16, 0.45])
    checkbox_labels = [f[1] for f in fields]
    colors = [f[3] for f in fields]
    actives = [f[4] for f in fields]
    check = CheckButtons(
        check_axes, checkbox_labels, actives,
        label_props={"color": colors, "fontsize": [12] * len(fields)},
        check_props={"facecolor": colors, "s": [200] * len(fields)},
        frame_props={"s": [200] * len(fields)},
    )

    def toggle(label):
        i = checkbox_labels.index(label)
        visible = not lines[i].get_visible()
        lines[i].set_visible(visible)
        if owns_axis[i]:
            axes_list[i].get_yaxis().set_visible(visible)
            axes_list[i].spines["left" if i == 0 else "right"].set_visible(visible)
        figure.canvas.draw_idle()

    check.on_clicked(toggle)

    max_window = min(SLIDER_WINDOW_LENGTH_MAX, len(position_mm))

    vel_window_slider = Slider(
        figure.add_axes([0.45, 0.20, 0.28, 0.02]), "Vel window",
        SLIDER_WINDOW_LENGTH_MIN, max_window,
        valinit=SAVGOL_VEL_WINDOW_LENGTH, valstep=2,
    )
    vel_poly_slider = Slider(
        figure.add_axes([0.45, 0.15, 0.28, 0.02]), "Vel polyorder",
        SLIDER_POLYORDER_MIN, SLIDER_POLYORDER_MAX,
        valinit=SAVGOL_VEL_POLYORDER, valstep=1,
    )

    def update_velocity(_):
        # Both velocity traces are re-derived together -- they're the same
        # smoothing applied to two calibrations of one capture, so tuning
        # the window on only one of them would make the comparison invalid.
        window, poly = int(vel_window_slider.val), int(vel_poly_slider.val)
        for label, source in (
            ("Velocity L", position_mm), ("Velocity N", position_mm_nonlinear),
        ):
            if label not in field_index:
                continue
            i = field_index[label]
            lines[i].set_ydata(compute_velocity(source, sample_period_s, window, poly))
        velocity_axes = axes_list[field_index["Velocity L"]]
        velocity_axes.relim()
        velocity_axes.autoscale_view()
        figure.canvas.draw_idle()

    vel_window_slider.on_changed(update_velocity)
    vel_poly_slider.on_changed(update_velocity)

    # Sliders/checkboxes stop responding if garbage-collected -- the Figure
    # doesn't hold a strong reference to widgets placed on it, only the
    # on_changed/on_clicked closures do (a reference cycle Python's GC can
    # still collect even while the Figure itself remains alive and
    # displayed). Stashing them here gives them a non-cyclic reference tied
    # to the figure's lifetime.
    figure._sliders = (vel_window_slider, vel_poly_slider)
    figure._checkboxes = check

    return figure


def build_sequence_figure(doc, info_text="", metrics_text=""):
    """Whole-sequence overview for one run (a capture_io.capture_document
    dict): position, commanded position and current for BOTH recording
    segments on one time axis -- the fast segment drawn as a solid line,
    the slow segment (coarser samples) as a dotted line continuing it --
    with the dwell and retract boundaries marked. Current shares a
    second Y-axis. Caller handles embedding/saving/displaying."""
    fast, slow = doc["fast"], doc["slow"]
    figure = Figure(figsize=(13, 7))
    figure.subplots_adjust(left=0.07, right=0.78, top=0.92, bottom=0.30)
    axes = figure.add_subplot(111)
    axes.set_title("Impact sequence (fast segment solid, slow segment dotted)")
    axes.set_xlabel("Time (s)")
    axes.set_ylabel("Position (mm)", color="tab:blue")
    axes.tick_params(axis="y", colors="tab:blue")
    axes.plot(fast["times_s"], fast["position_mm"], color="tab:blue", label="Position (fast)")
    axes.plot(slow["times_s"], slow["position_mm"], color="tab:blue", linestyle=":", label="Position (slow)")
    axes.plot(fast["times_s"], fast["commanded_position_mm"], color="tab:purple", alpha=0.7, label="Cmd position (fast)")
    axes.plot(slow["times_s"], slow["commanded_position_mm"], color="tab:purple", linestyle=":", alpha=0.7, label="Cmd position (slow)")
    current_axes = axes.twinx()
    current_axes.set_ylabel("Current (A)", color="tab:cyan")
    current_axes.tick_params(axis="y", colors="tab:cyan")
    current_axes.plot(fast["times_s"], fast["current_a"], color="tab:cyan", label="Current (fast)")
    current_axes.plot(slow["times_s"], slow["current_a"], color="tab:cyan", linestyle=":", label="Current (slow)")
    impact_mm = doc.get("impact_position_mm")
    if impact_mm is not None:
        axes.axhline(impact_mm, color="tab:red", linestyle="--", linewidth=0.8, label="Impact position")
    for key, label, color in (
        ("dwell_start_s", "dwell start", "tab:green"), ("retract_start_s", "retract start", "tab:orange"),
    ):
        t = doc.get(key)
        if t is not None:
            axes.axvline(t, color=color, linestyle="--", linewidth=0.8, label=label)
    handles, labels = axes.get_legend_handles_labels()
    h2, l2 = current_axes.get_legend_handles_labels()
    axes.legend(handles + h2, labels + l2, loc="upper left", bbox_to_anchor=(1.08, 1.0), fontsize=8)
    if metrics_text:
        figure.text(0.07, 0.22, metrics_text, fontsize=10, family="monospace", va="top", ha="left")
    if info_text:
        figure.text(0.07, 0.14, info_text, fontsize=7, family="monospace", va="top", ha="left")
    return figure
