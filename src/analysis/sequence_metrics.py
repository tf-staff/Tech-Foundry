"""Numbers an operator wants after a run, derived from a sequence
capture: velocity at the impact position, peak extension, rebound,
measured dwell. Pure data -- no plotting, no hardware.

Velocity comes from the same Savitzky-Golay derivative the plots use
(capture_analysis.compute_velocity), so a number reported here matches
the Velocity L trace at the same instant.
"""

from analysis.capture_analysis import compute_velocity


def _first_crossing(values, threshold):
    """Index of the first sample at or past `threshold` (rising), or
    None."""
    for i, v in enumerate(values):
        if v >= threshold:
            return i
    return None


def impact_metrics(
    position_mm, times_s, sample_period_s, impact_position_mm,
    dwell_start_s=None, retract_start_s=None,
    window_length=None, polyorder=None,
) -> dict:
    """position_mm/times_s: the FAST segment. Returns a dict with:

        impact_index / impact_time_s   first sample at/after the impact
                                       position (None if never reached)
        impact_velocity_m_s            Savitzky-Golay velocity there
        peak_velocity_m_s              max velocity before the peak
                                       position
        peak_position_mm / peak_time_s max extension
        rebound_mm                     peak minus the lowest position
                                       after the peak (within the fast
                                       segment) -- bounce off the stop
        dwell_measured_s               retract_start - dwell_start when
                                       both markers are given
    """
    kwargs = {}
    if window_length is not None:
        kwargs["window_length"] = window_length
    if polyorder is not None:
        kwargs["polyorder"] = polyorder
    metrics = {
        "impact_index": None, "impact_time_s": None, "impact_velocity_m_s": None,
        "peak_velocity_m_s": None, "peak_position_mm": None, "peak_time_s": None,
        "rebound_mm": None, "dwell_measured_s": None,
    }
    if len(position_mm) < 3:
        return metrics
    velocity = compute_velocity(position_mm, sample_period_s, **kwargs)
    peak_index = max(range(len(position_mm)), key=lambda i: position_mm[i])
    metrics["peak_position_mm"] = position_mm[peak_index]
    metrics["peak_time_s"] = times_s[peak_index]
    metrics["peak_velocity_m_s"] = max(velocity[: peak_index + 1])
    after = position_mm[peak_index:]
    metrics["rebound_mm"] = position_mm[peak_index] - min(after)
    impact_index = _first_crossing(position_mm, impact_position_mm)
    if impact_index is not None:
        metrics["impact_index"] = impact_index
        metrics["impact_time_s"] = times_s[impact_index]
        metrics["impact_velocity_m_s"] = velocity[impact_index]
    if dwell_start_s is not None and retract_start_s is not None:
        metrics["dwell_measured_s"] = retract_start_s - dwell_start_s
    return metrics


def format_metrics(metrics: dict) -> str:
    def fmt(value, unit, digits=3):
        return "n/a" if value is None else f"{value:.{digits}f}{unit}"
    return (
        f"Impact velocity: {fmt(metrics.get('impact_velocity_m_s'), ' m/s')}   "
        f"Peak velocity: {fmt(metrics.get('peak_velocity_m_s'), ' m/s')}\n"
        f"Peak position: {fmt(metrics.get('peak_position_mm'), ' mm', 2)}   "
        f"Rebound: {fmt(metrics.get('rebound_mm'), ' mm', 2)}   "
        f"Dwell: {fmt(None if metrics.get('dwell_measured_s') is None else metrics['dwell_measured_s'] * 1000, ' ms', 1)}"
    )
