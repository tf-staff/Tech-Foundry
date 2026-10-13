"""Generates the jerk-limited trajectory used by Current Profile mode --
a two-phase, two-quintic curve: 0 velocity at start_mm, ramping up to a
peak velocity exactly at peak_v_position_mm, then ramping back down to 0
velocity exactly at end_mm. Each phase is a quintic (5th-order)
"minimum-jerk" polynomial with acceleration pinned to exactly 0 at
start_mm, peak_v_position_mm, AND end_mm -- so acceleration (and
therefore the feedforward current curve derived from it, FF(t) =
m*a(t)/Kf) is continuous throughout the move, instead of stepping
instantaneously the way a constant-acceleration trapezoid would.

Peak velocity does not occur at the impact point (unlike this module's
original design, and unlike compute_contour_increments/
compute_pvt_segments' own two-phase trapezoids). Instead
impact_mm/impact_velocity_m_s describe a waypoint that the single,
continuous peak_v_position_mm->end_mm deceleration quintic must pass
through partway along the way down -- solved for numerically (peak
velocity is the one unknown pinned by that constraint), not given a
phase boundary of its own. That keeps acceleration continuous straight
through the impact point too, letting the actuator decelerate over a
longer span (starting before impact, ending after) than the plain
impact->end distance alone would allow -- useful when decelerating over
just impact->end saturates available current at high impact velocities.

Pure data/math -- no Tkinter, no gclib -- so this stays testable
independent of both the GUI and the controller connection. Deliberately
stops at raw physical units (mm, m/s^2), not counts -- unlike
compute_contour_increments/compute_pvt_segments, which convert to counts
themselves, this module's outputs get sampled at two DIFFERENT rates
(the CD grid and the FF grid) and only one of the two callers needs a
counts conversion at all.
"""

from typing import NamedTuple

from controller.controller import OF_MAX_V, OF_MIN_V


class _Phase(NamedTuple):
    """One quintic phase: position (m) and acceleration (m/s^2) as
    functions of time since this phase's own start, valid for
    t in [0, duration_s]. p(0)=p0_m, v(0)=v0, a(0)=0; p(duration_s)=p1_m
    (implied by the coefficients below), a(duration_s)=0."""

    p0_m: float
    v0: float
    c3: float
    c4: float
    c5: float
    duration_s: float

    def position_m(self, t: float) -> float:
        return self.p0_m + self.v0 * t + self.c3 * t**3 + self.c4 * t**4 + self.c5 * t**5

    def velocity_m_s(self, t: float) -> float:
        return self.v0 + 3 * self.c3 * t**2 + 4 * self.c4 * t**3 + 5 * self.c5 * t**4

    def acceleration_m_s2(self, t: float) -> float:
        return 6 * self.c3 * t + 12 * self.c4 * t**2 + 20 * self.c5 * t**3


class CurrentProfileCurve(NamedTuple):
    """The whole two-phase curve -- pure data, reusable at any sample
    rate via sample_position_mm/sample_acceleration_m_s2 below."""

    phase1: _Phase
    phase2: _Phase


def _solve_phase(p0_m: float, p1_m: float, v0: float, v1: float, duration_s: float) -> _Phase:
    # Quintic Hermite solve for boundary conditions p(0)=p0_m, v(0)=v0,
    # a(0)=0, p(T)=p1_m, v(T)=v1, a(T)=0 (a fixed 0 at both ends is what
    # makes this "minimum-jerk" and keeps acceleration continuous across
    # phase boundaries). c0=p0_m, c1=v0, c2=0 fall out of the t=0
    # conditions directly; c3/c4/c5 solved from the t=T conditions.
    t = duration_s
    delta_p = p1_m - p0_m
    delta_v = v1 - v0
    a = delta_p - v0 * t
    b = delta_v
    c3 = (10 * a - 4 * b * t) / t**3
    c4 = (7 * b * t - 15 * a) / t**4
    c5 = (6 * a - 3 * b * t) / t**5
    return _Phase(p0_m, v0, c3, c4, c5, t)


_BISECTION_ITERATIONS = 60


def _find_time_at_position(phase: _Phase, target_position_m: float, duration_s: float) -> float:
    # Bisects for t in [0, duration_s] where phase.position_m(t) ==
    # target_position_m -- position is monotonic increasing across a
    # single well-behaved deceleration-to-zero quintic phase (same
    # "single hump, non-reversing" shape already verified for this
    # module's two-phase curve), so plain bisection is safe.
    lo, hi = 0.0, duration_s
    for _ in range(_BISECTION_ITERATIONS):
        mid = (lo + hi) / 2
        if phase.position_m(mid) < target_position_m:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def _velocity_at_impact_in_phase2(
    peak_v_position_mm: float, impact_mm: float, end_mm: float, v_peak: float,
) -> float:
    # Builds the candidate peak->end phase for a trial v_peak, finds
    # where along it position reaches impact_mm, and returns the
    # velocity there. Used when impact_mm falls at/after peak_v_position_mm
    # (impact happens during deceleration).
    d2_m = (end_mm - peak_v_position_mm) / 1000
    t2 = 2 * d2_m / v_peak
    phase2 = _solve_phase(peak_v_position_mm / 1000, end_mm / 1000, v_peak, 0.0, t2)
    t_star = _find_time_at_position(phase2, impact_mm / 1000, t2)
    return phase2.velocity_m_s(t_star)


def _velocity_at_impact_in_phase1(
    start_mm: float, peak_v_position_mm: float, impact_mm: float, v_peak: float,
) -> float:
    # Mirror of _velocity_at_impact_in_phase2, for when impact_mm falls
    # before peak_v_position_mm (impact happens during acceleration, not
    # yet at peak) -- builds the candidate start->peak phase for a trial
    # v_peak, finds where along it position reaches impact_mm, and
    # returns the velocity there.
    d1_m = (peak_v_position_mm - start_mm) / 1000
    t1 = 2 * d1_m / v_peak
    phase1 = _solve_phase(start_mm / 1000, peak_v_position_mm / 1000, 0.0, v_peak, t1)
    t_star = _find_time_at_position(phase1, impact_mm / 1000, t1)
    return phase1.velocity_m_s(t_star)


def _solve_peak_velocity(
    start_mm: float, peak_v_position_mm: float, impact_mm: float, end_mm: float,
    impact_velocity_m_s: float,
) -> float:
    # Peak velocity is the one unknown that makes a single, continuous
    # quintic -- start->peak if impact falls before peak_v_position_mm,
    # peak->end otherwise (equality routes to peak->end; both branches
    # agree exactly at that boundary, so this is continuous) -- pass
    # through exactly (impact_mm, impact_velocity_m_s) partway through.
    # No closed form -- inverting a quintic's position-to-time relation
    # doesn't have one in general -- so bisect. residual(v_peak) is
    # monotonically increasing in v_peak in either branch (a faster peak
    # means more speed to cover the same fixed distance in less time, so
    # velocity at any fixed intermediate position is higher too), so
    # plain bisection is safe once bracketed.
    if impact_mm >= peak_v_position_mm:
        def residual(v_peak):
            return _velocity_at_impact_in_phase2(
                peak_v_position_mm, impact_mm, end_mm, v_peak,
            ) - impact_velocity_m_s
    else:
        def residual(v_peak):
            return _velocity_at_impact_in_phase1(
                start_mm, peak_v_position_mm, impact_mm, v_peak,
            ) - impact_velocity_m_s

    lo = impact_velocity_m_s
    hi = impact_velocity_m_s * 2
    for _ in range(_BISECTION_ITERATIONS):
        if residual(hi) > 0:
            break
        hi *= 2
    else:
        raise ValueError(
            "Could not find a Peak Velocity Position solution for the given "
            "Impact Position/Velocity -- check the Start/Peak/Impact/End ordering"
        )

    for _ in range(_BISECTION_ITERATIONS):
        mid = (lo + hi) / 2
        if residual(mid) < 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def compute_current_profile_curve(
    start_mm: float, peak_v_position_mm: float, impact_mm: float, end_mm: float,
    impact_velocity_m_s: float,
) -> CurrentProfileCurve:
    """impact_velocity_m_s must be positive, peak_v_position_mm must
    satisfy start_mm < peak_v_position_mm < end_mm, and impact_mm must
    satisfy start_mm < impact_mm < end_mm -- independently; peak and
    impact may fall in either order. Impact velocity/position are always
    honored exactly (see _solve_peak_velocity) regardless of that order.

    Peak velocity is not a direct input -- it's solved for numerically
    (_solve_peak_velocity) so that whichever quintic phase impact_mm
    actually falls in (start->peak if impact_mm < peak_v_position_mm,
    peak->end otherwise) passes through exactly (impact_mm,
    impact_velocity_m_s) partway through. When impact falls in the
    peak->end phase, that phase is one continuous quintic never broken
    at impact_mm, so acceleration stays continuous straight through the
    impact point -- no zero-acceleration dip there the way there would
    be if impact were its own phase boundary -- letting the actuator
    decelerate over a longer distance (starting before impact, ending
    after) than the plain impact->end span alone would allow, to avoid
    saturating current at high impact velocities."""
    if impact_velocity_m_s <= 0:
        raise ValueError("Impact velocity must be positive")
    if not (start_mm < peak_v_position_mm < end_mm):
        raise ValueError(
            "Peak Velocity Position must satisfy: start < peak velocity position < end"
        )
    if not (start_mm < impact_mm < end_mm):
        raise ValueError("Impact Position must satisfy: start < impact position < end")

    v_peak = _solve_peak_velocity(
        start_mm, peak_v_position_mm, impact_mm, end_mm, impact_velocity_m_s,
    )

    d2_m = (end_mm - peak_v_position_mm) / 1000
    t2 = 2 * d2_m / v_peak
    phase2 = _solve_phase(peak_v_position_mm / 1000, end_mm / 1000, v_peak, 0.0, t2)

    d1_m = (peak_v_position_mm - start_mm) / 1000
    t1 = 2 * d1_m / v_peak
    phase1 = _solve_phase(start_mm / 1000, peak_v_position_mm / 1000, 0.0, v_peak, t1)

    return CurrentProfileCurve(phase1, phase2)


def _sample_phase(phase: _Phase, sample_period_s: float, evaluate) -> list:
    # i in [1, n]: the first sample needs no t=0 point, since that's
    # wherever the axis already is (either the redundant retract to
    # start_mm, or exactly where phase1 left off) -- same convention as
    # compute_contour_increments. The last sample is clamped to the
    # phase's exact duration rather than n*sample_period_s, to land
    # exactly on the waypoint despite sample_period_s not evenly
    # dividing duration_s -- harmless here (no interpolation is declared
    # alongside these values the way PVT's PVA segments do; each is just
    # a plain data point).
    n = max(1, round(phase.duration_s / sample_period_s))
    return [
        evaluate(phase, i * sample_period_s if i < n else phase.duration_s)
        for i in range(1, n + 1)
    ]


def _find_accel_peak_time(phase: _Phase) -> float:
    # Bisects for the time in (0, duration_s) where |acceleration| peaks
    # (jerk = da/dt crosses zero) -- the single interior extremum
    # already relied on elsewhere in this module (a(0)=a(T)=0, single-
    # humped in between). Splits the phase into its Feedforward-stretch
    # "rising" (0..t_peak) and "falling" (t_peak..T) halves -- see
    # _sample_phase_stretched.
    def jerk(t):
        return 6 * phase.c3 + 24 * phase.c4 * t + 60 * phase.c5 * t ** 2

    lo, hi = 0.0, phase.duration_s
    j0_positive = jerk(0.0) > 0
    for _ in range(_BISECTION_ITERATIONS):
        mid = (lo + hi) / 2
        if (jerk(mid) > 0) == j0_positive:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def _sample_phase_stretched(
    phase: _Phase, sample_period_s: float, rise_stretch: float, fall_stretch: float, evaluate,
) -> list:
    # Like _sample_phase, but independently rescales the phase's rising
    # (0..t_peak) and falling (t_peak..duration_s) acceleration halves
    # in time before sampling -- 1.0/1.0 reproduces _sample_phase's own
    # sample times exactly. Only ever used for the Feedforward array
    # (compute_feedforward_v) -- never for sample_position_mm/
    # sample_acceleration_m_s2 -- so the physical CD move this pairs
    # with on the controller is completely unaffected by these factors.
    t_peak = _find_accel_peak_time(phase)
    rise_s = t_peak * rise_stretch
    fall_s = (phase.duration_s - t_peak) * fall_stretch
    total_s = rise_s + fall_s
    n = max(1, round(total_s / sample_period_s))

    def unwarp(tau):
        # Maps stretched/"virtual" time tau back to the phase's own real
        # time t, so evaluate() still samples the one true acceleration
        # curve -- just at a pace that spends more/less real Feedforward
        # playback time on each half than the physical move itself does.
        if tau <= rise_s:
            return tau / rise_stretch
        return t_peak + (tau - rise_s) / fall_stretch

    return [
        evaluate(phase, unwarp(i * sample_period_s if i < n else total_s))
        for i in range(1, n + 1)
    ]


def sample_position_mm(curve: CurrentProfileCurve, sample_period_s: float) -> list:
    """Position (mm), sampled every sample_period_s across the whole
    move. Raw mm values -- convert to counts/increments at the call
    site (current_profile_program.py), same as
    compute_contour_increments does internally for its own CD lines."""
    return [
        *_sample_phase(curve.phase1, sample_period_s, lambda p, t: p.position_m(t) * 1000),
        *_sample_phase(curve.phase2, sample_period_s, lambda p, t: p.position_m(t) * 1000),
    ]


def sample_acceleration_m_s2(curve: CurrentProfileCurve, sample_period_s: float) -> list:
    """Acceleration (m/s^2), sampled every sample_period_s across the
    whole move -- feeds the FF(t) = m*a(t)/Kf conversion."""
    return [
        *_sample_phase(curve.phase1, sample_period_s, lambda p, t: p.acceleration_m_s2(t)),
        *_sample_phase(curve.phase2, sample_period_s, lambda p, t: p.acceleration_m_s2(t)),
    ]


def compute_feedforward_v(
    curve: CurrentProfileCurve, sample_period_s: float,
    mass_kg: float, kf_accel: float, kf_decel: float,
    accel_rise_stretch: float = 1.0, accel_fall_stretch: float = 1.0,
    decel_rise_stretch: float = 1.0, decel_fall_stretch: float = 1.0,
) -> list:
    """FF(t) = -mass_kg * a(t) / kf, sampled every sample_period_s across
    the whole move -- clamped to OF's own documented range (Galil
    firmware reference, OF_MIN_V/OF_MAX_V in controller.py) rather than
    sent out-of-range and left to the controller's own (undocumented
    here) clamping behavior.

    Two separate Kf values, one per phase -- kf_accel for phase 1
    (ramping up toward impact), kf_decel for phase 2 (ramping down to
    end) -- since the actuator's real force constant can differ between
    the two current directions (bench-observed: Volt Output didn't
    track the single-Kf Feedforward curve equally well in both phases).

    Negated to match this project's established sign convention (see
    impact_program.py's own voltage fields: "Impact Accel Primary
    Voltage [<=0]" drives toward increasing position/impact, "Pre
    Impact Retract Voltage [>=0]" drives toward decreasing position) --
    negative voltage produces positive-direction force/acceleration, so
    a positive commanded acceleration needs a negative feedforward
    voltage, not a positive one.

    accel_rise_stretch/accel_fall_stretch/decel_rise_stretch/
    decel_fall_stretch (each 1.0 = unchanged): independently time-
    stretch the 4 quarter-segments of this curve (phase 1's rising-
    then-falling acceleration hump, phase 2's rising-then-falling
    deceleration-magnitude hump -- see _sample_phase_stretched/
    _find_accel_peak_time) -- bench-motivated (actual velocity coming
    out narrower/"pointier" than the broad-topped commanded velocity)
    knobs for reshaping the Feedforward current's own timeline without
    touching the physical move at all (sample_position_mm is untouched
    by these)."""
    def convert(phase, kf, rise_stretch, fall_stretch):
        return [
            max(OF_MIN_V, min(OF_MAX_V, -mass_kg * a / kf))
            for a in _sample_phase_stretched(
                phase, sample_period_s, rise_stretch, fall_stretch,
                lambda p, t: p.acceleration_m_s2(t),
            )
        ]
    return (
        convert(curve.phase1, kf_accel, accel_rise_stretch, accel_fall_stretch)
        + convert(curve.phase2, kf_decel, decel_rise_stretch, decel_fall_stretch)
    )
