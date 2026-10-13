"""Impact profile files -- the hand-off between the testing GUI (which
develops/calibrates a motion profile for one tip/velocity/depth
combination) and the user GUI (which runs it).

A profile is a plain JSON document. The testing GUI's "Save Profile"
writes one from its fields; the user GUI lists the ones in its profiles
directory and compiles the selected one with sequence/impact_sequence.py.
Nothing here touches hardware or Tkinter.

Layout (see default_profile() for every key and its default):

    format / version        file identification
    name, tip, notes        free text; `tip` and `name` are what the
                            operator picks by
    impact_velocity_m_s     nominal impact velocity (also the trajectory
                            constraint)
    impact_depth_mm         nominal depth = stop - impact position
    motion                  trajectory + feedforward parameters, Motion/
                            Rest PID -- the Current Profile section's
                            fields, one for one
      .mode                 which compiler builds the program
                            (impact_sequence.COMPILERS); "current_profile_
                            sequence" is the only one today. A different
                            motion program can be plugged in by adding a
                            compiler under a new mode without touching
                            either GUI.
    sequence                conditioning pulse, dwell/hold, retract,
                            recording options
    amp                     amp full-scale current (CME Analog Input
                            Scale) the profile was developed with -- the
                            amps<->volts conversion every current field
                            depends on
    calibration             LVDT calibration points in effect when the
                            profile was saved (reference only; the user
                            GUI keeps its own live calibration)
"""

import copy
import datetime
import json
from pathlib import Path

from controller.controller import (
    Controller,
    DEFAULT_AMP_FULL_SCALE_CURRENT_A,
    DEFAULT_MOTION_KD,
    DEFAULT_MOTION_KI,
    DEFAULT_MOTION_KP,
    DEFAULT_MOTION_OFFSET_V,
    DEFAULT_REST_KD,
    DEFAULT_REST_KI,
    DEFAULT_REST_KP,
    DEFAULT_REST_OFFSET_V,
    FULL_STROKE_MM,
    SLOW_ACCEL_M_S2,
    SLOW_DECEL_M_S2,
    SLOW_RETRACT_MARGIN_MM,
    SLOW_SPEED_M_S,
)
from program.conditioning_program import MODE2_PULSE_CURRENT_A, MODE2_PULSE_MS

PROFILE_FORMAT = "cci-impact-profile"
PROFILE_VERSION = 1
PROFILE_SUFFIX = ".json"

DWELL_MIN_MS = 100.0
DWELL_MAX_MS = 1000.0
DEFAULT_DWELL_MS = 200.0
DEFAULT_FAST_TAIL_MS = 20.0


def default_profile() -> dict:
    """A complete profile with every key present. Trajectory fields that
    have no sensible default (velocity, positions, mass, Kf) are left
    None and fail validation until filled in."""
    return {
        "format": PROFILE_FORMAT,
        "version": PROFILE_VERSION,
        "name": "",
        "tip": "",
        "notes": "",
        "created": "",
        "impact_velocity_m_s": None,
        "impact_depth_mm": None,
        "motion": {
            "mode": "current_profile_sequence",
            "start_mm": 0.0,
            "stop_mm": round(FULL_STROKE_MM, 2),
            "impact_position_mm": None,
            "peak_velocity_position_mm": None,
            "mass_kg": None,
            "kf_accel": None,
            "kf_decel": None,
            "ff_stretch": {
                "accel_rise": 1.0, "accel_fall": 1.0, "decel_rise": 1.0, "decel_fall": 1.0,
            },
            "ff_advance_us": 0.0,
            "pre_pause_ms": 2,
            "post_pause_ms": 2,
            "motion_pid": {
                "kp": DEFAULT_MOTION_KP, "kd": DEFAULT_MOTION_KD, "ki": DEFAULT_MOTION_KI,
                "offset_v": DEFAULT_MOTION_OFFSET_V,
            },
            "rest_pid": {
                "kp": DEFAULT_REST_KP, "kd": DEFAULT_REST_KD, "ki": DEFAULT_REST_KI,
                "offset_v": DEFAULT_REST_OFFSET_V,
            },
        },
        "sequence": {
            "condition": {
                "enabled": True,
                "current_a": MODE2_PULSE_CURRENT_A,
                "duration_ms": MODE2_PULSE_MS,
                "settle_ms": 50.0,
                "position_mm": 0.0,
            },
            "dwell": {
                "duration_ms": DEFAULT_DWELL_MS,
                "hold_current_a": 0.0,
                "hold_servo": False,
                "fast_tail_ms": DEFAULT_FAST_TAIL_MS,
            },
            "retract": {
                "kick_current_a": 0.0,
                "kick_ms": 0.0,
                "pid": {
                    "kp": DEFAULT_REST_KP, "kd": DEFAULT_REST_KD, "ki": DEFAULT_REST_KI,
                    "offset_v": DEFAULT_REST_OFFSET_V,
                },
                "speed_m_s": SLOW_SPEED_M_S,
                "accel_m_s2": SLOW_ACCEL_M_S2,
                "decel_m_s2": SLOW_DECEL_M_S2,
                # Same 2mm inset the Slow Retract button uses -- the coil
                # can't reliably hold right at the retract limit.
                "target_mm": SLOW_RETRACT_MARGIN_MM,
            },
            "recording": {"circular": True},
            "disable_at_end": True,
        },
        "amp": {"full_scale_current_a": DEFAULT_AMP_FULL_SCALE_CURRENT_A},
        "calibration": {"points": []},
    }


def merge_defaults(profile: dict) -> dict:
    """A deep copy of `profile` with every missing key filled from
    default_profile() -- so an older/partial file still has every field
    the compiler reads."""
    def merge(base, override):
        if not isinstance(base, dict) or not isinstance(override, dict):
            return copy.deepcopy(override)
        merged = copy.deepcopy(base)
        for key, value in override.items():
            merged[key] = merge(base.get(key), value) if key in base else copy.deepcopy(value)
        return merged
    return merge(default_profile(), profile or {})


def _number(value, name, errors, *, minimum=None, maximum=None, positive=False, nonzero=False):
    if value is None or isinstance(value, bool):
        errors.append(f"{name} is missing")
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        errors.append(f"{name} must be a number")
        return None
    if positive and value <= 0:
        errors.append(f"{name} must be positive")
    if nonzero and value == 0:
        errors.append(f"{name} must not be zero")
    if minimum is not None and value < minimum:
        errors.append(f"{name} must be at least {minimum}")
    if maximum is not None and value > maximum:
        errors.append(f"{name} must be at most {maximum}")
    return value


def validate_profile(profile: dict) -> list:
    """Every problem found, as human-readable strings -- an empty list
    means the profile can be compiled."""
    errors = []
    p = merge_defaults(profile)
    if p.get("format") != PROFILE_FORMAT:
        errors.append(f"Not a {PROFILE_FORMAT} file")
    m = p["motion"]
    if m.get("mode") not in KNOWN_MODES:
        errors.append(f"Unknown motion mode {m.get('mode')!r}")
    start = _number(m["start_mm"], "Start", errors, minimum=0.0)
    stop = _number(m["stop_mm"], "Stop", errors, maximum=FULL_STROKE_MM + 1e-6)
    _number(p["impact_velocity_m_s"], "Impact velocity", errors, positive=True)
    _number(p["impact_depth_mm"], "Impact depth", errors, positive=True)
    impact = _number(m["impact_position_mm"], "Impact position", errors)
    peak = _number(m["peak_velocity_position_mm"], "Peak velocity position", errors)
    if None not in (start, stop):
        if not start < stop:
            errors.append("Start must be less than Stop")
        else:
            if impact is not None and not (start < impact < stop):
                errors.append("Impact position must lie strictly between Start and Stop")
            if peak is not None and not (start < peak < stop):
                errors.append("Peak velocity position must lie strictly between Start and Stop")
    _number(m["mass_kg"], "Mass", errors, positive=True)
    _number(m["kf_accel"], "Force constant (accel)", errors, nonzero=True)
    _number(m["kf_decel"], "Force constant (decel)", errors, nonzero=True)
    for key in ("accel_rise", "accel_fall", "decel_rise", "decel_fall"):
        _number(m["ff_stretch"][key], f"FF stretch {key}", errors, positive=True)
    _number(m["ff_advance_us"], "Feedforward advance", errors, minimum=0.0)
    _number(m["pre_pause_ms"], "Pre-move pause", errors, minimum=0.0)
    _number(m["post_pause_ms"], "Post-move pause", errors, minimum=0.0)
    for group in ("motion_pid", "rest_pid"):
        for key in ("kp", "kd", "ki", "offset_v"):
            _number(m[group][key], f"{group}.{key}", errors)
    s = p["sequence"]
    c = s["condition"]
    _number(c["current_a"], "Condition pulse current", errors)
    _number(c["duration_ms"], "Condition pulse duration", errors, minimum=0.0)
    _number(c["settle_ms"], "Condition settle", errors, minimum=0.0)
    _number(c["position_mm"], "Condition position", errors, minimum=0.0, maximum=FULL_STROKE_MM + 1e-6)
    d = s["dwell"]
    _number(d["duration_ms"], "Dwell", errors, minimum=0.0)
    _number(d["hold_current_a"], "Hold current", errors)
    _number(d["fast_tail_ms"], "Fast capture tail", errors, minimum=0.0)
    r = s["retract"]
    _number(r["kick_current_a"], "Retract kick current", errors)
    _number(r["kick_ms"], "Retract kick duration", errors, minimum=0.0)
    for key in ("kp", "kd", "ki", "offset_v"):
        _number(r["pid"][key], f"retract pid.{key}", errors)
    _number(r["speed_m_s"], "Retract speed", errors, positive=True)
    _number(r["accel_m_s2"], "Retract accel", errors, positive=True)
    _number(r["decel_m_s2"], "Retract decel", errors, positive=True)
    _number(r["target_mm"], "Retract target", errors, minimum=0.0, maximum=FULL_STROKE_MM + 1e-6)
    _number(p["amp"]["full_scale_current_a"], "Amp full-scale current", errors, positive=True)
    return errors


# Motion modes a profile may name. impact_sequence.COMPILERS maps each to
# its compiler; kept here (as names only) so validation doesn't import
# the compiler module.
KNOWN_MODES = ("current_profile_sequence",)


def profile_impact_position_mm(profile: dict) -> float:
    """Explicit impact position if present, else Stop minus depth."""
    m = profile["motion"]
    if m.get("impact_position_mm") is not None:
        return float(m["impact_position_mm"])
    return float(m["stop_mm"]) - float(profile["impact_depth_mm"])


def profile_label(profile: dict) -> str:
    tip = profile.get("tip") or "?"
    v = profile.get("impact_velocity_m_s")
    depth = profile.get("impact_depth_mm")
    v_text = f"{float(v):g} m/s" if v is not None else "? m/s"
    d_text = f"{float(depth):g} mm" if depth is not None else "? mm"
    return f"{profile.get('name') or 'unnamed'}  [{tip}, {v_text}, {d_text}]"


def load_profile(path) -> dict:
    """Reads, fills defaults, validates. Raises ValueError with every
    problem listed if the file can't be compiled."""
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: not a profile object")
    profile = merge_defaults(raw)
    errors = validate_profile(profile)
    if errors:
        raise ValueError(f"{Path(path).name}: " + "; ".join(errors))
    return profile


def save_profile(profile: dict, path) -> dict:
    """Writes `profile` (with defaults filled, a timestamp and the live
    LVDT calibration points) as JSON. Validates first. Returns what was
    written."""
    profile = merge_defaults(profile)
    errors = validate_profile(profile)
    if errors:
        raise ValueError("; ".join(errors))
    profile["created"] = datetime.datetime.now().isoformat(timespec="seconds")
    profile["calibration"] = {"points": [list(pt) for pt in Controller.calibration_points()]}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(profile, f, indent=2)
    return profile


def list_profiles(directory) -> list:
    """(path, profile) for every loadable profile in `directory`, sorted
    by file name; unreadable/invalid files are returned as
    (path, ValueError) so the caller can show why."""
    directory = Path(directory)
    results = []
    if not directory.is_dir():
        return results
    for path in sorted(directory.glob(f"*{PROFILE_SUFFIX}")):
        try:
            results.append((path, load_profile(path)))
        except (OSError, ValueError, json.JSONDecodeError) as e:
            results.append((path, ValueError(str(e))))
    return results
