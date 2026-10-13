"""Controller interface layer for the Galil DMC-31011.

All gclib interaction lives here. No Tkinter import — this module must stay
usable headless (see src/headless/connect_and_read.py) and from the GUI
layer alike (project_spec_v1.txt, section 2).
"""

from typing import NamedTuple

import gclib

DEFAULT_ADDRESS = "192.168.1.20"

# Bounds worst-case GUI freeze from a stalled read; gclib's default is 5000ms.
SETUP_TIMEOUT_MS = 500

# Program downloads round-trip once per line (send + ACK), so a whole
# program's handshake can easily exceed SETUP_TIMEOUT_MS -- bench-confirmed:
# at 500ms, GProgramDownload returned without error but left a stray ACK
# (0x06) burst in the receive buffer, which then corrupted the next
# unrelated command's response (a poll-loop TP read came back as a string
# of 0x06 bytes instead of a position). Use a longer timeout just for the
# download itself; see download_program().
PROGRAM_DOWNLOAD_TIMEOUT_MS = 10000

# SETUP_COMMANDS puts analog input 1 on the +/-10V range (AQ 1,2) and feeds
# it to the axis as position (AF1), where the 16-bit reading spans the full
# range -- so raw counts and the LVDT voltage read back by "MG @AN[1]" are
# two views of one number. The calibration is entered in volts (what you
# read on the bench) and stored in counts (what the recordings contain),
# and this is the conversion between them.
COUNTS_PER_VOLT = 65536 / 20.0

# Calibration points for counts_to_mm, measured on the bench. The LVDT
# passes through its null partway along the stroke, so the retract end
# reads positive and the extend end negative.
RETRACT_VOLTS = 8.06
RETRACT_MM = 0.00
EXTEND_VOLTS = -3.90
EXTEND_MM = 39.85
RETRACT_COUNTS = RETRACT_VOLTS * COUNTS_PER_VOLT
EXTEND_COUNTS = EXTEND_VOLTS * COUNTS_PER_VOLT
FULL_STROKE_MM = EXTEND_MM - RETRACT_MM
# Used below to derive DEFAULT_SPEED/ACCEL/DECEL_COUNTS_PER_S(2) from
# physical-unit defaults -- same scale as Controller.meters_to_counts.
_COUNTS_PER_METER = (RETRACT_COUNTS - EXTEND_COUNTS) / FULL_STROKE_MM * 1000

# _TTA (recorded torque/output command, see start_recording()) is a raw
# signed value, not already in volts: -32767 counts = -10V commanded
# output, +32767 counts = +10V. Same full-scale magnitude both directions.
TORQUE_COUNTS_FULL_SCALE = 32767
TORQUE_VOLTS_FULL_SCALE = 10.0

# Volt Output is the analog reference voltage sent to the external amp
# (Copley Xenus XTL-230-36, .claude/reference/NCM15-15-032-2LBT.txt), which
# is configured in current mode (Desired State=2). Analog Input Scale
# (param 0x19, Parameter Dictionary.pdf) sets how many Amps the amp
# commands at a full-scale 10V reference -- this is a CME-side setting,
# not something the Galil controls, so it's GUI-editable rather than a
# fixed constant (see Controller.set_amp_current_per_volt).
DEFAULT_AMP_FULL_SCALE_CURRENT_A = 6.96
AMP_FULL_SCALE_CURRENT_MIN_A = 0.0
AMP_FULL_SCALE_CURRENT_MAX_A = 36.0

# No Galil software travel limits (BL/FL) in this application -- the app
# needs to be able to command the true physical extremes of travel without
# the controller rejecting anything, so range enforcement is pure
# Python-side checking against [0, FULL_STROKE_MM] instead (see app.py).
# This margin used to exist to dodge a Galil bug (error 22, "begin not
# possible due to limit switch") specific to re-issuing BG in the same
# direction while sitting exactly at an *enforced* BL/FL boundary -- with
# no BL/FL sent at all, that bug can't occur. Kept at 0 (not deleted) in
# case a similar margin is needed again later. Still governs the
# Extend/Stop-direction inset (unchanged) -- the retract direction uses
# the two margins below instead, since the voice coil can't reliably hold
# position right at the true mechanical retract limit (bench-found:
# reduced/nonlinear force output very close to the end of stroke, compounded
# by the vertical mount's gravity load -- not a control/tuning issue).
TRAVEL_MARGIN_MM = 0

# Slow Retract button, and Arm's/Contoured Motion Arm's host-side pre-arm
# retract (both go through _slow_move_to_limit in app.py).
SLOW_RETRACT_MARGIN_MM = 2.0

# Pre-enable setup, validated on the bench (example_dmc/(UC Davis) Slow
# DEMO.dmc): analog feedback, amp/motor/sign convention, PID gains, and
# speed/accel/decel. Params live in RAM and aren't guaranteed to survive a
# power cycle, so this is re-sent on every connect() rather than assumed to
# already be set on the controller. All of this must be in place *before*
# enable() is ever reachable — without it, SHA can enable the servo loop
# with an undefined sign convention and no speed limiting, driving hard
# until the mechanical stop.
DEFAULT_REST_KD = 0.5
DEFAULT_REST_KP = 0.5
DEFAULT_REST_KI = 0.01
DEFAULT_MOTION_KD = 10.0
DEFAULT_MOTION_KP = 3.0
DEFAULT_MOTION_KI = 0.01
# OF -- bias voltage added to the command output alongside KP/KD/KI,
# e.g. for gravity compensation. Galil range: OF_MIN_V to OF_MAX_V.
DEFAULT_REST_OFFSET_V = 0.2
DEFAULT_MOTION_OFFSET_V = 0.2
OF_MIN_V = -9.9982
OF_MAX_V = 9.9982

# FV -- velocity feedforward gain (Motion PID only, applies in Contour
# Mode). Bias is recomputed from commanded velocity every servo cycle, so
# a single static gain is inherently dynamic across a move.
DEFAULT_MOTION_FV = 0.0
FV_MIN = 0.0
FV_MAX = 8191.75

# PVT Motion's own FV/FA -- FA does not operate in Contour Mode but does
# in PVT mode (firmware reference), where it adds a torque bias scaled by
# AC (accel phase) or DC (decel phase): Bias = FA * AC * 1.5e-7 *
# (TM/1000)^2. Same range/resolution as FV.
DEFAULT_PVT_FV = 0.0
DEFAULT_PVT_FA = 0.0
FA_MIN = 0.0
FA_MAX = 8191.75

# End-of-program settle time before auto-disabling the motor, used by both
# Open Loop Motion and Contoured Motion after Rest gains/offset restore.
POST_TEST_DISABLE_DELAY_MS = 300

DEFAULT_SPEED_M_S = 0.02
DEFAULT_ACCEL_M_S2 = 0.1
DEFAULT_DECEL_M_S2 = 0.1
DEFAULT_SPEED_COUNTS_PER_S = round(DEFAULT_SPEED_M_S * _COUNTS_PER_METER)  # SP
DEFAULT_ACCEL_COUNTS_PER_S2 = round(DEFAULT_ACCEL_M_S2 * _COUNTS_PER_METER)  # AC
DEFAULT_DECEL_COUNTS_PER_S2 = round(DEFAULT_DECEL_M_S2 * _COUNTS_PER_METER)  # DC

# Fixed conservative profile for the GUI's Slow Retract/Extend buttons --
# deliberately decoupled from the Speed/Accel/Decel fields so a fast profile
# dialed in for impact-capture testing never carries over to a full-stroke
# repositioning move.
SLOW_SPEED_M_S = 0.1
SLOW_ACCEL_M_S2 = 0.2
SLOW_DECEL_M_S2 = 0.2

# Galil TM command: servo sampling period, usec. Valid range for this
# DMC-31011 per spec is 125-20000usec. DEFAULT_SAMPLE_PERIOD_US is the
# Rest/default rate; the GUI's Motion field can request a different rate for
# the duration of a move (see app.py's Rest/Motion PID split for the same
# switching pattern).
TM_MIN_US = 125
TM_MAX_US = 20000
DEFAULT_SAMPLE_PERIOD_US = 1000.0

SETUP_COMMANDS = (
    "AQ 1,2",  # analog input 1 range: +/-10V, full travel range
    "AF1",  # enable analog input 1 as axis position feedback (LVDT, not an encoder)
    "BR 1",  # brush type servo axis
    "MT -1",  # motor type servo with reverse polarity
    "CE 0",  # encoder configuration (unused for feedback here, but part of the validated sign-convention header)
    "CN-1",  # limit/home switch input polarity convention
    f"KD {DEFAULT_REST_KD}",  # position-loop derivative gain
    f"KP {DEFAULT_REST_KP}",  # position-loop proportional gain
    f"KI {DEFAULT_REST_KI}",  # position-loop integral gain
    f"SP {DEFAULT_SPEED_COUNTS_PER_S}",  # move speed, counts/s
    f"AC {DEFAULT_ACCEL_COUNTS_PER_S2}",  # acceleration, counts/s^2
    f"DC {DEFAULT_DECEL_COUNTS_PER_S2}",  # deceleration, counts/s^2
    f"TM {DEFAULT_SAMPLE_PERIOD_US}",  # servo sampling period, usec
)

# Galil DMC command reference: valid SP (speed) range for servo motors is
# 0 to 22,000,000 counts/s. AC/DC (accel/decel) range is 1024 to
# 1,073,740,800 counts/s^2, and the controller rounds values down to the
# nearest factor of 1024.
SP_MIN_COUNTS_PER_S = 0
SP_MAX_COUNTS_PER_S = 22_000_000
ACCEL_MIN_COUNTS_PER_S2 = 1024
ACCEL_MAX_COUNTS_PER_S2 = 1_073_740_800

# Galil built-in real-time data recording (DM/RA/RD/RC), driven directly via
# GCommand + GArrayUpload. start_recording() uses "RC 1" (m omitted, so
# recording is non-circular and auto-sized to the DM array -- always in
# sync with RECORD_ARRAY_SIZE, no risk of a stale mismatched value).
# "RC 1" at n=1 records every servo cycle -- i.e. every TM interval, not
# every 2nd cycle as the "2^n samples between records" doc language first
# suggested -- confirmed via stopwatch-timed moves and cross-checking
# plotted velocity against the commanded SP while TM was fixed at its
# default (1000usec/1ms), both of which were off by exactly 2x at an
# assumed 2ms period. TM is now dynamic (app.py's Motion field can request
# a different rate for a move), so the actual per-capture sample period is
# whatever TM was set to for that capture, not a fixed constant here.
#
# RECORD_ARRAY_SIZE=3000 was this DMC-31011's bench-confirmed fresh-power-
# cycle ceiling for a SINGLE array (DM posRec[3001] fails "66 Array space
# full", 3000 succeeds). Recording now uses 4 simultaneous arrays (position,
# commanded position, position error, torque/volt output). "66 Array space
# full" seen at 3000/array turned out to be stale controller-side state
# from repeated mid-session redimensioning without a full power cycle, not
# a real ceiling -- a fresh power cycle resolved it. 750/array (3000 total)
# is the current bench-confirmed-working value. Redimensioning an already-
# dimensioned array appears to skip the space check, so a larger size can
# look like it works mid-session -- don't trust that; the fresh-boot number
# is the real limit. At the default 1000usec/sample, 750 is 0.75s of
# one-shot recording capacity per channel (less at a faster TM) -- down
# from the single-channel design's 3s, since the memory budget is now
# split 4 ways.
RECORD_ARRAY_NAME = "posRec"  # actual position (_TPA)
RECORD_CMD_ARRAY_NAME = "cmdRec"  # commanded/reference position (_RPA)
RECORD_ERR_ARRAY_NAME = "errRec"  # position (following) error (_TEA)
RECORD_TRQ_ARRAY_NAME = "trqRec"  # output torque/volt command (_TTA)
RECORD_ARRAY_SIZE = 750

# The controller-wide array table's total bench-confirmed fresh-power-
# cycle ceiling (see above) -- the default 4-channel recording setup
# already uses the entire thing (4*RECORD_ARRAY_SIZE=3000). Any mode that
# needs array space beyond that default allocation (e.g. Current Profile
# mode's feedforward array) must size its own recording arrays smaller
# than RECORD_ARRAY_SIZE to make room, and check the total against this.
ARRAY_TABLE_MAX_ELEMENTS = 3000


class RecordedData(NamedTuple):
    """Raw (counts/native-scale) samples from a completed recording --
    same length across all 4 fields, one sample per real servo tick (RC 1
    at n=1). Each field is one of the 4 simultaneous channels armed by
    "RD _TPA,_RPA,_TEA,_TTA" (start_recording(), and duplicated inside
    every program builder -- contour_program.py/pvt_program.py/
    current_profile_program.py -- that embeds its own recording setup):

        position            _TPA  actual measured position (encoder
                                   feedback), raw counts.
        commanded_position  _RPA  the position servo's own internal
                                   reference/commanded position at that
                                   instant -- the real interpolated
                                   trajectory the controller is tracking
                                   against (e.g. the CD/PVA-derived path),
                                   not something reconstructed on the host.
        position_error      _TEA  following error: the live difference
                                   between _RPA and _TPA the PID is
                                   actively correcting, raw counts.
        torque              _TTA  the controller's analog output command
                                   to the amplifier, raw +/-32767 counts --
                                   a DIFFERENT representation than a live
                                   "MG _TTA" query, which returns volts
                                   directly (see read_output_voltage).

    None of these are converted to physical units here -- that happens
    at the call site (app.py's _show_capture): counts_to_mm for
    position/commanded_position, counts_delta_to_mm (a delta, not an
    absolute position) for position_error, and torque_counts_to_volts
    (then volts_to_current) for torque.

    See Controller.stop_recording()/upload_recorded_data()."""
    position: list
    commanded_position: list
    position_error: list
    torque: list


class Controller:
    # Mutable calibration state, not module constants -- every mm<->counts
    # conversion below is called as a bare Controller.foo(...) class access
    # throughout the app (never through an instance), so Set LVDT Voltage
    # (app.py's LVDT Calibration section) has to update state that lives here
    # at the class level to be visible everywhere. Starts at the original
    # bench-measured full-stroke endpoints; persists for the app session
    # until changed again (not reset on reconnect).
    # Ordered (counts, mm) calibration points, ascending in mm. Two points
    # is the classic straight-line calibration; up to 4 define up to 3
    # piecewise-linear regions (see counts_to_mm_piecewise). Defaults to the
    # original bench-measured full-stroke endpoints, so behavior before
    # anything is calibrated is identical to the old 2-point scheme.
    _calibration_points = [(RETRACT_COUNTS, RETRACT_MM), (EXTEND_COUNTS, EXTEND_MM)]
    # Same reasoning -- CME's Analog Input Scale, GUI-editable via the
    # Amp Full-Scale Current field, needs to be visible to every bare
    # Controller.volts_to_current(...) call across the app.
    _amp_current_per_volt = DEFAULT_AMP_FULL_SCALE_CURRENT_A / TORQUE_VOLTS_FULL_SCALE

    def __init__(self):
        self._g = gclib.py()
        self._connected = False
        # Tracks the largest size each named array (RECORD_ARRAY_NAME et
        # al., FF_ARRAY_NAME) has actually been DM'd to via this
        # Controller instance this session -- see _size_array.
        self._array_sizes = {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, address: str = DEFAULT_ADDRESS) -> None:
        self._g.GOpen(f"{address} --direct -s ALL")
        try:
            self._g.timeout = SETUP_TIMEOUT_MS
            for command in SETUP_COMMANDS:
                self._g.GCommand(command)
        except gclib.GclibError:
            self._g.GClose()
            raise
        self._connected = True
        # A fresh connection doesn't necessarily mean a fresh controller
        # boot (the array table may still hold whatever a prior session
        # left it at) -- but we have no way to query real array sizes
        # back from the controller, so this is the best available
        # approximation. Worst case (some other, untracked process left
        # an array at a smaller nonzero size than we now assume) is no
        # worse than before _size_array existed -- the DM would still
        # fail the same unreliable-growth way, just not caught
        # proactively that one time.
        self._array_sizes = {}

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._g.GClose()
        self._connected = False

    def read_position(self) -> int:
        return int(float(self._g.GCommand("TP")))

    def read_lvdt_voltage(self) -> float:
        return float(self._g.GCommand("MG @AN[1]"))

    def read_output_voltage(self) -> float:
        # _TTA is the same operand recorded via start_recording(), but its
        # representation differs by access path (Galil command reference,
        # "Data Record" vs. "TT Tell Torque"/interrogation): values landing
        # in a recorded array are raw +/-32767 counts (what
        # torque_counts_to_volts converts), but a live MG _TTA/TTA query
        # already returns the analog output directly in volts
        # (-9.9982..9.9982) -- no conversion here, unlike the recorded path.
        return float(self._g.GCommand("MG _TTA"))

    def read_servo_enabled(self) -> bool:
        return not bool(int(float(self._g.GCommand("MG _MOA"))))

    def enable(self) -> None:
        self._g.GCommand("SHA")

    def disable(self) -> None:
        # OF is a persistent bias on the command output that MOA alone
        # doesn't clear -- zero it first so no current keeps flowing.
        self.set_offset(0.0)
        self._g.GCommand("MOA")

    def abort(self) -> None:
        # Bare AB stops motion/any running program but does not reliably
        # turn off the motor on this controller (confirmed on the bench) --
        # follow it with an explicit disable to guarantee the servo drops.
        self._g.GCommand("AB")
        self.set_offset(0.0)
        self._g.GCommand("MOA")

    def move_absolute(self, counts: int) -> None:
        # No AMA (wait for motion complete) -- that would block the GUI
        # thread, and Disable/ABORT with it, for the whole move. Progress is
        # observed via the existing 5 Hz position poll instead.
        self._g.GCommand(f"PA {counts}")
        self._g.GCommand("BGA")

    def set_motion_profile(self, speed: int, accel: int, decel: int) -> None:
        self._g.GCommand(f"SP {speed}")
        self._g.GCommand(f"AC {accel}")
        self._g.GCommand(f"DC {decel}")

    def set_pid_gains(self, kp: float, kd: float, ki: float) -> None:
        self._g.GCommand(f"KP {kp}")
        self._g.GCommand(f"KD {kd}")
        self._g.GCommand(f"KI {ki}")

    def set_offset(self, volts: float) -> None:
        self._g.GCommand(f"OF {volts}")

    def set_velocity_feedforward(self, fv: float) -> None:
        self._g.GCommand(f"FV {fv}")

    @classmethod
    def set_calibration_points(cls, points) -> None:
        """Replace the calibration with `points`, an iterable of
        (counts, mm) in ascending-mm order. 2 points is a plain straight
        line; 3 or 4 define 2 or 3 piecewise-linear regions.

        Rejects anything that would make the map non-invertible: fewer
        than 2 points, non-ascending positions, or readings that don't
        change in one consistent direction across every segment (a flat
        or reversed segment would silently produce garbage positions
        rather than failing loudly)."""
        points = [(float(counts), float(mm)) for counts, mm in points]
        if len(points) < 2:
            raise ValueError("Need at least 2 calibration points")
        if any(b[1] <= a[1] for a, b in zip(points, points[1:])):
            raise ValueError("Calibration positions must strictly ascend (mm)")
        deltas = [b[0] - a[0] for a, b in zip(points, points[1:])]
        if any(d == 0 for d in deltas):
            raise ValueError(
                "Two calibration points captured the same reading -- each "
                "position needs a distinct LVDT value"
            )
        if not (all(d > 0 for d in deltas) or all(d < 0 for d in deltas)):
            raise ValueError(
                "Calibration readings must change in one consistent direction "
                "across all points -- re-capture whichever is out of order"
            )
        cls._calibration_points = points

    @classmethod
    def calibration_points(cls) -> list:
        return list(cls._calibration_points)

    @classmethod
    def calibration_region_count(cls) -> int:
        return len(cls._calibration_points) - 1

    @classmethod
    def calibration_regions(cls) -> list:
        """One (mm_lo, mm_hi, counts_per_mm, mm_per_volt) per region, in
        ascending-mm order. counts_per_mm is reported as a magnitude --
        counts run the opposite way to mm on this rig, and the sign is
        already handled by every conversion that needs it."""
        regions = []
        for (c0, m0), (c1, m1) in zip(cls._calibration_points, cls._calibration_points[1:]):
            counts_per_mm = abs((c1 - c0) / (m1 - m0))
            regions.append((m0, m1, counts_per_mm, 1.0 / cls.counts_to_volts(counts_per_mm)))
        return regions

    @staticmethod
    def volts_to_counts(volts: float) -> float:
        return volts * COUNTS_PER_VOLT

    @staticmethod
    def counts_to_volts(counts: float) -> float:
        return counts / COUNTS_PER_VOLT

    @classmethod
    def _linear_span(cls):
        # The straight line through the FIRST and LAST calibration points.
        # Every "linear" conversion below uses this, and -- deliberately --
        # so does all motion generation, even when a piecewise map is
        # configured: trajectories stay built on one consistent scale so
        # Position L/N on the plot are two readings of the same move.
        # span_counts keeps the historical retract-minus-extend sign, so
        # counts_delta_to_mm/counts_to_meters keep the polarity they have
        # always had.
        (base_counts, base_mm), (far_counts, far_mm) = (
            cls._calibration_points[0], cls._calibration_points[-1]
        )
        return base_counts, base_mm, base_counts - far_counts, far_mm - base_mm

    @classmethod
    def counts_to_mm_piecewise(cls, counts: float) -> float:
        """Position (mm) through the full piecewise map rather than the
        single straight line counts_to_mm uses. Identical to counts_to_mm
        when only 2 points are calibrated.

        Outside the calibrated range this extrapolates on the nearest
        segment's slope rather than clamping, so there's no discontinuity
        at the ends of travel."""
        points = cls._calibration_points
        last = len(points) - 2
        for i in range(last + 1):
            (c0, m0), (c1, m1) = points[i], points[i + 1]
            t = (counts - c0) / (c1 - c0)
            # The first segment also absorbs everything before the near
            # end (t < 0); the last absorbs everything past the far end.
            if t <= 1 or i == last:
                return m0 + t * (m1 - m0)

    @classmethod
    def counts_per_meter(cls) -> float:
        _, _, span_counts, span_mm = cls._linear_span()
        return abs(span_counts) / span_mm * 1000

    @classmethod
    def set_amp_current_per_volt(cls, full_scale_amps: float) -> None:
        cls._amp_current_per_volt = full_scale_amps / TORQUE_VOLTS_FULL_SCALE

    @classmethod
    def get_amp_full_scale_current(cls) -> float:
        return cls._amp_current_per_volt * TORQUE_VOLTS_FULL_SCALE

    def set_sample_period(self, microseconds: float) -> None:
        self._g.GCommand(f"TM {microseconds}")

    def download_program(self, text: str) -> None:
        self._g.timeout = PROGRAM_DOWNLOAD_TIMEOUT_MS
        try:
            self._g.GProgramDownload(text)
        finally:
            self._g.timeout = SETUP_TIMEOUT_MS

    def run_program(self, label: str = "MAIN") -> None:
        self._g.GCommand(f"XQ #{label}")

    def read_program_running(self) -> bool:
        # run_program() sends a bare "XQ #label" (no thread number), which
        # defaults to thread 0. _XQ0 is that thread's currently executing
        # line number, or -1 once it's stopped (reached EN, hit AB, etc).
        return int(float(self._g.GCommand("MG _XQ0"))) != -1

    def read_last_error(self) -> str:
        # TC 1: numeric error code + descriptive text for the last error
        # (e.g. "0" for none, or "20 UNRECOGNIZED COMMAND"). Distinguishes
        # a program that reached EN cleanly from one that halted partway
        # on an error -- read_program_running() alone can't tell those
        # apart, since both end with the thread "not running".
        return self._g.GCommand("TC 1")

    def read_motion_complete(self) -> bool:
        return not bool(int(float(self._g.GCommand("MG _BGA"))))

    def start_recording(self) -> None:
        # 4 simultaneous channels: size_shared_record_arrays (DA+DM if a
        # real grow is needed, e.g. a prior Current Profile Arm left
        # these smaller -- see its own comment) dimensions all 4, one
        # combined RA/RD listing all 4 (array order matches operand
        # order), one RC starts all 4 together. Multi-channel DR syntax,
        # bench-confirmed.
        self.size_shared_record_arrays(RECORD_ARRAY_SIZE)
        self._g.GCommand(
            f"RA {RECORD_ARRAY_NAME}[],{RECORD_CMD_ARRAY_NAME}[],"
            f"{RECORD_ERR_ARRAY_NAME}[],{RECORD_TRQ_ARRAY_NAME}[]"
        )
        self._g.GCommand("RD _TPA,_RPA,_TEA,_TTA")
        self._g.GCommand("RC 1")

    def stop_recording(self) -> RecordedData:
        # Query the sample count before stopping, mirroring the order in
        # Galil's official example.
        recorded_count = int(float(self._g.GCommand("MG _RD")))
        self._g.GCommand("RC 0")
        return self._upload_recorded_data(recorded_count)

    def upload_recorded_data(self) -> RecordedData:
        # For a program-driven capture (the program itself issues RC 0
        # before EN, unlike stop_recording()'s host-driven timing) -- just
        # retrieves whatever the program already recorded.
        recorded_count = int(float(self._g.GCommand("MG _RD")))
        return self._upload_recorded_data(recorded_count)

    def _upload_recorded_data(self, recorded_count: int) -> RecordedData:
        recorded_count = max(0, min(recorded_count, RECORD_ARRAY_SIZE))
        return RecordedData(
            position=self._upload_recorded_array(RECORD_ARRAY_NAME, recorded_count),
            commanded_position=self._upload_recorded_array(RECORD_CMD_ARRAY_NAME, recorded_count),
            position_error=self._upload_recorded_array(RECORD_ERR_ARRAY_NAME, recorded_count),
            torque=self._upload_recorded_array(RECORD_TRQ_ARRAY_NAME, recorded_count),
        )

    def _upload_recorded_array(self, array_name: str, recorded_count: int) -> list:
        if recorded_count == 0:
            return []
        return self._g.GArrayUpload(array_name, 0, recorded_count - 1)

    def _describe_last_error(self) -> str:
        # read_last_error() (TC 1) itself talks to the controller --
        # guard it so a broken query doesn't mask a real error with a
        # confusing new one.
        try:
            return self.read_last_error()
        except gclib.GclibError:
            return "<TC 1 query also failed>"

    def _size_array(self, name: str, target_size: int) -> None:
        # Dimensions `name` to `target_size` via DM -- but ONLY if
        # actually necessary: skip entirely if this array is already big
        # enough (just write into the leading target_size elements of
        # the existing, larger array -- every reader here (the FF-
        # playback loop's own JP#L,i<len(ff_values) bound, RC 1's own
        # record_samples-sized DM at Fire time) only ever touches
        # indices up to whatever length IT requested, so stale trailing
        # elements from a previous, larger download are never read).
        #
        # If a real size-INCREASE on an already-existing (tracked
        # nonzero) array is needed, first deallocate it with "DA
        # name[0]" (bench-confirmed reliable in isolation --
        # da_deallocate_bench.py) so the following DM counts as a fresh
        # creation again, avoiding the unreliable in-place growth path
        # (DM'ing straight to a larger size than an array's current
        # dimension fails outright on this firmware). This per-array DA
        # is only a defensive fallback here, though -- callers that can
        # instead do a full "DA *[0]" reset before resizing several
        # arrays at once (see size_shared_record_arrays) should prefer
        # that: bench-confirmed (da_leak_bench.py) that REPEATED
        # per-array DA/DM cycles across several arrays can silently
        # consume real array-table space beyond what this method's own
        # self._array_sizes bookkeeping can see, eventually failing with
        # a genuine "Array space full" even though the logical current
        # total looks nowhere near the ceiling; a full-table reset
        # before each resize round doesn't show that same drift.
        #
        # self._array_sizes only tracks what THIS Controller instance
        # has itself DM'd -- it can't see a differently-sized array left
        # by an untracked process. That's an accepted, narrow gap (see
        # connect()).
        current_size = self._array_sizes.get(name, 0)
        if current_size >= target_size:
            return
        if current_size > 0:
            da_cmd = f"DA {name}[0]"
            try:
                self._g.GCommand(da_cmd)
            except gclib.GclibError as e:
                raise gclib.GclibError(
                    f"{da_cmd!r} failed: {e} (TC 1: {self._describe_last_error()})"
                ) from e
            del self._array_sizes[name]
        dm_cmd = f"DM {name}[{target_size}]"
        try:
            self._g.GCommand(dm_cmd)
        except gclib.GclibError as e:
            raise gclib.GclibError(
                f"{dm_cmd!r} failed: {e} (TC 1: {self._describe_last_error()})"
            ) from e
        self._array_sizes[name] = target_size

    def download_array(self, array_name: str, values: list) -> None:
        # Mirror of _upload_recorded_array, but host -> controller (DMC QD
        # command via gclib's GArrayDownload). Dimensions the array to
        # len(values) itself (via _size_array -- see its own comment for
        # why that's not just a plain unconditional DM), same as
        # start_recording() does for its own arrays. QD pauses Data
        # Record output until the transfer completes, so callers must
        # download before start_recording()/RC 1, not during a move.
        #
        # round(v, 6) -- bench-confirmed (Phase 0 array round-trip test,
        # Current Profile mode): a value tiny enough that Python renders
        # it in scientific notation (e.g. a sin() residual like
        # 6.123233995736766e-17) comes back corrupted on readback (e.g.
        # -6.1232) -- QD's wire format doesn't parse an "e-NN" exponent.
        # Rounding to 6 decimal places (already far finer than any real
        # value here needs -- OF's own resolution is ~0.0003V) collapses
        # those residuals to a plain 0.0 before they ever reach gclib.
        #
        # "+ 0.0" -- bench-confirmed (Current Profile mode,
        # GArrayDownload failing with Galil TC error "6 Number out of
        # range" for a feedforward array containing a floating-point
        # residual landing on the NEGATIVE side of zero, e.g. right at a
        # curve's accel/decel sign change): round(v, 6) alone collapses
        # a near-zero residual's MAGNITUDE to 0.0, but doesn't clear a
        # negative sign bit -- Python still renders that as the literal
        # string "-0.0" (str(-0.0) == "-0.0"), which the controller
        # apparently doesn't parse as a valid array element. Adding 0.0
        # normalizes -0.0 to plain 0.0 (IEEE 754: (-0.0)+0.0 == 0.0) with
        # no effect on any other value.
        values = [round(v, 6) + 0.0 for v in values]
        self._size_array(array_name, len(values))
        try:
            self._g.GArrayDownload(array_name, 0, len(values) - 1, values)
        except gclib.GclibError as e:
            raise gclib.GclibError(
                f"GArrayDownload({array_name!r}, 0, {len(values) - 1}, "
                f"<{len(values)} values>) failed: {e} (TC 1: {self._describe_last_error()})"
            ) from e

    def read_variable(self, name: str) -> float:
        """Value of a controller variable (e.g. one a program stored a
        TIME stamp or _RD into before ending) -- "MG name"."""
        return float(self._g.GCommand(f"MG {name}"))

    def read_array_space(self) -> int:
        """Free array-table elements right now (MG _DM) -- diagnostic
        readback for the sequence modes' array budgeting."""
        return int(float(self._g.GCommand("MG _DM")))

    def upload_array(self, array_name: str, count: int) -> list:
        """First `count` elements of any named array (GArrayUpload). An
        empty list for count <= 0."""
        return self._upload_recorded_array(array_name, max(0, int(count)))

    def size_arrays(self, sizes: dict) -> None:
        """Resets the ENTIRE array table ("DA *[0]" -- see
        size_shared_record_arrays for why nothing less is reliable) and
        creates every array in `sizes` ({name: elements}) fresh. The
        sequence modes use this for their fast + slow recording arrays
        in one go; their feedforward array is then download_array()'d
        on top, so the caller must leave room for it (the program
        builders check the total against ARRAY_TABLE_MAX_ELEMENTS)."""
        try:
            self._g.GCommand("DA *[0]")
        except gclib.GclibError as e:
            raise gclib.GclibError(
                f"'DA *[0]' failed: {e} (TC 1: {self._describe_last_error()})"
            ) from e
        self._array_sizes.clear()
        for name, size in sizes.items():
            self._size_array(name, int(size))

    def size_shared_record_arrays(self, size: int) -> None:
        """Sizes the 4 shared recording arrays (RECORD_ARRAY_NAME et al.,
        also redimensioned by every mode's own program text at Fire
        time) to `size`, right now, via a full "DA *[0]" reset
        (deallocates EVERY array on the controller, not just these 4)
        followed by a fresh DM for each of the 4.

        Two distinct reasons a caller (any mode's own Arm handler, plus
        the generic Program section's Record flow via start_recording())
        needs this at ARM time, before Fire runs the new program:
        1. These 4 arrays are still sitting at whatever size the LAST
           program that ran left them at -- by default the full
           4x750=3000 (see ARRAY_TABLE_MAX_ELEMENTS), i.e. the entire
           array-table budget -- which can leave no room for a second,
           mode-specific array's own DM (Current Profile mode's own
           feedforward array).
        2. Relying on the new program's OWN matching DM lines to size
           these arrays when it later runs (XQ) was ALSO bench-
           confirmed unreliable on its own in one particular sequence
           (shrink now, regrow later) -- silently stayed at the smaller
           size ("only 1 sample(s) recorded") rather than actually
           growing. Sizing them here first, to the exact value the
           caller's own program text will also request, means that
           program's own DM lines become same-size redeclarations
           instead of a real size change.

        Callers should pass the exact size their own program's matching
        DM lines will request (e.g. current_profile_program.py's
        build_current_profile_program already returns its own
        record_samples for this). Safe any time recording isn't active
        (RC 0).

        Why a full "DA *[0]" reset rather than sizing just these 4
        (bench history, both confirmed real): these 4 arrays at their
        full size ALONE can consume the ENTIRE array-table budget
        (4*RECORD_ARRAY_SIZE can equal ARRAY_TABLE_MAX_ELEMENTS exactly
        -- zero slack), so anything else still resident (e.g. Current
        Profile mode's own feedforward array, never freed by Current
        Profile itself since it doesn't know when it's "done" for the
        session) leaves no room and this fails with a real Galil "Array
        space full" error -- even a single leftover element is enough
        (a version of this fix that shrank other arrays to 1 instead of
        deallocating them still failed by exactly 1 element). A second,
        DIFFERENT version of this fix then freed other arrays with a
        per-array "DA name[0]" before sizing these 4 individually (also
        per-array "DA name[0]" when growing) -- also bench-confirmed
        insufficient: a Current Profile -> Contour -> Current Profile
        Arm sequence still failed with a genuine "Array space full" on
        the THIRD resize, even though the logical total at that point
        was nowhere near the 3000 ceiling by simple addition (da_leak_
        bench.py). Leading theory: per-array "DA" doesn't fully return
        space to a reusable pool across repeated cycles the way a full
        "DA *[0]" reset does -- bench-confirmed (same script) that
        replacing the per-array approach with "DA *[0]" once per resize
        round made the identical 3-step sequence succeed cleanly. So:
        every call here wipes the ENTIRE array table first, unconditionally
        (no "already correct, skip" optimization -- safety over the
        small savings that gave), then rebuilds just these 4 fresh."""
        try:
            self._g.GCommand("DA *[0]")
        except gclib.GclibError as e:
            raise gclib.GclibError(
                f"'DA *[0]' failed: {e} (TC 1: {self._describe_last_error()})"
            ) from e
        self._array_sizes.clear()
        for name in (
            RECORD_ARRAY_NAME, RECORD_CMD_ARRAY_NAME, RECORD_ERR_ARRAY_NAME, RECORD_TRQ_ARRAY_NAME,
        ):
            self._size_array(name, size)

    @classmethod
    def counts_to_mm(cls, counts: int) -> float:
        # The straight line through the outermost calibration points --
        # "Position L" on the plot. See counts_to_mm_piecewise for the
        # multi-region version.
        base_counts, base_mm, span_counts, span_mm = cls._linear_span()
        return base_mm + (base_counts - counts) / span_counts * span_mm

    @classmethod
    def mm_to_counts(cls, mm: float) -> int:
        base_counts, base_mm, span_counts, span_mm = cls._linear_span()
        return round(base_counts - (mm - base_mm) / span_mm * span_counts)

    @classmethod
    def counts_delta_to_mm(cls, counts: float) -> float:
        # Same scale as counts_to_mm, no zero-point offset -- for a delta
        # quantity like position error, not an absolute position. Stays on
        # the linear scale even when a piecewise map is configured: a
        # piecewise delta would depend on *where* the delta sits, which
        # isn't information a bare error value carries.
        _, _, span_counts, span_mm = cls._linear_span()
        return counts / span_counts * span_mm

    @staticmethod
    def torque_counts_to_volts(counts: float) -> float:
        return counts / TORQUE_COUNTS_FULL_SCALE * TORQUE_VOLTS_FULL_SCALE

    @classmethod
    def volts_to_current(cls, volts: float) -> float:
        return volts * cls._amp_current_per_volt

    @classmethod
    def current_to_volts(cls, amps: float) -> float:
        # Inverse of volts_to_current -- for a caller that specifies a
        # desired amp current directly (Simple Open Loop mode) and needs
        # the OF command voltage that produces it.
        return amps / cls._amp_current_per_volt

    @staticmethod
    def ms_to_cycles(ms: float, sample_period_us: float) -> int:
        return round(ms * 1000 / sample_period_us)

    @classmethod
    def counts_to_meters(cls, counts: float) -> float:
        # Rate/delta conversion -- same counts<->length scale as
        # counts_to_mm, but no zero-point offset (rates aren't tied to the
        # calibration's zero reference). Always the linear scale: a rate
        # has no single position at which to pick a piecewise segment.
        _, _, span_counts, span_mm = cls._linear_span()
        return counts * span_mm / span_counts / 1000

    @classmethod
    def meters_to_counts(cls, meters: float) -> int:
        _, _, span_counts, span_mm = cls._linear_span()
        return round(meters * 1000 * span_counts / span_mm)
