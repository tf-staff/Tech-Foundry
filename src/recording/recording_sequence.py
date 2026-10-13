"""Motion sequencing: waiting for a commanded move to complete, and the
impact-capture recording sequence (before-recording hook -> pre-pause ->
move -> wait for motion complete -> post-pause -> retrieve recorded data ->
after-recording hook) built on top of it.

No Tkinter here -- the caller supplies a scheduling function (e.g.
tkinter.Tk.after) and status/completion callbacks, so this stays usable and
testable independent of the GUI.
"""

import time

import gclib

RECORD_PRE_PAUSE_MS = 10
RECORD_POST_PAUSE_MS = 10
MOTION_POLL_MS = 100
MOTION_TIMEOUT_S = 30
PROGRAM_POLL_MS = 100


def wait_for_motion_complete(controller, schedule, on_complete, on_error):
    """Polls until the current move finishes, then calls on_complete().
    Calls on_error(message) instead (never both) if a read fails, the servo
    drops mid-wait, or the wait times out.

    controller: a controller.controller.Controller instance.
    schedule(delay_ms, callback, *args): schedules callback after delay_ms,
        matching tkinter.Misc.after()'s signature.
    """
    start_time = time.monotonic()

    def check():
        try:
            motion_complete = controller.read_motion_complete()
            servo_enabled = controller.read_servo_enabled()
        except gclib.GclibError as e:
            on_error(f"Read error while waiting for motion: {e}")
            return

        if not servo_enabled:
            on_error("Motion wait stopped: servo disabled during move")
            return

        if motion_complete:
            on_complete()
            return

        if time.monotonic() - start_time > MOTION_TIMEOUT_S:
            on_error("Motion wait stopped: move did not complete in time")
            return

        schedule(MOTION_POLL_MS, check)

    # Bench-confirmed: checking synchronously, immediately after the move
    # was commanded, can race the controller's own status-bit update for
    # the just-issued BG -- an immediate read can still see "not moving"
    # from *before* the command took effect, reporting false-early
    # completion. Scheduling the first check after one poll interval
    # instead gives the controller time to register the move as started.
    schedule(MOTION_POLL_MS, check)


def wait_for_program_complete(controller, schedule, on_complete, on_error):
    """Polls until the program running on thread 0 (see
    Controller.run_program()) finishes, then calls on_complete(). Calls
    on_error(message) instead (never both) if a read fails (e.g. the
    connection drops).

    Unlike wait_for_motion_complete, there's no timeout and no servo-state
    check here: a program may legitimately run indefinitely (e.g. a looping
    demo, stopped only by Disable) or turn the servo off itself as one of
    its own steps before EN -- neither is an error condition to detect.

    controller: a controller.controller.Controller instance.
    schedule(delay_ms, callback, *args): schedules callback after delay_ms,
        matching tkinter.Misc.after()'s signature.
    """
    def check():
        try:
            running = controller.read_program_running()
        except gclib.GclibError as e:
            on_error(f"Read error while waiting for program: {e}")
            return

        if not running:
            on_complete()
            return

        schedule(PROGRAM_POLL_MS, check)

    # Same reasoning as wait_for_motion_complete's first-check delay: an
    # immediate read right after XQ can race the controller's own
    # thread-started status update, falsely reporting "not running" before
    # the program has actually begun.
    schedule(PROGRAM_POLL_MS, check)


class RecordingSequence:
    def __init__(
        self, controller, schedule, on_status,
        on_before_recording, on_after_recording, on_complete,
    ):
        """
        controller: a controller.controller.Controller instance.
        schedule(delay_ms, callback, *args): schedules callback after
            delay_ms, matching tkinter.Misc.after()'s signature.
        on_status(message: str): called with human-readable status updates.
        on_before_recording() -> bool: called first, before start_recording()
            issues DM/RA/RD/RC. Return False to abort before anything is
            sent to the controller (e.g. the motion gains/sample period
            failed to apply) -- the callback is responsible for its own
            status/error reporting. Called before the pre-move pause (not
            after it) so the whole recorded array -- pre-pause, move, and
            post-pause -- is captured at one uniform sample rate.
        on_after_recording(): called once the recorded data has been
            successfully retrieved (stop_recording()), before on_complete.
            Not called on abort/failure.
        on_complete(recorded_data): called with whatever stop_recording()
            returns (a controller.controller.RecordedData) once the
            sequence finishes successfully. Not called on abort/failure.
        """
        self._controller = controller
        self._schedule = schedule
        self._on_status = on_status
        self._on_before_recording = on_before_recording
        self._on_after_recording = on_after_recording
        self._on_complete = on_complete
        self._post_pause_ms = RECORD_POST_PAUSE_MS

    def start(
        self, target_counts,
        pre_pause_ms=RECORD_PRE_PAUSE_MS, post_pause_ms=RECORD_POST_PAUSE_MS,
    ):
        """pre_pause_ms/post_pause_ms: delay before the move begins and
        after it completes, respectively, both counted as part of the
        recording."""
        if not self._on_before_recording():
            return

        try:
            self._controller.start_recording()
        except gclib.GclibError as e:
            self._on_status(f"Recording setup failed: {e}")
            return

        self._post_pause_ms = post_pause_ms
        self._on_status("Recording: pre-move pause...")
        self._schedule(pre_pause_ms, self._trigger_move, target_counts)

    def _trigger_move(self, target_counts):
        try:
            self._controller.move_absolute(target_counts)
        except gclib.GclibError as e:
            self._on_status(f"Move failed: {e}")
            self._abort()
            return

        self._on_status("Recording: move in progress...")
        wait_for_motion_complete(
            self._controller, self._schedule,
            on_complete=self._handle_motion_complete,
            on_error=self._handle_motion_error,
        )

    def _handle_motion_complete(self):
        self._on_status("Recording: post-move pause...")
        self._schedule(self._post_pause_ms, self._finish)

    def _handle_motion_error(self, message):
        self._on_status(message)
        self._abort()

    def _abort(self):
        try:
            self._controller.stop_recording()
        except gclib.GclibError:
            pass

    def _finish(self):
        try:
            raw_positions = self._controller.stop_recording()
        except gclib.GclibError as e:
            self._on_status(f"Recording upload failed: {e}")
            return

        self._on_after_recording()
        self._on_status("Recording complete")
        self._on_complete(raw_positions)
