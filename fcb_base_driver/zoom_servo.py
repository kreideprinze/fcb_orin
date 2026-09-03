"""Closed-loop zoom control over VISCA, on its own thread.

Zoom is driven as a closed loop rather than by firing absolute-position
commands. VISCA gives a command two sockets; an absolute CAM_Zoom Direct
holds its socket until the lens physically arrives, so streaming Direct
commands from a continuously-moving RC knob exhausts the sockets within a
second and the camera starts refusing them with "command buffer full".
Variable-speed drive commands (CAM_Zoom Tele/Wide) complete immediately,
so this polls the zoom position, compares it against the requested target,
and drives towards it at a speed proportional to the error -- stopping
inside a deadband. That tracks a moving knob smoothly and never backs up
the command buffer.

Polling backs off once the lens is holding its target. The VISCA link and
the video stream share one USB3 connection on the NeoHD board, and there
is no reason to keep asking a stationary lens where it is at 20 Hz.

Two commanders are arbitrated: the RC knob, and whatever asks for a target
by hand. Any manual request takes over until control is handed back, since
the RC knob would otherwise immediately win -- it is always publishing.
"""
import threading
import time

from fcb_base_driver import visca, zoom_map
from fcb_base_driver.visca_link import ViscaError, ViscaTimeout

ACTIVE_INTERVAL = 0.05  # 20 Hz while actually driving towards a target
IDLE_INTERVAL = 0.2     # 5 Hz once parked -- just enough to stay current
DEADBAND = 24
MAX_SPEED = 7
MIN_SPEED = 1
ERROR_FOR_MAX_SPEED = 2000

#: Consecutive failed exchanges before the link is assumed dead rather
#: than merely busy. A camera that is mid-movement can refuse a command or
#: miss a reply occasionally, so one fault means nothing; a run of them
#: means the port went away -- unplugged, re-enumerated, or wedged.
FAULTS_BEFORE_RECONNECT = 10
RECONNECT_INTERVAL = 2.0


class ZoomServo:
    """Drives the lens towards a normalized 0.0-1.0 target.

    Pass `link_factory` to make the servo self-healing: if the VISCA link
    stops answering it is closed and rebuilt from the factory, which
    matters on a drone where nobody can replug anything mid-flight.
    """

    def __init__(self, link, curve="log", rc=None, source="rc", on_status=None,
                 link_factory=None):
        self.link = link
        self.curve = curve if curve in zoom_map.CURVES else "log"
        self.rc = rc
        self.source = source if rc is not None else "manual"
        self._status = on_status or (lambda msg: None)
        self._link_factory = link_factory
        self._faults = 0
        self._last_reconnect = 0.0

        self.unit = 0.0
        self.position = None
        self.ratio = None
        self.updated_at = None
        self.is_driving = False

        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # -- commanders ------------------------------------------------------

    def set_unit(self, value, source="manual"):
        """Request a normalized 0.0-1.0 zoom, taking over from the RC knob."""
        self.unit = max(0.0, min(1.0, float(value)))
        self.source = source
        return self.unit

    def nudge(self, delta):
        return self.set_unit(self.unit + delta)

    def hand_to_rc(self):
        """Give control back to the RC knob. False if there is no RC link."""
        if self.rc is None:
            return False
        self.source = "rc"
        return True

    def sync_from_camera(self):
        """Adopt the lens's current position as the setpoint.

        Called at startup so the first command continues from where the
        lens already is instead of snapping it across the range.
        """
        position = self.link.zoom_position()
        self.position = position
        self.ratio = zoom_map.position_to_ratio(position)
        self.unit = zoom_map.position_to_unit(position, self.curve)
        self.updated_at = time.monotonic()
        return self.ratio

    # -- servo loop ------------------------------------------------------

    def _run(self):
        while self._running:
            try:
                self._tick()
                self._faults = 0
            except (ViscaError, ViscaTimeout, ValueError) as exc:
                self._faults += 1
                # Only shout about the first one; a dead link would
                # otherwise fill the log at 5 Hz for the rest of the flight.
                if self._faults == 1:
                    self._status(f"zoom: {exc}")
                if self._faults >= FAULTS_BEFORE_RECONNECT:
                    self._reconnect()
            time.sleep(ACTIVE_INTERVAL if self.is_driving else IDLE_INTERVAL)

    def _reconnect(self):
        """Rebuild a VISCA link that has stopped answering."""
        if self._link_factory is None:
            return
        now = time.monotonic()
        if now - self._last_reconnect < RECONNECT_INTERVAL:
            return
        self._last_reconnect = now

        try:
            self.link.close()
        except Exception:
            pass
        try:
            link = self._link_factory()
        except Exception as exc:
            self._status(f"zoom: VISCA reconnect failed: {exc}")
            return
        if link is None:
            return
        self.link = link
        self.is_driving = False
        self._faults = 0
        self._status("zoom: VISCA link re-established")

    def _tick(self):
        # A manual request already set unit and switched source itself;
        # only pull from RC while it is the active commander.
        if self.source == "rc" and self.rc is not None:
            unit = self.rc.unit()
            if unit is not None:
                self.unit = unit

        self.position = self.link.zoom_position()
        self.ratio = zoom_map.position_to_ratio(self.position)
        self.updated_at = time.monotonic()

        target = zoom_map.unit_to_position(self.unit, self.curve)
        error = target - self.position
        if abs(error) <= DEADBAND:
            self._stop_if_driving()
            return

        speed = int(max(MIN_SPEED, min(
            MAX_SPEED, round(abs(error) / ERROR_FOR_MAX_SPEED * MAX_SPEED)
        )))
        if error > 0:
            self.link.zoom_tele(speed)
        else:
            self.link.zoom_wide(speed)
        self.is_driving = True

    def _stop_if_driving(self):
        if not self.is_driving:
            return
        try:
            self.link.zoom_stop()
            self.is_driving = False
        except (ViscaError, ViscaTimeout) as exc:
            self._status(f"zoom stop: {exc}")

    # -- other camera commands -------------------------------------------

    def set_icr(self, mode):
        """Switch the IR cut filter: day, night, night_color, or auto."""
        address = self.link.address
        if mode == "auto":
            self.link.command(visca.auto_icr(True, address=address))
        else:
            # Leave automatic switching first, or the camera would override
            # a hand-picked mode at the next light change.
            self.link.command(visca.auto_icr(False, address=address))
            self.link.command(visca.icr(mode, address))

    def one_push_af(self):
        self.link.command(visca.focus_one_push(self.link.address))

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            self.link.zoom_stop()
        except Exception:
            pass
