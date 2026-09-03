"""Serial transport for VISCA, framed properly and safe to share.

VISCA is a framed protocol: every packet ends in 0xFF, and packets vary in
length. Replies to a command arrive as an ACK (`y0 4z FF`) as soon as the
camera accepts it, then a separate completion (`y0 5z FF`) once the action
has actually finished -- which for a zoom movement can be seconds later.

That distinction matters. Waiting for completion after every command would
stall the control loop for the whole duration of the physical movement, so
`command()` waits only for the ACK by default and late completions are
drained on the next exchange.

The link takes an exclusive `flock` on the serial device so a second node
(or a stray CLI tool) cannot interleave bytes into the same stream.
"""
import fcntl
import threading
import time

import serial

from fcb_base_driver import visca


class ViscaError(RuntimeError):
    """The camera replied with a VISCA error packet."""


class ViscaTimeout(RuntimeError):
    """No complete reply frame arrived within the timeout."""


class ViscaLink:
    """Exclusive, framed serial link to a VISCA camera."""

    def __init__(self, port: str, baud: int = 9600, address: int = 1,
                 timeout: float = 0.25):
        self.address = address
        self.timeout = timeout
        self._lock = threading.Lock()

        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        try:
            fcntl.flock(self._ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._ser.close()
            raise RuntimeError(
                f"{port} is locked by another process; refusing to share the bus"
            ) from exc

        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    # -- lifecycle -----------------------------------------------------

    def close(self) -> None:
        try:
            fcntl.flock(self._ser.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- framing -------------------------------------------------------

    def _read_frame(self, deadline: float) -> bytes:
        """Read bytes up to and including the next 0xFF terminator."""
        frame = bytearray()
        while time.monotonic() < deadline:
            byte = self._ser.read(1)
            if not byte:
                continue
            frame += byte
            if byte[0] == 0xFF:
                return bytes(frame)
        raise ViscaTimeout(
            f"no complete frame within {self.timeout}s (partial: {frame.hex()})"
        )

    def _await(self, want: str, timeout: float = None,
               min_payload: int = 0) -> bytes:
        """Read frames until one of kind `want` arrives, or time out.

        Frames of other kinds (a stale completion from an earlier movement,
        for instance) are skipped rather than mistaken for this reply.

        `min_payload` separates the two shapes of completion frame. A command
        completes with a bare `y0 5z FF` (one payload byte), while an inquiry
        answers with `y0 50 <data> FF`. Interleaving a position inquiry with
        movement commands means a lens arriving mid-inquiry can drop its
        completion into the stream first, so an inquiry asks for a payload
        long enough to actually carry data and skips the bare one.
        """
        timeout = self.timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while True:
            frame = self._read_frame(deadline)
            kind, payload = visca.classify(frame)
            if kind == visca.ERROR:
                raise ViscaError(visca.error_message(payload))
            if kind == want and len(payload) >= min_payload:
                return payload
            # Otherwise: an ACK we are not waiting on, or a late completion
            # from a previous command. Drop it and keep reading.

    def _exchange(self, packet: bytes, want: str, timeout: float = None,
                  min_payload: int = 0) -> bytes:
        """Write `packet`, then wait for a reply frame of kind `want`."""
        self._ser.write(packet)
        self._ser.flush()
        return self._await(want, timeout, min_payload)

    # -- public API ----------------------------------------------------

    def command(self, packet: bytes, wait_completion: bool = False,
                expect_ack: bool = True, timeout: float = None) -> None:
        """Send a command packet and wait for the camera to accept it.

        By default this returns once the camera ACKs, not once the movement
        finishes -- see the module docstring. Pass `wait_completion=True`
        for commands whose completion is fast and worth confirming.

        IF_Clear is a documented exception to the normal ACK-then-completion
        pattern -- the manual states plainly that no ACK is returned for it,
        only a bare completion -- so `expect_ack=False` skips straight to
        waiting on the completion instead of timing out on an ACK that is
        never coming.
        """
        with self._lock:
            if expect_ack:
                self._exchange(packet, want=visca.ACK, timeout=timeout)
                if wait_completion:
                    self._await(visca.COMPLETION, timeout=timeout)
            else:
                self._exchange(packet, want=visca.COMPLETION, timeout=timeout)

    def inquiry(self, packet: bytes, timeout: float = None) -> bytes:
        """Send an inquiry packet and return its completion payload.

        Inquiries are answered with a single completion frame carrying data;
        they are not ACKed first.
        """
        with self._lock:
            return self._exchange(
                packet, want=visca.COMPLETION, timeout=timeout, min_payload=2
            )

    # -- convenience wrappers -----------------------------------------

    def zoom_position(self) -> int:
        payload = self.inquiry(visca.zoom_pos_inq(self.address))
        return visca.parse_zoom_pos(payload)

    def zoom_direct(self, position: int) -> None:
        self.command(visca.zoom_direct(position, self.address))

    def zoom_tele(self, speed: int) -> None:
        self.command(visca.zoom_tele(speed, self.address))

    def zoom_wide(self, speed: int) -> None:
        self.command(visca.zoom_wide(speed, self.address))

    def zoom_stop(self) -> None:
        self.command(visca.zoom_stop(self.address))

    def focus_position(self) -> int:
        payload = self.inquiry(visca.focus_pos_inq(self.address))
        return visca.parse_focus_pos(payload)

    def if_clear(self) -> None:
        """Clear the camera's command buffer -- useful on startup.

        Per the technical manual: "Acknowledge is not returned for this
        command" -- the camera replies with a bare completion only.
        """
        self.command(visca.if_clear(self.address), expect_ack=False)
