"""Finding the camera and the flight controller among the /dev nodes.

The FCB's video node, its VISCA control port, and the Pixhawk's MAVLink
port all appear as generic devices -- /dev/video* and /dev/ttyACM* -- and
which number each gets depends on the order things were plugged in. On a
machine with both, the numbers move whenever either is replugged, so
nothing here trusts a fixed path: a device is identified by answering its
own protocol, or by the name v4l2 reports for it.
"""
import glob
import subprocess
import sys

import cv2

from fcb_base_driver import visca
from fcb_base_driver.visca_link import ViscaLink

#: Substrings (case-insensitive) seen in v4l2's "Card type" for this camera
#: across the boards it ships on -- a bare Sony FCB block reports "FCB", the
#: Active Silicon Harrier board reports "Harrier", and the Twiga board this
#: particular camera turned out to be on reports "NeoHD".
FCB_NAME_HINTS = ("neohd", "fcb", "harrier")

#: The camera's baud is a persistent register setting (9600 out of the box;
#: 38400/115200 selectable and surviving a power cycle), so autodetection
#: checks all three documented rates rather than only the one requested.
VISCA_BAUDS = (9600, 38400, 115200)


def list_v4l2_devices():
    """Group /dev/video* nodes by USB device, as `v4l2-ctl --list-devices` sees them.

    Returns {card_name: [device_paths]}, or None if v4l2-ctl is missing. A
    UVC camera creates one node per logical stream (capture, metadata, ...)
    under one physical device, and only the card name -- not the node's own
    properties -- says which physical camera a node belongs to.
    """
    try:
        output = subprocess.run(
            ["v4l2-ctl", "--list-devices"],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except (FileNotFoundError, subprocess.SubprocessError):
        return None

    groups = {}
    card = None
    for line in output.splitlines():
        if not line.strip():
            card = None
        elif not line.startswith(("\t", " ")):
            card = line.split(" (")[0].strip()
            groups[card] = []
        elif card is not None and line.strip().startswith("/dev/video"):
            groups[card].append(line.strip())
    return groups


def capture_capable(device):
    """True if this node reports Video Capture support, not just metadata."""
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-formats"],
            capture_output=True, text=True, timeout=5,
        )
        return "Video Capture" in result.stdout
    except (FileNotFoundError, subprocess.SubprocessError):
        return True  # can't tell -- don't rule it out


def opens_and_reads(device):
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        return capture.isOpened() and capture.read()[0]
    finally:
        capture.release()


def autodetect_video(log=print):
    """Find the FCB's video node by USB identity, not by guessing at /dev order.

    A laptop's own webcam and the FCB camera both just look like "a video
    device that yields a frame" to the naive approach of trying /dev/video0
    first -- and the built-in webcam usually wins that race since it has no
    USB3 negotiation to do. This asks v4l2 which physical camera each node
    belongs to and only considers ones whose name matches this camera's
    known boards.

    Once a card matches by name this commits to it rather than falling
    through to the generic scan, because silently substituting some other
    camera for the real one is worse than reporting no video at all. The
    choice is made on what v4l2 reports, without a trial open-and-read:
    this board wedges after enough open/release churn -- every later open
    then times out until it is replugged -- and probing meant opening it
    twice on every startup.
    """
    groups = list_v4l2_devices()
    if groups is not None:
        for card, devices in groups.items():
            if not any(hint in card.lower() for hint in FCB_NAME_HINTS):
                continue
            for device in devices:
                if capture_capable(device):
                    log(f"matched \"{card}\" -> {device}")
                    return device
            log(f"matched \"{card}\" but none of its nodes report video "
                f"capture -- not falling back to an unrelated camera")
            return None
        # v4l2 answered and nothing it listed is this camera, so this camera
        # is not attached. Scanning every node from here would just find
        # whatever else is (a laptop's built-in webcam, typically) and hand
        # back the wrong picture, which is worse than no picture.
        log(f"no /dev/video* device matched a known FCB board name "
            f"({', '.join(FCB_NAME_HINTS)}) -- is the camera plugged in? "
            f"Pass --video to use one anyway")
        return None

    # Only reached when v4l2-ctl is missing, so there is no way to ask which
    # camera is which and a scan is the only option left.
    log("v4l2-ctl not available to identify cameras by name; "
        "falling back to scanning every node")
    for device in sorted(glob.glob("/dev/video*")):
        if opens_and_reads(device):
            return device
    return None


def is_zoom_reply(payload):
    """True if `payload` really is a CAM_ZoomPosInq answer.

    A zoom inquiry is answered with `50 0p 0q 0r 0s`: a completion byte
    then four bytes carrying one nibble each, so every data byte is 0x0F or
    below. Checking that shape matters because the flight controller sits
    on a neighbouring /dev/ttyACM* port, and MAVLink is binary traffic full
    of 0xFF bytes -- which is also the VISCA frame terminator. Slicing a
    telemetry stream at those bytes yields "frames" that decode into a
    plausible-looking zoom position often enough to hijack autodetection,
    and the four-high-nibbles-clear test is what MAVLink noise fails.
    """
    return (
        len(payload) == 5
        and payload[0] == 0x50
        and all(byte <= 0x0F for byte in payload[1:5])
    )


def answers_visca(port, baud, address=1, timeout=0.3):
    """Whether a real FCB camera is listening on this port and baud.

    Asks twice: a stray byte sequence can pass the structural check once,
    but not twice in a row, and a camera always can.
    """
    try:
        with ViscaLink(port, baud, address, timeout=timeout) as link:
            for _ in range(2):
                payload = link.inquiry(visca.zoom_pos_inq(address))
                if not is_zoom_reply(payload):
                    return False
                if not 0 <= visca.parse_zoom_pos(payload) <= visca.ZOOM_OPTICAL_TELE_END:
                    return False
        return True
    except Exception:
        return False


def autodetect_visca(ports=None, bauds=None, address=1):
    """First (port, baud) that actually answers a VISCA zoom position inquiry.

    Only a real protocol exchange -- not merely opening the device -- proves
    a port is the camera's control interface, since a board can expose other
    ACM ports (a debug/config console, for instance) that open fine but
    never speak VISCA. Pass a single-element `ports` and/or `bauds` to pin
    whichever side the caller already knows and sweep only the other.

    A candidate that fails verification does not end the sweep; the camera
    may still be on a later port, and giving up at the first plausible-
    looking-but-wrong answer used to leave the camera undetected entirely.
    """
    if ports is None:
        ports = serial_candidates()
    if bauds is None:
        bauds = VISCA_BAUDS
    for port in ports:
        for baud in bauds:
            if answers_visca(port, baud, address):
                return port, baud
    return None, None


def autodetect_mavlink(bauds=(115200,), exclude=None, timeout=3.0):
    """First serial candidate that actually answers a MAVLink heartbeat.

    Mirrors autodetect_visca: only a real protocol exchange proves a port is
    the flight controller's link. Pass `exclude` to skip a port already
    claimed for something else -- no point spending a timeout probing the
    port the camera is known to be on.
    """
    try:
        from pymavlink import mavutil
    except ImportError:
        return None, None

    candidates = [p for p in serial_candidates() if p != exclude]
    for candidate in candidates:
        for baud in bauds:
            connection = None
            try:
                connection = mavutil.mavlink_connection(candidate, baud=baud)
                if connection.wait_heartbeat(timeout=timeout):
                    return candidate, baud
            except Exception:
                pass
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
    return None, None


def serial_candidates():
    """Serial ports worth probing, in a stable order.

    ttyTHS* is where a Jetson's own UARTs land, so a flight controller
    wired to TELEM2 rather than USB is still found.
    """
    return sorted(
        glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyTHS*")
    )


def open_capture(device, width=1920, height=1080, fourcc=None):
    """Open a V4L2 capture set up to hand back raw frames.

    CONVERT_RGB is turned off so the colour conversion happens on the
    consumer's thread instead of inside read() -- see FrameGrabber for why
    that matters. Backends that ignore it still work; FrameGrabber checks
    what it actually got.
    """
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        return None
    if fourcc:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return capture
