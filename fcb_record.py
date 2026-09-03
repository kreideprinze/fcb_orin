#!/usr/bin/env python3
"""Headless flight recorder for the Sony FCB-EV9520L on a Jetson Orin.

Runs on the drone's companion computer with no display: you SSH in, start
it, and fly. Recording is armed from the transmitter -- RC channel 8 high
starts, low stops -- and zoom follows RC channel 7 the whole time.

Every recording produces a matched pair, named after the same timestamp:

    fcb-20260903-114530.mp4     the video
    fcb-20260903-114530.csv     one row per recorded frame

The CSV is the point of this script. Each row carries the video frame
number it belongs to, so the two can be replayed together afterwards,
plus the vehicle's position, altitude, attitude, heading and groundspeed,
and the camera's live zoom.

Telemetry is sampled asynchronously and far slower than 60 fps video --
GPS lands at a few hertz -- so each group of values carries its age in
milliseconds. A row is only as frame-accurate as those ages say it is,
and pretending otherwise would quietly corrupt any analysis built on it.
Each row also carries the flight controller's own time_boot_ms, which is
what lets the CSV be lined up against the Pixhawk's .bin log later.

    ./fcb_record.py                       # autodetect everything
    ./fcb_record.py --encoder cpu         # if GStreamer/NVENC is missing
    ./fcb_record.py --rc-url /dev/ttyACM0 # pin the flight controller
    ./fcb_record.py --log-level DEBUG     # every message, to the log file
"""
import argparse
import csv
import logging
import logging.handlers
import os
import shutil
import signal
import sys
import time
from datetime import datetime, timezone


def _find_driver_root():
    """Where fcb_base_driver lives.

    The bundle copied onto the drone keeps the package next to this script
    so the whole thing is one self-contained directory, so that is checked
    first; a development checkout falls through to the usual repo path.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.environ.get("FCB_DRIVER_ROOT"),
        here,                                    # bundled: ./fcb_base_driver
        os.path.expanduser("~/fcb_base_driver"),  # the git checkout
    ]
    for candidate in candidates:
        if candidate and os.path.isdir(os.path.join(candidate, "fcb_base_driver")):
            return candidate
    return os.path.expanduser("~/fcb_base_driver")


_DRIVER_ROOT = _find_driver_root()
if os.path.isdir(_DRIVER_ROOT) and _DRIVER_ROOT not in sys.path:
    sys.path.insert(0, _DRIVER_ROOT)

try:
    import cv2
except ImportError as exc:
    sys.exit(f"{exc}\n\nThis needs OpenCV (cv2).")

try:
    from fcb_base_driver import devices, zoom_map
    from fcb_base_driver.frame_grabber import FrameGrabber
    from fcb_base_driver.mavlink_source import MavlinkSource
    from fcb_base_driver.rc_source import RcZoomSource
    from fcb_base_driver.visca_link import ViscaLink
    from fcb_base_driver.zoom_servo import ZoomServo
except ImportError as exc:
    sys.exit(
        f"{exc}\n\nCould not import the driver modules from {_DRIVER_ROOT}.\n"
        f"Set FCB_DRIVER_ROOT if the repo lives elsewhere."
    )

log = logging.getLogger("fcb")

CSV_COLUMNS = [
    "frame",              # index into the video, 0-based
    "t_wall_utc",         # wall clock at capture, ISO 8601
    "t_mono_s",           # seconds since recording started
    "lat_deg", "lon_deg",
    "alt_msl_m",          # above mean sea level
    "alt_rel_m",          # above the home/arming point
    "gps_age_ms",
    "roll_deg", "pitch_deg", "yaw_deg",
    "att_age_ms",
    "heading_deg", "groundspeed_ms",
    "hud_age_ms",
    "zoom_ratio",         # e.g. 5.7 for 5.7x
    "zoom_position",      # raw VISCA counts, lossless
    "zoom_age_ms",
    "fc_time_boot_ms",    # the Pixhawk's clock, to align with its .bin log
]


class LiveLine:
    """A single status line that repaints in place at the bottom.

    Log records and a fast-updating status line share one terminal, so the
    line is erased before anything else prints and redrawn afterwards --
    otherwise a log message lands in the middle of it and both are
    unreadable. Anywhere that is not a terminal (a pipe, nohup, a systemd
    journal) this does nothing at all, so redirected output stays clean.
    """

    def __init__(self, stream=None):
        self.stream = stream or sys.stdout
        self.enabled = self.stream.isatty()
        self._shown = False

    def show(self, text):
        if not self.enabled:
            return
        width = shutil.get_terminal_size((120, 24)).columns
        self.stream.write("\r\033[K" + text[:width - 1])
        self.stream.flush()
        self._shown = True

    def clear(self):
        if not self.enabled or not self._shown:
            return
        self.stream.write("\r\033[K")
        self.stream.flush()
        self._shown = False


LIVE = LiveLine()


class LiveAwareHandler(logging.StreamHandler):
    """Console handler that steps around the live status line."""

    def emit(self, record):
        LIVE.clear()
        super().emit(record)


def setup_logging(directory, level, quiet):
    """Log to the console and, in full detail, to a file next to the video."""
    os.makedirs(directory, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = os.path.join(directory, f"fcb_record-{stamp}.log")

    root = logging.getLogger("fcb")
    root.setLevel(logging.DEBUG)
    root.handlers.clear()

    fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    to_file = logging.FileHandler(path)
    to_file.setLevel(getattr(logging, level))
    to_file.setFormatter(fmt)
    root.addHandler(to_file)

    if not quiet:
        to_console = LiveAwareHandler(sys.stdout)
        to_console.setLevel(logging.INFO)
        to_console.setFormatter(fmt)
        root.addHandler(to_console)

    return path


class VideoRecorder:
    """Writes frames to an .mp4, on the GPU where one is available.

    Software encoding 1080p60 is roughly 250 MB/s of raw input to chew
    through, which an Orin's CPU will not keep up with while also running
    everything else. nvv4l2h264enc hands that to the hardware encoder. The
    CPU path stays available for machines without GStreamer, and is fallen
    back to loudly rather than silently, since the difference shows up as
    dropped frames rather than an error.
    """

    def __init__(self, path, width, height, fps, encoder="nvenc", bitrate_mbps=25):
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.frames = 0
        self.started = time.monotonic()

        self.encoder, self.writer = self._open(encoder, bitrate_mbps)
        if self.writer is None:
            raise RuntimeError(f"could not open any video writer for {path}")

    def _open(self, requested, bitrate_mbps):
        if requested in ("nvenc", "auto"):
            writer = self._open_nvenc(bitrate_mbps)
            if writer is not None:
                return "nvenc", writer
            log.warning(
                "NVENC/GStreamer writer would not open -- falling back to "
                "CPU mp4v. Expect dropped frames at 1080p60. Check that this "
                "OpenCV was built with GStreamer (cv2.getBuildInformation())."
            )
        writer = cv2.VideoWriter(
            self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps,
            (self.width, self.height),
        )
        return ("cpu", writer) if writer.isOpened() else ("cpu", None)

    def _open_nvenc(self, bitrate_mbps):
        pipeline = (
            f"appsrc ! video/x-raw,format=BGR,width={self.width},"
            f"height={self.height},framerate={int(round(self.fps))}/1 "
            f"! queue ! videoconvert ! video/x-raw,format=NV12 "
            f"! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc bitrate={int(bitrate_mbps * 1_000_000)} "
            f"insert-sps-pps=1 maxperf-enable=1 "
            f"! h264parse ! qtmux ! filesink location={self.path}"
        )
        log.debug("trying GStreamer pipeline: %s", pipeline)
        try:
            writer = cv2.VideoWriter(
                pipeline, cv2.CAP_GSTREAMER, 0, self.fps,
                (self.width, self.height),
            )
        except Exception as exc:
            log.debug("GStreamer writer raised: %s", exc)
            return None
        return writer if writer.isOpened() else None

    def write(self, frame):
        self.writer.write(frame)
        self.frames += 1

    def close(self):
        self.writer.release()
        return time.monotonic() - self.started


class TelemetryCsv:
    """One row per recorded frame, flushed as it goes.

    Flushing every row costs little at 60 rows a second and means a power
    cut mid-flight leaves a readable file up to the last moment, rather
    than a buffer that never reached the disk.
    """

    def __init__(self, path):
        self.path = path
        self.rows = 0
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_COLUMNS)
        self._file.flush()

    def write(self, frame_number, wall_utc, t_mono, telemetry, servo, now):
        def ms(age):
            return None if age is None else round(age * 1000, 1)

        def num(value, digits):
            return None if value is None else round(value, digits)

        zoom_age = None if servo.updated_at is None else now - servo.updated_at
        self._writer.writerow([
            frame_number,
            wall_utc.isoformat(timespec="milliseconds"),
            round(t_mono, 4),
            num(telemetry.lat_deg, 7), num(telemetry.lon_deg, 7),
            num(telemetry.alt_msl_m, 2), num(telemetry.alt_rel_m, 2),
            ms(telemetry.position_age_s),
            num(telemetry.roll_deg, 2), num(telemetry.pitch_deg, 2),
            num(telemetry.yaw_deg, 2),
            ms(telemetry.attitude_age_s),
            num(telemetry.heading_deg, 1), num(telemetry.groundspeed_ms, 2),
            ms(telemetry.hud_age_s),
            num(servo.ratio, 2), servo.position, ms(zoom_age),
            telemetry.position_boot_ms if telemetry.position_boot_ms is not None
            else telemetry.attitude_boot_ms,
        ])
        self.rows += 1
        self._file.flush()

    def close(self):
        self._file.close()


class Recorder:

    def __init__(self, args):
        self.args = args
        self.link = None
        self.control_port = None
        self.servo = None
        self.mavlink = None
        self.rc = None
        self.grabber = None

        self.video = None
        self.csv = None
        self.base = None
        self.record_started_mono = None
        self.record_started_wall = None

        self.frames_seen = 0
        self.frames_dropped = 0
        self._last_status = 0.0
        self._last_live = 0.0
        self._status_broken = False
        self.running = True

    # -- setup -----------------------------------------------------------

    def open_control(self):
        self.link = self.make_visca_link()
        if self.link is None:
            log.error("no VISCA port answered -- zoom control and zoom "
                      "logging will be unavailable")

    def make_visca_link(self):
        """Find and open the camera's control port. None if it is not there.

        Doubles as the servo's reconnect factory, so a link lost in flight
        is rebuilt by rediscovering the port rather than assuming it came
        back with the same name -- a re-enumerated USB device usually does
        not.
        """
        ports = [self.args.port] if self.args.port else None
        bauds = [self.args.baud] if self.args.baud else None
        port, baud = devices.autodetect_visca(ports, bauds)
        if port is None:
            return None
        try:
            link = ViscaLink(port, baud, self.args.address)
            link.if_clear()
        except Exception as exc:
            log.warning("VISCA: %s at %d baud would not open: %s",
                        port, baud, exc)
            return None
        self.control_port = port
        log.info("VISCA: %s at %d baud", port, baud)
        return link

    def open_mavlink(self):
        url, baud = self.args.rc_url, self.args.rc_baud
        if url is None:
            log.info("probing for the flight controller...")
            url, baud = devices.autodetect_mavlink(exclude=self.control_port)
            if url is None:
                log.error("no MAVLink heartbeat on any serial port -- no RC "
                          "control, no telemetry. Pass --rc-url to pin it.")
                return
        self.mavlink = MavlinkSource(
            url=url, baud=baud, stream_rate_hz=self.args.stream_rate,
            on_status=lambda msg: log.info("MAVLink: %s", msg),
        )
        log.info("MAVLink: %s at %d baud, zoom on ch%d, record on ch%d",
                 url, baud, self.args.rc_channel, self.args.rec_channel)
        self.rc = RcZoomSource(
            channel=self.args.rc_channel,
            pwm_min=self.args.rc_pwm_min, pwm_max=self.args.rc_pwm_max,
            pwm_deadband=self.args.rc_deadband, reverse=self.args.rc_reverse,
            rc_timeout=self.args.rc_timeout,
            source=self.mavlink,
        )

    def start_servo(self):
        if self.link is None:
            return
        self.servo = ZoomServo(
            self.link, curve=self.args.curve, rc=self.rc,
            on_status=lambda msg: log.warning("%s", msg),
            link_factory=self.make_visca_link,
        )
        try:
            ratio = self.servo.sync_from_camera()
            log.info("zoom: currently %.1fx, following %s",
                     ratio, "RC" if self.rc else "nothing (no RC link)")
        except Exception as exc:
            log.warning("could not read the starting zoom position: %s", exc)

    def open_camera(self, quiet=False):
        device = self.args.video or devices.autodetect_video(
            log=(lambda msg: None) if quiet else
                (lambda msg: log.info("video: %s", msg))
        )
        if device is None:
            if not quiet:
                log.critical("no FCB camera found")
            return False

        capture = devices.open_capture(
            device, self.args.width, self.args.height, self.args.fourcc
        )
        if capture is None:
            if not quiet:
                log.critical("could not open %s", device)
            return False

        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info("video: %s at %dx%d", device, width, height)
        self.grabber = FrameGrabber(capture)
        return True

    def reopen_camera(self):
        """Tear the capture down and build it again after the stream died.

        A USB camera that browns out, is knocked loose, or wedges its
        streaming state comes back as a device that opens but never
        delivers, and nothing short of reopening it recovers. Nobody can
        replug anything mid-flight, so this keeps retrying on its own.
        """
        log.warning("video: reopening the camera")
        if self.grabber is not None:
            try:
                self.grabber.stop()
            except Exception:
                pass
            self.grabber = None

        # An in-progress recording cannot span the gap -- the frame stream
        # it was built around is gone -- so close it out now. If the switch
        # is still armed when frames return, run() opens a fresh segment.
        if self.video is not None:
            log.warning("video: closing the current recording at the break")
            self.stop_recording()

        if self.open_camera(quiet=True):
            log.info("video: camera recovered")
            return True
        return False

    # -- recording -------------------------------------------------------

    def start_recording(self, frame):
        height, width = frame.shape[:2]
        measured = self.grabber.fps()
        fps = measured if measured >= 1.0 else self.args.fps

        os.makedirs(self.args.record_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.base = os.path.join(self.args.record_dir, f"fcb-{stamp}")

        try:
            self.video = VideoRecorder(
                f"{self.base}.mp4", width, height, fps,
                encoder=self.args.encoder, bitrate_mbps=self.args.bitrate,
            )
        except RuntimeError as exc:
            log.error("could not start recording: %s", exc)
            self.base = None
            return

        self.csv = TelemetryCsv(f"{self.base}.csv")
        self.record_started_mono = time.monotonic()
        self.record_started_wall = datetime.now(timezone.utc)
        log.info("RECORDING STARTED  %s.mp4 (%dx%d @ %.2f fps, %s encoder)",
                 os.path.basename(self.base), width, height, fps,
                 self.video.encoder)
        log.info("                   %s.csv", os.path.basename(self.base))

    def stop_recording(self):
        if self.video is None:
            return
        frames = self.video.frames
        elapsed = self.video.close()
        rows = self.csv.rows
        self.csv.close()
        size_mb = os.path.getsize(f"{self.base}.mp4") / 1e6
        log.info("RECORDING STOPPED  %s.mp4 -- %d frames, %.1fs, %.1f MB "
                 "(%.1f fps average), %d CSV rows",
                 os.path.basename(self.base), frames, elapsed, size_mb,
                 frames / elapsed if elapsed > 0 else 0.0, rows)
        self.video = None
        self.csv = None

    def wants_recording(self):
        """Whether the RC switch is asking for recording right now."""
        if self.mavlink is None:
            return False
        age = self.mavlink.rc_age()
        if age is None or age > self.args.rc_timeout:
            return None  # unknown -- RC is stale, hold whatever we are doing
        pwm = self.mavlink.channel(self.args.rec_channel)
        if pwm <= 0:
            return None
        high = pwm >= self.args.rec_channel_threshold
        return not high if self.args.rec_channel_reverse else high

    # -- main loop -------------------------------------------------------

    def run(self):
        last_seq = 0
        last_rc_warning = 0.0
        waiting_logged = False
        last_frame_at = time.monotonic()

        while self.running:
            frame, seq, captured_at = self.grabber.wait_for_frame(
                last_seq, timeout=0.2
            )
            now = time.monotonic()

            if frame is None:
                if not waiting_logged:
                    log.warning("no frames from the camera")
                    waiting_logged = True
                # Give the stream a grace period, then rebuild it. Without
                # this a camera that drops out stays dropped out and the
                # rest of the flight records nothing.
                if now - last_frame_at > self.args.camera_timeout:
                    if self.reopen_camera():
                        last_seq = 0
                    last_frame_at = time.monotonic()
                self.update_display(now)
                continue
            last_frame_at = now
            if waiting_logged:
                log.info("frames flowing again")
                waiting_logged = False

            if self.frames_seen == 0:
                log.info("first frame received (%dx%d)",
                         frame.shape[1], frame.shape[0])
            dropped = seq - last_seq - 1
            if dropped > 0:
                self.frames_dropped += dropped
                log.debug("skipped %d frame(s) -- consumer behind the camera",
                          dropped)
            last_seq = seq
            self.frames_seen += 1

            wanted = self.wants_recording()
            if wanted is None:
                if now - last_rc_warning > 5.0:
                    log.warning("no RC data -- holding recording state")
                    last_rc_warning = now
            elif wanted and self.video is None:
                self.start_recording(frame)
            elif not wanted and self.video is not None:
                self.stop_recording()

            if self.video is not None:
                frame_number = self.video.frames
                self.video.write(frame)
                wall = self.record_started_wall + _delta(
                    captured_at - self.record_started_mono
                )
                self.csv.write(
                    frame_number, wall, captured_at - self.record_started_mono,
                    self.mavlink.telemetry() if self.mavlink else _NO_TELEMETRY,
                    self.servo if self.servo else _NO_ZOOM,
                    now,
                )

            self.update_display(now)

    def update_display(self, now):
        """Repaint the live line often; write a log line occasionally.

        Two different rates on purpose. The live line is what you watch
        over SSH, so it wants to be quick; the log file is what you read
        after the flight, and filling it at the same rate would bury the
        events that matter in thousands of near-identical lines.
        """
        # Nothing about drawing a status line is worth losing a recording
        # over, so a fault in here is reported once and swallowed rather
        # than being allowed to unwind the capture loop.
        try:
            if now - self._last_live >= self.args.status_interval:
                self._last_live = now
                LIVE.show(self.status_text(now))

            if now - self._last_status >= self.args.log_status_interval:
                self._last_status = now
                log.info("status: %s", self.status_text(now))
        except Exception:
            if not self._status_broken:
                self._status_broken = True
                log.exception("status display failed; continuing without it")

    def status_text(self, now):
        parts = [f"{self.grabber.fps():.1f} fps" if self.grabber else "no video",
                 f"frames {self.frames_seen}"]
        if self.frames_dropped:
            parts.append(f"skipped {self.frames_dropped}")

        if self.video is not None:
            elapsed = now - self.record_started_mono
            parts.append(f"REC {os.path.basename(self.base)}.mp4 "
                         f"({self.video.frames} fr, {elapsed:.0f}s)")
        else:
            parts.append("not recording")

        if self.servo is not None and self.servo.ratio is not None:
            parts.append(f"zoom {self.servo.ratio:.1f}x")

        if self.mavlink is not None:
            telemetry = self.mavlink.telemetry()
            if telemetry.lat_deg is not None:
                parts.append(f"GPS {telemetry.lat_deg:.6f},{telemetry.lon_deg:.6f}")
            else:
                parts.append("GPS no fix")
            if telemetry.alt_rel_m is not None:
                # Altitude is barometric, so it is there with or without a fix.
                parts.append(f"alt {telemetry.alt_msl_m:.1f}m "
                             f"rel {telemetry.alt_rel_m:.1f}m")
            if telemetry.roll_deg is not None:
                parts.append(f"rpy {telemetry.roll_deg:.0f}/"
                             f"{telemetry.pitch_deg:.0f}/{telemetry.yaw_deg:.0f}")
            if telemetry.groundspeed_ms is not None:
                parts.append(f"gs {telemetry.groundspeed_ms:.1f}m/s "
                             f"hdg {telemetry.heading_deg:.0f}")
            if not self.mavlink.is_connected():
                parts.append("MAVLINK DOWN")
        return " | ".join(parts)

    # -- shutdown --------------------------------------------------------

    def close(self):
        LIVE.clear()
        self.stop_recording()
        if self.grabber is not None:
            self.grabber.stop()
        if self.servo is not None:
            self.servo.stop()
        if self.rc is not None:
            self.rc.stop()
        if self.mavlink is not None:
            self.mavlink.stop()
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
        log.info("shut down cleanly -- %d frames seen, %d skipped",
                 self.frames_seen, self.frames_dropped)


class _NoTelemetry:
    lat_deg = lon_deg = alt_msl_m = alt_rel_m = position_age_s = None
    roll_deg = pitch_deg = yaw_deg = attitude_age_s = None
    heading_deg = groundspeed_ms = hud_age_s = None
    position_boot_ms = attitude_boot_ms = None


class _NoZoom:
    ratio = position = updated_at = None


_NO_TELEMETRY = _NoTelemetry()
_NO_ZOOM = _NoZoom()


def _delta(seconds):
    from datetime import timedelta
    return timedelta(seconds=seconds)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Headless RC-armed flight recorder for the FCB-EV9520L",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('"""')[0],
    )
    parser.add_argument("--video", help="UVC device (default: autodetect)")
    parser.add_argument("--port", help="VISCA serial device (default: autodetect)")
    parser.add_argument("--baud", type=int, default=None,
                        help="VISCA baud (default: try 9600, 38400, 115200)")
    parser.add_argument("--address", type=int, default=1, help="VISCA address 1-7")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=60.0,
                        help="fallback fps tag if the rate cannot be measured")
    parser.add_argument("--fourcc", help="force a capture format, e.g. MJPG")
    parser.add_argument("--curve", default="log", choices=list(zoom_map.CURVES),
                        help="zoom shaping (default: log)")
    parser.add_argument("--record-dir",
                        default=os.path.expanduser("~/fcb_recordings"))
    parser.add_argument("--encoder", default="nvenc",
                        choices=("nvenc", "cpu"),
                        help="nvenc uses the Jetson's hardware encoder via "
                             "GStreamer and falls back to cpu if unavailable")
    parser.add_argument("--bitrate", type=float, default=25.0,
                        help="NVENC bitrate in Mbps (default: 25)")

    parser.add_argument("--rc-url", default=None,
                        help="MAVLink connection (default: autodetect by "
                             "probing for a heartbeat)")
    parser.add_argument("--rc-baud", type=int, default=115200)
    parser.add_argument("--rc-channel", type=int, default=7,
                        help="RC channel driving zoom (default: 7)")
    parser.add_argument("--rc-pwm-min", type=int, default=1000)
    parser.add_argument("--rc-pwm-max", type=int, default=2000)
    parser.add_argument("--rc-deadband", type=int, default=8)
    parser.add_argument("--rc-reverse", action="store_true")
    parser.add_argument("--rc-timeout", type=float, default=2.0)
    parser.add_argument("--stream-rate", type=int, default=10,
                        help="Hz to ask the autopilot to stream telemetry at")

    parser.add_argument("--rec-channel", type=int, default=8,
                        help="RC channel arming the recording (default: 8)")
    parser.add_argument("--rec-channel-threshold", type=int, default=1500,
                        help="PWM at or above this is the 'record' position")
    parser.add_argument("--rec-channel-reverse", action="store_true",
                        help="treat low PWM as the 'record' position")

    parser.add_argument("--status-interval", type=float, default=0.25,
                        help="seconds between live status refreshes on the "
                             "terminal (default: 0.25, i.e. 4 Hz)")
    parser.add_argument("--log-status-interval", type=float, default=5.0,
                        help="seconds between status lines written to the log "
                             "file (default: 5; the live line is faster)")
    parser.add_argument("--camera-timeout", type=float, default=5.0,
                        help="seconds without frames before the camera is "
                             "reopened (default: 5)")
    parser.add_argument("--log-level", default="DEBUG",
                        choices=("DEBUG", "INFO", "WARNING"),
                        help="detail written to the log file (default: DEBUG)")
    parser.add_argument("--quiet", action="store_true",
                        help="log to the file only, not the console")
    return parser.parse_args()


def main():
    args = parse_args()
    log_path = setup_logging(args.record_dir, args.log_level, args.quiet)

    log.info("fcb_record starting -- recordings and logs in %s",
             args.record_dir)
    log.info("log file: %s", log_path)
    log.debug("arguments: %s", vars(args))

    recorder = Recorder(args)

    def on_signal(signum, _frame):
        log.info("caught %s -- finishing the recording and shutting down",
                 signal.Signals(signum).name)
        recorder.running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        # Control before video: the grab thread streams 1080p continuously
        # over the same USB3 link the NeoHD board carries VISCA on, and
        # probing baud rates against a busy bus makes the handshake flaky.
        recorder.open_control()
        recorder.open_mavlink()
        recorder.start_servo()
        if not recorder.open_camera():
            return 1
        log.info("ready -- channel %d high starts recording",
                 args.rec_channel)
        recorder.run()
    except Exception:
        log.exception("unhandled error")
        return 1
    finally:
        recorder.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
