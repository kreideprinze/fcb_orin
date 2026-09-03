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

Everything else is flown from the transmitter, but the day/night imaging
mode is not on a spare RC channel, so it is switched from the keyboard of
whatever terminal is attached -- your laptop, over SSH, into the tmux
session the recorder runs in:

    1   RGB      daylight colour, IR cut filter in
    2   IR       filter out, infrared-sensitive mono
    3   RGB+IR   filter out, colour retained (false colours under IR)
    4   AUTO     the camera switches on scene brightness
    i   cycle through the four in turn
    ?   print a status line now

Keys are read only when stdin is a terminal, so running under nohup, a
pipe or systemd simply has no keyboard and changes nothing else. There is
deliberately no quit key: a stray keypress must not be able to end a
recording mid-flight. Ctrl-C, or ./start_recorder.sh --stop, does that.

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
import select
import shutil
import signal
import sys
import termios
import threading
import time
import tty
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

# Beside this script rather than in the driver package: it is about
# showing a picture in a terminal, which is a recorder concern, and
# sync_driver.sh would delete it from the bundle's copy of the driver.
import snapshot

try:
    from fcb_base_driver import devices, visca, zoom_map
    from fcb_base_driver.frame_grabber import FrameGrabber
    from fcb_base_driver.mavlink_source import MavlinkSource
    from fcb_base_driver.rc_source import RcZoomSource
    from fcb_base_driver.rc_switch import RcButton, RcSelector
    from fcb_base_driver.visca_link import ViscaError, ViscaLink, ViscaTimeout
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

#: Imaging modes, in the operator's vocabulary rather than Sony's.
#:
#: The FCB-EV9520L has one visible-light sensor behind a mechanically
#: removable IR cut filter -- it is not a thermal camera, and there is no
#: second stream to switch to. Removing the filter lets infrared reach that
#: same sensor, which is what "IR" means here. The manual calls these ICR
#: On/Off, named after the filter rather than the picture, and the sense is
#: the inverse of what anyone expects; visca.ICR_MODES does the translation.
#:
#: Each entry is (key, name shown, ICR mode, what it does).
ICR_MODES = (
    ("1", "RGB",    "day",         "daylight colour, IR cut filter in"),
    ("2", "IR",     "night",       "filter out, infrared-sensitive mono"),
    ("3", "RGB+IR", "night_color", "filter out, colour retained"),
    ("4", "AUTO",   "auto",        "camera switches on scene brightness"),
)

#: Argument/key lookups built from the table, so the table stays the one
#: place a mode is defined.
ICR_BY_KEY = {key: mode for key, _, mode, _ in ICR_MODES}
ICR_BY_NAME = {name.lower(): mode for _, name, mode, _ in ICR_MODES}
ICR_LABELS = {mode: f"{name} ({what})" for _, name, mode, what in ICR_MODES}
ICR_SHORT = {mode: name for _, name, mode, _ in ICR_MODES}
ICR_ORDER = [mode for _, _, mode, _ in ICR_MODES]


class TerminalKeys:
    """Single keypresses from the terminal, without waiting for Enter.

    The recorder is otherwise flown entirely from the transmitter, and this
    is the one control that is not: the terminal attached to the tmux
    session -- your laptop over SSH -- is where the imaging mode is
    chosen. So stdin is put in cbreak mode and keys are read as they are
    pressed, the same way fcb_teleop.py does it.

    cbreak rather than raw: it leaves signal generation on, so Ctrl-C still
    stops the recorder cleanly and `start_recorder.sh --stop`, which works
    by sending one, keeps working.

    Without a terminal on stdin -- piped, backgrounded under nohup, run
    from systemd -- this quietly does nothing and get() always returns
    None, so an unattended recorder behaves exactly as it did before.
    """

    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self._fd = sys.stdin.fileno() if self.enabled else None
        self._saved = None

    def __enter__(self):
        if self.enabled:
            try:
                self._saved = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
            except termios.error as exc:
                # A tty that will not go into cbreak is not worth losing a
                # flight over; carry on without the keyboard.
                log.warning("keyboard unavailable (%s) -- RC control only", exc)
                self.enabled = False
        return self

    def __exit__(self, *exc):
        self.restore()

    def restore(self):
        """Put the terminal back. Safe to call more than once."""
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            self._saved = None

    def get(self):
        """The next key as a string, or None if nothing is waiting.

        Read from the file descriptor rather than through sys.stdin.
        sys.stdin is buffered, so reading one character from it pulls the
        whole burst out of the kernel and keeps the rest in a Python-side
        buffer that select() cannot see -- which makes two keys pressed in
        quick succession look like one, with the second arriving only once
        some later keypress wakes the loop again. os.read leaves nothing
        anywhere but the fd, so select() stays the truth.

        Escape sequences (arrow keys, and anything else starting 0x1B) are
        swallowed rather than decoded -- nothing here is bound to one, and
        letting the bytes through would fire two unrelated actions.
        """
        if not self.enabled or not self._ready():
            return None
        try:
            data = os.read(self._fd, 1)
        except OSError:
            return None
        if not data:
            return None
        if data != b"\x1b":
            return data.decode("utf-8", "replace")
        # Escape, alone or opening the sequence an arrow key sends. Consume
        # exactly that sequence -- up to and including its final byte, which
        # is what 0x40-0x7E marks -- rather than draining everything
        # pending, or a key typed right behind an arrow key is eaten with it.
        if not self._ready():
            return None
        try:
            if os.read(self._fd, 1) not in (b"[", b"O"):
                return None
            while self._ready():
                final = os.read(self._fd, 1)
                if not final or 0x40 <= final[0] <= 0x7E:
                    break
        except OSError:
            pass
        return None

    def pending(self):
        """Whether there is anything left to read.

        The caller drains on this rather than on get() returning None: a
        swallowed escape sequence also returns None, and stopping there
        would leave whatever was typed behind an arrow key unread.
        """
        return self.enabled and self._ready()

    def _ready(self, timeout=0.0):
        return bool(select.select([self._fd], [], [], timeout)[0])


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
        # The snapshot preview is drawn from a worker thread while the
        # capture loop is still repainting this line. Held across the
        # write *and* the flag, so a preview cannot land between a clear
        # and its redraw. Re-entrant because a log record emitted while
        # the preview holds it would otherwise deadlock on clear().
        self.lock = threading.RLock()

    def show(self, text):
        if not self.enabled:
            return
        width = shutil.get_terminal_size((120, 24)).columns
        with self.lock:
            self.stream.write("\r\033[K" + text[:width - 1])
            self.stream.flush()
            self._shown = True

    def clear(self):
        if not self.enabled or not self._shown:
            return
        with self.lock:
            self.stream.write("\r\033[K")
            self.stream.flush()
            self._shown = False


LIVE = LiveLine()


class LiveAwareHandler(logging.StreamHandler):
    """Console handler that steps around the live status line."""

    def emit(self, record):
        with LIVE.lock:
            LIVE.clear()
            super().emit(record)


def setup_logging(directory, level, quiet):
    """Log to the console and, in full detail, to one file per day.

    A file per process start piles up fast. The tmux supervisor restarts
    the recorder after every crash, so a camera that will not come back
    used to strand a fresh near-empty log every few seconds, burying the
    recordings among them. One file a day, appended to, keeps every run of
    that day in order in one place and leaves the directory readable.
    """
    os.makedirs(directory, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d")
    path = os.path.join(directory, f"fcb_record-{stamp}.log")

    # Appending, so say where this run begins. Otherwise a restart reads
    # as a hiccup in the middle of the previous one, and the per-line
    # timestamps are clock time only -- no date, no run boundary.
    if os.path.exists(path) and os.path.getsize(path) > 0:
        try:
            with open(path, "a") as fh:
                fh.write("\n===== run started %s =====\n"
                         % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        except OSError:
            pass  # the handler below will report a real write problem

    root = logging.getLogger("fcb")
    root.setLevel(logging.DEBUG)
    root.handlers.clear()

    fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    to_file = logging.FileHandler(path, mode="a")
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
        # The framerate has to be a whole number here. OpenCV copies the fps
        # it is handed straight into the appsrc caps, so a measured rate
        # like 56.39 arrives as framerate=5639/100, which nvv4l2h264enc will
        # not negotiate. The pipeline then opens, reports itself ready, and
        # refuses every buffer -- leaving an empty file while the frame
        # counter and the CSV carry on as though it were recording. So the
        # rate is rounded, and the same rounded value is given to both the
        # caps and the writer so the two cannot disagree.
        fps = max(1, int(round(self.fps)))
        pipeline = (
            f"appsrc ! video/x-raw,format=BGR,width={self.width},"
            f"height={self.height},framerate={fps}/1 "
            f"! queue ! videoconvert ! video/x-raw,format=NV12 "
            f"! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc bitrate={int(bitrate_mbps * 1_000_000)} "
            f"insert-sps-pps=1 maxperf-enable=1 "
            f"! h264parse ! qtmux ! filesink location={self.path}"
        )
        log.debug("trying GStreamer pipeline: %s", pipeline)
        try:
            writer = cv2.VideoWriter(
                pipeline, cv2.CAP_GSTREAMER, 0, float(fps),
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


def draw_snapshot(args, frame, why, number):
    """Write the JPEG and print the preview. Logs its own outcome.

    Split out of Recorder.take_snapshot so the worker thread and the
    inline fallback cannot drift apart.
    """
    path = size = None
    try:
        path, size = snapshot.save(
            frame, args.snap_dir or os.path.join(args.record_dir, "snapshots"),
            max_width=args.snap_width, quality=args.snap_quality,
        )
    except Exception as exc:
        log.error("snapshot: could not write the JPEG: %s", exc)

    drawn = ""
    if args.preview and sys.stdout.isatty():
        try:
            text, cols, rows = snapshot.render(frame, max_cols=args.preview_cols)
            # Step around the live status line the same way a log record
            # does, or the preview lands on top of it.
            with LIVE.lock:
                LIVE.clear()
                sys.stdout.write(text + "\n")
                sys.stdout.flush()
            drawn = ", %dx%d preview (%.0f KB) to this pane" % (
                cols, rows, len(text.encode("utf-8")) / 1024.0
            )
        except Exception:
            log.exception("snapshot: preview failed; the JPEG is still on "
                          "the drone")

    if path is not None:
        log.info("snapshot %d [%s]: %s (%.0f KB)%s",
                 number, why, path, size / 1024.0, drawn)
    else:
        log.info("snapshot %d [%s]: preview only%s", number, why, drawn)


class SnapshotWorker:
    """Renders and files snapshots off the capture thread.

    Measured on the Orin, one press costs about 21 ms -- 17 ms of it the
    100-column render -- against a 16.7 ms frame budget at 59.94 fps. Done
    inline that drops a frame or two from the recording every time the
    pilot looks at the picture, which is a poor trade for a preview nobody
    is timing. So the capture loop hands the frame over and goes straight
    back to grabbing.

    The queue is one deep and the newest wins. A pilot leaning on the
    button wants the frame they are looking at now, not a backlog of stale
    ones rendered one after another well after the moment has passed.

    Holding a frame reference is safe: the grabber rebinds its buffer to a
    fresh array per capture rather than writing into the old one, so the
    frame handed over here does not change underneath the render.
    """

    def __init__(self, args):
        self.args = args
        self.superseded = 0
        self._pending = None
        self._cond = threading.Condition()
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="snapshot", daemon=True
        )
        self._thread.start()

    def submit(self, frame, why, number):
        """Queue a frame, displacing any not yet drawn. Never blocks."""
        with self._cond:
            displaced = self._pending is not None
            self._pending = (frame, why, number)
            self._cond.notify()
        if displaced:
            self.superseded += 1
        return not displaced

    def stop(self, timeout=3.0):
        """Let a queued snapshot finish, then retire the thread."""
        with self._cond:
            self._running = False
            self._cond.notify()
        self._thread.join(timeout)

    def _run(self):
        while True:
            with self._cond:
                while self._running and self._pending is None:
                    self._cond.wait()
                if self._pending is None:
                    return
                job, self._pending = self._pending, None
            try:
                draw_snapshot(self.args, *job)
            except Exception:
                log.exception("snapshot: failed; continuing")


class Recorder:

    def __init__(self, args):
        self.args = args
        self.link = None
        self.control_port = None
        self.declared_fps = None
        self.servo = None
        self.mavlink = None
        self.rc = None
        self.grabber = None

        self.video = None
        self.csv = None
        self.base = None
        self.record_started_mono = None
        self.record_started_wall = None

        self.icr_mode = None
        self.keys = None

        # RC channels that are switches rather than knobs.
        self.snap_button = None
        self.snap_worker = None
        self._state_broken = False
        self.icr_switch = None
        self.snapshots = 0
        self.last_frame = None

        # Digits typed after 'z', or None when not entering a zoom.
        self.zoom_entry = None

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

        if self.args.snap_channel:
            self.snap_button = RcButton(
                self.mavlink, self.args.snap_channel,
                threshold=self.args.snap_threshold,
                reverse=self.args.snap_reverse,
                rc_timeout=self.args.rc_timeout,
            )
            log.info("snapshot: ch%d, one press sends a frame to this pane",
                     self.args.snap_channel)

        if self.args.icr_channel:
            # Low, centre, high -- in the order the switch travels, which is
            # the order the modes are listed in on the transmitter.
            self.icr_switch = RcSelector(
                self.mavlink, self.args.icr_channel,
                ["day", "night_color", "night"],
                pwm_min=self.args.rc_pwm_min, pwm_max=self.args.rc_pwm_max,
                reverse=self.args.icr_channel_reverse,
                rc_timeout=self.args.rc_timeout,
            )
            log.info("imaging mode: ch%d, low RGB / centre RGB+IR / high IR",
                     self.args.icr_channel)

    def start_servo(self):
        if self.link is None:
            return
        self.servo = ZoomServo(
            self.link, curve=self.args.curve, rc=self.rc,
            on_status=lambda msg: log.warning("%s", msg),
            link_factory=self.make_visca_link,
            ratio_step=self.args.zoom_step,
        )
        if self.args.zoom_step > 0:
            steps = zoom_map.ratio_steps(self.args.zoom_step)
            log.info("zoom: detented in %.2gx steps -- %d positions from "
                     "%.1fx to %.1fx", self.args.zoom_step, len(steps),
                     steps[0], steps[-1])
        try:
            ratio = self.servo.sync_from_camera()
            log.info("zoom: currently %.1fx, following %s",
                     ratio, "RC" if self.rc else "nothing (no RC link)")
        except Exception as exc:
            log.warning("could not read the starting zoom position: %s", exc)

        self.start_icr()

    def start_icr(self):
        """Set the imaging mode if one was asked for, otherwise adopt it.

        With no --icr the camera is left exactly as it was found -- it
        keeps the mode across a power cycle, so whatever was chosen last
        flight is presumably still wanted -- and the mode is only read back
        so the status line and the 'i' cycle start from the truth rather
        than from an assumption.
        """
        if self.args.icr:
            self.set_icr(ICR_BY_NAME[self.args.icr], announce="imaging mode")
            return
        try:
            payload = self.link.inquiry(visca.icr_mode_inq(self.link.address))
            self.icr_mode = visca.parse_icr_mode(payload)
        except (ViscaError, ViscaTimeout, ValueError) as exc:
            log.warning("could not read the current imaging mode: %s", exc)
            return
        # The inquiry reports where the filter is, not how it got there, so
        # a camera left in automatic reads back as whichever mode it has
        # currently selected. Said plainly rather than displayed as though
        # the mode had been pinned.
        log.info("imaging mode: %s (as found -- pass --icr to set it, or "
                 "press 1-4 while running)", ICR_LABELS[self.icr_mode])

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
        # What the camera says it runs at. Preferred over measuring, which
        # needs a second or two of frames to mean anything and is wrong if
        # recording starts before then.
        declared = capture.get(cv2.CAP_PROP_FPS)
        self.declared_fps = declared if 1.0 < declared < 1000.0 else None
        log.info("video: %s at %dx%d, %s", device, width, height,
                 f"{self.declared_fps:.2f} fps" if self.declared_fps
                 else "rate not reported")
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
        # Say so plainly. Nobody is watching the drone, so the pane and the
        # log are the only places a dead camera can announce itself.
        log.error("video: camera did not come back -- retrying every %gs",
                  self.args.camera_timeout)
        return False

    # -- imaging mode ----------------------------------------------------

    def set_icr(self, mode, announce="imaging mode"):
        """Switch the IR cut filter. Logged either way, including failures.

        A mode change is a change to the footage being recorded, so it
        belongs in the log next to the recording it applies to -- and a
        change that did not take has to say so, since there is no monitor
        on the drone to notice it on.
        """
        if self.servo is None:
            log.warning("no VISCA control -- cannot change the imaging mode")
            return False
        try:
            self.servo.set_icr(mode)
        except (ViscaError, ViscaTimeout, ValueError) as exc:
            log.error("imaging mode: %s would not take: %s",
                      ICR_LABELS.get(mode, mode), exc)
            return False
        self.icr_mode = mode
        log.info("%s: %s", announce, ICR_LABELS[mode])
        return True

    def cycle_icr(self):
        """Step to the next mode, starting from whatever is current."""
        try:
            index = ICR_ORDER.index(self.icr_mode)
        except ValueError:
            index = -1  # unknown, so the cycle starts at the first mode
        self.set_icr(ICR_ORDER[(index + 1) % len(ICR_ORDER)])

    # -- zoom by hand ----------------------------------------------------

    def begin_zoom_entry(self):
        if self.servo is None:
            log.warning("no VISCA control -- cannot set the zoom")
            return
        self.zoom_entry = ""
        self.show_zoom_entry()

    def show_zoom_entry(self):
        """Draw the prompt where the status line normally lives.

        The status line repaints four times a second and would scribble
        over anything typed underneath it, so while an entry is in progress
        the prompt *is* the status line. Kept short deliberately: in the
        GCS layout this pane is half a window wide and the line is
        truncated to fit.
        """
        LIVE.show(f"zoom> {self.zoom_entry}x  [1-30, Enter]")

    def handle_zoom_entry(self, key):
        if key in ("\r", "\n"):
            text, self.zoom_entry = self.zoom_entry, None
            if not text:
                log.info("zoom: entry cancelled")
                return
            try:
                wanted = float(text)
            except ValueError:
                log.warning("zoom: %r is not a number", text)
                return
            self.apply_zoom(wanted)
            return
        if key in ("\x7f", "\b"):
            self.zoom_entry = self.zoom_entry[:-1]
        elif key.isdigit() or (key == "." and "." not in self.zoom_entry):
            # Four characters covers 30.0; anything longer is a typo.
            if len(self.zoom_entry) < 4:
                self.zoom_entry += key
        self.show_zoom_entry()

    def apply_zoom(self, wanted):
        try:
            got = self.servo.set_ratio(wanted)
        except Exception as exc:
            log.error("zoom: could not set %.2fx: %s", wanted, exc)
            return
        if abs(got - wanted) > 0.01:
            log.info("zoom: %.2fx requested, clamped to %.2fx (the lens does "
                     "%.0fx-%.0fx)", wanted, got,
                     zoom_map.MIN_RATIO, zoom_map.MAX_RATIO)
        else:
            log.info("zoom: %.2fx set by hand -- the ch%d knob is ignored "
                     "until you press 'a'", got, self.args.rc_channel)

    def hand_zoom_to_rc(self):
        if self.servo is None:
            return
        if self.servo.hand_to_rc():
            log.info("zoom: back under the ch%d knob", self.args.rc_channel)
        else:
            log.warning("zoom: no RC link to hand back to")

    # -- snapshots -------------------------------------------------------

    def take_snapshot(self, why="ch%d"):
        """Put the current frame in front of whoever is on the ground.

        Two outputs for two purposes: a JPEG on the drone, which is the one
        with detail in it, and a coarse colour rendering printed into this
        pane, which is the one that actually crosses the link. The pane is
        the ground station -- there is no video downlink -- so the render
        is what makes the button worth pressing in flight.
        """
        frame = self.last_frame
        if frame is None:
            log.warning("snapshot: no frame yet")
            return False

        # Counted on the press, not on the draw, so the status line
        # acknowledges the button immediately.
        self.snapshots += 1
        if self.snap_worker is not None:
            self.snap_worker.submit(frame, why, self.snapshots)
        else:
            # Outside run() there is no worker. Do it here rather than not
            # at all -- nothing is recording on that path, so the cost of
            # doing it inline buys nothing to avoid.
            draw_snapshot(self.args, frame, why, self.snapshots)
        return True

    # -- RC switches -----------------------------------------------------

    def poll_rc_switches(self):
        """Service the momentary and selector channels once per frame.

        Both are edge-triggered, so this is cheap and silent while nothing
        is being touched. Failures are contained: a switch that misbehaves
        must not be able to end a recording that is in progress.
        """
        try:
            if self.snap_button is not None and self.snap_button.pressed():
                self.take_snapshot(why=f"ch{self.args.snap_channel}")
        except Exception:
            log.exception("snapshot channel failed; continuing")

        try:
            if self.icr_switch is not None:
                mode = self.icr_switch.changed()
                # Only ever on a transition, which is what leaves the 1-4
                # keys usable: the switch reasserts nothing until the pilot
                # actually moves it.
                if mode is not None and mode != self.icr_mode:
                    self.set_icr(mode, announce=
                                 f"imaging mode (ch{self.args.icr_channel})")
        except Exception:
            log.exception("imaging mode channel failed; continuing")

    # -- keys ------------------------------------------------------------

    def handle_key(self, key):
        """Act on one keypress. Unknown keys are ignored, not reported.

        There is no quit key on purpose. This runs unattended on a flying
        drone with a terminal that anyone might lean on; ending a recording
        must take Ctrl-C or start_recorder.sh --stop, not one keystroke.
        """
        # An entry in progress swallows everything until it ends, or the
        # digits of a zoom would be read as mode changes.
        if self.zoom_entry is not None:
            self.handle_zoom_entry(key)
            return

        if key in ICR_BY_KEY:
            self.set_icr(ICR_BY_KEY[key])
        elif key in ("i", "I"):
            self.cycle_icr()
        elif key in ("z", "Z"):
            self.begin_zoom_entry()
        elif key in ("a", "A"):
            self.hand_zoom_to_rc()
        elif key in ("s", "S"):
            # The same thing the RC button does, for when the transmitter
            # is off or you are already looking at the pane.
            self.take_snapshot(why="key")
        elif key == "?":
            log.info("status: %s", self.status_text(time.monotonic()))
            self.announce_keys()

    # -- recording -------------------------------------------------------

    def start_recording(self, frame, captured_at=None):
        height, width = frame.shape[:2]
        # The container's frame rate decides playback speed, so it has to be
        # right from the first frame. The camera's own figure is used where
        # it has one; a measured rate is only trusted once enough frames
        # have arrived for it to be meaningful.
        measured = self.grabber.fps()
        if self.declared_fps:
            fps, source = self.declared_fps, "camera"
        elif measured >= 1.0:
            fps, source = measured, "measured"
        else:
            fps, source = self.args.fps, "--fps default"

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
        # Time zero is the first frame that gets written, not the instant
        # this function happens to run -- otherwise the opening rows carry
        # small negative timestamps, having been captured just before
        # recording was asked for.
        now_mono = time.monotonic()
        self.record_started_mono = now_mono if captured_at is None else captured_at
        self.record_started_wall = datetime.now(timezone.utc) - _delta(
            now_mono - self.record_started_mono
        )
        log.info("RECORDING STARTED  %s.mp4 (%dx%d @ %.2f fps from %s, "
                 "%s encoder)",
                 os.path.basename(self.base), width, height, fps, source,
                 self.video.encoder)
        log.info("                   %s.csv", os.path.basename(self.base))

    def stop_recording(self):
        if self.video is None:
            return
        frames = self.video.frames
        encoder = self.video.encoder
        elapsed = self.video.close()
        rows = self.csv.rows
        self.csv.close()
        size_bytes = os.path.getsize(f"{self.base}.mp4")
        size_mb = size_bytes / 1e6
        log.info("RECORDING STOPPED  %s.mp4 -- %d frames, %.1fs, %.1f MB "
                 "(%.1f fps average), %d CSV rows",
                 os.path.basename(self.base), frames, elapsed, size_mb,
                 frames / elapsed if elapsed > 0 else 0.0, rows)

        # Frames counted but nothing on disk means the encoder rejected
        # every buffer. That failure is otherwise completely silent -- the
        # counters, the CSV and this summary all look healthy -- so it is
        # called out rather than left to be discovered after the flight.
        if frames > 0 and size_bytes < 10_000:
            log.error(
                "THE VIDEO IS EMPTY (%d bytes) despite %d frames -- the %s "
                "encoder accepted the file but rejected the frames, so this "
                "recording has no usable video. The CSV is still valid. "
                "Re-run with --encoder cpu to keep recording while the "
                "encoder problem is investigated.",
                size_bytes, frames, encoder,
            )
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
        """Capture and record until asked to stop, watching the keyboard.

        The keyboard wraps the capture loop rather than living inside it so
        the terminal is put back the way it was found on every exit path,
        including an unhandled error -- a raw terminal left behind would
        make the SSH session it was flown from unusable.
        """
        with TerminalKeys() as keys:
            self.keys = keys
            self.announce_keys()
            self.snap_worker = SnapshotWorker(self.args)
            try:
                self.capture_loop()
            finally:
                worker, self.snap_worker = self.snap_worker, None
                worker.stop()

    def announce_keys(self):
        if self.keys is not None and self.keys.enabled:
            log.info("keys: %s  i cycle  s snap  z zoom to an exact value  "
                     "a knob back  ? status  (Ctrl-C stops)",
                     "  ".join(f"{key} {name}" for key, name, _, _ in ICR_MODES))
        else:
            log.info("stdin is not a terminal -- no keyboard, so the imaging "
                     "mode stays as it is; RC still drives everything else")

    def poll_keys(self):
        """Drain whatever has been typed since the last frame.

        Drained rather than read one per frame: keys typed faster than the
        loop turns would otherwise queue up and replay afterwards, which on
        a mode switch means the camera stepping through modes seconds after
        the last keypress.

        Nothing here is worth losing a recording over, so a fault is
        reported and swallowed the same way the status line's is.
        """
        if self.keys is None:
            return
        try:
            while self.keys.pending():
                key = self.keys.get()
                if key is not None:
                    self.handle_key(key)
        except Exception:
            log.exception("keyboard handling failed; continuing")

    def capture_loop(self):
        last_seq = 0
        last_rc_warning = 0.0
        waiting_logged = False
        last_frame_at = time.monotonic()

        while self.running:
            if self.grabber is None:
                # The last reopen failed, so the camera is gone rather than
                # merely stalled. Stand in for the frame wait -- same shape,
                # same cadence -- so the loop keeps running and retries on
                # the timeout below instead of dereferencing nothing.
                time.sleep(0.2)
                frame, seq, captured_at = None, last_seq, None
            else:
                frame, seq, captured_at = self.grabber.wait_for_frame(
                    last_seq, timeout=0.2
                )
            now = time.monotonic()
            self.poll_keys()

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
            # Held for the snapshot button, which fires between frames and
            # needs the most recent one rather than waiting for the next.
            self.last_frame = frame
            self.poll_rc_switches()

            wanted = self.wants_recording()
            if wanted is None:
                if now - last_rc_warning > 5.0:
                    log.warning("no RC data -- holding recording state")
                    last_rc_warning = now
            elif wanted and self.video is None:
                self.start_recording(frame, captured_at)
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

    def publish_state(self, text):
        """Write the full status line where other processes can read it.

        The ground-station panes cannot get this from the terminal. The
        live line is truncated to the pane width, and splitting the window
        for the GCS layout makes that pane narrower than the line is long,
        so heading and groundspeed fall off the end. Publishing it whole
        is the only way the flight-data panel sees every field.

        Goes to a tmpfs by default, and is replaced atomically: a reader
        never sees a half-written line, and the card the recordings are on
        takes no extra writes.
        """
        path = self.args.state_file
        if not path or self._state_broken:
            return
        try:
            tmp = f"{path}.{os.getpid()}.tmp"
            with open(tmp, "w") as handle:
                handle.write(text + "\n")
            os.replace(tmp, path)
        except OSError as exc:
            # Once, then never again: a status file nobody can write is
            # not worth a message per frame.
            self._state_broken = True
            log.warning("could not publish state to %s: %s -- the flight-data "
                        "pane will fall back to reading the pane", path, exc)

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
                text = self.status_text(now)
                # An entry in progress owns the line; the state file still
                # gets the real status, so the GCS panes are unaffected.
                if self.zoom_entry is not None:
                    self.show_zoom_entry()
                else:
                    LIVE.show(text)
                self.publish_state(text)

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
            target = self.servo.target_ratio
            # While the lens is still travelling to a detent, showing only
            # where it is now reads as if the knob did nothing. Show both
            # until it arrives.
            if (target is not None
                    and abs(target - self.servo.ratio) >= 0.05):
                zoom = f"zoom {self.servo.ratio:.1f}x>{target:.1f}x"
            else:
                zoom = f"zoom {self.servo.ratio:.1f}x"
            # A knob that has been overridden looks broken to whoever is
            # holding it, so the override says so on the status line.
            if self.servo.source != "rc":
                zoom += " BY HAND"
            parts.append(zoom)

        if self.icr_mode is not None:
            parts.append(ICR_SHORT[self.icr_mode])

        if self.snapshots:
            parts.append(f"{self.snapshots} snap")

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
        # Leave no stale state behind: a reader that finds this file after
        # the recorder is gone would otherwise show the last frame of the
        # flight as though it were current. Readers also age it out, which
        # is what covers a crash that never reaches this line.
        if self.args.state_file:
            try:
                os.unlink(self.args.state_file)
            except OSError:
                pass
        if self.keys is not None:
            self.keys.restore()
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
    parser.add_argument("--curve", default="ratio",
                        choices=list(zoom_map.CURVES),
                        help="zoom shaping (default: ratio, which gives every "
                             "0.5x detent an equal slice of knob travel; log "
                             "feels more like a camera rocker but crowds 20 "
                             "detents into the top 12%% of the knob)")
    parser.add_argument("--icr", default=None, choices=list(ICR_BY_NAME),
                        help="imaging mode to start in: rgb (daylight "
                             "colour), ir (IR-sensitive mono), rgb+ir (IR "
                             "with colour), auto. Default: leave the camera "
                             "in whatever mode it is already in. Switchable "
                             "while running with keys 1-4.")
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

    parser.add_argument("--zoom-step", type=float,
                        default=zoom_map.DEFAULT_RATIO_STEP,
                        help="magnification step for the zoom knob, in x "
                             "(default: 0.5, giving 1.0x 1.5x 2.0x ... 30x). "
                             "0 restores a continuously variable knob.")

    parser.add_argument("--snap-channel", type=int, default=9,
                        help="RC channel that sends a frame to the terminal "
                             "on each press (default: 9; 0 disables)")
    parser.add_argument("--snap-threshold", type=int, default=1500,
                        help="PWM at or above this counts as pressed")
    parser.add_argument("--snap-reverse", action="store_true",
                        help="treat low PWM as pressed")
    parser.add_argument("--snap-dir", default=None,
                        help="where snapshot JPEGs go "
                             "(default: <record-dir>/snapshots)")
    parser.add_argument("--snap-width", type=int, default=960,
                        help="JPEG width in pixels (default: 960)")
    parser.add_argument("--snap-quality", type=int, default=70,
                        help="JPEG quality 1-100 (default: 70)")
    parser.add_argument("--state-file",
                        default=os.path.join(
                            os.environ.get("XDG_RUNTIME_DIR") or "/tmp",
                            "fcb_gcs_state"),
                        help="where to publish the status line for the GCS "
                             "panes (default: $XDG_RUNTIME_DIR/fcb_gcs_state); "
                             "empty string disables it")
    parser.add_argument("--preview-cols", type=int,
                        default=snapshot.DEFAULT_MAX_COLS,
                        help="widest the terminal preview may be, in "
                             "characters (default: 100). Cost over the link "
                             "grows with the square of this.")
    parser.add_argument("--no-preview", dest="preview", action="store_false",
                        help="save the JPEG but do not draw in the terminal")

    parser.add_argument("--icr-channel", type=int, default=10,
                        help="RC channel selecting the imaging mode: low RGB, "
                             "centre RGB+IR, high IR (default: 10; 0 disables)")
    parser.add_argument("--icr-channel-reverse", action="store_true",
                        help="reverse the switch, so low selects IR")

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
