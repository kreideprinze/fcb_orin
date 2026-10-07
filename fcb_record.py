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
import glob
import logging
import logging.handlers
import os
import re
import select
import shutil
import signal
import subprocess
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

import prefer_cv2  # noqa: F401  -- before cv2, see prefer_cv2.py

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
    "filled",             # 1: a repeat of the previous frame, written to
                          # cover frames the camera link lost, keeping the
                          # video in real time; 0: a captured frame
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
        except OSError as exc:
            # A closed descriptor reports readable and then fails, for ever.
            self._retire(f"the terminal went away ({exc})")
            return None
        if not data:
            # EOF. The tmux pane was closed, or the window it lived in.
            # select() now calls this fd readable permanently and every
            # read returns nothing, so a caller draining on pending()
            # spins at full tilt -- measured at over a million iterations
            # in three seconds -- and the capture loop never turns again.
            # That wedge ends a recording as surely as a crash, which is
            # why the keyboard is retired here and the flight goes on
            # without it.
            self._retire("the terminal closed")
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

    def _retire(self, why):
        """Give up on the keyboard for good, once, and say so."""
        if not self.enabled:
            return
        self.enabled = False
        log.warning("%s -- keyboard control is gone for this run, but "
                    "RECORDING CONTINUES and the RC switches still work. "
                    "Reattach with ./fly.sh --attach; restart the recorder "
                    "to get the keys back.", why)

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
            self._write("\r\033[K" + text[:width - 1])
            self._shown = True

    def clear(self):
        if not self.enabled or not self._shown:
            return
        with self.lock:
            self._write("\r\033[K")
            self._shown = False

    def _write(self, text):
        """Write, or give up on the terminal for good if it has gone.

        A closed pane leaves this an EIO on every write. Raised from here it
        came out of the logging call that was reporting the closed terminal,
        and out of whatever was doing the logging -- a status line must never
        be able to take the recorder down with it.
        """
        try:
            self.stream.write(text)
            self.stream.flush()
        except (OSError, ValueError):
            self.enabled = False


LIVE = LiveLine()

#: Seconds in which the second 'x' must land to confirm a stop.
STOP_CONFIRM_WINDOW = 5.0


class LiveAwareHandler(logging.StreamHandler):
    """Console handler that steps around the live status line."""

    def emit(self, record):
        with LIVE.lock:
            LIVE.clear()
            try:
                super().emit(record)
            except Exception:
                pass  # the console is gone; the log file still has it


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


#: Container defaults. AVI is the default because of how the two behave
#: when a write is cut short -- a power cut, a kill -9, a card pulled. An
#: mp4 keeps its index (the moov atom) at the end, so a truncated one is
#: not a short video, it is no video at all. AVI's frames are recoverable
#: without the index. Measured on the drone by truncating a 60-frame clip
#: to 75%: 43 frames came back out of the AVI and 0 out of the mp4.
CONTAINERS = {
    #             GStreamer muxer, OpenCV fourcc for the CPU fallback
    "avi": ("avimux", "XVID"),
    "mp4": ("qtmux", "mp4v"),
}


def _to_bgr(frame, yuv_code=cv2.COLOR_YUV2BGR_YUY2):
    """Whatever the camera handed over, as three-channel BGR.

    The FCB offers only 4:2:2 YUV, and whether OpenCV converts it depends on
    the backend: with CAP_PROP_CONVERT_RGB off it passes the raw pairs
    through as a *two*-channel image. Measured that way on a laptop against
    this same board. The encoder would accept those frames and write
    nonsense, which is the worst kind of failure here -- a full card and no
    usable video. Cheap to check, so it is checked. `yuv_code` says which
    packing those pairs are: YUYV on the NeoHD board, UYVY on the Oppila one.
    """
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    channels = frame.shape[2]
    if channels == 3:
        return frame
    if channels == 2:
        return cv2.cvtColor(frame, yuv_code)
    if channels == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def _unique_base(base):
    """A base path no existing recording is already using.

    The timestamp only resolves to the second, so a stop and an immediate
    restart -- a twitchy ch8, or the stop key followed by the switch --
    produces the same name twice, and the second session would overwrite
    the first without a word. Letters for the suffix, because digits are
    what the video segments of one session use.
    """
    if not glob.glob(glob.escape(base) + ".*"):
        return base
    for letter in "bcdefghijklmnopqrstuvwxyz":
        candidate = f"{base}{letter}"
        if not glob.glob(glob.escape(candidate) + ".*"):
            return candidate
    return f"{base}-{int(time.time() * 1000) % 100000}"


class VideoRecorder:
    """Writes frames to a video file, on the GPU where one is available.

    Software encoding 1080p60 is roughly 250 MB/s of raw input to chew
    through, which an Orin's CPU will not keep up with while also running
    everything else. nvv4l2h264enc hands that to the hardware encoder. The
    CPU path stays available for machines without GStreamer, and is fallen
    back to loudly rather than silently, since the difference shows up as
    dropped frames rather than an error.
    """

    def __init__(self, path, width, height, fps, encoder="nvenc", bitrate_mbps=25):
        self.path = path
        extension = os.path.splitext(path)[1].lstrip(".").lower()
        self.muxer, self.fourcc = CONTAINERS.get(extension, CONTAINERS["avi"])
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
                "CPU %s. Expect dropped frames at 1080p60. Check that this "
                "OpenCV was built with GStreamer "
                "(cv2.getBuildInformation()).", self.fourcc
            )
        # Whole frames per second here too, for the same reason as NVENC: a
        # measured rate like 96.67 reaches FFmpeg's mpeg4 encoder as a time
        # base it rejects ("Could not open codec mpeg4", -22) -- and since
        # the measurement differs run to run, the CPU fallback opened or
        # failed at random, leaving recordings with no video.
        writer = cv2.VideoWriter(
            self.path, cv2.VideoWriter_fourcc(*self.fourcc),
            float(max(1, int(round(self.fps)))), (self.width, self.height),
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
            f"! h264parse ! {self.muxer} ! filesink location={self.path}"
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

    def write(self, frame_number, wall_utc, t_mono, telemetry, servo, now,
              filled=0):
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
            filled if frame_number is not None else None,
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


#: Seconds between looks for a control port that did not answer at launch.
CONTROL_RETRY = 10.0


class Recorder:

    #: How raw two-channel frames are decoded; replaced by open_camera()
    #: with whatever packing the camera actually negotiated.
    yuv_code = cv2.COLOR_YUV2BGR_YUY2
    #: Whether the control port was found on a known USB camera board.
    control_on_board = False
    control_baud = None
    # Defaults for state set in __init__, so a Recorder built without it
    # (the tests do) still has every field the loop reads.
    _reopens = 0
    _blank_checked = 0.0
    picture_blank = False
    _switch_raw = None
    _switch_since = 0.0
    _switch_settled = None
    _disk_checked = 0.0
    disk_free_gb = None
    _write_failed = False
    _last_rc_warning = 0.0
    _segment_failed_at = -1e9
    repeating_board = False
    _last_written = None
    #: Most frames filled in one go. A longer hole is a camera outage, which
    #: the reopen path handles -- not something to paper over with a still.
    MAX_FILL_PER_GAP = 3
    _last_written_at = None
    _board_cycled_at = -1e9
    _board_advice_given = False
    _blank_since = None
    #: The current recording was started with r or --autostart, not ch8.
    #: A switch merely sitting low -- or no flight controller at all --
    #: does not end it; x does, or ch8 flicked up and back down.
    _by_hand = False

    def __init__(self, args):
        self.args = args
        self.link = None
        self.control_port = None
        self.declared_fps = None
        self.servo = None
        self.mavlink = None
        self.rc = None
        self.grabber = None

        # A recording session and a video file are no longer the same
        # thing. `recording` is the flight: it owns the CSV and lasts from
        # the ch8 flick to a deliberate stop. `video` is one mp4 inside it,
        # and can come and go if the camera does -- an mp4 cannot span a
        # gap in its own frame stream, but the flight track can and must.
        self.recording = False
        self.segment = 0
        self.video = None
        self.video_path = None
        self.csv = None
        self.base = None
        self.record_started_mono = None
        self.record_started_wall = None
        self._last_gap_row = 0.0
        self._stop_armed = 0.0
        # Set when a recording is ended by hand while ch8 is still high.
        # Without it the very next loop iteration sees "switch high, not
        # recording" and starts a new one, so the stop key would be
        # useless -- it would just chop the flight into two files.
        # Cleared when the switch is next seen low, so the transmitter
        # takes over again as soon as the pilot actually uses it.
        self._start_suppressed = False

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
        # Characters typed at the "name this flight" prompt, or None. The
        # base path being named is held alongside, because a new recording
        # could in principle start while the prompt is still open.
        self.name_entry = None
        self._name_base = None

        self.frames_seen = 0
        self.frames_dropped = 0
        # Reopens since the last frame, for backing off (see capture_loop).
        self._reopens = 0
        # Blank-picture watch: when last checked, and whether it is blank.
        self._blank_checked = 0.0
        self.picture_blank = False
        # ch8 debounce: the raw reading, since when, and the settled one.
        self._switch_raw = None
        self._switch_since = 0.0
        self._switch_settled = None
        # Free-space watch.
        self._disk_checked = 0.0
        self.disk_free_gb = None
        self._write_failed = False
        self._last_status = 0.0
        self._last_live = 0.0
        self._status_broken = False
        self.running = True

    # -- setup -----------------------------------------------------------

    def open_control(self):
        self.link = self.make_visca_link()
        if self.link is None:
            log.error("no VISCA port answered -- zoom and imaging mode are "
                      "unavailable until it does; still looking every %gs",
                      CONTROL_RETRY)
            threading.Thread(target=self._find_control_later, daemon=True,
                             name="visca-retry").start()

    def _find_control_later(self):
        """Keep looking for a control port that did not answer at startup.

        Without this a camera that was slow to answer at launch -- a board
        just powered up, or one holding replies from before -- left the whole
        run with no zoom and no imaging mode. Once found, the servo is
        started exactly as it would have been at launch.
        """
        attempts = 0
        while self.running and self.link is None:
            time.sleep(CONTROL_RETRY)
            attempts += 1
            link = self.make_visca_link()
            if link is not None:
                self.link = link
                log.warning("VISCA: camera control found after %d retries", attempts)
                self.start_servo()
                return
            if attempts % 6 == 0:
                log.warning("VISCA: still no camera control (%d retries); if "
                            "the camera board was unplugged or reset it may "
                            "need its 12 V power cycled", attempts)

    def make_visca_link(self):
        """Find and open the camera's control port. None if it is not there.

        Doubles as the servo's reconnect factory, so a link lost in flight
        is rebuilt by rediscovering the port rather than assuming it came
        back with the same name -- a re-enumerated USB device usually does
        not.
        """
        ports = [self.args.port] if self.args.port else None
        bauds = [self.args.baud] if self.args.baud else None
        if ports is None and self.control_on_board:
            # Coming back to a board found by USB ID: only its own ports.
            # The full sweep would also probe the autopilot's -- every 2 s
            # for as long as the camera is unplugged, writing VISCA into the
            # MAVLink stream and reading telemetry out from under it.
            ports = devices.camera_serial_ports()
            if not ports:
                return None
            # And the baud it answered at: it is a camera register, not
            # something a replug changes, and against a board that has
            # stopped answering, every extra rate is another slow open.
            if bauds is None and self.control_baud:
                bauds = [self.control_baud]
        port, baud = devices.autodetect_visca(ports, bauds)
        if port is None:
            return None
        link = None
        try:
            link = ViscaLink(port, baud, self.args.address)
            link.if_clear()
        except Exception as exc:
            log.warning("VISCA: %s at %d baud would not open: %s",
                        port, baud, exc)
            # Closed here, or its exclusive lock outlives it and every later
            # reconnect finds the port "locked by another process".
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass
            return None
        self.control_port = port
        self.control_on_board = (
            devices.usb_id(port) in devices.CAMERA_BOARD_USB_IDS)
        self.control_baud = baud
        log.info("VISCA: %s at %d baud", port, baud)
        self.apply_stabilizer(link)
        return link

    def apply_stabilizer(self, link):
        """Put the camera's image stabilizer where --stabilizer asks.

        Done for every new link -- startup, the retry after a camera that
        did not answer at launch, and the servo's reconnects -- because the
        setting lives in the camera and does not survive it losing power:
        after the 12 V power cycles the Oppila board needs, the camera read
        back "off". The Twiga NeoHD driver (neohd-ptz-ros) switched it on at
        startup; this recorder never did, so the feed went unstabilised.

        Read back afterwards rather than trusting the acknowledgement, and
        never fatal: a recording without stabilisation still beats none.
        """
        wanted = getattr(self.args, "stabilizer", "on")
        try:
            if wanted != "keep":
                link.command(visca.image_stabilizer(wanted == "on", link.address))
            state = visca.parse_stabilizer(
                link.inquiry(visca.stabilizer_inq(link.address)))
        except Exception as exc:
            log.warning("image stabilizer: could not %s: %s",
                        "read it" if wanted == "keep" else "turn it %s" % wanted,
                        exc)
            return
        if wanted == "keep":
            log.info("image stabilizer: %s (as found -- --stabilizer keep)", state)
        elif state == wanted:
            log.info("image stabilizer: %s (confirmed by the camera)", state)
        else:
            log.warning("image stabilizer: asked for %s, camera reports %s",
                        wanted, state)

    def open_mavlink(self):
        if getattr(self.args, "no_mavlink", False):
            log.info("--no-mavlink: not looking for a flight controller -- "
                     "r starts a recording, x x stops it")
            return
        url, baud = self.args.rc_url, self.args.rc_baud
        if url is None:
            log.info("probing for the flight controller...")
            url, baud = devices.autodetect_mavlink(exclude=self.control_port)
            if url is None:
                log.critical("=" * 68)
                log.critical("NO FLIGHT CONTROLLER FOUND -- ch%d CANNOT START "
                             "A RECORDING.", self.args.rec_channel)
                log.critical("Recording is armed by RC channel %d, which "
                             "arrives over MAVLink. Without",
                             self.args.rec_channel)
                log.critical("it, press r in the pane to record (x x stops), "
                             "or run with --autostart.")
                log.critical("Zoom, imaging mode and snapshots still work. "
                             "--no-mavlink skips all this.")
                log.critical("Fix: check the autopilot's USB lead and that "
                             "it is powered, or pass")
                log.critical("--rc-url to pin the port. Pass "
                             "--require-mavlink to make this fatal.")
                log.critical("Still looking every %gs -- plug it in and it "
                             "is picked up without a restart.", CONTROL_RETRY)
                log.critical("=" * 68)
                threading.Thread(target=self._find_mavlink_later, daemon=True,
                                 name="mavlink-retry").start()
                return
        self._attach_mavlink(url, baud)

    def _find_mavlink_later(self):
        """Keep looking for an autopilot that was not there at startup.

        Without this an autopilot plugged in -- or powered -- after the
        recorder started meant ch8 did nothing for the rest of the run,
        however long it went, until someone restarted it.
        """
        while self.running and self.mavlink is None:
            time.sleep(CONTROL_RETRY)
            url, baud = devices.autodetect_mavlink(exclude=self.control_port)
            if url is not None:
                self._attach_mavlink(url, baud)
                log.warning("MAVLink: flight controller found -- ch%d now "
                            "arms recording", self.args.rec_channel)
                return

    def _attach_mavlink(self, url, baud):
        """Bring up MAVLink and everything read through it.

        The switches are built before self.mavlink is set, because the
        capture loop takes self.mavlink being there to mean all of it is.
        """
        mavlink = MavlinkSource(
            url=url, baud=baud, stream_rate_hz=self.args.stream_rate,
            on_status=lambda msg: log.info("MAVLink: %s", msg),
        )
        log.info("MAVLink: %s at %d baud, zoom on ch%d, record on ch%d",
                 url, baud, self.args.rc_channel, self.args.rec_channel)
        rc = RcZoomSource(
            channel=self.args.rc_channel,
            pwm_min=self.args.rc_pwm_min, pwm_max=self.args.rc_pwm_max,
            pwm_deadband=self.args.rc_deadband, reverse=self.args.rc_reverse,
            rc_timeout=self.args.rc_timeout,
            source=mavlink,
        )

        if self.args.snap_channel:
            self.snap_button = RcButton(
                mavlink, self.args.snap_channel,
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
                mavlink, self.args.icr_channel,
                ["day", "night_color", "night"],
                pwm_min=self.args.rc_pwm_min, pwm_max=self.args.rc_pwm_max,
                reverse=self.args.icr_channel_reverse,
                rc_timeout=self.args.rc_timeout,
            )
            log.info("imaging mode: ch%d, low RGB / centre RGB+IR / high IR",
                     self.args.icr_channel)
        self.rc = rc
        if self.servo is not None and self.servo.rc is None:
            self.servo.rc = rc      # the zoom knob, for an autopilot found late
        self.mavlink = mavlink

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
        # YUYV and UYVY arrive as the same two-channel shape when OpenCV
        # does not convert, so the negotiated format is what tells them apart.
        pixel_format = devices.fourcc_of(capture)
        self.yuv_code = devices.yuv422_to_bgr_code(pixel_format)
        # What the camera says it runs at. Preferred over measuring, which
        # needs a second or two of frames to mean anything and is wrong if
        # recording starts before then.
        declared = capture.get(cv2.CAP_PROP_FPS)
        self.declared_fps = declared if 1.0 < declared < 1000.0 else None
        repeats = devices.usb_id(device) in devices.REPEATING_FRAME_USB_IDS
        log.info("video: %s at %dx%d %s, %s", device, width, height,
                 pixel_format or "format not reported",
                 f"{self.declared_fps:.2f} fps" if self.declared_fps
                 else "rate not reported")
        self.repeating_board = repeats
        if repeats:
            # Its figure is not the camera's (devices.REPEATING_FRAME_USB_IDS),
            # and tagging a recording with it would play it back at the wrong
            # speed. The camera's own nominal rate (--fps) is used instead.
            self.declared_fps = None
            log.info("video: this board repeats frames and misreports its "
                     "rate -- dropping the repeats and measuring the rate")
        # Ask the backend to hand over BGR; _to_bgr covers it declining.
        try:
            capture.set(cv2.CAP_PROP_CONVERT_RGB, 1.0)
        except Exception:
            pass
        self.grabber = FrameGrabber(capture, self.yuv_code,
                                    drop_repeats=repeats)
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

        # An mp4 cannot span a gap in its own frame stream, so this one is
        # finalised now. The *flight* is not over: the CSV stays open, the
        # telemetry track keeps being written, and frames returning open a
        # new numbered segment against the same session.
        if self.video is not None:
            log.warning("video: closing segment %d at the break -- telemetry "
                        "keeps recording", self.segment)
            self.close_segment()

        if self.open_camera(quiet=True):
            # Opened is not recovered: a wedged board opens fine and then
            # sends nothing. "frames flowing again" is the real recovery.
            log.info("video: camera reopened -- waiting for frames")
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
        except Exception as exc:
            # The link itself is gone (a closed or vanished port) -- the
            # servo is already reconnecting, so this is said in one line
            # rather than as a traceback per flick of the switch.
            log.error("imaging mode: %s not set -- no camera control right "
                      "now (%s); it reconnects by itself",
                      ICR_LABELS.get(mode, mode), type(exc).__name__)
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

    def request_start_recording(self, why="r in the pane"):
        """Start a recording without the RC switch -- no Pixhawk needed.

        For bench runs and for flying with no flight controller wired in,
        where ch8 does not exist. The recording holds until x x, or until
        ch8 (if there is one) is flicked up and back down; a switch that is
        merely sitting low does not end it. Video joins on the next frame,
        so this works with the camera still coming up.
        """
        if self.recording:
            log.info("already recording")
            return
        if self._write_failed:
            log.error("not starting: the last write failed (full disk?) -- "
                      "free space and restart the recorder")
            return
        self._by_hand = True
        self.start_recording(None)
        log.info("recording started by %s -- stop it with r r (or x x)", why)

    def request_stop_recording(self, key="x"):
        """Stop the recording -- the only way a flight's footage ends.

        Two presses, not one. This is the single irreversible key on a
        terminal that anyone might lean on, and the whole point of moving
        the stop off ch8 was that a recording should not end by accident.
        """
        if not self.recording:
            log.info("not recording -- nothing to stop")
            return
        now = time.monotonic()
        if now - self._stop_armed > STOP_CONFIRM_WINDOW:
            self._stop_armed = now
            log.warning("press %s again within %.0fs to STOP the recording",
                        key, STOP_CONFIRM_WINDOW)
            return
        self._stop_armed = 0.0
        base = self.base
        # Suppress BEFORE stopping, not after. ch8 is almost certainly
        # still high -- that is how the recording started -- and any gap
        # between "no longer recording" and "not allowed to start" is a
        # gap in which a new session begins and the stop key has merely
        # chopped the flight in two.
        still_high = self.wants_recording()
        if still_high:
            self._start_suppressed = True
        self.stop_recording(why=f"{key} in the pane")
        if still_high:
            log.info("ch%d is still high; flick it low and back to start "
                     "another recording", self.args.rec_channel)
        # Ask now, while the pilot is still thinking about the flight they
        # just made. A timestamp is a fine filename and a poor label, and
        # ten minutes later nobody remembers which one was the good pass.
        self.begin_name_entry(base)

    # -- naming a finished flight ----------------------------------------

    def begin_name_entry(self, base):
        """Offer to rename the files a finished recording just produced."""
        if base is None or not self.recorded_files(base):
            return
        if self.keys is None or not self.keys.enabled:
            # No terminal to ask on -- a detached or piped run. The
            # timestamp name stands, which is why it is the default.
            return
        self._name_base = base
        self.name_entry = ""
        self.show_name_entry()

    def recorded_files(self, base):
        """Every file that belongs to one recording session.

        The video may be several numbered segments, so this globs rather
        than assuming ".mp4" -- renaming only the first segment would
        scatter one flight across two names.
        """
        stem = os.path.basename(base)
        directory = os.path.dirname(base)
        try:
            names = os.listdir(directory)
        except OSError:
            return []
        out = []
        for name in sorted(names):
            if not name.startswith(stem):
                continue
            rest = name[len(stem):]
            # "" + .mp4/.csv, or "-2" + .mp4 -- not "-something-else".
            if re.fullmatch(r"(-\d+)?\.(%s|csv)" % "|".join(CONTAINERS), rest):
                out.append(os.path.join(directory, name))
        return out

    def show_name_entry(self):
        LIVE.show(f"name this flight> {self.name_entry}"
                  f"   [Enter keeps {os.path.basename(self._name_base or '')}]")

    def handle_name_entry(self, key):
        if key in ("\r", "\n"):
            text, self.name_entry = self.name_entry, None
            base, self._name_base = self._name_base, None
            if not text.strip():
                log.info("kept the timestamp name: %s",
                         os.path.basename(base or "?"))
                return
            self.rename_recording(base, text)
            return
        if key in ("\x7f", "\b"):
            self.name_entry = self.name_entry[:-1]
        elif key.isprintable() and len(self.name_entry) < 60:
            self.name_entry += key
        self.show_name_entry()

    @staticmethod
    def safe_name(text):
        """A filename from whatever was typed, or None if nothing survives.

        Takes the basename first, so a slash cannot walk the rename out of
        the recordings directory, and keeps only characters that are
        painless in a shell and on any filesystem.
        """
        text = os.path.basename(text.strip()).strip(". ")
        text = re.sub(r"\s+", "_", text)
        text = re.sub(r"[^A-Za-z0-9._-]", "", text)
        return text or None

    def rename_recording(self, base, text):
        """Rename every file of one session, or none of them."""
        name = self.safe_name(text)
        if name is None:
            log.warning("that name has no usable characters -- keeping %s",
                        os.path.basename(base))
            return
        files = self.recorded_files(base)
        if not files:
            log.warning("nothing left to rename for %s",
                        os.path.basename(base))
            return

        directory = os.path.dirname(base)
        stem = os.path.basename(base)
        # Never overwrite an existing recording, and never half-rename a
        # session: a suffix is found that clears every file at once.
        suffix, target = "", name
        while True:
            planned = [(f, os.path.join(
                directory, target + os.path.basename(f)[len(stem):])) for f in files]
            if not any(os.path.exists(dst) for _, dst in planned):
                break
            suffix = f"-{int(suffix[1:] or 1) + 1}" if suffix else "-2"
            target = name + suffix
        if suffix:
            log.warning("%s was taken -- using %s", name, target)

        for src, dst in planned:
            try:
                os.rename(src, dst)
            except OSError as exc:
                log.error("could not rename %s: %s -- the rest of this "
                          "flight keeps its old name",
                          os.path.basename(src), exc)
                return
        log.info("RENAMED  %s -> %s  (%d file(s))", stem, target, len(planned))
        for _, dst in planned:
            log.info("         %s", os.path.basename(dst))

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
        if self.name_entry is not None:
            self.handle_name_entry(key)
            return

        if key in ICR_BY_KEY:
            self.set_icr(ICR_BY_KEY[key])
        elif key in ("i", "I"):
            self.cycle_icr()
        elif key in ("r", "R"):
            # A toggle, so one key does both from the laptop. Stopping still
            # takes two presses, exactly like x.
            if self.recording:
                self.request_stop_recording(key="r")
            else:
                self.request_start_recording()
        elif key in ("x", "X"):
            self.request_stop_recording()
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
        """Begin a flight: open the CSV, then the first video segment."""
        os.makedirs(self.args.record_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.base = _unique_base(
            os.path.join(self.args.record_dir, f"fcb-{stamp}"))

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
        self.recording = True
        self.segment = 0
        self._last_gap_row = 0.0
        log.info("RECORDING STARTED  %s.csv", os.path.basename(self.base))
        self.open_segment(frame)

    def open_segment(self, frame):
        """Open one mp4 inside the current session.

        Called again after a camera outage, so a flight that loses video
        for a while comes back as a second numbered file rather than
        ending. Returns True if there is a video to write to.
        """
        if frame is None or self.grabber is None:
            return False
        height, width = frame.shape[:2]
        # The container's frame rate decides playback speed, so it has to be
        # right from the first frame. The camera's own figure is used where
        # it has one; a measured rate is only trusted once enough frames
        # have arrived for it to be meaningful.
        measured = self.grabber.fps()
        if self.declared_fps:
            fps, source = self.declared_fps, "camera"
        elif self.repeating_board:
            # Not measured: a measurement taken while the link is losing
            # frames tagged one recording 43 fps for a 60 fps camera, so it
            # played back at the wrong speed. The camera runs at its nominal
            # rate; frames lost on the way are filled in (see write_frame).
            fps, source = self.args.fps, "camera nominal"
        elif measured >= 1.0:
            fps, source = measured, "measured"
        else:
            fps, source = self.args.fps, "--fps default"

        # This is retried on every frame while there is no video, so it is
        # rate-limited, and the segment number only advances on success --
        # it used to advance per attempt, numbering the file -57 after a
        # second of failures and logging an error per frame.
        now = time.monotonic()
        if now - self._segment_failed_at < 5.0:
            return False
        number = self.segment + 1
        suffix = "" if number == 1 else f"-{number}"
        path = f"{self.base}{suffix}.{self.args.container}"
        try:
            self.video = VideoRecorder(
                path, width, height, fps,
                encoder=self.args.encoder, bitrate_mbps=self.args.bitrate,
            )
        except RuntimeError as exc:
            # The flight is not over: telemetry keeps going without video.
            self._segment_failed_at = now
            log.error("could not open video segment %d (%s) -- telemetry "
                      "keeps recording; retrying every 5s", number, exc)
            self.video = None
            return False
        self.segment = number
        self._last_written = None
        self._last_written_at = None

        self.video_path = path
        log.info("  video segment %d: %s (%dx%d @ %.2f fps from %s, "
                 "%s encoder)", self.segment, os.path.basename(path),
                 width, height, fps, source, self.video.encoder)
        return True

    def close_segment(self):
        """Finalise the current mp4, leaving the session and CSV open."""
        if self.video is None:
            return
        frames = self.video.frames
        encoder = self.video.encoder
        path = self.video_path
        elapsed = self.video.close()
        self.video = None
        try:
            size_bytes = os.path.getsize(path)
        except OSError:
            size_bytes = 0
        log.info("  video segment %d closed: %s -- %d frames, %.1fs, "
                 "%.1f MB (%.1f fps average)", self.segment,
                 os.path.basename(path), frames, elapsed, size_bytes / 1e6,
                 frames / elapsed if elapsed > 0 else 0.0)

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

    def stop_recording(self, why="stopped"):
        """End the flight: close the last video segment and the CSV.

        Called for ch8 going low (once it has held there for --rec-debounce,
        see settled_switch), for x in the pane, on a write failure, and at
        shutdown. A range dropout does not stop it: stale RC reads as "cannot
        tell", which holds whatever is happening.
        """
        if not self.recording:
            return
        # Marked stopped first, so nothing writes to half-closed files, and
        # each close is attempted whatever happened to the other -- a full
        # disk can make the CSV's final flush fail too.
        self.recording = False
        self._by_hand = False
        try:
            self.close_segment()
        except Exception:
            log.exception("could not close the video cleanly")
            self.video = None
        rows = self.csv.rows if self.csv is not None else 0
        if self.csv is not None:
            try:
                self.csv.close()
            except Exception as exc:
                log.error("could not close the CSV cleanly: %s", exc)
        self.csv = None
        elapsed = (time.monotonic() - self.record_started_mono
                   if self.record_started_mono else 0.0)
        log.info("RECORDING STOPPED (%s)  %s -- %.1fs, %d video segment(s), "
                 "%d telemetry rows", why, os.path.basename(self.base or "?"),
                 elapsed, self.segment, rows)

    def write_gap_row(self, now):
        """A telemetry row with no frame behind it.

        Video and telemetry are two records of the same flight and must not
        take each other down. The camera dropping out used to end the CSV
        as well, losing the position track for the rest of the sortie --
        the half of the data that is hardest to fly again. These rows carry
        an empty frame number, which is how you tell them apart afterwards.
        """
        if self.csv is None or self.record_started_mono is None:
            return
        if now - self._last_gap_row < self.args.telemetry_interval:
            return
        self._last_gap_row = now
        t_mono = now - self.record_started_mono
        try:
            self.csv.write(
                None, self.record_started_wall + _delta(t_mono), t_mono,
                self.mavlink.telemetry() if self.mavlink else _NO_TELEMETRY,
                self.servo if self.servo else _NO_ZOOM, now,
            )
        except Exception:
            log.exception("telemetry row failed; continuing")

    def wants_recording(self):
        """Whether the RC switch is asking for recording right now.

        None means "cannot tell", and no flight controller is exactly that
        -- not a considered "no". The difference matters: returning False
        here made the capture loop treat an unanswerable question as a
        settled answer, so it recorded nothing and never said why. A whole
        flight was lost to that.
        """
        if self.mavlink is None:
            return None
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
            log.info("keys: %s  i cycle  s snap  z zoom  a knob back  "
                     "r REC/STOP (r r)  x STOP+NAME (twice)  ? status  (Ctrl-C quits)",
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
            # Bounded. Draining is meant to clear a burst of typing, and no
            # human produces more than a handful between frames; an
            # unbounded loop here is one broken descriptor away from
            # holding the capture loop for ever.
            for _ in range(64):
                if not self.keys.pending():
                    break
                key = self.keys.get()
                if key is not None:
                    self.handle_key(key)
        except Exception:
            log.exception("keyboard handling failed; continuing")

    def capture_loop(self):
        last_seq = 0
        self._last_rc_warning = 0.0
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
                # rest of the flight records nothing. Backing off, though:
                # 5 s, 10, 20, 40, then every 60 s. A reopen cannot revive a
                # board that has stopped streaming (the Oppila one needs its
                # 12 V cycled), and the open/close churn of retrying every
                # 5 s for ever is itself what wedges it.
                wait = min(60.0, self.args.camera_timeout * (2 ** min(self._reopens, 4)))
                if now - last_frame_at > wait:
                    self._reopens += 1
                    if self.reopen_camera():
                        last_seq = 0
                    if self._reopens >= 2:
                        self.board_is_stuck("no frames after %d reopens" % self._reopens)
                    if self._reopens >= 3:
                        log.warning("video: %d reopens without a frame -- next "
                                    "try in %.0fs. If the camera board was "
                                    "unplugged or reset, cycle its 12 V supply",
                                    self._reopens, min(60.0, self.args.camera_timeout
                                                       * (2 ** min(self._reopens, 4))))
                    last_frame_at = time.monotonic()
                # No video does not mean no flight: the switch is still
                # obeyed, and the position and attitude track keeps going.
                self.apply_record_switch(now, None, None)
                if self.recording:
                    self.write_gap_row(now)
                self.check_disk(now)
                self.update_display(now)
                continue
            last_frame_at = now
            self._reopens = 0
            frame = _to_bgr(frame, self.yuv_code)
            if waiting_logged:
                log.info("frames flowing again")
                waiting_logged = False
            self.check_blank(frame, now)
            self.check_disk(now)

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

            self.apply_record_switch(now, frame, captured_at)

            if self.recording:
                # Video coming back mid-flight rejoins the same session as
                # a new numbered segment, against the same CSV.
                if self.video is None:
                    self.open_segment(frame)
                # Read both once. A stop that lands between the check and
                # the write would otherwise dereference a closed CSV --
                # unreachable while keys are handled inside this loop, but
                # this is the only copy of a flight and the guard is free.
                video, sheet = self.video, self.csv
                if video is not None and sheet is not None:
                    try:
                        self.write_frame(video, sheet, frame, captured_at, now,
                                         seq_gap=max(1, dropped + 1))
                    except Exception as exc:
                        self.on_write_failure(exc)
                else:
                    self.write_gap_row(now)

            self.update_display(now)

    def write_frame(self, video, sheet, frame, captured_at, now, seq_gap=1):
        """Write one captured frame, after filling a gap the camera left.

        A frame the camera link lost leaves a hole in the timeline, and the
        container plays at a fixed rate, so without a fill the video runs
        short and plays fast. Only those holes are filled -- seq_gap == 1,
        the grabber had nothing in between -- each judged on its own against
        the frame before it, and never more than MAX_FILL_PER_GAP at once.

        The first version kept a running total against the segment's start
        and filled whatever it said was missing, including frames this loop
        had merely been too slow to collect. Every fill costs an encode, the
        cost made the next frame late, and it spiralled: 291 real frames and
        1,795 copies in a 35 s recording. Frames skipped here are counted in
        frames_dropped instead, as before.
        """
        telemetry = self.mavlink.telemetry() if self.mavlink else _NO_TELEMETRY
        servo = self.servo if self.servo else _NO_ZOOM
        prev, prev_t = self._last_written, self._last_written_at
        if prev is not None and seq_gap == 1 and prev_t is not None:
            gap = captured_at - prev_t
            # Over 1.75 frame periods: the board's own arrival jitter (frames
            # 10-27 ms apart around a 16.7 ms mean) never reaches that.
            if gap > 1.75 / video.fps:
                missing = min(int(round(gap * video.fps)) - 1,
                              self.MAX_FILL_PER_GAP)
                for i in range(1, missing + 1):
                    t = prev_t + i / video.fps
                    sheet.write(video.frames,
                                self.record_started_wall + _delta(t - self.record_started_mono),
                                t - self.record_started_mono, telemetry, servo, now,
                                filled=1)
                    video.write(prev)
        sheet.write(video.frames,
                    self.record_started_wall + _delta(captured_at - self.record_started_mono),
                    captured_at - self.record_started_mono, telemetry, servo, now)
        video.write(frame)
        self._last_written, self._last_written_at = frame, captured_at

    def settled_switch(self, now):
        """wants_recording(), acted on only once it has held still.

        A single stray reading used to be enough: one low sample ended a
        recording and the next high one began another, which is where the
        one-frame recordings came from. The reading has to hold for
        --rec-debounce seconds before it counts; until then the last settled
        answer stands. "Cannot tell" (None) is passed straight through, as
        it always meant "hold".
        """
        raw = self.wants_recording()
        if raw is None:
            self._switch_raw = None
            return None
        if raw != self._switch_raw:
            self._switch_raw, self._switch_since = raw, now
        if now - self._switch_since >= getattr(self.args, "rec_debounce", 0.3):
            self._switch_settled = raw
        return self._switch_settled

    def apply_record_switch(self, now, frame, captured_at):
        """Start or stop on ch8, with or without a video frame in hand.

        Done on every turn of the loop, not only when a frame arrives: with
        the camera out, flipping ch8 used to do nothing at all -- no
        telemetry-only recording on the way up, no stop on the way down.
        """
        wanted = self.settled_switch(now)
        if wanted is None:
            if now - self._last_rc_warning > 5.0:
                if self.mavlink is None:
                    # Nothing to say when it is not wanted, or when a
                    # recording started by hand is running fine without it.
                    if not (getattr(self.args, "no_mavlink", False)
                            or self.recording):
                        log.error("NO FLIGHT CONTROLLER -- channel %d cannot "
                                  "be read, so nothing is being recorded. "
                                  "Press r to record without it. Still "
                                  "looking; check the autopilot's USB lead "
                                  "and power.", self.args.rec_channel)
                elif self._switch_raw is None:
                    log.warning("no RC data -- holding recording state")
                self._last_rc_warning = now
            return
        if not wanted:
            # The switch is genuinely low: whatever was stopped by hand is
            # now stopped by the pilot too, so let it arm again.
            if self._start_suppressed:
                log.info("ch%d is low again -- the switch can start a new "
                         "recording", self.args.rec_channel)
                self._start_suppressed = False
            if self.recording and self._by_hand:
                return  # started without the switch; it has not been used
            if self.recording:
                # Deliberately no naming prompt here. That prompt swallows
                # every keystroke until it is answered, and this path fires
                # mid-flight with nobody necessarily at the keyboard -- it
                # would leave the mode and snapshot keys dead. Name it with
                # 'x', or afterwards.
                self.stop_recording(why=f"ch{self.args.rec_channel} low")
            return
        # The switch is up: from here on it owns the recording, so bringing
        # it back down stops one that was started by hand.
        self._by_hand = False
        if not self.recording and not self._start_suppressed:
            if self._write_failed:
                return  # said once already, in on_write_failure
            self.start_recording(frame, captured_at)
            if frame is None:
                log.warning("recording with NO VIDEO yet -- telemetry is being "
                            "written, and video joins as soon as frames arrive")

    def on_write_failure(self, exc):
        """A frame or row could not be written: say so, and stop cleanly.

        Almost always a full disk. Left to propagate, it unwound the capture
        loop and killed the recorder mid-flight; stopping here at least
        closes the files that were written, so they stay readable. No new
        recording starts until the recorder is restarted -- one that cannot
        be written is not a recording.
        """
        free = self.free_gb()
        log.critical("WRITE FAILED: %s -- %s. Stopping the recording so what "
                     "was written stays readable.", exc,
                     f"{free:.1f} GB free" if free is not None else
                     "free space unknown")
        self._write_failed = True
        try:
            self.stop_recording(why="write failed")
        except Exception:
            log.exception("could not close the recording cleanly")

    def free_gb(self):
        try:
            return shutil.disk_usage(self.args.record_dir).free / 1e9
        except (OSError, AttributeError):
            return None

    def check_disk(self, now):
        """Warn as the recordings disk fills, every 30 s while it is low."""
        if now - self._disk_checked < 30.0:
            return
        self._disk_checked = now
        self.disk_free_gb = self.free_gb()
        if self.disk_free_gb is None:
            return
        if self.disk_free_gb < 1.0:
            log.critical("DISK NEARLY FULL: %.1f GB left in %s -- about %.0f s "
                         "of video", self.disk_free_gb, self.args.record_dir,
                         self.disk_free_gb * 1e9 / (getattr(self.args, 'bitrate', 25.0) * 1e6 / 8))
        elif self.disk_free_gb < 5.0:
            log.warning("disk low: %.1f GB left in %s (about %.0f min of video)",
                        self.disk_free_gb, self.args.record_dir,
                        self.disk_free_gb * 1e9 / (getattr(self.args, 'bitrate', 25.0) * 1e6 / 8) / 60)

    def board_is_stuck(self, why):
        """The board's video is wedged: power-cycle it if we can, else say so.

        The Oppila board runs on its own 12 V, so it stays up while the Orin
        boots or reboots -- and from its side that is the USB cable being
        pulled, which it does not recover from (Oppila's docs; reproduced 5
        Oct: after an Orin reboot the board never came back on USB at all,
        and after a cold start of the whole drone it streamed 0 fps). Only
        cutting its 12 V clears it. With --board-power-cycle-cmd set (a
        relay or MOSFET on that 12 V line, driven by the Orin) the recorder
        does it itself; at most once every 90 s, so a board that will not
        come back is not cycled for ever.
        """
        cmd = getattr(self.args, "board_power_cycle_cmd", "")
        now = time.monotonic()
        if not cmd:
            if not self._board_advice_given:
                self._board_advice_given = True
                log.error("CAMERA BOARD IS STUCK (%s). Cycle its 12 V supply. It "
                          "does not recover from the Orin booting or rebooting "
                          "while it stays powered; set --board-power-cycle-cmd "
                          "to have the recorder do this itself.", why)
            return
        if now - self._board_cycled_at < 90.0:
            return
        self._board_cycled_at = now
        log.warning("CAMERA BOARD IS STUCK (%s) -- power-cycling it: %s", why, cmd)
        try:
            done = subprocess.run(cmd, shell=True, timeout=60,
                                  capture_output=True, text=True)
            if done.returncode != 0:
                log.error("board power cycle failed (exit %d): %s",
                          done.returncode, (done.stderr or done.stdout).strip()[:200])
        except Exception as exc:
            log.error("board power cycle failed: %s", exc)
        self._reopens = 0
        self._blank_since = None

    def check_blank(self, frame, now):
        """Notice a picture that is one flat colour, every couple of seconds.

        An all-green frame is the board streaming nothing but zeros: the
        board is up and on USB, VISCA still answers (zoom and RGB/IR keep
        working), but no image is reaching it from the camera block. It
        looked like a working feed and recorded green; now it is said.
        A real picture -- even pitch dark or lens-capped -- carries sensor
        noise, so a spread of under one grey level means nothing is there.
        """
        if now - self._blank_checked < 2.0:
            return
        self._blank_checked = now
        try:
            blank = float(frame[::24, ::24].std()) < 1.0
        except Exception:
            return
        if blank:
            if self._blank_since is None:
                self._blank_since = now
            elif now - self._blank_since > 10.0:
                self.board_is_stuck("picture blank for %.0fs" % (now - self._blank_since))
        else:
            self._blank_since = None
        if blank and not self.picture_blank:
            log.error("PICTURE IS BLANK -- every pixel the same colour (all "
                      "green means all-zero frames). The board is streaming "
                      "but receiving no image from the camera: check the "
                      "camera's LVDS cable and power, then cycle the board's "
                      "12 V. Zoom and RGB/IR still respond; that is a "
                      "separate path.")
        elif not blank and self.picture_blank:
            log.info("picture is back")
        self.picture_blank = blank

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
                elif self.name_entry is not None:
                    self.show_name_entry()
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

        if self.recording:
            elapsed = now - self.record_started_mono
            if self.video is not None:
                parts.append(f"REC {os.path.basename(self.base)} "
                             f"(seg {self.segment}, {self.video.frames} fr, "
                             f"{elapsed:.0f}s)")
            else:
                # Still a live recording -- the telemetry track is being
                # written. Saying "not recording" here would read as data
                # loss when there is none.
                parts.append(f"REC TELEMETRY ONLY -- NO VIDEO "
                             f"({elapsed:.0f}s)")
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

        # First, and every quarter second, for as long as it is true. The
        # one startup error scrolls off in seconds; this cannot.
        if self.mavlink is None and not getattr(self.args, "no_mavlink", False):
            parts.insert(0, "NO FC" if self.recording
                         else "NO FC -- r TO RECORD")
        if self.picture_blank and self.grabber is not None:
            parts.insert(0, "BLANK PICTURE")
        if self._write_failed:
            parts.insert(0, "WRITE FAILED -- NOT RECORDING")
        if self.disk_free_gb is not None and self.disk_free_gb < 5.0:
            parts.append(f"DISK {self.disk_free_gb:.1f}GB")

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
    parser.add_argument("--stabilizer", default="on",
                        choices=["on", "off", "keep"],
                        help="the camera's image stabilizer, set every time "
                             "the camera is connected (default on, as the "
                             "Twiga NeoHD driver did). The camera comes back "
                             "with it off after losing power. keep = leave "
                             "it as found.")
    parser.add_argument("--icr", default=None, choices=list(ICR_BY_NAME),
                        help="imaging mode to start in: rgb (daylight "
                             "colour), ir (IR-sensitive mono), rgb+ir (IR "
                             "with colour), auto. Default: leave the camera "
                             "in whatever mode it is already in. Switchable "
                             "while running with keys 1-4.")
    parser.add_argument("--record-dir",
                        default=os.path.expanduser(
                            os.environ.get("FCB_RECORD_DIR", "~/flight_recordings")),
                        help="where recordings, snapshots and logs go "
                             "(default $FCB_RECORD_DIR, else ~/flight_recordings)")
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
    parser.add_argument("--board-power-cycle-cmd",
                        default=os.environ.get("FCB_BOARD_POWER_CYCLE", ""),
                        help="shell command that power-cycles the camera "
                             "board's 12 V (e.g. drives a relay from a GPIO). "
                             "Run when its video is stuck -- it does not "
                             "recover from the Orin booting while it stays "
                             "powered. Default $FCB_BOARD_POWER_CYCLE; empty "
                             "means just say so.")
    parser.add_argument("--rec-debounce", type=float, default=0.3,
                        help="seconds ch8 must hold a new position before it "
                             "starts or stops a recording (default: 0.3), so "
                             "a single glitched reading cannot")
    parser.add_argument("--container", default="avi",
                        choices=sorted(CONTAINERS),
                        help="video container (default: avi). AVI is the "
                             "default because a truncated mp4 loses every "
                             "frame -- its index lives at the end -- while a "
                             "truncated AVI is still mostly readable.")
    parser.add_argument("--telemetry-interval", type=float, default=0.1,
                        help="seconds between telemetry rows written while "
                             "there is no video (default: 0.1, i.e. 10 Hz). "
                             "Frame-backed rows are unaffected -- they are "
                             "written one per frame as before.")
    parser.add_argument("--no-mavlink", action="store_true",
                        help="run without a flight controller (Pixhawk): do "
                             "not look for one or warn about it. Start "
                             "recordings with r in the pane (x x stops), or "
                             "use --autostart. The CSV then has no position "
                             "or attitude.")
    parser.add_argument("--autostart", action="store_true",
                        help="start recording as soon as the recorder is up, "
                             "without waiting for ch8 -- for runs with no "
                             "Pixhawk or nobody at the keyboard. Stops on x x, "
                             "or on ch8 flicked up and back down.")
    parser.add_argument("--require-mavlink", action="store_true",
                        help="refuse to start without a flight controller. "
                             "Off by default, because the camera, zoom, "
                             "imaging modes and snapshots all work without "
                             "one; turn it on for a real flight, where a "
                             "recorder that cannot arm is useless.")

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
    if args.no_mavlink and args.require_mavlink:
        sys.exit("--no-mavlink and --require-mavlink contradict each other")
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

    # An SSH session that dies of range sends SIGHUP to what was attached
    # to it, and the default action is to die immediately -- which would
    # leave the mp4 without its moov atom, i.e. unplayable, at exactly the
    # moment a flight ends badly. tmux normally shields the recorder from
    # this, but "normally" is not good enough for the only copy of a
    # flight, so the signal is ignored outright: a dropped link is not a
    # request to stop recording, and every deliberate way to stop is still
    # there (x in the pane, ./fly.sh --stop, SIGTERM).
    signal.signal(signal.SIGHUP, signal.SIG_IGN)

    try:
        # Control before video: the grab thread streams 1080p continuously
        # over the same USB3 link the NeoHD board carries VISCA on, and
        # probing baud rates against a busy bus makes the handshake flaky.
        recorder.open_control()
        recorder.open_mavlink()
        if recorder.mavlink is None and args.require_mavlink:
            log.critical("--require-mavlink was given and there is no flight "
                         "controller, so this run would record nothing. "
                         "Refusing to start.")
            return 2
        recorder.start_servo()
        if not recorder.open_camera():
            # Not fatal. Exiting here only had the supervisor restart it
            # every few seconds -- churn the board does not survive -- while
            # recording nothing, not even telemetry. The loop keeps trying.
            log.error("no camera at startup -- carrying on: ch8 still records "
                      "telemetry, and video joins when the camera appears")
        if args.no_mavlink:
            log.info("ready -- no flight controller in use: r starts "
                     "recording, x x stops it")
        elif recorder.mavlink is None:
            log.error("ready -- NO FLIGHT CONTROLLER: channel %d cannot start "
                      "a recording until one is found. Press r to record "
                      "without it.", args.rec_channel)
        else:
            log.info("ready -- channel %d high starts recording, or press r",
                     args.rec_channel)
        if args.autostart:
            recorder.request_start_recording(why="--autostart")
        recorder.run()
    except Exception:
        log.exception("unhandled error")
        return 1
    finally:
        recorder.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
