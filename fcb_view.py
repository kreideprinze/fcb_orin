#!/usr/bin/env python3
"""Minimal desktop viewer and recorder for the Sony FCB-EV9520L.

For a laptop with the camera plugged into it: a window showing the live
feed, and a key to record. No autopilot, no RC, no telemetry, no tmux --
none of that exists on a desk. What is kept is the part that is fiddly to
get right and worth reusing: VISCA zoom with detents, and the IR cut
filter modes.

    ./fcb_view.py                  find the camera, show it
    ./fcb_view.py --video /dev/video2
    ./fcb_view.py --scale 1.0      full size window
    ./fcb_view.py --no-visca       video only, no serial control

Keys, in the window (not the terminal):

    r          start / stop recording
    1 2 3 4    RGB / IR / RGB+IR / AUTO
    i          cycle those four
    + -        zoom in / out by one 0.5x detent
    0          back to 1x
    s          save a JPEG of this frame
    q or Esc   quit

Recordings and snapshots land in --dir (default ~/fcb_laptop), named for
the moment recording started. The feed is rotated 180 degrees by default,
because the camera is mounted upside down -- --rotate 0 turns that off.
"""
import argparse
import glob
import os
import signal
import sys
import time
from datetime import datetime

try:
    import prefer_cv2  # noqa: F401  -- before cv2
    import cv2
except ImportError as exc:
    sys.exit(f"{exc}\n\nThis needs OpenCV (cv2).")

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

try:
    from fcb_base_driver import devices, visca, zoom_map
    from fcb_base_driver.visca_link import ViscaError, ViscaLink, ViscaTimeout
    from fcb_base_driver.zoom_servo import ZoomServo
    from fcb_base_driver.frame_grabber import _same_frame
except ImportError as exc:
    sys.exit(f"{exc}\n\nRun this from the fcb_orin directory, or beside "
             f"the fcb_base_driver package.")

#: key -> (label, CAM_ICR sub-command). Same four modes as the flight
#: recorder, and the same names, so notes from one apply to the other.
MODES = {
    "1": ("RGB", "day"),
    "2": ("IR", "night"),
    "3": ("RGB+IR", "night_color"),
    "4": ("AUTO", "auto"),
}
MODE_ORDER = ["day", "night", "night_color", "auto"]
MODE_LABEL = {mode: label for label, mode in MODES.values()}

WINDOW = "FCB-EV9520L"


def to_bgr(frame, yuv_code=cv2.COLOR_YUV2BGR_YUY2):
    """Whatever the camera handed over, as three-channel BGR.

    Measured on this FCB over USB3: the board offers only YUYV, and with
    CAP_PROP_CONVERT_RGB off OpenCV passes the raw pairs straight through
    as a *two*-channel image. Nothing downstream copes with that -- the
    overlay refuses to draw on it and the video writer would have happily
    encoded garbage -- and the shape differs between machines, so it is
    normalised here rather than assumed anywhere. `yuv_code` is the packing
    of those pairs: YUYV normally, UYVY when the NeoHD board comes up under
    its generic FX3 identity.
    """
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    channels = frame.shape[2]
    if channels == 2:
        return cv2.cvtColor(frame, yuv_code)
    if channels == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def unique_base(base):
    """A base path no existing recording is already using.

    The timestamp only resolves to the second, so stopping and immediately
    starting again produces the same name twice -- and the second
    recording would overwrite the first without a word. Letters rather
    than numbers for the suffix: numbers are what video segments use.
    """
    if not glob.glob(glob.escape(base) + ".*"):
        return base
    for letter in "bcdefghijklmnopqrstuvwxyz":
        candidate = f"{base}{letter}"
        if not glob.glob(glob.escape(candidate) + ".*"):
            return candidate
    # Twenty-five in one second is not a thing, but never return a name
    # that would overwrite something.
    return f"{base}-{int(time.time() * 1000) % 100000}"


def log(message, *args):
    print(f"{datetime.now():%H:%M:%S}  " + (message % args if args else message),
          flush=True)


class Viewer:

    def __init__(self, args):
        self.args = args
        self.link = None
        self.servo = None
        self.capture = None
        self.writer = None
        self.base = None
        self.mode = None
        self.frames = 0
        self.rec_frames = 0
        self.rec_started = None
        self.fps_measured = 0.0
        self.declared_fps = None
        self.repeats = False
        self._dropped_last = False
        self._last_raw = None
        self.yuv_code = cv2.COLOR_YUV2BGR_YUY2
        self.running = True
        self.message = ""
        self.message_until = 0.0
        self._warned_no_visca = False
        self._fps_marker = (0, time.monotonic())
        self.frame_size = (args.width, args.height)

    # -- setup -----------------------------------------------------------

    def open_control(self):
        """VISCA, for zoom and the IR cut filter. Optional on purpose.

        No serial port is a degraded viewer, not a broken one -- the feed
        and the recording are the point, and they do not need it. So this
        warns and carries on rather than refusing to start.
        """
        if self.args.no_visca:
            return
        port, baud = self.args.port, self.args.baud
        if port is None:
            port, baud = devices.autodetect_visca()
        if port is None:
            log("no VISCA port answered -- zoom and mode keys will do "
                "nothing. The feed and recording still work.")
            return
        try:
            self.link = ViscaLink(port, baud=baud or 9600,
                                  address=self.args.address)
        except Exception as exc:
            log("could not open %s: %s -- carrying on without zoom control",
                port, exc)
            return
        log("VISCA: %s at %d baud", port, baud or 9600)

        self.servo = ZoomServo(
            self.link, curve=self.args.curve, rc=None,
            ratio_step=self.args.zoom_step,
            on_status=lambda msg: log("zoom: %s", msg),
        )
        try:
            ratio = self.servo.sync_from_camera()
            log("zoom: currently %.1fx", ratio)
        except Exception as exc:
            log("could not read the zoom position: %s", exc)
        self.read_mode()

    def read_mode(self):
        """Adopt whatever mode the camera is already in, without changing it.

        The FCB remembers this across a power cycle, so the honest thing at
        startup is to report it rather than impose one.
        """
        if self.link is None:
            return
        try:
            reply = self.link.inquiry(visca.icr_mode_inq(self.link.address))
            self.mode = visca.parse_icr_mode(reply)
        except (ViscaError, ViscaTimeout, ValueError) as exc:
            log("could not read the imaging mode: %s", exc)
            return
        # The inquiry reports where the filter is, not how it got there, so
        # a camera left in AUTO reads back as whichever mode it has picked.
        log("imaging mode: %s (as found)", MODE_LABEL.get(self.mode, self.mode))

    def open_camera(self):
        device = self.args.video
        if device is None:
            device = devices.autodetect_video(log=lambda m: log("video: %s", m))
        if device is None:
            log("no FCB camera found. If it is attached under a name this "
                "does not recognise, pass --video /dev/videoN explicitly -- "
                "it is not going to quietly record from your webcam instead.")
            return False

        self.capture = devices.open_capture(
            device, self.args.width, self.args.height, self.args.fourcc)
        if self.capture is None:
            log("could not open %s", device)
            return False

        # Ask for the conversion at the source where the backend will do
        # it; to_bgr() covers the case where it declines.
        try:
            self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 1.0)
        except cv2.error:
            pass

        width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_size = (width, height)
        pixel_format = devices.fourcc_of(self.capture)
        self.yuv_code = devices.yuv422_to_bgr_code(pixel_format)
        declared = self.capture.get(cv2.CAP_PROP_FPS)
        self.declared_fps = declared if 1.0 < declared < 1000.0 else None
        log("video: %s at %dx%d %s, %s", device, width, height,
            pixel_format or "format not reported",
            f"{self.declared_fps:.2f} fps" if self.declared_fps
            else "rate not reported")
        # The Oppila board sends every fourth frame twice and reports 30 fps
        # for a 60 fps camera; recorded as-is a clip played back at ~1/2.6
        # speed. Repeats are dropped in run() and the rate is measured.
        self.repeats = devices.usb_id(device) in devices.REPEATING_FRAME_USB_IDS
        if self.repeats:
            self.declared_fps = None
            log("video: this board repeats frames and misreports its rate -- "
                "dropping the repeats and measuring the rate")
        return True

    # -- recording -------------------------------------------------------

    def toggle_recording(self, frame):
        if self.writer is None:
            self.start_recording(frame)
        else:
            self.stop_recording()

    def start_recording(self, frame):
        height, width = frame.shape[:2]
        fps = self.declared_fps or (self.fps_measured
                                    if self.fps_measured >= 1.0
                                    else self.args.fps)
        os.makedirs(self.args.dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.base = unique_base(os.path.join(self.args.dir, f"fcb-{stamp}"))
        path = f"{self.base}.{self.args.container}"

        # Whole frames per second: a measured rate like 96.67 is a time base
        # FFmpeg's encoders reject, so the writer opened or not at random.
        writer = cv2.VideoWriter(
            path, cv2.VideoWriter_fourcc(*self.args.codec),
            float(max(1, int(round(fps)))), (width, height))
        if not writer.isOpened():
            self.flash(f"could not open {os.path.basename(path)}")
            log("could not open a writer for %s -- try --codec MJPG", path)
            return
        self.writer = writer
        self.rec_frames = 0
        self.rec_started = time.monotonic()
        log("RECORDING  %s  (%dx%d @ %.2f fps, %s)",
            os.path.basename(path), width, height, fps, self.args.codec)
        self.flash("recording")

    def stop_recording(self):
        if self.writer is None:
            return
        self.writer.release()
        self.writer = None
        path = f"{self.base}.{self.args.container}"
        elapsed = time.monotonic() - (self.rec_started or time.monotonic())
        size = os.path.getsize(path) if os.path.exists(path) else 0
        log("STOPPED    %s -- %d frames, %.1fs, %.1f MB",
            os.path.basename(path), self.rec_frames, elapsed, size / 1e6)
        # The same silent failure the flight recorder guards against: a
        # writer that accepted the file and rejected every frame leaves a
        # healthy-looking counter and an empty file.
        if self.rec_frames > 0 and size < 10_000:
            log("THE VIDEO IS EMPTY (%d bytes) despite %d frames -- the "
                "%s codec rejected them. Try --codec MJPG.",
                size, self.rec_frames, self.args.codec)
        self.flash("saved " + os.path.basename(path))
        self.rec_started = None

    def snapshot(self, frame):
        os.makedirs(self.args.dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")[:-3]
        path = os.path.join(self.args.dir, f"snap-{stamp}.jpg")
        if cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
            log("snapshot: %s", path)
            self.flash("snap " + os.path.basename(path))
        else:
            log("could not write %s", path)

    # -- camera control --------------------------------------------------

    def need_visca(self):
        if self.servo is not None:
            return True
        if not self._warned_no_visca:
            self._warned_no_visca = True
            log("no VISCA link -- zoom and mode keys do nothing this run")
        self.flash("no VISCA link")
        return False

    def set_mode(self, mode):
        if not self.need_visca():
            return
        try:
            self.servo.set_icr(mode)
        except (ViscaError, ViscaTimeout, ValueError) as exc:
            log("imaging mode %s would not take: %s",
                MODE_LABEL.get(mode, mode), exc)
            self.flash("mode failed")
            return
        self.mode = mode
        log("imaging mode: %s", MODE_LABEL.get(mode, mode))
        self.flash(MODE_LABEL.get(mode, mode))

    def cycle_mode(self):
        try:
            index = MODE_ORDER.index(self.mode)
        except ValueError:
            index = -1
        self.set_mode(MODE_ORDER[(index + 1) % len(MODE_ORDER)])

    def step_zoom(self, detents):
        """Move by whole detents, so the same key always does the same thing."""
        if not self.need_visca():
            return
        step = self.args.zoom_step or 0.5
        current = self.servo.target_ratio or self.servo.ratio or 1.0
        wanted = zoom_map.snap_ratio(current + detents * step, step)
        got = self.servo.set_ratio(wanted)
        self.flash(f"zoom {got:.1f}x")

    def zoom_wide(self):
        if not self.need_visca():
            return
        self.servo.set_ratio(zoom_map.MIN_RATIO)
        self.flash("zoom 1.0x")

    # -- display ---------------------------------------------------------

    def flash(self, text, seconds=1.5):
        """A short message over the video, for things with no other home."""
        self.message = text
        self.message_until = time.monotonic() + seconds

    def status_text(self):
        parts = [f"{self.fps_measured:.1f} fps"]
        if self.writer is not None:
            elapsed = time.monotonic() - (self.rec_started or 0)
            parts.append(f"REC {self.rec_frames} fr  {elapsed:.0f}s")
        else:
            parts.append("not recording")
        if self.servo is not None and self.servo.ratio is not None:
            target = self.servo.target_ratio
            if target is not None and abs(target - self.servo.ratio) >= 0.05:
                parts.append(f"zoom {self.servo.ratio:.1f}>{target:.1f}x")
            else:
                parts.append(f"zoom {self.servo.ratio:.1f}x")
        if self.mode is not None:
            parts.append(MODE_LABEL.get(self.mode, self.mode))
        return "   ".join(parts)

    def draw_overlay(self, shown):
        height, width = shown.shape[:2]
        bar = max(22, int(height * 0.055))
        scale = bar / 34.0

        # Darken a strip rather than drawing text straight onto the picture:
        # white text on a bright sky is unreadable exactly when you need it.
        strip = shown[:bar].copy()
        cv2.rectangle(shown, (0, 0), (width, bar), (0, 0, 0), -1)
        cv2.addWeighted(strip, 0.45, shown[:bar], 0.55, 0, dst=shown[:bar])

        recording = self.writer is not None
        cv2.putText(shown, self.status_text(), (int(8 * scale) + (bar if recording else 0),
                    int(bar * 0.72)), cv2.FONT_HERSHEY_SIMPLEX, 0.6 * scale,
                    (255, 255, 255), max(1, int(round(1.4 * scale))),
                    cv2.LINE_AA)
        if recording:
            # A blinking dot, because a static one reads as a UI decoration
            # and this needs to say "still going" at a glance.
            if int(time.monotonic() * 2) % 2 == 0:
                cv2.circle(shown, (int(bar * 0.55), int(bar * 0.5)),
                           max(4, int(bar * 0.22)), (60, 60, 255), -1)

        if time.monotonic() < self.message_until and self.message:
            cv2.putText(shown, self.message, (int(8 * scale),
                        height - int(10 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6 * scale, (80, 255, 80),
                        max(1, int(round(1.4 * scale))), cv2.LINE_AA)
        return shown

    def measure_fps(self):
        self.frames += 1
        frames, since = self._fps_marker
        elapsed = time.monotonic() - since
        if elapsed >= 0.5:
            self.fps_measured = (self.frames - frames) / elapsed
            self._fps_marker = (self.frames, time.monotonic())

    # -- main loop -------------------------------------------------------

    def handle_key(self, key, frame):
        if key in (ord("q"), 27):                 # q, Esc
            self.running = False
        elif key == ord("r"):
            self.toggle_recording(frame)
        elif key == ord("s"):
            self.snapshot(frame)
        elif key == ord("i"):
            self.cycle_mode()
        elif key in (ord("+"), ord("=")):
            self.step_zoom(+1)
        elif key in (ord("-"), ord("_")):
            self.step_zoom(-1)
        elif key == ord("0"):
            self.zoom_wide()
        elif chr(key) in MODES:
            self.set_mode(MODES[chr(key)][1])

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        if self.args.scale:
            cv2.resizeWindow(WINDOW,
                             int(self.frame_size[0] * self.args.scale),
                             int(self.frame_size[1] * self.args.scale))
        log("keys: r record   1-4 modes   i cycle   +/- zoom   0 wide   "
            "s snap   q quit")

        misses = 0
        while self.running:
            ok, frame = self.capture.read()
            if not ok or frame is None:
                misses += 1
                if misses == 1:
                    log("no frame from the camera")
                if misses > self.args.give_up_after:
                    log("the camera stopped delivering -- giving up")
                    break
                # A recording must not silently accumulate a gap, so it is
                # closed cleanly rather than left open across an outage.
                if self.writer is not None:
                    log("closing the recording at the break")
                    self.stop_recording()
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
                continue
            misses = 0
            # At most one in a row, as in FrameGrabber: the board repeats a
            # single frame, while a run of identical ones is the picture.
            if self.repeats and not self._dropped_last \
                    and _same_frame(frame, self._last_raw):
                self._dropped_last = True
                continue
            self._dropped_last = False
            self._last_raw = frame
            frame = to_bgr(frame, self.yuv_code)
            if self.args.rotate == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            self.measure_fps()

            # Written before anything is drawn on it: the overlay is for
            # the window, not for the file.
            if self.writer is not None:
                self.writer.write(frame)
                self.rec_frames += 1

            shown = frame
            if self.args.scale and self.args.scale != 1.0:
                shown = cv2.resize(frame, None, fx=self.args.scale,
                                   fy=self.args.scale,
                                   interpolation=cv2.INTER_AREA)
            else:
                shown = frame.copy()
            cv2.imshow(WINDOW, self.draw_overlay(shown))

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                self.handle_key(key, frame)

            # The window's close button has to finalise the recording too,
            # or clicking X leaves an unplayable file.
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    log("window closed")
                    break
            except cv2.error:
                break

    def close(self):
        self.stop_recording()
        if self.capture is not None:
            self.capture.release()
        if self.servo is not None:
            self.servo.stop()
        elif self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
        cv2.destroyAllWindows()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Show the FCB feed on this machine and record with 'r'.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--video", default=None,
                        help="capture device (default: autodetect an FCB / "
                             "NeoHD board by name)")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=30.0,
                        help="rate written into the file when the camera "
                             "does not report one")
    parser.add_argument("--fourcc", default=None,
                        help="capture fourcc, e.g. MJPG, if the default "
                             "negotiation gives a low frame rate")
    parser.add_argument("--rotate", type=int, default=180, choices=(0, 180),
                        help="rotate the feed, in degrees. 180 by default "
                             "because the camera is mounted upside down; it "
                             "is applied once, so the window, the recording "
                             "and snapshots all agree")
    parser.add_argument("--scale", type=float, default=0.5,
                        help="window size relative to the capture; the file "
                             "is always written at full resolution")

    parser.add_argument("--port", default=None,
                        help="VISCA serial port (default: autodetect)")
    parser.add_argument("--baud", type=int, default=None)
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--no-visca", action="store_true",
                        help="do not touch the serial port at all")
    parser.add_argument("--curve", default="ratio",
                        choices=list(zoom_map.CURVES))
    parser.add_argument("--zoom-step", type=float,
                        default=zoom_map.DEFAULT_RATIO_STEP,
                        help="magnification per +/- press")

    parser.add_argument("--dir",
                        default=os.path.expanduser("~/fcb_laptop"),
                        help="where clips and snapshots go. Its own folder, "
                             "not ~/fcb_recordings -- that one holds flights "
                             "pulled off the drone and these should not mix "
                             "in with them")
    parser.add_argument("--container", default="avi", choices=("avi", "mp4"),
                        help="AVI is far more tolerant of an interrupted "
                             "write; a truncated mp4 loses every frame")
    parser.add_argument("--codec", default="XVID",
                        help="fourcc for the writer: XVID or MJPG for avi, "
                             "mp4v for mp4")
    parser.add_argument("--give-up-after", type=int, default=200,
                        help="consecutive empty reads before exiting")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    viewer = Viewer(args)

    def on_signal(signum, _frame):
        log("caught %s -- finishing up", signal.Signals(signum).name)
        viewer.running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        viewer.open_control()
        if not viewer.open_camera():
            return 1
        viewer.run()
    finally:
        viewer.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
