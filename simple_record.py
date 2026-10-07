#!/usr/bin/env python3
"""Minimal FCB recorder: video only, controlled from the keyboard.

    r / space   start or stop recording
    q           quit (stops any recording first)

No MAVLink, no CSV, no zoom -- fcb_record.py does all that. Files go to
~/flight_recordings/rec-YYYYmmdd-HHMMSS.avi, hardware-encoded where possible.
"""
import argparse
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime

import prefer_cv2  # noqa: F401  -- before cv2
import cv2

from fcb_base_driver.devices import (
    REPEATING_FRAME_USB_IDS, autodetect_video, fourcc_of, open_capture,
    usb_id, yuv422_to_bgr_code)
from fcb_base_driver.frame_grabber import FrameGrabber


def open_writer(path, width, height, fps, bitrate_mbps):
    fps = max(1, int(round(fps)))  # nvv4l2h264enc will not take a fractional rate
    pipeline = (
        f"appsrc ! video/x-raw,format=BGR,width={width},height={height},"
        f"framerate={fps}/1 ! queue ! videoconvert ! video/x-raw,format=NV12 "
        f"! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
        f"! nvv4l2h264enc bitrate={int(bitrate_mbps * 1_000_000)} "
        f"insert-sps-pps=1 maxperf-enable=1 "
        f"! h264parse ! avimux ! filesink location={path}"
    )
    writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(fps),
                             (width, height))
    if writer.isOpened():
        return writer, "nvenc"
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"XVID"), float(fps),
                             (width, height))
    return (writer, "cpu") if writer.isOpened() else (None, None)


def read_key():
    """One key if pressed, else None. Read from the fd so select() stays honest."""
    if select.select([sys.stdin], [], [], 0)[0]:
        data = os.read(sys.stdin.fileno(), 32)
        return data[:1].decode(errors="ignore") if data else "q"  # EOF: quit
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--video", help="video device (default: autodetect the FCB)")
    parser.add_argument("--out", default=os.path.expanduser(
        os.environ.get("FCB_RECORD_DIR", "~/flight_recordings")))
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--bitrate", type=float, default=25, help="Mbps")
    args = parser.parse_args()

    if not sys.stdin.isatty():
        sys.exit("needs a terminal for key presses")

    device = args.video or autodetect_video()
    if not device:
        sys.exit("FCB camera not found")
    capture = open_capture(device, args.width, args.height)
    if capture is None:
        sys.exit(f"could not open {device}")
    grabber = FrameGrabber(capture, yuv422_to_bgr_code(fourcc_of(capture)),
                           drop_repeats=usb_id(device) in REPEATING_FRAME_USB_IDS)
    os.makedirs(args.out, exist_ok=True)

    saved = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())  # cbreak keeps Ctrl-C working
    writer = path = None
    frames = 0
    started = last_status = 0.0
    seq = 0

    def stop():
        nonlocal writer
        writer.release()
        writer = None
        print(f"\r\nSTOPPED  {path}  ({frames} frames, {time.monotonic() - started:.1f}s)")

    print(f"camera {device}  --  [r/space] start/stop   [q] quit")
    try:
        while True:
            key = read_key()
            if key == "q":
                break
            if key in ("r", " "):
                if writer:
                    stop()
                else:
                    fps = grabber.fps()
                    if fps <= 0:
                        print("\rno frames from the camera yet -- try again")
                    else:
                        path = os.path.join(
                            args.out, datetime.now().strftime("rec-%Y%m%d-%H%M%S.avi"))
                        writer, encoder = open_writer(path, args.width, args.height,
                                                      fps, args.bitrate)
                        if writer is None:
                            print("\rcould not open a video writer")
                        else:
                            frames, started = 0, time.monotonic()
                            print(f"\rRECORDING {path}  ({encoder}, {fps:.1f} fps)")

            frame, new_seq, _ = grabber.wait_for_frame(seq, timeout=0.1)
            if frame is not None:
                seq = new_seq
                if writer:
                    if frame.shape[1] != args.width or frame.shape[0] != args.height:
                        frame = cv2.resize(frame, (args.width, args.height))
                    writer.write(frame)
                    frames += 1

            now = time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                state = (f"REC {now - started:6.1f}s {frames} fr" if writer
                         else "idle")
                sys.stdout.write(f"\r{grabber.fps():5.1f} fps | {state}   ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if writer:
            stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        grabber.stop()
        print()


if __name__ == "__main__":
    main()
