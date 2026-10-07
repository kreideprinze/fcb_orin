#!/usr/bin/env python3
"""Can recording keep up? 15 s of the recorder's own capture + NVENC path.

    python3 enc_bench.py          # prints frames written per second and CPU

Measured 5 Oct 2026 on a fresh board: 60.0 fps, ~167% of one core. Stop
the recorder first. (A BGRx->nvvidconv variant was tried and wrote 0-byte
files while accepting every frame -- do not use it.)
"""
import os, sys, time
import cv2
sys.path.insert(0, os.path.expanduser("~/fcb_orin"))
import fcb_record as fr
from fcb_base_driver import devices
from fcb_base_driver.frame_grabber import FrameGrabber

dev = devices.autodetect_video(log=lambda m: None)
cap = devices.open_capture(dev)
cap.set(cv2.CAP_PROP_CONVERT_RGB, 1.0)
g = FrameGrabber(cap, devices.yuv422_to_bgr_code(devices.fourcc_of(cap)), drop_repeats=True)
time.sleep(2)
v = fr.VideoRecorder("/tmp/bench.avi", 1920, 1080, 60.0, encoder="nvenc")
seq = n = 0
t0, cpu0 = time.monotonic(), os.times()
while time.monotonic() - t0 < 15:
    f, s, _ = g.wait_for_frame(seq, 0.2)
    if f is None:
        continue
    seq = s
    v.write(fr._to_bgr(f)); n += 1
el = time.monotonic() - t0
cpu = sum(os.times()[:2]) - sum(cpu0[:2])
v.close(); g.stop()
size = os.path.getsize("/tmp/bench.avi") / 1e6
print(f"wrote {n} frames in {el:.1f}s = {n / el:.1f} fps, grabber {g.fps():.1f} fps, "
      f"cpu {100 * cpu / el:.0f}% of one core, file {size:.1f} MB")
os._exit(0)
