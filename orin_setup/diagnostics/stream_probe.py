#!/usr/bin/env python3
"""Is the camera board streaming a real picture? Grabs 3 raw frames.

    python3 stream_probe.py      -> "N frames, luma std X -> PICTURE|BLANK" or "NO FRAMES"

BLANK (std ~0) is the all-green, all-zero stream; NO FRAMES is a wedged
board. Both need the board's 12 V cycled. Stop the recorder first -- it
holds the device.
"""
import os, subprocess, sys
import numpy as np

F = "/tmp/probe.uyvy"
if os.path.exists(F):
    os.unlink(F)
subprocess.run(["timeout", "8", "v4l2-ctl", "-d", sys.argv[1] if len(sys.argv) > 1 else "/dev/video0",
                "--set-fmt-video=width=1920,height=1080,pixelformat=UYVY",
                "--stream-mmap", "--stream-count=3", "--stream-to=" + F], capture_output=True)
up = float(open("/proc/uptime").read().split()[0])
d = np.fromfile(F, np.uint8) if os.path.exists(F) else np.zeros(0, np.uint8)
fr = 1920 * 1080 * 2
n = len(d) // fr
if n == 0:
    print(f"uptime {up:5.0f}s: NO FRAMES")
    sys.exit(1)
y = d[:fr][1::2]
verdict = "PICTURE" if y.std() > 2 else "BLANK"
print(f"uptime {up:5.0f}s: {n} frames, luma std {y.std():.1f} -> {verdict}")
