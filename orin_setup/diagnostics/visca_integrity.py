#!/usr/bin/env python3
"""How many VISCA replies survive the board? Sends 400 Address Set pads.

    python3 visca_integrity.py idle      # control port only
    python3 visca_integrity.py stream    # with 1080p video streaming too

Every reply should be exactly 88 30 02 FF. Measured on the Oppila board,
5 Oct 2026: idle 399/400 intact; streaming 296/400, 61 malformed -- bytes
lost and corrupted by its USB serial bridge. Stop the recorder first.
"""
import os, sys, threading, time
import serial
sys.path.insert(0, os.path.expanduser("~/fcb_orin"))
from fcb_base_driver import devices

port = devices.camera_serial_ports()[0]
stream = len(sys.argv) > 1 and sys.argv[1] == "stream"
run, frames = [True], [0]
if stream:
    cap = devices.open_capture(devices.autodetect_video(log=lambda m: None))
    def grab():
        while run[0]:
            frames[0] += cap.read()[0]
    threading.Thread(target=grab, daemon=True).start()
    time.sleep(2)
s = serial.Serial(port, 9600, timeout=0.05)
PAD = bytes([0x88, 0x30, 0x01, 0xFF])
s.write(PAD * 16); time.sleep(1.5); s.read(4096)
N, got = 400, bytearray()
for _ in range(N // 8):
    s.write(PAD * 8); time.sleep(0.12)
    got += s.read(4096)
end = time.monotonic() + 3
while time.monotonic() < end:
    got += s.read(4096)
run[0] = False
parts = bytes(got).split(b"\xff")
good = sum(1 for f in parts if f == bytes([0x88, 0x30, 0x02]))
bad = [f.hex(" ") for f in parts[:-1] if f != bytes([0x88, 0x30, 0x02])]
print(f"{'stream' if stream else 'idle'}: sent {N}, {good} intact, {len(bad)} malformed, "
      f"{N - good} missing; video frames {frames[0]}")
print("malformed samples:", bad[:10])
os._exit(0)
