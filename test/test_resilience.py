"""Video and telemetry must not take each other down, and a recording must
end only on purpose."""
import sys, types, time, threading, logging, shutil, csv as csvmod
import numpy as np
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fcb_record as fr

import tempfile
OUT = os.path.join(tempfile.gettempdir(), "fcb_test_" + os.path.basename(__file__)[:-3])
shutil.rmtree(OUT, ignore_errors=True)
logging.basicConfig(level=logging.INFO, format="  %(levelname)-7s %(message)s",
                    stream=sys.stdout)

fails = []
def check(label, ok, detail=""):
    if not ok: fails.append(label)
    print(("  ok   " if ok else "  FAIL ") + label + (("  -- " + detail) if detail and not ok else ""))

FRAME = np.zeros((64, 64, 3), "uint8")

class Grab:
    """A camera that can be made to stop delivering."""
    def __init__(self): self.seq = 0; self.alive = True
    def wait_for_frame(self, after, timeout=1.0):
        time.sleep(0.005)
        if not self.alive: return None, after, None
        self.seq += 1
        return FRAME, self.seq, time.monotonic()
    def fps(self): return 30.0
    def stop(self): pass

class Mav:
    def __init__(self): self.ch8 = 1900; self.alive = True
    def rc_age(self): return 0.1 if self.alive else None
    def channel(self, n): return self.ch8
    def telemetry(self): return fr._NO_TELEMETRY
    def is_connected(self): return self.alive

def build(mav):
    r = fr.Recorder.__new__(fr.Recorder)
    r.args = types.SimpleNamespace(
        rec_channel=8, rec_channel_threshold=1500, rec_channel_reverse=False,
        rc_timeout=2.0, camera_timeout=0.4, record_dir=OUT, container="avi", encoder="cpu",
        bitrate=5.0, fps=30.0, telemetry_interval=0.05, snap_channel=0,
        icr_channel=0, status_interval=0.25, log_status_interval=99.0)
    r.mavlink = mav; r.grabber = Grab(); r.servo = None
    r.recording = False; r.segment = 0; r.video = None; r.video_path = None
    r.csv = None; r.base = None; r.record_started_mono = None
    r.record_started_wall = None; r._last_gap_row = 0.0; r._stop_armed = 0.0
    r.icr_mode = "day"; r.snapshots = 0; r.frames_seen = 0; r.frames_dropped = 0
    r.declared_fps = 30.0; r.running = True; r.keys = None; r.zoom_entry = None
    r._start_suppressed = False
    r.name_entry = None; r._name_base = None; r.keys = None
    r.zoom_entry = None
    r.poll_keys = lambda: None
    r.poll_rc_switches = lambda: None
    r.update_display = lambda now: None
    r.open_camera = lambda quiet=False: False   # camera stays gone once lost
    return r

def run_for(r, seconds):
    t = time.monotonic() + seconds
    def stop():
        while time.monotonic() < t: time.sleep(0.02)
        r.running = False
    th = threading.Thread(target=stop, daemon=True); th.start()
    r.capture_loop(); th.join()

print("1. camera dies mid-recording -> telemetry keeps going")
mav = Mav(); r = build(mav)
th = threading.Thread(target=lambda: run_for(r, 3.0), daemon=True); th.start()
time.sleep(1.0)
check("recording started from ch8 high", r.recording)
rows_before = r.csv.rows
seg1 = r.video_path
r.grabber.alive = False                    # camera unplugged
time.sleep(1.6)
check("session survives the camera loss", r.recording, "the flight ended with the camera")
check("video segment was closed", r.video is None)
rows_after = r.csv.rows if r.csv else 0
check("telemetry rows still accruing (%d -> %d)" % (rows_before, rows_after),
      rows_after > rows_before, "the CSV stopped when video did")
th.join(timeout=5)

print("\n2. ch8 going low stops the recording again (RC stop restored)")
mav = Mav(); r = build(mav)
th = threading.Thread(target=lambda: run_for(r, 2.5), daemon=True); th.start()
time.sleep(0.8)
check("recording", r.recording)
mav.ch8 = 1000                              # switch flicked low
time.sleep(1.0)
check("ch8 low stopped it", not r.recording,
      "the RC stop did not take effect")
mav.ch8 = 1900                              # and high starts a new one
time.sleep(0.8)
check("ch8 high starts another", r.recording)

print("\n3. 'x' twice stops it; once does not")
r.request_stop_recording()
check("one press only arms", r.recording)
r.request_stop_recording()
check("second press stops", not r.recording)
time.sleep(0.6)
check("does not restart while ch8 stays high", not r.recording,
      "the stop key just chopped the flight in two")
th.join(timeout=5)

print("\n3b. an RC stop must NOT open the naming prompt")
mav = Mav(); r = build(mav)
r.keys = types.SimpleNamespace(enabled=True)
th = threading.Thread(target=lambda: run_for(r, 2.2), daemon=True); th.start()
time.sleep(0.8)
check("recording", r.recording)
mav.ch8 = 1000
time.sleep(0.8)
check("stopped by RC", not r.recording)
check("no prompt swallowing keys mid-flight", r.name_entry is None,
      "the mode and snapshot keys would be dead until someone pressed Enter")
th.join(timeout=5)

print("\n4. MAVLink dies mid-recording -> video keeps going")
mav = Mav(); r = build(mav)
th = threading.Thread(target=lambda: run_for(r, 2.5), daemon=True); th.start()
time.sleep(0.8)
check("recording", r.recording)
frames_before = r.video.frames
mav.alive = False                           # telemetry link lost
time.sleep(1.2)
check("still recording without MAVLink", r.recording)
check("video frames still accruing (%d -> %d)" % (frames_before, r.video.frames if r.video else -1),
      r.video is not None and r.video.frames > frames_before)
r.request_stop_recording(); r.request_stop_recording()
th.join(timeout=5)

print("\n5. the CSV distinguishes gap rows from frame rows")
found = [f for f in os.listdir(OUT) if f.endswith(".csv")]
gap = frame = 0
for f in found:
    with open(os.path.join(OUT, f)) as fh:
        for row in csvmod.DictReader(fh):
            if row["frame"] == "": gap += 1
            else: frame += 1
check("frame-backed rows exist (%d)" % frame, frame > 0)
check("gap rows exist and are marked by an empty frame (%d)" % gap, gap > 0)

print("\n6. SIGHUP is ignored, so an SSH drop cannot kill a flight")
import signal
src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "fcb_record.py")).read()
check("SIGHUP set to SIG_IGN", "signal.signal(signal.SIGHUP, signal.SIG_IGN)" in src)

print("\n%s" % ("ALL PASS" if not fails else "FAILURES: %s" % fails))
sys.exit(1 if fails else 0)
