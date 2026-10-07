"""Camera comes back mid-flight: a new segment, the same CSV, one session."""
import sys, types, time, threading, logging, shutil
import numpy as np
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fcb_record as fr
import tempfile
OUT = os.path.join(tempfile.gettempdir(), "fcb_test_" + os.path.basename(__file__)[:-3])
shutil.rmtree(OUT, ignore_errors=True)
logging.basicConfig(level=logging.WARNING, format="  %(levelname)-7s %(message)s", stream=sys.stdout)
FRAME = np.zeros((64, 64, 3), "uint8")
fails=[]
def check(l, ok, d=""):
    if not ok: fails.append(l)
    print(("  ok   " if ok else "  FAIL ")+l+(("  -- "+d) if d and not ok else ""))

class Grab:
    def __init__(self): self.seq=0; self.alive=True
    def wait_for_frame(self, a, timeout=1.0):
        time.sleep(0.005)
        if not self.alive: return None, a, None
        self.seq+=1; return FRAME, self.seq, time.monotonic()
    def fps(self): return 30.0
    def stop(self): pass
class Mav:
    def rc_age(self): return 0.1
    def channel(self, n): return 1900
    def telemetry(self): return fr._NO_TELEMETRY
    def is_connected(self): return True

r = fr.Recorder.__new__(fr.Recorder)
r.args = types.SimpleNamespace(rec_channel=8, rec_channel_threshold=1500,
    rec_channel_reverse=False, rc_timeout=2.0, camera_timeout=0.4,
    record_dir=OUT, container="avi", encoder="cpu", bitrate=5.0, fps=30.0,
    telemetry_interval=0.05, snap_channel=0, icr_channel=0,
    status_interval=0.25, log_status_interval=99.0)
r.mavlink=Mav(); r.grabber=Grab(); r.servo=None
r.recording=False; r.segment=0; r.video=None; r.video_path=None; r.csv=None
r.base=None; r.record_started_mono=None; r.record_started_wall=None
r._last_gap_row=0.0; r._stop_armed=0.0; r.icr_mode="day"; r.snapshots=0
r.frames_seen=0; r.frames_dropped=0; r.declared_fps=30.0; r.running=True
r.keys=None; r.zoom_entry=None; r._start_suppressed=False
r.poll_keys=lambda: None; r.poll_rc_switches=lambda: None
r.update_display=lambda now: None
# The real open_camera() builds a new FrameGrabber and assigns it, and
# reopen_camera() has already set self.grabber to None by this point.
attempts = {"n": 0}
def _reopen(quiet=False):
    attempts["n"] += 1
    if attempts["n"] < 3:          # still unplugged
        return False
    r.grabber = Grab()
    return True
r.open_camera = _reopen

th=threading.Thread(target=r.capture_loop, daemon=True); th.start()
time.sleep(0.8)
check("segment 1 open", r.segment==1 and r.video is not None)
base = r.base
r.grabber.alive=False                       # camera drops
time.sleep(0.9)
check("segment 1 closed during outage", r.video is None)
check("session still live", r.recording)
time.sleep(2.5)                             # reopen_camera brings it back
check("segment 2 opened on return (seg=%d)" % r.segment, r.segment>=2)
check("video writing again", r.video is not None)
check("same session, same base", r.base==base)
rows = r.csv.rows
# Production handles keys inside the capture loop, so the stop must be
# driven from there too -- calling it from this thread races the loop and
# tests an ordering that never happens in the real program.
r.poll_keys = lambda: (r.request_stop_recording(), r.request_stop_recording(),
                       setattr(r, "poll_keys", lambda: None))
time.sleep(0.6)
check("stopped by hand", not r.recording)
time.sleep(0.8)                             # ch8 is STILL high
check("does NOT restart while ch8 stays high", not r.recording,
      "the stop key just chopped the flight in two")
r.running=False; th.join(timeout=5)

files = sorted(os.listdir(OUT))
vid = [f for f in files if f.endswith(".avi")]
csvs = [f for f in files if f.endswith(".csv")]
check("two numbered video files: %s" % vid, len(vid)==2)
check("exactly one CSV for the flight: %s" % csvs, len(csvs)==1)
check("CSV spans both segments (%d rows)" % rows, rows>0)
print("\n%s" % ("ALL PASS" if not fails else "FAILURES: %s"%fails))
sys.exit(1 if fails else 0)
