"""Regression: a recorder with no flight controller must not fail silently.

On 2026-09-08 a flight was lost because wants_recording() returned False
when self.mavlink was None. The loop treated that as a settled "no", so it
recorded nothing, said nothing, and showed a healthy 59.9 fps for the
whole session.
"""
import sys, types, time, threading, logging, io
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fcb_record as fr

fails = []
def check(label, ok, detail=""):
    if not ok: fails.append(label)
    print(("  ok   " if ok else "  FAIL ") + label + (("  -- " + detail) if detail and not ok else ""))

def make(mavlink):
    r = fr.Recorder.__new__(fr.Recorder)
    r.args = types.SimpleNamespace(rec_channel=8, rec_channel_threshold=1500,
                                   rec_channel_reverse=False, rc_timeout=2.0,
                                   camera_timeout=5.0, snap_channel=0,
                                   icr_channel=0, status_interval=0.25,
                                   log_status_interval=5.0,
                                   telemetry_interval=0.1)
    r.mavlink = mavlink
    r.servo = None; r.grabber = None; r.video = None; r.csv = None
    r.recording = False; r.segment = 0; r.video_path = None
    r._last_gap_row = 0.0; r._stop_armed = 0.0; r._start_suppressed = False
    r.icr_mode = "day"; r.snapshots = 0; r.frames_seen = 0; r.frames_dropped = 0
    r.base = None; r.record_started_mono = None
    return r

print("1. wants_recording with no flight controller")
r = make(None)
got = r.wants_recording()
check("returns None (unknown), not False (a settled 'no')", got is None,
      "got %r -- this is the exact bug that lost the flight" % got)

print("\n2. the status line carries a permanent warning")
text = r.status_text(time.monotonic())
check("status says NO FC -- r TO RECORD", "NO FC -- r TO RECORD" in text,
      "status was %r" % text)
check("it is first, where it cannot be missed", text.startswith("NO FC"))

print("\n3. a healthy recorder is unaffected")
class Mav:
    def rc_age(self): return 0.1
    def channel(self, n): return 1900
    def telemetry(self): return fr._NO_TELEMETRY
    def is_connected(self): return True
ok = make(Mav())
check("ch8 high still starts recording", ok.wants_recording() is True)
check("no false warning on a good link",
      "NO FC" not in ok.status_text(time.monotonic()))

print("\n4. the loop shouts, repeatedly, and does not just fall quiet")
buf = io.StringIO()
h = logging.StreamHandler(buf); h.setLevel(logging.DEBUG)
log = logging.getLogger("fcb"); log.handlers = [h]; log.setLevel(logging.DEBUG)

class Grab:
    def __init__(self): self.seq = 0
    def wait_for_frame(self, after, timeout=1.0):
        self.seq += 1
        time.sleep(0.005)
        import numpy as np
        return np.zeros((4, 4, 3), "uint8"), self.seq, time.monotonic()
    def fps(self): return 59.9
    def stop(self): pass

loop = make(None)
loop.grabber = Grab(); loop.running = True
loop.poll_keys = lambda: None
loop.poll_rc_switches = lambda: None
loop.update_display = lambda now: None
stop_at = time.monotonic() + 11.5
def stopper():
    while time.monotonic() < stop_at: time.sleep(0.05)
    loop.running = False
threading.Thread(target=stopper, daemon=True).start()
loop.capture_loop()

out = buf.getvalue()
shouts = out.count("NO FLIGHT CONTROLLER")
check("warns at all", shouts > 0, "it stayed silent, as before")
check("re-warns while it stays broken (%d times in 11s)" % shouts, shouts >= 2)
check("names the channel that cannot arm", "channel 8" in out)
check("says nothing is being recorded, and how to fix it", "nothing is being recorded" in out and "Press r" in out)

print("\n%s" % ("ALL PASS" if not fails else "FAILURES: %s" % fails))
sys.exit(1 if fails else 0)
