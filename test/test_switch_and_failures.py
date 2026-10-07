"""ch8 debounce, recording with no video, write failures, a late autopilot.

Each of these was a way to lose a flight or make a junk one:
  - one glitched ch8 sample stopped a recording and the next started
    another, leaving one-frame files behind;
  - with the camera out, ch8 did nothing at all;
  - a write error (a full disk) killed the recorder mid-flight;
  - an autopilot plugged in after startup was never picked up;
  - a closed terminal made the status line raise from inside logging.
"""
import sys, os, io, types, tempfile, shutil, logging
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fcb_record as fr

logging.basicConfig(level=logging.INFO, format="  %(levelname)-7s %(message)s",
                    stream=sys.stdout)
OUT = os.path.join(tempfile.gettempdir(), "fcb_test_switch")

fails = []
def check(label, ok, detail=""):
    if not ok: fails.append(label)
    print(("  ok   " if ok else "  FAIL ") + label + (("  -- " + detail) if detail and not ok else ""))


class FakeMavlink:
    def __init__(self, pwm=1000):
        self.pwm = pwm
    def rc_age(self):
        return 0.05
    def channel(self, n):
        return self.pwm
    def telemetry(self):
        return fr._NO_TELEMETRY
    def is_connected(self):
        return True


def recorder():
    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT)
    r = fr.Recorder.__new__(fr.Recorder)
    r.args = types.SimpleNamespace(
        record_dir=OUT, rec_channel=8, rec_channel_threshold=1500,
        rec_channel_reverse=False, rc_timeout=2.0, rec_debounce=0.3,
        telemetry_interval=0.1, container="avi", encoder="cpu", bitrate=25.0,
        fps=60.0)
    r.mavlink = FakeMavlink()
    r.recording = False; r.video = None; r.csv = None; r.base = None
    r.segment = 0; r.record_started_mono = None; r.record_started_wall = None
    r._last_gap_row = 0.0; r._start_suppressed = False; r.servo = None
    r.grabber = None; r.video_path = None
    r.zoom_entry = None; r.name_entry = None; r._name_base = None
    r._stop_armed = 0.0; r.keys = None; r.frames_seen = 0; r.frames_dropped = 0
    r.icr_mode = None; r.snapshots = 0
    return r


# -- debounce -------------------------------------------------------------
r = recorder()
t = 100.0
r.apply_record_switch(t, None, None)              # low, settles
r.apply_record_switch(t + 0.5, None, None)
r.mavlink.pwm = 1900                               # a single glitch high...
r.apply_record_switch(t + 0.6, None, None)
r.mavlink.pwm = 1000                               # ...and straight back
r.apply_record_switch(t + 0.65, None, None)
r.apply_record_switch(t + 1.2, None, None)
check("a 50 ms glitch high does not start a recording", not r.recording)

r.mavlink.pwm = 1900
r.apply_record_switch(t + 2.0, None, None)
check("a fresh switch-up is not acted on instantly", not r.recording)
r.apply_record_switch(t + 2.35, None, None)
check("held high for the debounce time, it starts", r.recording)

r.mavlink.pwm = 1000                               # glitch low mid-flight
r.apply_record_switch(t + 3.0, None, None)
r.mavlink.pwm = 1900
r.apply_record_switch(t + 3.05, None, None)
r.apply_record_switch(t + 3.5, None, None)
check("a glitch low does not stop it", r.recording)

r.mavlink.pwm = 1000
r.apply_record_switch(t + 4.0, None, None)
r.apply_record_switch(t + 4.4, None, None)
check("switched low and held, it stops", not r.recording)

# -- recording with no video ---------------------------------------------
check("that recording was telemetry-only, with a CSV on disk",
      any(n.endswith(".csv") for n in os.listdir(OUT)), str(os.listdir(OUT)))

# -- stale RC holds --------------------------------------------------------
r = recorder()
r.mavlink.pwm = 1900
r.apply_record_switch(10.0, None, None); r.apply_record_switch(10.4, None, None)
r.mavlink.rc_age = lambda: 9.0                    # transmitter lost
r.mavlink.pwm = 1000
r.apply_record_switch(11.0, None, None); r.apply_record_switch(12.0, None, None)
check("RC lost mid-flight holds the recording", r.recording)
r.stop_recording()

# -- write failure ---------------------------------------------------------
r = recorder()
r.mavlink.pwm = 1900
r.apply_record_switch(20.0, None, None); r.apply_record_switch(20.4, None, None)
try:
    r.on_write_failure(OSError(28, "No space left on device"))
    check("a write failure does not raise", True)
except Exception as exc:
    check("a write failure does not raise", False, repr(exc))
check("and stops the recording cleanly", not r.recording)
r.apply_record_switch(21.0, None, None); r.apply_record_switch(21.5, None, None)
check("no new recording starts onto a failing disk", not r.recording)

# -- recording by hand, no Pixhawk ----------------------------------------
r = recorder()
r.mavlink = None
r.request_start_recording()
check("r starts a recording with no flight controller", r.recording)
for i in range(10):
    r.apply_record_switch(30.0 + i, None, None)
check("and no flight controller does not stop it", r.recording)
r.handle_key("x"); r.handle_key("x")
check("x x stops it", not r.recording)
r.name_entry = None
r.handle_key("r")
check("r starts one", r.recording)
r.handle_key("r")
check("a single r while recording does not stop it", r.recording)
r.handle_key("r")
check("r r stops it", not r.recording)
r.name_entry = None

r = recorder()                                     # Pixhawk there, ch8 low
r.apply_record_switch(40.0, None, None); r.apply_record_switch(40.5, None, None)
r.handle_key("r")
r.apply_record_switch(41.0, None, None); r.apply_record_switch(42.0, None, None)
check("a ch8 merely sitting low does not stop a recording started by hand",
      r.recording)
r.mavlink.pwm = 1900
r.apply_record_switch(43.0, None, None); r.apply_record_switch(43.5, None, None)
r.mavlink.pwm = 1000
r.apply_record_switch(44.0, None, None); r.apply_record_switch(44.5, None, None)
check("ch8 flicked up and back down does stop it", not r.recording)
r.apply_record_switch(45.0, None, None)
check("and the switch is in charge again afterwards", not r.recording)

r = recorder(); r.mavlink = None; r._write_failed = True
r.request_start_recording()
check("r does not start onto a failing disk", not r.recording)

r = recorder(); r.mavlink = None; r.args.no_mavlink = True
check("--no-mavlink drops the NO FC warning from the status",
      "NO FC" not in r.status_text(50.0), r.status_text(50.0))
r.args.no_mavlink = False
check("without it the status says how to record",
      "NO FC -- r TO RECORD" in r.status_text(50.0), r.status_text(50.0))

# -- a late autopilot ------------------------------------------------------
r = fr.Recorder.__new__(fr.Recorder)
r.args = types.SimpleNamespace(stream_rate=10, rc_channel=7, rec_channel=8,
                               rc_pwm_min=1000, rc_pwm_max=2000, rc_deadband=8,
                               rc_reverse=False, rc_timeout=2.0, snap_channel=0,
                               icr_channel=0)
r.mavlink = None; r.servo = types.SimpleNamespace(rc=None)
made = []
class FakeSource:
    def __init__(self, **kw): made.append(kw)
orig = fr.MavlinkSource
fr.MavlinkSource = FakeSource
try:
    r._attach_mavlink("/dev/serial/by-id/usb-CubePilot", 115200)
finally:
    fr.MavlinkSource = orig
check("a late autopilot is attached", r.mavlink is not None and made)
check("and the zoom servo gets its knob", r.servo.rc is r.rc and r.rc is not None)

# -- a dead terminal -------------------------------------------------------
class Dead(io.StringIO):
    def isatty(self): return True
    def write(self, s): raise OSError(5, "Input/output error")
line = fr.LiveLine(Dead())
try:
    line.show("status"); line.clear(); line.show("again")
    check("a dead terminal does not raise from the status line", True)
except Exception as exc:
    check("a dead terminal does not raise from the status line", False, repr(exc))
check("and the status line gives up on it", not line.enabled)

# -- frames lost on the link are filled, keeping the video in real time ----
class FakeVideo:
    def __init__(self, fps): self.fps, self.frames, self.written = fps, 0, []
    def write(self, f): self.written.append(f); self.frames += 1
class FakeSheet:
    def __init__(self): self.rows = []
    def write(self, n, wall, t, tel, servo, now, filled=0): self.rows.append((n, filled))
r = recorder()
r.record_started_mono = 0.0
from datetime import datetime, timezone
r.record_started_wall = datetime.now(timezone.utc)
video, sheet = FakeVideo(60.0), FakeSheet()
for i, t in enumerate([0.0, 1/60, 2/60, 6/60, 7/60]):     # 3 frames lost before 6/60
    r.write_frame(video, sheet, f"frame{i}", t, t)
check("lost frames are filled so the video keeps real time", video.frames == 8,
      f"{video.frames} frames written")
check("filled frames repeat the last real one", video.written[3:6] == ["frame2"] * 3,
      str(video.written))
check("the CSV marks exactly the filled rows", [f for _, f in sheet.rows] == [0, 0, 0, 1, 1, 1, 0, 0],
      str(sheet.rows))
check("CSV frame numbers stay in step with the video",
      [n for n, _ in sheet.rows] == list(range(8)))
video, sheet = FakeVideo(60.0), FakeSheet()
r._last_written = None; r._last_written_at = None
r.write_frame(video, sheet, "a", 10.0, 10.0)
r.write_frame(video, sheet, "b", 20.0, 20.0)                # a 10 s outage
check("a long outage is not papered over with a still", video.frames <= 2 + 3,
      f"{video.frames} frames")

video, sheet = FakeVideo(60.0), FakeSheet()
r._last_written = None; r._last_written_at = None
r.write_frame(video, sheet, "a", 0.0, 0.0)
r.write_frame(video, sheet, "b", 5/60, 5/60, seq_gap=5)       # this loop was slow
check("frames the loop was too slow to collect are not filled (the spiral)",
      video.frames == 2, f"{video.frames} frames")

video, sheet = FakeVideo(60.0), FakeSheet()
r._last_written = None; r._last_written_at = None
t = 0.0
for dt in [0.0104, 0.0104, 0.0125, 0.0271, 0.0104, 0.0208, 0.0167] * 20:
    t += dt
    r.write_frame(video, sheet, "f", t, t)
check("the board's normal arrival jitter fills nothing",
      sum(1 for _, f in sheet.rows if f) == 0)

# -- a stuck board is power-cycled when a command is set, at most every 90 s
marker = os.path.join(OUT, "cycled")
r = recorder()
r.args.board_power_cycle_cmd = f"echo x >> {marker}"
r.board_is_stuck("test"); r.board_is_stuck("test again")
check("a stuck board is power-cycled once, not hammered",
      os.path.exists(marker) and open(marker).read().count("x") == 1)
r = recorder()
r.args.board_power_cycle_cmd = ""
try:
    r.board_is_stuck("no command"); r.board_is_stuck("no command")
    check("with no command it only says so", r._board_advice_given)
except Exception as exc:
    check("with no command it only says so", False, repr(exc))

sys.exit(1 if fails else 0)
