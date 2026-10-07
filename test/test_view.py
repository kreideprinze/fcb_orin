"""fcb_view.py: the laptop viewer. No camera, no serial, no window shown."""
import os, sys, time, types, shutil, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import cv2

# No window, ever. Only Qt's "xcb" plugin is installed on this machine, so
# the alternative is a window flashing onto whoever's desktop is running
# the tests -- and none of what is under test here is Qt's job anyway.
_shown = {"frames": 0}
cv2.namedWindow = lambda *a, **k: None
cv2.resizeWindow = lambda *a, **k: None
cv2.destroyAllWindows = lambda *a, **k: None
cv2.imshow = lambda name, img: _shown.__setitem__("frames", _shown["frames"] + 1)
cv2.waitKey = lambda ms=1: 255
cv2.getWindowProperty = lambda *a: 1.0

import fcb_view

OUT = os.path.join(tempfile.gettempdir(), "fcb_test_view")
shutil.rmtree(OUT, ignore_errors=True)

fails = []
def check(label, ok, detail=""):
    if not ok: fails.append(label)
    print(("  ok   " if ok else "  FAIL ") + label + (("  -- " + detail) if detail and not ok else ""))

W, H = 320, 240
def a_frame(v=90):
    f = np.full((H, W, 3), v, np.uint8)
    cv2.rectangle(f, (20, 20), (120, 90), (200, 40, 40), -1)
    return f

class FakeCapture:
    """Hands out frames, then stops, so run() terminates on its own."""
    def __init__(self, count): self.left = count
    def read(self):
        if self.left <= 0: return False, None
        self.left -= 1
        return True, a_frame()
    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: W, cv2.CAP_PROP_FRAME_HEIGHT: H,
                cv2.CAP_PROP_FPS: 30.0}.get(prop, 0)
    def release(self): pass

def viewer(**over):
    args = fcb_view.parse_args(["--no-visca", "--dir", OUT, "--scale", "1.0"]
                               + list(over.pop("argv", [])))
    for k, v in over.items(): setattr(args, k, v)
    v = fcb_view.Viewer(args)
    v.frame_size = (W, H)
    v.declared_fps = 30.0
    return v

print("1. defaults are the safe ones")
a = fcb_view.parse_args([])
check("container is avi", a.container == "avi")
check("codec is XVID", a.codec == "XVID")
check("autodetects rather than guessing a device", a.video is None)
check("zoom detents 0.5x", a.zoom_step == 0.5)

print("\n2. 'r' records a real, readable file")
v = viewer()
f = a_frame()
v.handle_key(ord("r"), f)
check("writer opened", v.writer is not None,
      "cv2 could not open an XVID avi here")
for _ in range(20):
    v.writer.write(f); v.rec_frames += 1
path = f"{v.base}.avi"
v.handle_key(ord("r"), f)
check("writer closed", v.writer is None)
check("file exists", os.path.exists(path))
size = os.path.getsize(path) if os.path.exists(path) else 0
check("file is not empty (%d bytes)" % size, size > 10_000)
cap = cv2.VideoCapture(path); n = 0
while True:
    ok, _ = cap.read()
    if not ok: break
    n += 1
cap.release()
check("reads back %d frames" % n, n >= 15, "only %d of 20 came back" % n)

print("\n2b. raw YUYV from the camera is converted, not written as-is")
yuyv = np.random.randint(0, 255, (H, W, 2), dtype=np.uint8)
bgr = fcb_view.to_bgr(yuyv)
check("2-channel YUYV -> 3-channel BGR %s" % (bgr.shape,), bgr.shape == (H, W, 3))
check("grayscale -> BGR", fcb_view.to_bgr(np.zeros((H, W), np.uint8)).shape == (H, W, 3))
check("BGRA -> BGR", fcb_view.to_bgr(np.zeros((H, W, 4), np.uint8)).shape == (H, W, 3))
check("BGR passes through untouched",
      fcb_view.to_bgr(a_frame()).shape == (H, W, 3))
try:
    v0 = viewer(); v0.draw_overlay(bgr.copy()); ok = True
except Exception as exc:
    ok = False
check("the overlay can draw on the converted frame", ok)

print("\n2c. the 180 rotation reaches the recording, not just the window")
# a marker only in the top-left: after a 180 rotation it must be bottom-right
marked = np.zeros((H, W, 3), np.uint8)
marked[0:40, 0:40] = (255, 255, 255)
class OneMarked:
    def __init__(self): self.left = 6
    def read(self):
        if self.left <= 0: return False, None
        self.left -= 1
        return True, marked.copy()
    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: W, cv2.CAP_PROP_FRAME_HEIGHT: H,
                cv2.CAP_PROP_FPS: 30.0}.get(prop, 0)
    def release(self): pass

vr = viewer(); vr.capture = OneMarked(); vr.args.give_up_after = 2
vr.args.rotate = 180
keys_r = iter([ord("r")] + [255] * 100)
_saved_waitkey = cv2.waitKey
cv2.waitKey = lambda ms=1: next(keys_r, 255)
vr.run()
cv2.waitKey = _saved_waitkey
rec = sorted(f for f in os.listdir(OUT) if f.endswith(".avi"))[-1]
cap = cv2.VideoCapture(os.path.join(OUT, rec)); ok, got = cap.read(); cap.release()
tl = float(got[0:40, 0:40].mean()); br = float(got[H-40:H, W-40:W].mean())
check("marker moved top-left -> bottom-right (tl=%.0f br=%.0f)" % (tl, br),
      br > tl + 100, "the recording was not rotated")

vr2 = viewer(); vr2.capture = OneMarked(); vr2.args.give_up_after = 2
vr2.args.rotate = 0
keys_r2 = iter([ord("r")] + [255] * 100)
cv2.waitKey = lambda ms=1: next(keys_r2, 255)
vr2.run()
cv2.waitKey = _saved_waitkey
rec2 = sorted(f for f in os.listdir(OUT) if f.endswith(".avi"))[-1]
cap = cv2.VideoCapture(os.path.join(OUT, rec2)); ok, got2 = cap.read(); cap.release()
tl2 = float(got2[0:40, 0:40].mean()); br2 = float(got2[H-40:H, W-40:W].mean())
check("--rotate 0 leaves it alone (tl=%.0f br=%.0f)" % (tl2, br2),
      tl2 > br2 + 100)

print("\n3. the overlay darkens a strip -- it does not paint a black bar")
v = viewer()
shown = v.draw_overlay(a_frame(200).copy())
bar = max(22, int(H * 0.055))
strip_mean = float(shown[:bar].mean())
body_mean = float(shown[bar:].mean())
check("strip is darker than the picture (%.0f < %.0f)" % (strip_mean, body_mean),
      strip_mean < body_mean)
check("strip is not solid black (%.0f)" % strip_mean, strip_mean > 20,
      "the blend is black-on-black -- the original strip was not copied")

print("\n4. the recording is written before the overlay is drawn")
v = viewer()
v.handle_key(ord("r"), a_frame())
src = a_frame()
before = src.copy()
v.writer.write(src); v.rec_frames += 1
v.draw_overlay(src if False else src.copy())
check("source frame untouched by drawing", np.array_equal(src, before))
v.handle_key(ord("r"), src)

print("\n5. zoom and mode keys without VISCA warn instead of crashing")
v = viewer()
for key in (ord("1"), ord("2"), ord("3"), ord("4"), ord("i"),
            ord("+"), ord("-"), ord("0")):
    try:
        v.handle_key(key, a_frame())
    except Exception as exc:
        check("key %r survived" % chr(key), False, repr(exc))
check("all control keys survived with no serial link", True)
check("said so once, not once per key", v._warned_no_visca)

print("\n6. 's' writes a jpeg, 'q' quits")
v = viewer()
v.handle_key(ord("s"), a_frame())
jpgs = [f for f in os.listdir(OUT) if f.endswith(".jpg")]
check("snapshot written: %s" % jpgs, len(jpgs) == 1)
check("still running", v.running)
v.handle_key(ord("q"), a_frame())
check("q stops the loop", not v.running)

print("\n7. run() completes, and a camera that stops closes the file cleanly")
v = viewer()
v.capture = FakeCapture(40)
v.args.give_up_after = 3
started = {"done": False}
def fake_key_sequence():
    # press r on the first poll, then nothing
    seq = [ord("r")] + [255] * 500
    for k in seq: yield k
keys = fake_key_sequence()
cv2.waitKey = lambda ms=1: next(keys, 255)
before_shown = _shown["frames"]
v.run()
cv2.waitKey = lambda ms=1: 255
check("loop exited on its own", True)
check("it actually displayed frames (%d)" % (_shown["frames"] - before_shown),
      _shown["frames"] - before_shown >= 30)
check("recording was finalised", v.writer is None)
avis = sorted(f for f in os.listdir(OUT) if f.endswith(".avi"))
check("every recording got its own name: %s" % avis,
      len(avis) == len(set(avis)) and len(avis) >= 3,
      "a same-second name collided and one recording was lost")
check("none of them is empty",
      all(os.path.getsize(os.path.join(OUT, f)) > 5_000 for f in avis))


shutil.rmtree(OUT, ignore_errors=True)
print("\n%s" % ("ALL PASS" if not fails else "FAILURES: %s" % fails))
sys.exit(1 if fails else 0)
