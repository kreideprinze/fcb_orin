"""A closing tmux pane must not wedge the capture loop.

When the pane (or the window it lived in) goes away, stdin hits EOF. A
closed descriptor is reported *readable* by select() for ever and every
read returns nothing, so draining on pending() spins at full tilt --
measured at over a million iterations in three seconds -- and the capture
loop never turns again. Recording stops as surely as if the process had
crashed, which is what this guards.
"""
import os, pty, sys, threading, time, types, logging, tty
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fcb_record as fr

logging.basicConfig(level=logging.INFO, format="  %(levelname)-7s %(message)s",
                    stream=sys.stdout)
fails = []
def check(label, ok, detail=""):
    if not ok: fails.append(label)
    print(("  ok   " if ok else "  FAIL ") + label + (("  -- " + detail) if detail and not ok else ""))


def keys_on(fd):
    """A TerminalKeys bound to an arbitrary fd, without touching stdin.

    cbreak matters: a pty defaults to canonical mode, where the line
    discipline holds every byte until a newline, so single keypresses
    would never arrive. The real TerminalKeys sets this in __enter__.
    """
    try:
        tty.setcbreak(fd)
    except Exception:
        pass
    k = fr.TerminalKeys.__new__(fr.TerminalKeys)
    k._fd = fd
    k.enabled = True
    return k


print("1. a live terminal still delivers keys")
master, slave = pty.openpty()
k = keys_on(slave)
os.write(master, b"2")
time.sleep(0.05)
check("key read back", k.get() == "2")
check("still enabled", k.enabled)

print("\n2. EOF retires the keyboard instead of spinning")
os.close(master)
time.sleep(0.05)
got = k.get()
check("get() returns None", got is None, "got %r" % (got,))
check("keyboard disabled", not k.enabled, "still enabled -- pending() will spin")
check("pending() is now False", not k.pending(),
      "pending() still True: the drain loop cannot terminate")

print("\n3. the drain loop terminates")
spins = {"n": 0}
done = threading.Event()
def drain():
    while k.pending() and spins["n"] < 2_000_000:
        k.get(); spins["n"] += 1
    done.set()
threading.Thread(target=drain, daemon=True).start()
done.wait(timeout=3.0)
check("drain finished (%d spins)" % spins["n"], done.is_set() and spins["n"] < 10,
      "wedged after %d iterations" % spins["n"])

print("\n4. poll_keys survives it and the loop keeps turning")
master2, slave2 = pty.openpty()
r = fr.Recorder.__new__(fr.Recorder)
r.keys = keys_on(slave2)
r.zoom_entry = None; r.name_entry = None
os.close(master2)
t0 = time.monotonic()
r.poll_keys()
elapsed = time.monotonic() - t0
check("poll_keys returned promptly (%.3fs)" % elapsed, elapsed < 1.0,
      "it blocked, which is the capture loop stalling")
r.poll_keys()   # and again, now that it is retired
check("keyboard stays retired", not r.keys.enabled)

print("\n5. an unbounded burst cannot hold the loop either")
master3, slave3 = pty.openpty()
r2 = fr.Recorder.__new__(fr.Recorder)
r2.keys = keys_on(slave3)
r2.zoom_entry = None; r2.name_entry = None
seen = []
r2.handle_key = lambda key: seen.append(key)
os.write(master3, b"1" * 4000)
time.sleep(0.05)
t0 = time.monotonic()
r2.poll_keys()
elapsed = time.monotonic() - t0
check("one poll is bounded (%d keys, %.3fs)" % (len(seen), elapsed),
      len(seen) <= 64 and elapsed < 1.0)
os.close(master3)

print("\n%s" % ("ALL PASS" if not fails else "FAILURES: %s" % fails))
sys.exit(1 if fails else 0)
