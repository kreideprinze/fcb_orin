"""Naming a finished flight from the tmux pane.

A timestamp is a fine filename and a poor label: ten minutes after
landing nobody remembers which of five files was the good pass. So the
stop key asks, and whatever is typed renames every file of that session --
all of it, or none of it.
"""
import sys, os, types, tempfile, shutil, logging
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fcb_record as fr

logging.basicConfig(level=logging.INFO, format="  %(levelname)-7s %(message)s",
                    stream=sys.stdout)
OUT = os.path.join(tempfile.gettempdir(), "fcb_test_naming")

fails = []
def check(label, ok, detail=""):
    if not ok: fails.append(label)
    print(("  ok   " if ok else "  FAIL ") + label + (("  -- " + detail) if detail and not ok else ""))

class Keys:
    enabled = True

def fresh(segments=1, extra=()):
    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT)
    base = os.path.join(OUT, "fcb-20260909-120000")
    open(base + ".csv", "w").write("frame\n")
    open(base + ".mp4", "w").write("v")
    for i in range(2, segments + 1):
        open(f"{base}-{i}.mp4", "w").write("v")
    for name in extra:
        open(os.path.join(OUT, name), "w").write("x")
    r = fr.Recorder.__new__(fr.Recorder)
    r.args = types.SimpleNamespace(record_dir=OUT, rec_channel=8)
    r.base = base; r.keys = Keys(); r.name_entry = None; r._name_base = None
    r.recording = False
    return r, base

def typed(r, text):
    for ch in text:
        r.handle_name_entry(ch)

print("1. a plain name renames every file of the session")
r, base = fresh(segments=2)
r.begin_name_entry(base)
check("prompt opened", r.name_entry == "")
typed(r, "morning pass 3\r")
files = sorted(os.listdir(OUT))
check("spaces become underscores, all 3 renamed: %s" % files,
      files == ["morning_pass_3-2.mp4", "morning_pass_3.csv", "morning_pass_3.mp4"])
check("prompt closed", r.name_entry is None)

print("\n2. empty Enter keeps the timestamp name")
r, base = fresh()
r.begin_name_entry(base)
typed(r, "\r")
check("untouched", sorted(os.listdir(OUT)) ==
      ["fcb-20260909-120000.csv", "fcb-20260909-120000.mp4"])

print("\n3. backspace edits")
r, base = fresh()
r.begin_name_entry(base)
typed(r, "abcX")
typed(r, "\x7f")
check("buffer is 'abc'", r.name_entry == "abc", "got %r" % r.name_entry)
typed(r, "\r")
check("renamed to abc", "abc.mp4" in os.listdir(OUT))

print("\n4. a name that is only junk is refused, files kept")
r, base = fresh()
r.begin_name_entry(base)
typed(r, "!!!\r")
check("originals still there", "fcb-20260909-120000.mp4" in os.listdir(OUT))

print("\n5. path traversal cannot escape the recordings directory")
r, base = fresh()
r.begin_name_entry(base)
typed(r, "../../etc/passwd\r")
check("stayed put as 'passwd'", "passwd.mp4" in os.listdir(OUT))
check("nothing escaped", not os.path.exists("/tmp/etc"))

print("\n6. a taken name is disambiguated, never overwritten")
r, base = fresh(segments=2, extra=("sortie.mp4", "sortie.csv"))
before = open(os.path.join(OUT, "sortie.mp4")).read()
r.begin_name_entry(base)
typed(r, "sortie\r")
names = sorted(os.listdir(OUT))
check("existing file untouched",
      open(os.path.join(OUT, "sortie.mp4")).read() == before)
check("new session took sortie-2: %s" % names,
      "sortie-2.mp4" in names and "sortie-2.csv" in names and
      "sortie-2-2.mp4" in names)

print("\n7. no prompt when there is no terminal to ask on")
r, base = fresh()
r.keys = None
r.begin_name_entry(base)
check("stays silent", r.name_entry is None)

print("\n8. unrelated files sharing the directory are not swept up")
r, base = fresh(extra=("fcb-20260909-120000-notes.txt", "other.mp4"))
r.begin_name_entry(base)
typed(r, "flight\r")
left = sorted(os.listdir(OUT))
check("only the session moved: %s" % left,
      "fcb-20260909-120000-notes.txt" in left and "other.mp4" in left
      and "flight.mp4" in left and "flight.csv" in left)

shutil.rmtree(OUT, ignore_errors=True)
print("\n%s" % ("ALL PASS" if not fails else "FAILURES: %s" % fails))
sys.exit(1 if fails else 0)
