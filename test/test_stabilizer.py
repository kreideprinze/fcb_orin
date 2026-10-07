"""The image stabilizer is switched on whenever the camera is connected.

The setting lives in the camera and is lost when it loses power -- after
the Oppila board's 12 V power cycles the camera read back "off", and the
feed went unstabilised because this recorder never set it (the Twiga
NeoHD driver always had). So every new control link sets it and reads it
back, and a camera that will not cooperate is reported, not fatal.
"""
import sys, os, types, logging
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fcb_record as fr
from fcb_base_driver import visca

logging.basicConfig(level=logging.INFO, format="  %(levelname)-7s %(message)s",
                    stream=sys.stdout)

fails = []
def check(label, ok, detail=""):
    if not ok: fails.append(label)
    print(("  ok   " if ok else "  FAIL ") + label + (("  -- " + detail) if detail and not ok else ""))


class FakeLink:
    """A camera whose stabilizer starts off, as it does after power-up."""

    address = 1

    def __init__(self, ignore_commands=False, dead=False):
        self.state = 0x03                       # off
        self.ignore_commands = ignore_commands
        self.dead = dead
        self.sent = []

    def command(self, packet, **kwargs):
        if self.dead:
            raise fr.ViscaTimeout("no complete frame")
        self.sent.append(packet)
        if packet[:4] == bytes([0x81, 0x01, 0x04, 0x34]) and not self.ignore_commands:
            self.state = packet[4]

    def inquiry(self, packet, **kwargs):
        if self.dead:
            raise fr.ViscaTimeout("no complete frame")
        assert packet == visca.stabilizer_inq(1), packet.hex()
        return bytes([0x50, self.state])


def recorder(stabilizer):
    r = fr.Recorder.__new__(fr.Recorder)
    r.args = types.SimpleNamespace(stabilizer=stabilizer)
    return r


link = FakeLink()
recorder("on").apply_stabilizer(link)
check("default 'on' sends CAM_Stabilizer On",
      visca.image_stabilizer(True) in link.sent, [p.hex() for p in link.sent])
check("and the camera ends up stabilised", link.state == 0x02)

link = FakeLink(); link.state = 0x02
recorder("off").apply_stabilizer(link)
check("'off' switches it off", link.state == 0x03)

link = FakeLink()
recorder("keep").apply_stabilizer(link)
check("'keep' sends no command, only asks", link.sent == [], [p.hex() for p in link.sent])
check("'keep' leaves it as found", link.state == 0x03)

link = FakeLink(ignore_commands=True)
try:
    recorder("on").apply_stabilizer(link)
    check("a camera that acknowledges but does not switch is not fatal", True)
except Exception as exc:
    check("a camera that acknowledges but does not switch is not fatal", False, repr(exc))

try:
    recorder("on").apply_stabilizer(FakeLink(dead=True))
    check("a camera that does not answer is not fatal", True)
except Exception as exc:
    check("a camera that does not answer is not fatal", False, repr(exc))

r = fr.Recorder.__new__(fr.Recorder)
r.args = types.SimpleNamespace()
link = FakeLink()
r.apply_stabilizer(link)
check("args without the option (older callers) still default to on", link.state == 0x02)

sys.exit(1 if fails else 0)
