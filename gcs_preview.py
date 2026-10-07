#!/usr/bin/env python3
"""The last snapshot, in its own pane.

    ./gcs_preview.py [snapshot-dir] [poll-seconds]

The recorder already writes a JPEG for every ch9 press, so this needs no
cooperation from it at all: it watches that directory and draws whatever
arrived most recently. That keeps the pane a pure add-on -- it can be
killed, restarted, or resized at any point in a flight without the
recorder noticing, and a snapshot taken while it was dead still appears
the moment it comes back.

Redraws on a new file *or* a pane resize, and otherwise sits idle, so an
unattended pane costs one directory listing per poll.
"""
import os
import shutil
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                    # noqa: E402
import prefer_cv2  # noqa: E402,F401  -- before cv2
import cv2                                            # noqa: E402
import snapshot                                       # noqa: E402

# Quadrant block elements, indexed by which of the cell's four sub-pixels
# belong to the foreground colour: bit 0 top-left, 1 top-right, 2 bottom-
# left, 3 bottom-right. Every one of these is in the Block Elements range
# that has been in essentially every monospace font for decades, so this
# needs no special font -- unlike the sextant characters that would give
# another 50% vertically.
QUADRANTS = [
    " ", "\u2598", "\u259d", "\u2580", "\u2596", "\u258c", "\u259e", "\u259b",
    "\u2597", "\u259a", "\u2590", "\u259c", "\u2584", "\u2599", "\u259f", "\u2588",
]

#: Rec. 601 luma weights on BGR, for splitting a cell's four sub-pixels
#: into a light group and a dark one.
_LUMA = np.array([0.114, 0.587, 0.299], dtype=np.float32)


def to_ansi_quadrant(frame, cols, rows, quantum=None):
    """Render a frame at two sub-pixels per cell horizontally and two
    vertically, instead of the one-by-two a half block gives.

    A cell can only hold two colours, so the four sub-pixels are split by
    luma into a light group and a dark one and each group is averaged.
    That trades a little colour fidelity on smooth gradients for twice the
    horizontal detail, which on a camera frame is the better bargain --
    edges, horizons and the shape of things on the ground are what you are
    looking for in a preview, and those are exactly what extra spatial
    resolution recovers.
    """
    quantum = snapshot.COLOR_QUANTUM if quantum is None else quantum
    frame = snapshot._as_bgr(frame)
    small = cv2.resize(frame, (cols * 2, rows * 2), interpolation=cv2.INTER_AREA)
    if quantum > 1:
        small = (small // quantum) * quantum

    # (rows, 2, cols, 2, 3) -> one 2x2x3 block per cell.
    cells = small.reshape(rows, 2, cols, 2, 3).transpose(0, 2, 1, 3, 4)
    cells = cells.reshape(rows, cols, 4, 3).astype(np.float32)

    luma = cells @ _LUMA                                  # (rows, cols, 4)
    # Split at the cell's own mean rather than a fixed threshold: what
    # matters is the contrast within the cell, not its absolute brightness.
    light = luma > luma.mean(axis=2, keepdims=True)

    n_light = light.sum(axis=2)
    n_dark = 4 - n_light
    # A flat cell has no light group at all. Divide by a floor of 1 to keep
    # numpy quiet, then fix those cells up by using the dark mean for both.
    fg = (cells * light[..., None]).sum(axis=2) / np.maximum(n_light, 1)[..., None]
    bg = (cells * ~light[..., None]).sum(axis=2) / np.maximum(n_dark, 1)[..., None]
    flat = n_light == 0
    fg[flat] = bg[flat]

    bits = (light * np.array([1, 2, 4, 8])).sum(axis=2)

    fg = fg.astype(np.uint8).tolist()
    bg = bg.astype(np.uint8).tolist()
    bits = bits.tolist()

    out = []
    for row in range(rows):
        frow, brow, birow = fg[row], bg[row], bits[row]
        last_fg = last_bg = None
        line = []
        for col in range(cols):
            b, g, r = frow[col]
            f = (r, g, b)
            b2, g2, r2 = brow[col]
            k = (r2, g2, b2)
            # Only re-state the colours that changed, as the half-block
            # renderer does. Runs are shorter here because a cell now
            # carries two colours, but it still saves most of the bytes.
            if f != last_fg and k != last_bg:
                line.append("\x1b[38;2;%d;%d;%d;48;2;%d;%d;%dm" % (f + k))
            elif f != last_fg:
                line.append("\x1b[38;2;%d;%d;%dm" % f)
            elif k != last_bg:
                line.append("\x1b[48;2;%d;%d;%dm" % k)
            line.append(QUADRANTS[birow[col]])
            last_fg, last_bg = f, k
        line.append("\x1b[0m")
        out.append("".join(line))
    return "\n".join(out)

DEFAULT_DIR = os.path.join(os.path.expanduser(
    os.environ.get("FCB_RECORD_DIR", "~/flight_recordings")), "snapshots")

#: Set FCB_PREVIEW_HALFBLOCK=1 if a terminal or font renders the quadrant
#: characters badly and the plainer one-pixel-per-half-cell look is wanted.
HALF_BLOCK_ONLY = os.environ.get("FCB_PREVIEW_HALFBLOCK", "") not in ("", "0")

AMBER = "\033[38;5;214m"
DIM = "\033[38;5;240m"
LABEL = "\033[38;5;245m"
VALUE = "\033[38;5;252m"
BOLD = "\033[1m"
OFF = "\033[0m"


def newest(directory):
    """Path and mtime of the most recent JPEG, or (None, None)."""
    try:
        names = [n for n in os.listdir(directory) if n.endswith(".jpg")]
    except OSError:
        return None, None
    if not names:
        return None, None
    best = max(names, key=lambda n: os.path.getmtime(os.path.join(directory, n)))
    path = os.path.join(directory, best)
    try:
        return path, os.path.getmtime(path)
    except OSError:
        return None, None


def draw_waiting(directory, cols):
    sys.stdout.write("\033[H\033[2J")
    sys.stdout.write(f"\n  {LABEL}No snapshot yet.{OFF}\n\n")
    sys.stdout.write(f"  {LABEL}Press {VALUE}ch9{LABEL} on the transmitter, or "
                     f"{VALUE}s{LABEL} in the log pane.{OFF}\n\n")
    sys.stdout.write(f"  {DIM}watching {directory}{OFF}\n")
    sys.stdout.flush()


def draw(path, cols, rows):
    frame = cv2.imread(path)
    if frame is None:
        sys.stdout.write("\033[H\033[2J")
        sys.stdout.write(f"\n  {LABEL}could not read {os.path.basename(path)}{OFF}\n")
        sys.stdout.flush()
        return

    # Two lines of caption, one of margin: the picture is what matters, but
    # a picture with no time on it is worth much less when you are trying
    # to work out which pass over the target it came from.
    c, r = snapshot.fit(frame.shape, cols - 1, max(4, rows - 3))
    if HALF_BLOCK_ONLY:
        text = snapshot.to_ansi(frame, c, r)
    else:
        text = to_ansi_quadrant(frame, c, r)

    stamp = datetime.fromtimestamp(os.path.getmtime(path)).strftime("%H:%M:%S")
    height, width = frame.shape[0], frame.shape[1]
    size_kb = os.path.getsize(path) / 1024.0

    out = ["\033[H\033[2J"]
    out.append(f" {AMBER}{BOLD}LAST SNAPSHOT{OFF}  {LABEL}{stamp}{OFF}"
               f"  {DIM}{width}x{height}  {size_kb:.0f} KB{OFF}\n")
    out.append(text + "\n")
    out.append(f" {DIM}{os.path.basename(path)}{OFF}\n")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    poll = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    sys.stdout.write("\033[?25l")                     # hide the cursor
    shown = None
    shown_size = None
    try:
        while True:
            path, mtime = newest(directory)
            size = shutil.get_terminal_size((80, 24))
            key = (path, mtime)
            if key != shown or size != shown_size:
                try:
                    if path is None:
                        draw_waiting(directory, size.columns)
                    else:
                        draw(path, size.columns, size.lines)
                except Exception as exc:                # a bad file must not
                    sys.stdout.write(f"\n  preview failed: {exc}\n")  # end the pane
                    sys.stdout.flush()
                shown, shown_size = key, size
            time.sleep(poll)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\033[?25h")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
