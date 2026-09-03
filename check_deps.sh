#!/usr/bin/env bash
# Check everything fcb_record.py needs, before a flight rather than during.
#
# Exits non-zero only for things that stop it running at all. Anything that
# merely degrades it -- no NVENC, no v4l2-ctl -- is reported as a warning,
# because a CPU-encoded recording still beats no recording.
set -uo pipefail

PYTHON="${FCB_PYTHON:-python3}"
FAIL=0
WARN=0

ok()   { printf '  \033[32mok\033[0m    %s\n' "$1"; }
warn() { printf '  \033[33mwarn\033[0m  %s\n' "$1"; WARN=$((WARN+1)); }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FAIL=$((FAIL+1)); }

echo "checking dependencies with $PYTHON ($($PYTHON -V 2>&1))"

# -- python packages ---------------------------------------------------
if $PYTHON -c 'import cv2' 2>/dev/null; then
    ok "opencv $($PYTHON -c 'import cv2; print(cv2.__version__)' 2>/dev/null)"
else
    bad "opencv (cv2) not importable
        On a Jetson, OpenCV comes from JetPack and lives in the system
        python. If you are inside a virtualenv it will be hidden unless the
        venv was made with --system-site-packages. Either run outside the
        venv, recreate it with that flag, or set FCB_PYTHON=/usr/bin/python3."
fi

if $PYTHON -c 'import pymavlink' 2>/dev/null; then
    ok "pymavlink $($PYTHON -c 'import pymavlink; print(pymavlink.__version__)' 2>/dev/null)"
else
    bad "pymavlink not importable -- pip3 install pymavlink
        (or: sudo apt install python3-pymavlink)"
fi

if $PYTHON -c 'import serial' 2>/dev/null; then
    ok "pyserial $($PYTHON -c 'import serial; print(serial.__version__)' 2>/dev/null)"
else
    bad "pyserial not importable -- pip3 install pyserial
        (or: sudo apt install python3-serial)"
fi

# -- hardware encoding -------------------------------------------------
# The camera only offers raw 1080p60 YUYV, about 250 MB/s. Software
# encoding that on an Orin drops frames, so NVENC is what makes the
# default settings work.
if $PYTHON -c 'import cv2' 2>/dev/null; then
    if $PYTHON -c 'import cv2,sys; sys.exit(0 if "GStreamer:                   YES" in cv2.getBuildInformation() or "GStreamer:" in cv2.getBuildInformation() and "YES" in [l for l in cv2.getBuildInformation().splitlines() if "GStreamer:" in l][0] else 1)' 2>/dev/null; then
        ok "opencv has GStreamer (NVENC path available)"
    else
        warn "opencv was built WITHOUT GStreamer -- NVENC is unavailable and
        the recorder will fall back to CPU mp4v, which will drop frames at
        1080p60. On a Jetson this usually means you are using a pip-installed
        opencv instead of JetPack's. Check with:
          $PYTHON -c 'import cv2; print(cv2.getBuildInformation())' | grep -i gstreamer"
    fi
fi

if command -v gst-inspect-1.0 >/dev/null 2>&1; then
    if gst-inspect-1.0 nvv4l2h264enc >/dev/null 2>&1; then
        ok "nvv4l2h264enc (Jetson hardware H.264 encoder)"
    else
        # nvidia-l4t-multimedia is not the one that ships the GStreamer
        # elements -- nvidia-l4t-gstreamer is, and JetPack does not always
        # pull it in. Without it there is no nvv4l2h264enc and no nvvidconv,
        # which is the whole NVENC pipeline.
        hint="CPU encoding will be used, which drops frames at 1080p60."
        # Captured rather than piped into grep: with pipefail set, grep -q
        # closes the pipe on its first match, the producer dies of SIGPIPE,
        # and the pipeline reports failure even though the pattern matched.
        policy="$(apt-cache policy nvidia-l4t-gstreamer 2>/dev/null || true)"
        if [[ "$policy" == *"Installed: (none)"* ]]; then
            hint="install it with:
          sudo apt install nvidia-l4t-gstreamer
        then re-run this check."
        fi
        warn "nvv4l2h264enc is missing -- the Jetson GStreamer plugins are not
        installed. $hint"
    fi
else
    warn "gst-inspect-1.0 not found; cannot confirm the hardware encoder.
        sudo apt install gstreamer1.0-tools"
fi

# -- tools and permissions --------------------------------------------
command -v v4l2-ctl >/dev/null 2>&1 \
    && ok "v4l2-ctl" \
    || warn "v4l2-ctl missing -- the camera is identified by name with it.
        Without it the recorder cannot tell the FCB from another camera.
        sudo apt install v4l-utils"

command -v tmux >/dev/null 2>&1 \
    && ok "tmux $(tmux -V | awk '{print $2}')" \
    || bad "tmux missing -- sudo apt install tmux"

if [[ $EUID -eq 0 ]]; then
    # Group membership is irrelevant to root, so checking it would only
    # produce a scary and useless "add root to dialout" instruction.
    ok "running as root -- device permissions are bypassed"
    warn "running as root is not what you want here: recordings would be
        written to $HOME and owned by root, and the tmux session would
        belong to root rather than to you. Run it as your normal user."
else
    groups_of_user="$(id -nG "$USER" 2>/dev/null || true)"
    if [[ " $groups_of_user " == *" dialout "* ]]; then
        ok "user '$USER' is in the dialout group (serial access)"
    else
        bad "user '$USER' is NOT in the dialout group, so /dev/ttyACM* will be
        permission-denied. Fix with:
          sudo usermod -aG dialout $USER
        then log out and back in (a new SSH session is not enough if the
        session predates the change)."
    fi

    if [[ " $groups_of_user " == *" video "* ]]; then
        ok "user '$USER' is in the video group (camera access)"
    else
        warn "user '$USER' is not in the video group; /dev/video* may be
        permission-denied. sudo usermod -aG video $USER"
    fi
fi

# -- devices present ---------------------------------------------------
shopt -s nullglob
serial_ports=(/dev/ttyACM* /dev/ttyUSB* /dev/ttyTHS*)
video_nodes=(/dev/video*)
shopt -u nullglob

[[ ${#serial_ports[@]} -gt 0 ]] \
    && ok "serial ports present: ${serial_ports[*]}" \
    || warn "no /dev/ttyACM*, ttyUSB* or ttyTHS* -- camera control and the
        flight controller link will both be unavailable"

[[ ${#video_nodes[@]} -gt 0 ]] \
    && ok "video nodes present: ${video_nodes[*]}" \
    || bad "no /dev/video* at all -- the camera is not attached"

if command -v v4l2-ctl >/dev/null 2>&1; then
    cameras="$(v4l2-ctl --list-devices 2>/dev/null || true)"
    if grep -qiE 'neohd|fcb|harrier' <<<"$cameras"; then
        ok "an FCB/NeoHD camera is attached"
    else
        warn "no camera matching neohd/fcb/harrier is attached; the recorder
        will refuse to start rather than record from the wrong camera"
    fi
fi

# -- writable output ---------------------------------------------------
RECORD_DIR="${FCB_RECORD_DIR:-$HOME/fcb_recordings}"
mkdir -p "$RECORD_DIR" 2>/dev/null
if [[ -w "$RECORD_DIR" ]]; then
    free_gb=$(df -BG --output=avail "$RECORD_DIR" 2>/dev/null | tail -1 | tr -dc '0-9')
    # 1080p60 H.264 at the default 25 Mbps is about 11 GB an hour, so free
    # space is only meaningful as flight time. Reporting "12 GB free" as
    # fine would be misleading when that is barely one sortie.
    # Never blocking, however little is left: a short recording beats no
    # recording, and how much flight time is worth having is the operator's
    # call, not this script's. Reported as time rather than gigabytes so
    # the number means something at a glance.
    if [[ -n "$free_gb" ]]; then
        hours=$(awk "BEGIN{printf \"%.1f\", $free_gb/11}")
        if [[ "$free_gb" -lt 25 ]]; then
            warn "$RECORD_DIR has ${free_gb} GB free -- about ${hours} h of
        recording at 25 Mbps."
        else
            ok "$RECORD_DIR is writable (${free_gb} GB free, ~${hours} h at 25 Mbps)"
        fi
    else
        ok "$RECORD_DIR is writable"
    fi
else
    bad "$RECORD_DIR is not writable"
fi

echo
if [[ $FAIL -gt 0 ]]; then
    echo "$FAIL blocking problem(s), $WARN warning(s)"
    exit 1
fi
echo "all clear${WARN:+ ($WARN warning(s))}"
exit 0
