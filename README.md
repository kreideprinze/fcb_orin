# fcb_orin — flight recorder bundle for the Jetson Orin

Everything the drone needs to record the Sony FCB-EV9520L with time-synced
Pixhawk telemetry. One directory: copy it to the Orin and run it.

## Deploy

From the laptop:

    ./deploy.sh orin              # ssh host or alias
    ./deploy.sh nvidia@10.0.0.5

Then on the Orin:

    cd fcb_orin
    ./check_deps.sh               # fix anything it flags first
    ./start_recorder.sh           # runs inside tmux

## Flying it

| RC channel | Function |
|---|---|
| 7 | zoom (proportional — a knob, not a switch) |
| 8 | recording: **high starts, low stops** |

Each recording writes a matched pair plus a log, all sharing a timestamp:

    ~/fcb_recordings/fcb-20260903-114530.mp4
    ~/fcb_recordings/fcb-20260903-114530.csv
    ~/fcb_recordings/fcb_record-20260903-114530.log

## Why tmux

The recorder runs inside a tmux server owned by the Orin, so your SSH
session is just a viewer. Losing the link — flying out of range, closing
the laptop, a flat battery on your end — detaches the terminal and leaves
the recorder running.

    ./start_recorder.sh           # start, or re-attach to a running one
    Ctrl-b then d                 # detach, leaving it running
    ./start_recorder.sh --status  # is it up?
    ./start_recorder.sh --stop    # stop it properly (closes the mp4/CSV)

Always stop with `--stop` rather than killing the process: it sends an
interrupt so the video file gets its index written, and an mp4 killed
outright can be unplayable.

Inside tmux the recorder runs under a supervision loop, so if it exits
non-zero it restarts after a few seconds rather than leaving the drone
with nothing recording and nobody on the link to notice.

## The CSV

One row per recorded video frame, with `frame` as the join key into the
mp4:

| column | meaning |
|---|---|
| `frame` | frame number in the video, 0-based |
| `t_wall_utc`, `t_mono_s` | wall clock, and seconds since recording began |
| `lat_deg`, `lon_deg` | GPS position |
| `alt_msl_m`, `alt_rel_m` | altitude above sea level, and above home |
| `roll_deg`, `pitch_deg`, `yaw_deg` | attitude |
| `heading_deg`, `groundspeed_ms` | course and speed over ground |
| `zoom_ratio`, `zoom_position` | live zoom, as magnification and raw VISCA counts |
| `*_age_ms` | how old each reading was when the frame was captured |
| `fc_time_boot_ms` | the Pixhawk's own clock |

**Read the age columns.** Telemetry arrives far slower than 60 fps video —
GPS at a few hertz — so a row's position can be a couple of hundred
milliseconds stale. The ages tell you exactly how stale, instead of the
file implying a precision it does not have. Missing readings are left
blank rather than zero, because `0.0` latitude is a real place.

`fc_time_boot_ms` is there so the CSV can be lined up against the
Pixhawk's own `.bin` log after the flight.

## When things go wrong mid-flight

Nobody can replug anything at altitude, so the recorder recovers on its own:

- **Camera stops delivering frames** — after `--camera-timeout` (5s) the
  capture is torn down and reopened, rediscovering the device. Any
  recording in progress is closed cleanly at the break, and a fresh
  segment starts once frames return and the switch is still armed.
- **VISCA link dies** — after a run of failures the port is rediscovered
  and reopened, so zoom control and zoom logging come back.
- **MAVLink link drops** — reconnects continuously; the recording switch
  holds its current state rather than stopping, since a dropout is not a
  command to stop.
- **The recorder process dies** — the tmux supervision loop restarts it.

All of these are logged with timestamps, so the log file says what
happened and when.

## Dependencies

`check_deps.sh` covers these; the two that actually catch people:

**OpenCV must be JetPack's, not pip's.** The hardware encoder is reached
through GStreamer, and a pip-installed `opencv-python` is built without
it. If you use a virtualenv, create it with `--system-site-packages` or
JetPack's OpenCV will be invisible inside it. To check:

    python3 -c 'import cv2; print(cv2.getBuildInformation())' | grep -i gstreamer

Without it the recorder still runs, but falls back to CPU encoding and
will drop frames at 1080p60. It warns loudly when this happens.

**Serial permissions.** The camera's VISCA port and the Pixhawk are both
`/dev/ttyACM*`, which need the `dialout` group:

    sudo usermod -aG dialout $USER    # then log out and back in

Everything else: `pymavlink`, `pyserial`, `v4l-utils`, `tmux`.

    sudo apt install tmux v4l-utils python3-serial python3-pymavlink

## Useful options

    ./start_recorder.sh -- --encoder cpu        # force software encoding
    ./start_recorder.sh -- --rec-channel 6      # different recording switch
    ./start_recorder.sh -- --rc-url /dev/ttyACM0  # pin the flight controller
    ./start_recorder.sh -- --status-interval 0.1  # faster live display
    ./start_recorder.sh -- --bitrate 40         # higher quality, bigger files

Ports are autodetected by protocol rather than by path — the camera by a
VISCA exchange, the Pixhawk by a MAVLink heartbeat — because both appear
as `/dev/ttyACM*` and the numbers shuffle whenever something is replugged.

## Layout

    fcb_record.py        the recorder
    fcb_base_driver/     driver modules, generated by sync_driver.sh
    start_recorder.sh    tmux launcher and supervisor
    check_deps.sh        pre-flight dependency check
    deploy.sh            copy this bundle to the Orin
    sync_driver.sh       refresh fcb_base_driver/ from the git checkout

`fcb_base_driver/` here is a **generated copy**. The source of truth is the
`~/fcb_base_driver` git checkout; edit there, then run `./sync_driver.sh`
(or `./deploy.sh`, which does it first).
