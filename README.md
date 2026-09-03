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
| 7 | zoom knob — detented in **0.5x** steps (1.0x, 1.5x ... 30x) |
| 8 | recording: **high starts, low stops** |
| 9 | momentary: **one press sends a frame to the tmux pane** |
| 10 | 3-position: **low RGB, centre RGB+IR, high IR** |

Each recording writes a matched pair -- video and per-frame telemetry --
stamped with the moment recording started:

    ~/fcb_recordings/fcb-20260903-114530.mp4
    ~/fcb_recordings/fcb-20260903-114530.csv

The log is separate: one file per day, appended to, covering every run
and every segment of that day. Each restart writes a `===== run started
... =====` line so the boundaries are visible:

    ~/fcb_recordings/fcb_record-20260903.log

## The zoom knob (ch7)

The knob is continuously variable, but what it *asks for* is quantized:
magnification snaps to the nearest 0.5x, so the same knob position gives
the same shot twice and a framing can be called out over the radio as a
number. `--zoom-step 0` restores the old continuous behaviour, and any
other step works -- `--zoom-step 1` for whole multiples, `0.25` for finer.

The default curve is **`ratio`**, which gives every detent an equal slice
of knob travel -- one nudge is one step, anywhere in the range:

| | 1-2x | 2-5x | 5-10x | 10-20x | 20-30x |
|---|---|---|---|---|---|
| detents | 2 | 6 | 10 | 20 | 20 |
| `ratio` travel | 2.5% | 10.3% | 17.1% | 34.4% | 34.3% |
| `log` travel | 16.4% | 29.3% | 21.0% | 20.7% | 11.9% |

`--curve log` is the camera-rocker feel and is still there, but it crowds
20 detents into the top 12% of the knob -- a nudge up there jumps several
steps -- while spending the bottom 16% getting from 1x to 2x.

Two honest limits, both at the tele end:

- The lens lands within about 50 position counts of a detent (measured:
  worst case 0.04x off across 2x-15x). Above **22x** the 0.5x detents are
  closer together than that -- 22.0x to 22.5x is 43 counts -- so up there
  the step you get may be the neighbouring one. 1x to 22x is exact.
- The default `log` curve spends equal knob travel per *factor* of zoom,
  so near 30x one percent of knob travel already crosses more than one
  detent. The detents are all still reachable; you just cannot feel every
  one of them up there. `--curve ratio` spreads them evenly instead.

### Why the lens no longer hunts

Worth knowing, because it constrains anything built on the servo. The
FCB does not stop where it is told to: `zoom_stop` is followed by a coast
of 25 counts at the slowest speed and 402 at the fastest, and a position
inquiry is a 60 ms serial round-trip, so the position being acted on is
always stale. A servo that stops inside a symmetric deadband therefore
*cannot* -- it overshoots, sees an error the other way, drives back, and
oscillates for as long as the knob is held still.

So the stop is issued early, by the distance the lens is about to cover
anyway; the speed is chosen as the fastest one that still leaves room to
stop, which makes it decelerate on approach; and once parked on a target
the servo does nothing at all until that target changes. It may spend up
to three slow trims squaring up on the detent -- bounded, because an
unbounded "correct until close" is the hunting all over again.

The rate and coast figures live in `zoom_servo.py` as measured constants.
`test/test_zoom_servo.py` in the driver repo models the lens from them and
fails if hunting comes back.

The detent being driven towards shows on the status line while the lens
is still travelling: `zoom 3.0x>4.5x`.

### Typing an exact zoom

The detents are an affordance for a knob; a number is not. Press `z` in
the recorder pane, type a value, press Enter:

    zoom> 7.3x  [1-30, Enter]

That drives to exactly 7.3x -- off-detent values are honoured, not
snapped. Backspace edits, and Enter on an empty entry cancels. Verified on
the camera: 7.3x and 2.2x land dead on, 18x within 0.06x.

While a value is set by hand the ch7 knob is **ignored**, because the knob
is always publishing and would otherwise take control back on the next
tick. The status line says so -- `zoom 7.3x BY HAND` -- and **`a`** hands
it back to the knob.

## Sending a frame to the ground (ch9)

There is no video downlink. Press ch9 and the recorder writes a JPEG on
the drone *and* draws the frame directly into the tmux pane you are
already SSH'd into, in colour, as text:

    ~/fcb_recordings/snapshots/snap-20260904-021242-930.jpg

The drawing is what crosses the link, and it is deliberately cheap -- a
64x18 rendering of a 1080p frame measured 24 KB, once, per press, on a
real (noisy, dim) frame off the drone; a clean daylight scene compresses
into colour runs better and costs less. Nothing
needs installing on the ground station: the terminal that is already open
draws it. The JPEG stays on the drone at full 960 px width for pulling off
later. `s` in the pane does the same thing without the transmitter.

Tuning: `--preview-cols` (default 100; cost grows with its square),
`--no-preview` for the JPEG alone, `--snap-width` / `--snap-quality` for
the JPEG, `--snap-channel 0` to switch the feature off.

## Switching RGB / IR in flight

The imaging mode is on **ch10** — low RGB, centre RGB+IR, high IR — and
also on the keyboard. Press these in the tmux pane, on your laptop, over
SSH, while it is recording:

| key | mode | what the camera does |
|---|---|---|
| `1` | RGB | daylight colour, IR cut filter in |
| `2` | IR | filter out, infrared-sensitive mono |
| `3` | RGB+IR | filter out, colour retained |
| `4` | AUTO | camera switches on scene brightness |
| `i` | | cycle through the four in turn |
| `s` | | send a frame to the pane, as ch9 does |
| `z` | | type an exact zoom, e.g. `z` `7` `.` `3` Enter |
| `a` | | hand zoom back to the ch7 knob |
| `?` | | print a status line now |

The switch and the keys do not fight. Ch10 acts only when it *moves*, so a
key press holds until you physically flick the switch — otherwise the
switch, which reports its position tens of times a second, would win every
time and the keys would be dead. AUTO is keyboard-only; the switch has
three positions and covers the three fixed modes.

The FCB-EV9520L is one visible-light sensor behind a mechanically
removable IR cut filter, not a thermal camera: "IR" means that filter
swings out and infrared reaches the same sensor. There is no second
stream, so a mode change changes the video being recorded from that
moment on -- switch between recordings rather than during one if the
footage needs to be uniform. Sony warns that RGB+IR gives false colours
under IR illumination.

Every change is logged with its timestamp, so the log says which mode any
part of a flight was shot in. The current mode is on the status line:

    59.9 fps | frames 12480 | REC fcb-20260903-114530.mp4 (9012 fr, 150s) | zoom 4.2x | IR | GPS ...

The mode survives a power cycle in the camera itself, so the recorder
leaves it alone at startup and only reports what it found. `--icr rgb`
(or `ir`, `rgb+ir`, `auto`) pins it at launch instead:

    ./start_recorder.sh -- --icr rgb

Keys only work where there is a terminal to read them from. Under nohup,
a pipe or systemd there is no keyboard, the recorder says so on startup,
and the mode stays wherever it was -- pass `--icr` in that case.

There is deliberately no quit key: ending a recording takes Ctrl-C or
`./start_recorder.sh --stop`, not one stray keystroke on a leaned-on
laptop.

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

## Getting the footage home

The drone has the recordings and no screen. When `fly.sh` finishes -- you
detached, you pressed Ctrl-C, or you ran `./fly.sh --stop` -- it offers to
copy the latest recording to your machine:

    ==> latest recording: fcb-20260904-032557.mp4 + .csv  (13M, finished 03:26)
    ==> would go to: 10.42.0.1:fcb_recordings
        copy it over? [y] yes  [a] all of today  [e] change destination  [n] no:

It works out where "home" is on its own: an SSH session carries the
client's address in `SSH_CONNECTION`, so running this on the drone from a
laptop already says where to send it. The *username* does not come with
that, so pick `e` the first time and the answer is remembered in
`~/.config/fcb_orin/gcs_dest` and offered as the default from then on.
`FCB_GCS_DEST` overrides it.

Run from the laptop instead, it pulls into `~/fcb_recordings` rather than
pushing. Either way `./fly.sh --fetch` does just the copy and touches
nothing else, and `--no-fetch` skips the question.

Two things it will not do. It never copies a recording that is still being
written -- an mp4 whose moov atom has not been written yet is unplayable,
so it says so and tells you to stop the recording first. And it never asks
when there is no terminal to ask on, so scripts and `--detached` runs are
unaffected.

**One-time setup for the push direction.** Sending *from* the drone needs
the drone to be able to log in to your laptop, which is the opposite of
how you set the session up, so it usually is not arranged yet. Until it
is, the offer fails cleanly and prints both remedies:

    ssh-copy-id r2d2@10.42.0.1        # once, from the drone
    scp uas@10.42.0.223:'/home/uas/fcb_recordings/fcb-...mp4' .   # or just pull

## When things go wrong mid-flight

Nobody can replug anything at altitude, so the recorder recovers on its own:

- **Camera stops delivering frames** — after `--camera-timeout` (5s) the
  capture is torn down and reopened, rediscovering the device. Any
  recording in progress is closed cleanly at the break, and a fresh
  segment starts once frames return and the switch is still armed.
- **Camera disappears entirely** (cable out, USB brown-out) — reopening
  fails because there is no device to open. The recorder does not exit;
  it prints `camera did not come back` and keeps retrying on the same
  timeout, so a lead that reseats itself is picked up automatically.
  Telemetry, zoom control and the mode keys carry on working meanwhile.
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

    ./start_recorder.sh -- --icr rgb            # start in a fixed mode
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
