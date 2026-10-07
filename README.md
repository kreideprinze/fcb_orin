# fcb_orin — flight recorder bundle for the Jetson Orin

Everything the drone needs to record the Sony FCB-EV9520L with time-synced
Pixhawk telemetry. One directory: copy it to the Orin and run it.

## Read this first — caveats

### 1. Never stop the running recorder. Keep it running.

**If you stop `fcb_record.py` you will have to power-cycle the camera
board.** With the Oppila LVDS-USB3 board, closing the video stream and
opening it again usually leaves the board sending nothing (0 fps). Only
switching its **12 V supply off and on** brings video back; restarting
software, replugging USB or rebooting the Orin does not.

So start the recorder once after power-up and leave it running for the
whole session:

- **Start and stop recordings with `r` / `r r` (or `x x`) or RC ch8**, never
  by stopping the script. The recorder can sit idle for hours.
- **Leave the pane with `Ctrl-b` then `d`** (detach). Do **not** press
  Ctrl-C, run `./fly.sh --stop`, or kill the tmux session unless you are
  about to power-cycle the board anyway.
- **`./fly.sh` restarts the recorder whenever it copies new code** (and with
  `--restart`, `--no-pixhawk`, `--autostart`). To copy code without
  restarting, use `./deploy.sh <user>@<host>`; the new code is used at the
  next power-up.
- If the feed shows **0 fps / "no frames"** or the log says **CAMERA
  BOARD IS STUCK**: switch the board's 12 V off for a few seconds and back
  on. The running recorder picks the camera up again by itself.

### 2. Power-up order (Oppila board)

The board runs on its own 12 V and **does not survive the Orin booting or
rebooting while it stays powered**: after a cold start of the whole drone it
can enumerate but stream 0 fps. Power the board **after** the Orin has
booted, or cycle its 12 V once the recorder is up. The proper fix is an
Orin-controlled relay/MOSFET on the 12 V line; `--board-power-cycle-cmd` (or
`FCB_BOARD_POWER_CYCLE`) is the hook the recorder calls when video is stuck,
at most once every 90 s.

### 3. Other Oppila board quirks (handled in code, but know them)

- **Unplugging USB while it streams** also needs a 12 V power cycle.
- **VISCA replies are held back** until 32 bytes have built up. The code
  pads every exchange (broadcast `88 30 01 FF`), so each reply costs about
  120–250 ms. Commands act at once.
- **The USB VISCA channel is unreliable while video streams**: about 25 % of
  reply bytes are lost or corrupted (measured 296 of 400 intact), and after a
  few day/IR switches it can stop answering altogether until a 12 V cycle.
  The code resyncs and validates replies, but expect the odd zoom or
  mode command to need repeating. The real fix is wiring VISCA to the
  board's **UART header J7** (TXD → Orin pin 10, RXD → pin 8, GND → pin 6;
  DIP switch UART on, USB off); the recorder already probes `ttyTHS*`.
- **Every 4th frame is a repeat** (78.7 frames/s delivered for a 60 fps
  camera) and the board **reports 30 fps**. Repeats are dropped and the
  container is written at the camera's nominal 60 fps. Frames lost on the
  link are filled with the previous one (at most 3 in a row) and marked
  `filled=1` in the CSV.
- **An all-green picture** is the board sending all-zero frames (no image
  from the camera block over LVDS) while control still works. The status
  says `BLANK PICTURE`. Fix: 12 V power cycle, then check the LVDS cable.
- **Two serial ports**: interface 0 is VISCA, the other is debug output.
  Only interface 0 is used.

The **Twiga USB3 NeoHD** board (`04b4:00f9`) is USB bus-powered and has none
of these 12 V issues; for it, replug the USB cable instead.

### 4. Flight controller (Pixhawk)

- Recording is armed by **RC ch8 over MAVLink**. With no flight controller
  there is no ch8: the status shows `NO FC -- r TO RECORD`, and only `r`
  (or `--autostart`) starts a recording. `--no-pixhawk` silences the search.
- A recording started with `r` is not stopped by ch8 sitting low; flicking
  ch8 up and back down stops it.
- **RC lost mid-flight holds the current state**: a dropout never starts or
  stops a recording. ch8 must hold a new position for 0.3 s to count.
- Without a Pixhawk the CSV's position and attitude columns are empty.

### 5. The Orin

- **No RTC battery**: after power-off the clock restarts at a stale date
  until it reaches an NTP server, so recordings get wrong-time names.
  `fly.sh` sets the clock from the laptop; run it from the laptop, or check
  `date` on the Orin before flying.
- **Only one program may use the camera.** A second recorder (an old tmux
  session, `fcb_view.py`, ROS nodes, `~/new_fcb`'s `neohd` session) holds the
  video and VISCA devices and makes this one look broken. Check `tmux ls`.
- **OpenCV must have GStreamer** for the hardware encoder (NVENC). A pip
  `opencv-python` hides the system one and has no GStreamer, so recording
  falls back to CPU and drops frames at 1080p60. `prefer_cv2.py` loads the
  system OpenCV automatically; `./check_deps.sh` reports which one is used.
- **The Orin's IP changes** (10.42.0.223 / .224 on the direct cable). `fly.sh`
  tries the known addresses; otherwise pass `user@host`.
- **Disk full**: the recorder stops the recording cleanly, shows
  `WRITE FAILED`, and starts no new one until it is restarted (see rule 1).
  It warns below 5 GB free.
- Recordings live on the Orin in `~/flight_recordings`. **Copy-on-detach**
  (`Ctrl-b d`, then `y`) needs key login from the Orin to the laptop **with
  the laptop's user name** (e.g. `r2d2@10.42.0.1:fcb_recordings`; set it with
  `[e]`). Without it, nothing is copied.
- JetPack 7 on the AVerMedia D131 carrier: `nvidia-l4t-bootloader` must stay
  on hold (`apt-mark hold`); its post-install script rejects the board.
  See `orin_setup/README.md` for every change made on each Orin.

### 6. Recording files

- **AVI, not mp4**, so a recording cut short by a power loss is still
  playable (see below).
- Stopping by key takes **two presses** (`r r` / `x x`) so a stray tap
  cannot end a flight.
- A camera dropout mid-flight starts a new numbered segment (`-2.avi`) on
  the same CSV; telemetry keeps being written while there is no video.

## Deploy

From the laptop:

    ./deploy.sh orin              # ssh host or alias
    ./deploy.sh nvidia@10.0.0.5

Then on the Orin:

    cd fcb_orin
    ./check_deps.sh               # fix anything it flags first
    ./start_recorder.sh           # runs inside tmux

Or do it all in one step from the laptop (deploy, start, attach):

    ./fly.sh shadow2@10.42.0.223

## Flying it

| RC channel | Function |
|---|---|
| 7 | zoom knob — detented in **0.5x** steps (1.0x, 1.5x ... 30x) |
| 8 | recording: **high starts, low stops** (or `r` / `r r` in the pane) |
| 9 | momentary: **one press sends a frame to the tmux pane** |
| 10 | 3-position: **low RGB, centre RGB+IR, high IR** |

Each flight writes one CSV and one or more AVI files, stamped with the
moment recording started:

    ~/flight_recordings/fcb-20260903-114530.avi     video
    ~/flight_recordings/fcb-20260903-114530-2.avi   more video, if the camera dropped out
    ~/flight_recordings/fcb-20260903-114530.csv     telemetry, covering the whole flight

The container is **AVI**, and that is a deliberate choice about what
happens when a write is cut short -- a power cut, a kill -9, a card pulled
at the wrong moment. An mp4 keeps its index (the moov atom) at the *end*
of the file, so a truncated mp4 is not a short video, it is no video at
all. AVI's frames can be recovered without the index. Measured on the
drone, truncating a 60-frame clip to 75% of its length:

| container | frames recovered |
|---|---|
| **AVI** | **43** |
| mp4 | 0 |

`--container mp4` switches back. Both are read by `./fly.sh --fetch`, so
flights recorded before this change still come home normally.

## Running without a Pixhawk

With no flight controller there is no ch8, so recordings are started from
the pane instead:

    ./fly.sh --no-pixhawk        # no flight controller: no search, no warnings
    ./fly.sh --autostart         # records as soon as it is up, nobody needed
    ./fly.sh --no-pixhawk --autostart

- **r** starts a recording; **r r** (or **x x**) stops it and offers to name
  it. Two presses, so one stray tap cannot end a flight. These are typed
  on the laptop in the pane `fly.sh` opens -- it is the Orin's tmux session.
- A recording started with r or `--autostart` is not ended by ch8 merely
  sitting low, or by the flight controller being absent. If there is a
  Pixhawk, flicking ch8 up and back down still stops it, and the switch is
  in charge again from then on.
- Video, zoom, day/IR and snapshots work as normal. The CSV is still
  written, with the position and attitude columns empty.
- Both options restart the recorder (they only take effect at start), and
  like any restart `fly.sh` will not do it in the middle of a recording
  without `--force`. Directly: `fcb_record.py --no-mavlink --autostart`.

## Nothing takes anything else down

The video and the telemetry are two records of the same flight, and losing
one must not lose the other. Nor should a bad radio moment, or an SSH
session that dies of range, end a recording.

**The camera drops out mid-flight.** An mp4 cannot span a gap in its own
frame stream, so the current one is finalised — but the *flight* is not
over. The CSV stays open and the position and attitude track keeps being
written at 10 Hz (`--telemetry-interval`) for as long as the outage lasts.
When frames return they open a new numbered segment against the same
session and the same CSV. Rows written during an outage have an **empty
`frame` column**, which is how you tell them from frame-backed ones.

**MAVLink drops out mid-flight.** Video keeps recording; the telemetry
columns simply go empty for that stretch. The recording switch holds its
state rather than stopping, because a dropout is not a command.

**Ch8 goes low.** The recording stops, and the files are finalised
properly. This is the normal way to end a flight from the transmitter.

**The tmux pane or window is closed mid-flight.** Recording continues.
This one was a real bug: closing the pane puts stdin at EOF, and a closed
descriptor is reported *readable* by `select()` for ever while every read
returns nothing -- so the key-drain loop spun at over a million iterations
per three seconds and the capture loop never turned again. Recording
stopped as surely as a crash. The keyboard is now retired on EOF, once,
with a message, and the flight carries on under RC control alone. Detach
properly (`Ctrl-b` then `d`) and nothing is disturbed either way.

**A recording can also be ended from the console**, which is the only way
to name it: press **`x` twice** within five seconds, or run
`./fly.sh --stop`.

Stopping with `x` then asks what to call the flight:

    name this flight> morning pass 3   [Enter keeps fcb-20260909-120000]

Whatever you type renames **every file of that session** — both the CSV
and all of its video segments — or none of them, if any target name is
already taken. A timestamp is a fine filename and a poor label; ten
minutes after landing nobody remembers which of five files was the good
pass, and this is the moment you still do. Enter on its own keeps the
timestamp. Spaces become underscores, anything awkward for a shell or a
filesystem is dropped, and the name cannot walk out of the recordings
directory. A clash never overwrites: it becomes `name-2`. The rename is
logged old-to-new, so the day's log still ties the file to its flight. After a manual stop the switch is ignored until you
flick it low and back — otherwise ch8, still sitting high, would start a
new recording on the very next frame and the stop key would just chop the
flight in two.

**The SSH session dies of range.** The recorder does not notice. It lives
in tmux on the drone, and `SIGHUP` — what a dying terminal sends, whose
default action is to kill — is ignored outright, so the mp4 can never be
left without its moov atom. Verified by killing an SSH client mid-write:
the process kept running and kept writing. Reconnect and
`./fly.sh --attach` picks it back up.

The log is separate: one file per day, appended to, covering every run
and every segment of that day. Each restart writes a `===== run started
... =====` line so the boundaries are visible:

    ~/flight_recordings/fcb_record-20260903.log

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
| `x` | | **stop the recording** — twice within 5s, then name it |
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

## Camera interface boards

The camera reaches the Orin through a USB interface board. Two are
recognised by USB ID, and `fly.sh` prints which one it found before
starting the recorder (`python3 -m fcb_base_driver.devices` asks the same
question by hand):

| Board | USB ID | Shows up as |
|---|---|---|
| Twiga USB3 NeoHD | `04b4:00f9` (`04b4:00f8` on a USB 2 link) | "USB3 NeoHD", YUYV |
| Oppila LVDS-USB3 | `04b4:0040` | "FX3 CAMERA", UYVY |

The Oppila board has three quirks the code works around on its own:

- **VISCA replies are held back** until the board has 32 bytes of them, so
  a lone reply never arrives. The link pads each exchange with broadcast
  Address Set packets, whose answers push the real reply out -- about
  120-250 ms per answer. Commands themselves act at once.
- **Every fourth frame is a repeat** of the one before (78.7 frames/s for a
  60 fps camera), and the board reports 30 fps whatever it is sending. The
  repeats are dropped and the rate is measured, so recordings play at the
  right speed.
- **Two serial ports:** interface 0 is VISCA, the other is the board's
  debug output. The VISCA one is probed first.

**Image stabilizer.** The recorder switches the camera's stabilizer on
every time it connects to the camera, startup and reconnects alike, and
logs what the camera reads back (`image stabilizer: on (confirmed by the
camera)`). The setting lives in the camera and is lost when it loses
power, so it has to be re-applied rather than set once. `--stabilizer off`
or `--stabilizer keep` (leave it as found) override it.

Set up per Oppila's docs: 12 V / 2 A on J9 (the board powers the camera
from it), and the **USB** control switch ON with **UART** OFF. If the USB
cable is pulled while it is streaming, video does not come back until
the 12 V supply is switched off and on.

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

## On a laptop, without the drone

`fcb_view.py` is the desk version: a window with the live feed, `r` to
record, and the zoom and imaging-mode controls. No autopilot, no RC, no
telemetry, no tmux -- none of that exists on a desk.

    ./fcb_view.py                    find the camera, show it
    ./fcb_view.py --scale 1.0        full-size window
    ./fcb_view.py --no-visca         video only, no serial control

| key | |
|---|---|
| `r` | start / stop recording |
| `1` `2` `3` `4` | RGB / IR / RGB+IR / AUTO |
| `i` | cycle those four |
| `+` `-` | zoom by one 0.5x detent |
| `0` | back to 1x |
| `s` | save a JPEG |
| `q` or Esc | quit |

The keys go to the **window**, not the terminal. Recordings land in
`--dir` (default `~/fcb_laptop`) as AVI, at full capture resolution --
`--scale` only affects the window, and the overlay is never written into
the file. Measured on this FCB over USB3: 1920x1080 at 59.94 fps, 350
frames recorded in 5.8 s with no drops.

Two things worth knowing, both found by running it:

- **The board offers only YUYV**, and whether OpenCV converts that to BGR
  depends on the backend. With `CAP_PROP_CONVERT_RGB` off it hands over a
  *two*-channel image, which the writer will happily encode as nonsense.
  Both programs now ask for the conversion and convert anything that still
  arrives raw.
- **No VISCA port is not an error.** The feed and the recording do not need
  one; the zoom and mode keys say so once and do nothing.

## Tests

    ./test/run_all.sh

Each one encodes a fault that actually happened in flight, so a failure
there is a repeat of a bad day: a recorder that could never arm and never
said so, a camera outage that took the telemetry track with it, a stop
that chopped a flight in two.

## When things go wrong mid-flight

Nobody can replug anything at altitude, so the recorder recovers on its own:

- **Camera stops delivering frames** — after `--camera-timeout` (5s) the
  capture is torn down and reopened, rediscovering the device. The video
  segment is finalised at the break; the recording session and its CSV
  carry on, and frames returning open the next numbered segment.
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
- **No flight controller at all** — the recorder still runs, because the
  camera, zoom, imaging modes and snapshots do not need one. But **ch8
  cannot start a recording**: the arming switch is RC channel 8, which
  arrives over MAVLink, so with no link there is no channel 8. This is
  impossible to miss -- a CRITICAL block at startup, `ready -- NO FLIGHT
  CONTROLLER`, `NO FC -- r TO RECORD` at the *front* of the live status
  line, and an error every five seconds. Press **r** in the pane to record
  anyway (see "Running without a Pixhawk"). Pass `--require-mavlink` and it
  refuses to start at all (exit 2), which is what you want for a real
  flight.
- **The recorder process dies** — the tmux supervision loop restarts it.

> A flight was lost to this on 2026-09-08. The autopilot was unplugged,
> `wants_recording()` returned `False` rather than `None` when there was no
> MAVLink, and the capture loop treated an unanswerable question as a
> settled "no". The pane showed a healthy 59.9 fps and `not recording` for
> two minutes and said nothing else. The single startup error had scrolled
> away. That is why the warnings above are so insistent.

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
    ./start_recorder.sh -- --stabilizer off     # leave the image stabilizer off
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
