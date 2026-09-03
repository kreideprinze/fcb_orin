#!/usr/bin/env bash
# One command to get the recorder running and fly. Works from either end:
# run it on the laptop and it deploys first, run it on the drone -- already
# SSH'd in, which is the usual case -- and it skips straight to starting.
#
#   ./fly.sh                     deploy if needed, (re)start, attach
#   ./fly.sh uas@10.42.0.223     a different drone (from the laptop)
#   ./fly.sh -- --icr ir         pass the rest to fcb_record.py
#   ./fly.sh --status            is it up, and is it recording?
#   ./fly.sh --stop              stop the recorder so the mp4 closes properly
#   ./fly.sh --no-deploy         skip the copy, but still restart and attach
#   ./fly.sh --no-attach         leave it running, do not open the pane
#   ./fly.sh --fetch             just copy recordings off, change nothing
#   ./fly.sh --no-fetch          do not offer to copy anything on the way out
#   ./fly.sh --force             act even while a recording is in progress
#
# This is deploy.sh + start_recorder.sh + tmux attach in one step, with the
# checks that matter between them:
#
#   - it refuses to disturb a recording that is in progress, because
#     restarting the recorder to load new code would end that recording;
#   - it restarts the recorder after a deploy, since a process already
#     running keeps the old code no matter what was just copied over;
#   - it notices when it is already running on the drone, instead of
#     SSHing into itself for a password and a deploy with nothing to copy;
#   - it holds one SSH connection open for the whole run, so a password
#     prompt (if you have not set up keys) happens once instead of four
#     times;
#   - it finds the drone itself. The Orin answers on the direct link and
#     on WiFi, and the WiFi address moves when the router hands out a
#     different lease, so each known address is tried and the first that
#     answers on port 22 is used. Override with FCB_ORIN=user@host, or
#     name the host on the command line.
#
# Once the pane is open:  1 RGB  2 IR  3 RGB+IR  4 AUTO  i cycle  ? status
#                         Ctrl-b then d detaches and leaves it recording.
set -uo pipefail

# Addresses the drone is known to answer on, tried in this order. The
# first is the wired/hotspot link, which is the one that does not change;
# the rest are WiFi leases, which do. FCB_ORIN overrides the lot.
CANDIDATES=(${FCB_ORIN_CANDIDATES:-uas@10.42.0.223 uas@192.168.1.103 uas@192.168.0.32})
TARGET="${FCB_ORIN:-}"
REMOTE_DIR="${FCB_REMOTE_DIR:-fcb_orin}"
SESSION="${FCB_TMUX_SESSION:-fcb}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

DEPLOY=1
ATTACH=1
OFFER=1
FORCE=0
ACTION=fly
EXTRA=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --status)     ACTION=status; shift ;;
        --stop)       ACTION=stop; shift ;;
        --no-deploy)  DEPLOY=0; shift ;;
        --no-attach)  ATTACH=0; shift ;;
        --fetch)      ACTION=fetch; shift ;;
        --no-fetch)   OFFER=0; shift ;;
        --force)      FORCE=1; shift ;;
        --help|-h)    sed -n '2,36p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        --)           shift; EXTRA=("$@"); break ;;
        -*)           die "unknown option $1 (--help for the list)" ;;
        *)            TARGET="$1"; shift ;;
    esac
done

[[ -f "$HERE/fcb_record.py" ]] || die "run this from the fcb_orin bundle ($HERE has no fcb_record.py)"

# -- are we already on the drone? --------------------------------------
#
# Running this from the drone is the ordinary case: you are usually
# already SSH'd in when you decide to restart something. Left undetected
# that means SSHing from the machine into itself -- a password prompt for
# your own shell, followed by a deploy that has nothing to copy and a
# driver refresh that looks for a git checkout only the laptop has. So the
# target's address is checked against this machine's own first, and
# everything runs locally when they are the same box.
is_local_target() {
    local host="${1#*@}" addr
    case "$host" in
        localhost|127.0.0.1|::1) return 0 ;;
        "$(hostname 2>/dev/null)"|"$(hostname -s 2>/dev/null)") return 0 ;;
    esac
    for addr in $( { ip -o addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1
                     hostname -I 2>/dev/null; } ); do
        [[ "$addr" == "$host" ]] && return 0
    done
    return 1
}

# Can this address be reached at all? A bare TCP connect to the SSH port,
# because it answers in milliseconds and needs no credentials -- worth
# doing before committing the whole run to an address that is not there.
reachable() {
    local host="${1#*@}"
    timeout 2 bash -c "exec 3<>/dev/tcp/$host/22" 2>/dev/null
}

resolve_target() {
    local cand
    # Being one of the candidates ourselves settles it with no network at
    # all, and is the common case: you are SSH'd into the drone already.
    for cand in "${CANDIDATES[@]}"; do
        if is_local_target "$cand"; then TARGET="$cand"; return; fi
    done
    for cand in "${CANDIDATES[@]}"; do
        if reachable "$cand"; then
            TARGET="$cand"
            [[ "$cand" == "${CANDIDATES[0]}" ]] || say "drone found at $cand"
            return
        fi
    done
    # Nothing answered. Keep the first candidate so the failure that
    # follows names an address instead of an empty string.
    TARGET="${CANDIDATES[0]}"
    warn "no drone answered on: ${CANDIDATES[*]}"
}

[[ -n "$TARGET" ]] || resolve_target

LOCAL=0
is_local_target "$TARGET" && LOCAL=1

# -- one connection for the whole run ----------------------------------
#
# Every step below needs the drone, and without multiplexing each one is a
# separate login. ControlPersist keeps the first connection alive for the
# rest of the script, so authentication happens once -- which on a laptop
# in a field with a password prompt is the difference between one
# interruption and four.
CTL="${TMPDIR:-/tmp}/fcb-fly-$$"
SSH_OPTS=(-o ControlMaster=auto -o ControlPath="$CTL" -o ControlPersist=120
          -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new)

if [[ $LOCAL -eq 1 ]]; then
    # Same commands, no transport. The rest of the script is written
    # against orin()/attach() so it does not care which of these it got.
    WORK_DIR="$HERE"
    orin() { bash -c "$1"; }
    attach() {
        # Attaching to the session from inside tmux is what nesting warnings
        # are made of; switch the client instead.
        if [[ -n "${TMUX:-}" ]]; then
            tmux switch-client -t "$SESSION"
        else
            tmux attach -t "$SESSION"
        fi
    }
    say "running on the drone itself ($(hostname)) -- no SSH, no copy needed"
else
    WORK_DIR="$REMOTE_DIR"
    cleanup() { ssh "${SSH_OPTS[@]}" -O exit "$TARGET" 2>/dev/null; }
    trap cleanup EXIT
    orin()   { ssh "${SSH_OPTS[@]}" "$TARGET" "$1"; }
    attach() { ssh -t "${SSH_OPTS[@]}" "$TARGET" "tmux attach -t '$SESSION'"; }
    say "drone: $TARGET"
    orin true 2>/dev/null || die "cannot reach $TARGET over SSH.
       Check the link is up (ping ${TARGET#*@}), that the drone is powered,
       and that the username is right -- it is the part before the @."
fi

# -- getting the footage home ------------------------------------------
#
# The drone has the recordings and no screen. Whoever just finished flying
# is sitting at a terminal on another machine, and the thing they want
# next is the file. Asking on the way out is the moment they are actually
# thinking about it -- ten minutes later they are somewhere else and the
# card fills up.
#
# Which machine to send it to is known without being told: an SSH session
# carries the client's address in SSH_CONNECTION, so running this on the
# drone from a laptop already says where "home" is. The username does not
# come with it, so the first answer is remembered and offered as the
# default from then on.
OFFERED=0
DEST_MEMO="${XDG_CONFIG_HOME:-$HOME/.config}/fcb_orin/gcs_dest"

default_destination() {
    [[ -n "${FCB_GCS_DEST:-}" ]] && { printf '%s' "$FCB_GCS_DEST"; return 0; }
    [[ -r "$DEST_MEMO" ]] && { head -n1 "$DEST_MEMO"; return 0; }
    # "client_ip client_port server_ip server_port". Not set at all when
    # this is not an SSH session, which is the ordinary case on the laptop.
    local client="${SSH_CONNECTION:-}"
    client="${client%% *}"
    [[ -n "$client" ]] && printf '%s:fcb_recordings' "$client"
}

remember_destination() {
    mkdir -p "$(dirname "$DEST_MEMO")" 2>/dev/null || return 0
    printf '%s\n' "$1" > "$DEST_MEMO" 2>/dev/null || true
}

#: Files, newest first, as "<mp4> <csv>" pairs. Only pairs: a video whose
#: telemetry is still being written is half a recording.
recording_pairs() {
    local which="$1"      # newest | today
    orin "ls -t ~/fcb_recordings/fcb-*.mp4 2>/dev/null" 2>/dev/null | {
        local count=0 mp4 base
        while read -r mp4; do
            [[ -n "$mp4" ]] || continue
            base="${mp4%.mp4}"
            if [[ "$which" == today ]]; then
                case "$mp4" in *"fcb-$(date +%Y%m%d)-"*) ;; *) continue ;; esac
            fi
            printf '%s %s.csv\n' "$mp4" "$base"
            count=$((count + 1))
            [[ "$which" == newest && $count -ge 1 ]] && break
        done
    }
}

#: A file the recorder is still writing is not worth copying: the mp4
#: would land unplayable, its moov atom never written.
still_growing() {
    orin "a=\$(stat -c%s '$1' 2>/dev/null); sleep 1; \
          b=\$(stat -c%s '$1' 2>/dev/null); [ \"\$a\" != \"\$b\" ]"
}

copy_out() {
    local dest="$1"; shift
    local files=("$@")
    local host="${dest%%:*}" path="${dest#*:}"

    if [[ $LOCAL -eq 1 ]]; then
        # On the drone: push to the machine that SSH'd in. That direction
        # needs the drone to be able to authenticate *to* the laptop, which
        # is the opposite of how the session was set up and is often not
        # arranged -- so a failure here gets a usable answer, not a stack
        # of scp errors.
        say "copying to $dest"
        if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$host" \
                 "mkdir -p '$path'" 2>/dev/null; then
            warn "cannot log in to $host from the drone without a password.
         Nothing was copied. Either set that up once, from the drone:
             ssh-copy-id $host
         or pull them from the other machine instead:
             scp $(whoami)@$(hostname -I | awk '{print $1}'):'${files[0]}' ."
            return 1
        fi
        scp -p "${files[@]}" "$dest/" || { warn "scp failed"; return 1; }
    else
        # On the laptop: pull off the drone into the local recordings dir.
        local into="${HOME}/fcb_recordings"
        mkdir -p "$into"
        say "copying into $into"
        local remote=()
        local f
        for f in "${files[@]}"; do remote+=("$TARGET:$f"); done
        scp -p "${SSH_OPTS[@]}" "${remote[@]}" "$into/" \
            || { warn "scp failed"; return 1; }
    fi
    say "copied $(( ${#files[@]} )) file(s)"
    return 0
}

offer_recordings() {
    [[ $OFFER -eq 1 && $OFFERED -eq 0 ]] || return 0
    [[ -t 0 && -t 1 ]] || return 0          # nothing to ask, nobody to ask
    OFFERED=1

    local pair
    pair="$(recording_pairs newest)"
    [[ -n "$pair" ]] || return 0

    local mp4 csv
    read -r mp4 csv <<<"$pair"

    if still_growing "$mp4"; then
        warn "the newest recording is still being written -- not copying it.
         Stop the recording (ch8 low, or ./fly.sh --stop) and run
         ./fly.sh --fetch"
        return 0
    fi

    local size when
    size="$(orin "du -ch '$mp4' '$csv' 2>/dev/null | tail -1 | cut -f1")"
    when="$(orin "date -r '$mp4' '+%H:%M' 2>/dev/null")"

    local dest
    dest="$(default_destination)"
    if [[ $LOCAL -eq 1 && -z "$dest" ]]; then
        # Not an SSH session and nothing remembered: there is no sensible
        # guess, and inventing one would copy someone's flight somewhere
        # they did not ask for.
        say "recordings are in ~/fcb_recordings on this machine"
        return 0
    fi

    echo
    say "latest recording: $(basename "$mp4") + .csv  ($size, finished $when)"
    [[ $LOCAL -eq 1 ]] && say "would go to: $dest"

    local answer
    read -r -p "$(printf '    copy it over? [y] yes  [a] all of today  [e] change destination  [n] no: ')" answer
    echo

    case "${answer:-n}" in
        e|E)
            read -r -p "    destination (user@host:dir): " dest
            [[ -n "$dest" ]] || return 0
            remember_destination "$dest"
            ;;
        y|Y|a|A) : ;;
        *) say "left on the drone; ./fly.sh --fetch copies them any time"
           return 0 ;;
    esac

    local files=()
    local which=newest
    case "${answer}" in a|A) which=today ;; esac
    while read -r m c; do
        [[ -n "$m" ]] && files+=("$m" "$c")
    done < <(recording_pairs "$which")

    [[ ${#files[@]} -gt 0 ]] || return 0
    if copy_out "$dest" "${files[@]}"; then
        [[ $LOCAL -eq 1 ]] && remember_destination "$dest"
    fi
}

# -- what is it doing right now? ---------------------------------------
#
# Asked before anything is changed. A recorder that is mid-recording must
# not be restarted underneath a flight, and a deploy that lands without a
# restart is a deploy that has not taken effect -- both need this answer.
# -- make the pane look like a ground station --------------------------
#
# Cosmetics, and deliberately confined to tmux options on this one
# session: nothing here touches the recorder, and deleting the whole
# function would change no behaviour. The point is that the four things
# worth knowing at a glance while flying -- is it recording, what zoom,
# which filter, is the frame rate holding -- are readable without parsing
# the log scroll, and that the channel and key legend is always on screen
# instead of in a README on the other machine.
style_session() {
    orin "cd '$WORK_DIR' 2>/dev/null && ./gcs_style.sh '$SESSION' '$WORK_DIR'" \
        >/dev/null 2>&1 || true
}

read -r SESSION_UP RECORDER_UP RECORDING < <(
    orin "SESSION='$SESSION' bash -s" <<'EOS'
session=no; recorder=no; recording=no
if tmux has-session -t "$SESSION" 2>/dev/null; then
    session=yes
    # The recorder is a child of the pane's shell: the supervisor loop is
    # what the pane itself runs, so the pane's own command is always bash.
    # ":.0" is pane 0 of the session's window -- the recorder. A bare
    # "$SESSION" means the *active* pane, which stops being the recorder
    # the moment the window is split into the GCS layout, and would make
    # the recording guard below read the wrong pane.
    pane="$(tmux display-message -p -t "$SESSION:.0" '#{pane_pid}' 2>/dev/null)"
    for pid in $(pgrep -P "$pane" 2>/dev/null); do
        case "$(ps -o comm= -p "$pid" 2>/dev/null)" in python*) recorder=yes ;; esac
    done
    if tmux capture-pane -p -J -t "$SESSION:.0" 2>/dev/null | tail -4 | grep -q '| REC '; then
        recording=yes
    fi
fi
echo "$session $recorder $recording"
EOS
)
: "${SESSION_UP:=no}" "${RECORDER_UP:=no}" "${RECORDING:=no}"

describe() {
    if [[ $RECORDER_UP == yes && $RECORDING == yes ]]; then
        say "recorder is UP and RECORDING"
    elif [[ $RECORDER_UP == yes ]]; then
        say "recorder is up, armed, not recording"
    elif [[ $SESSION_UP == yes ]]; then
        say "tmux session '$SESSION' exists but the recorder is not running"
    else
        say "nothing running on the drone"
    fi
}

case "$ACTION" in
status)
    describe
    orin "cd '$WORK_DIR' 2>/dev/null && tmux capture-pane -p -t '$SESSION:.0' 2>/dev/null | grep -v '^\$' | tail -3"
    [[ $RECORDER_UP == yes ]] && exit 0 || exit 1
    ;;
stop)
    describe
    orin "cd '$WORK_DIR' && ./start_recorder.sh --stop"
    code=$?
    # Landing and fetching are the same moment for most flights, so the
    # offer follows the stop rather than making it a second command.
    [[ $code -eq 0 ]] && offer_recordings
    exit $code
    ;;
fetch)
    offer_recordings
    exit 0
    ;;
esac

# Ctrl-C is how a flight ends, so it is a way out of this script, not an
# error in it. While the pane is attached the key belongs to the recorder
# and never reaches here -- that path is covered after attach returns.
trap 'echo; offer_recordings; exit 130' INT

describe

# A recording in progress is the one thing here worth protecting: the
# restart that loads new code would close it, and a half-flight of footage
# cannot be taken again. Say so and change nothing, rather than deciding
# for the pilot -- --force is there when it really is meant.
if [[ $RECORDING == yes && $FORCE -eq 0 ]]; then
    warn "a recording is in progress -- nothing has been deployed or restarted.
         Flip the recording switch (ch8) low to close the file, then run this
         again. To override and lose the tail of that recording: ./fly.sh --force"
    if [[ $ATTACH -eq 1 ]]; then
        say "attaching to watch it (Ctrl-b then d to detach)"
        style_session
        attach
    fi
    exit 0
fi

# -- deploy ------------------------------------------------------------
#
# There is nothing to copy when the bundle is already on the machine that
# will run it.
if [[ $LOCAL -eq 1 ]]; then
    DEPLOY=0
fi

if [[ $DEPLOY -eq 1 ]]; then
    # A refresh that cannot happen is not a reason to abandon the flight.
    # ~/fcb_base_driver is the laptop's development checkout; without it
    # the bundle's own copy of the driver is simply what gets deployed,
    # which is exactly what was flying yesterday.
    if [[ -d "${FCB_DRIVER_REPO:-$HOME/fcb_base_driver}/fcb_base_driver" ]]; then
        say "refreshing the bundled driver from the git checkout"
        "$HERE/sync_driver.sh" || die "sync_driver.sh failed; nothing was copied"
    else
        warn "no driver checkout at ${FCB_DRIVER_REPO:-$HOME/fcb_base_driver} --
         deploying the bundle's own fcb_base_driver/ as it stands. Set
         FCB_DRIVER_REPO if the checkout lives somewhere else."
    fi

    say "copying the bundle to $TARGET:$REMOTE_DIR"
    if command -v rsync >/dev/null 2>&1; then
        rsync -a --delete \
            --exclude '__pycache__' --exclude '*.pyc' \
            --exclude '.git' --exclude '.pytest_cache' \
            -e "ssh ${SSH_OPTS[*]}" \
            "$HERE/" "$TARGET:$REMOTE_DIR/" || die "copy failed"
    else
        warn "rsync not found, falling back to scp"
        orin "mkdir -p '$REMOTE_DIR'"
        scp -q "${SSH_OPTS[@]}" -r "$HERE"/*.py "$HERE"/*.sh "$HERE/README.md" \
            "$HERE/fcb_base_driver" "$TARGET:$REMOTE_DIR/" || die "copy failed"
    fi
    orin "chmod +x '$REMOTE_DIR'/*.sh '$REMOTE_DIR'/fcb_record.py"
elif [[ $LOCAL -eq 1 ]]; then
    say "the bundle is already on this machine; nothing to copy"
else
    say "skipping the copy (--no-deploy)"
fi

# -- start -------------------------------------------------------------
#
# Unconditionally restarted after a deploy: a running process holds the
# code it started with, so copying a new fcb_record.py over the top of a
# live recorder changes nothing until it comes back.
if [[ $SESSION_UP == yes ]]; then
    say "stopping the running recorder so the new code is what runs"
    orin "cd '$WORK_DIR' && ./start_recorder.sh --stop" || die "could not stop it"
fi

ARGS=""
if ((${#EXTRA[@]})); then
    ARGS=" -- $(printf '%q ' "${EXTRA[@]}")"
    say "recorder options:${ARGS#* -- }"
fi

say "starting the recorder (this runs check_deps.sh first)"
# shellcheck disable=SC2029  # $ARGS is deliberately expanded here, not remotely
orin "cd '$WORK_DIR' && ./start_recorder.sh --detached$ARGS" \
    || die "the recorder would not start -- see the output above"

# start_recorder.sh returns as soon as tmux has the session; the recorder
# behind it still has to find the camera, the VISCA port and the autopilot,
# and it is those that usually fail. So wait for it to say it is ready
# rather than reporting success the moment tmux accepted the command.
say "waiting for it to come up..."
for _ in $(seq 1 20); do
    sleep 1
    if orin "tmux capture-pane -p -t '$SESSION:.0' 2>/dev/null | grep -q 'ready --'"; then
        break
    fi
done

echo
orin "tmux capture-pane -p -t '$SESSION:.0' 2>/dev/null | grep -v '^\$' \
      | grep -E 'VISCA|MAVLink:|video:|zoom:|imaging mode|ready --|keys:|ERROR|CRITICAL' \
      | tail -12"
echo

style_session

# The whole pane, not its last 20 lines: by the time this runs the status
# line has printed several times and pushed 'ready --' well past a 20-line
# tail, which made a healthy recorder report as a failed one on every launch.
if orin "tmux capture-pane -p -t '$SESSION:.0' 2>/dev/null | grep -q 'ready --'"; then
    say "recorder is up. Recording is armed on RC ch8: high starts, low stops."
else
    warn "the recorder did not report 'ready' within 20s -- it may still be
         probing, or something above failed. The pane has the detail."
fi

cat <<KEYS

  In the pane:  1 RGB  2 IR  3 RGB+IR  4 AUTO  i cycle  s snapshot
                z 7.3 Enter  zoom to an exact value    a  knob back
                Ctrl-b then d  detach, leaving it recording
                ./fly.sh --stop  land it, then offer the footage
                ./fly.sh --fetch copy recordings over, change nothing

KEYS

if [[ $ATTACH -eq 1 ]]; then
    attach
    offer_recordings
else
    # Not "./fly.sh --no-deploy": that skips the copy but still restarts
    # the recorder, which is the opposite of what someone who just wants
    # to watch it is asking for.
    say "left running detached; watch it with: tmux attach -t $SESSION"
fi
