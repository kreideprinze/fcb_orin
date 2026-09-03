#!/usr/bin/env bash
# Start (or re-attach to) the flight recorder inside tmux.
#
# The point of tmux here is that the recorder must not care about your SSH
# session. It runs inside a tmux server owned by the Orin, so dropping the
# link -- flying out of WiFi range, closing the laptop, a dead battery on
# your end -- detaches the terminal and leaves the recorder running. SSH
# back in, run this again, and you are looking at the same session.
#
#   ./start_recorder.sh              start it, then attach
#   ./start_recorder.sh --detached   start it, do not attach
#   ./start_recorder.sh --stop       stop the recorder and kill the session
#   ./start_recorder.sh --status     is it running?
#   ./start_recorder.sh -- --encoder cpu --rc-channel 6
#                                    pass the rest to fcb_record.py
#
# Detach without stopping anything: Ctrl-b then d.
set -uo pipefail

SESSION="${FCB_TMUX_SESSION:-fcb}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECORDER="$HERE/fcb_record.py"
PYTHON="${FCB_PYTHON:-python3}"
RESTART_DELAY="${FCB_RESTART_DELAY:-3}"

die() { echo "error: $*" >&2; exit 1; }

attach() {
    if [[ -n "${TMUX:-}" ]]; then
        echo "already inside tmux; switching to session '$SESSION'"
        tmux switch-client -t "$SESSION"
    else
        tmux attach-session -t "$SESSION"
    fi
}

running() { tmux has-session -t "$SESSION" 2>/dev/null; }

recorder_alive() {
    # Is the python process itself still up?
    #
    # Not `pgrep -f fcb_record.py`: the supervisor is a bash script whose
    # own command line contains that path, so a pattern match finds the
    # wrapper and reports the recorder alive forever. Not
    # #{pane_current_command} either: bash runs python in its own process
    # group, so tmux reports the wrapper there too. The children of the
    # pane's process are what actually answer the question.
    local pane_pid children comm
    pane_pid="$(tmux display-message -p -t "$SESSION" '#{pane_pid}' 2>/dev/null)"
    [[ -n "$pane_pid" ]] || return 1
    children="$(pgrep -P "$pane_pid" 2>/dev/null)" || return 1
    for pid in $children; do
        comm="$(ps -o comm= -p "$pid" 2>/dev/null)"
        [[ "$comm" == python* ]] && return 0
    done
    return 1
}

DETACHED=0
ACTION=start
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --detached|-d) DETACHED=1; shift ;;
        --stop)        ACTION=stop; shift ;;
        --status)      ACTION=status; shift ;;
        --help|-h)     sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        --)            shift; EXTRA_ARGS=("$@"); break ;;
        *)             EXTRA_ARGS+=("$1"); shift ;;
    esac
done

command -v tmux >/dev/null 2>&1 || die "tmux is not installed. sudo apt install tmux"

case "$ACTION" in
status)
    if running; then
        if recorder_alive; then
            echo "session '$SESSION' is running, recorder is up"
        else
            echo "session '$SESSION' exists but the recorder is NOT running"
            echo "  (attach to see why: tmux attach -t $SESSION)"
        fi
        exit 0
    fi
    echo "session '$SESSION' is not running"
    exit 1
    ;;
stop)
    running || { echo "session '$SESSION' is not running"; exit 0; }
    # SIGINT first so the recorder closes the video and CSV properly; an
    # mp4 killed outright can be missing the index that makes it playable.
    echo "asking the recorder to stop..."
    tmux send-keys -t "$SESSION" C-c 2>/dev/null

    # Wait on the recorder itself, not on the session: the supervisor keeps
    # the pane open afterwards so its last output stays readable, so the
    # session outliving the recorder is normal rather than a hang.
    stopped=0
    for _ in $(seq 1 40); do
        sleep 0.5
        if ! recorder_alive; then
            stopped=1
            break
        fi
    done
    if [[ $stopped -eq 1 ]]; then
        echo "recorder stopped cleanly"
    else
        echo "recorder did not stop within 20s; killing it"
    fi
    tmux kill-session -t "$SESSION" 2>/dev/null
    echo "session '$SESSION' closed"
    exit 0
    ;;
esac

[[ -f "$RECORDER" ]] || die "fcb_record.py not found next to this script ($RECORDER)"

if running; then
    echo "session '$SESSION' is already running -- attaching"
    [[ $DETACHED -eq 1 ]] || attach
    exit 0
fi

"$HERE/check_deps.sh" || die "dependency check failed; fix the above first"

# A supervision loop, not a bare command: if the recorder dies -- an
# unhandled error, a USB controller reset that takes the camera with it --
# it comes back by itself instead of leaving the drone with no recorder
# and nobody on the link to notice. Ctrl-C stops the recorder and the loop
# together, so a deliberate stop does not fight the restart.
read -r -d '' SUPERVISE <<EOF || true
trap 'echo; echo "supervisor: stopping"; exit 0' INT TERM
while true; do
    echo "supervisor: starting fcb_record.py at \$(date '+%H:%M:%S')"
    $PYTHON -u "$RECORDER" ${EXTRA_ARGS[@]+"\${@}"}
    code=\$?
    if [ \$code -eq 0 ]; then
        echo "supervisor: recorder exited cleanly"
        break
    fi
    echo "supervisor: recorder exited with status \$code, restarting in ${RESTART_DELAY}s"
    sleep ${RESTART_DELAY}
done
echo "supervisor: done -- this pane stays open so you can read the output"
exec bash
EOF

tmux new-session -d -s "$SESSION" -n recorder \
    bash -c "$SUPERVISE" bash ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}

# Keep the pane alive if the shell inside ever exits, so a crash leaves its
# last words on screen instead of a vanished session.
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null 2>&1

echo "started tmux session '$SESSION'"
if [[ $DETACHED -eq 1 ]]; then
    echo "attach with: tmux attach -t $SESSION"
else
    sleep 1
    attach
fi
