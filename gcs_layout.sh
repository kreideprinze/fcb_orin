#!/usr/bin/env bash
# Split the recorder's window into a ground-station layout.
#
#   ./gcs_layout.sh [session] [bundle-dir]
#
#   +------------------------+----------------------------+
#   |  recorder log          |  LAST SNAPSHOT             |
#   |  (the pane keys go to) |  (the whole column, because|
#   +------------------------+   the picture is the one   |
#   |  FLIGHT DATA           |   thing that wants pixels) |
#   +------------------------+----------------------------+
#
# The preview takes the full height of its column rather than sharing it
# with the panel. The rendered image is bounded by whichever of width or
# height runs out first, and with the panel stacked beside it the height
# ran out with a third of the pane's width left unused.
#
# Idempotent: run it again and it does nothing, so restyling an already
# laid-out session cannot end up with six panes.
#
# The recorder keeps pane 0 and keeps the focus. That matters -- keys go
# to the *active* pane, so 1/2/3/4 and s only reach the recorder while its
# pane is the selected one.
set -uo pipefail

SESSION="${1:-${FCB_TMUX_SESSION:-fcb}}"
DIR="${2:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

tmux has-session -t "$SESSION" 2>/dev/null || exit 0

panes=$(tmux list-panes -t "$SESSION" 2>/dev/null | wc -l)
[[ "$panes" -ge 3 ]] && exit 0

REC=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' 2>/dev/null | head -1)
[[ -n "$REC" ]] || exit 0

# Lay the panes out against a standard size rather than against whatever
# the window happens to be. A detached session is 80x24, and splitting 22
# lines off 24 leaves the preview a sliver -- which tmux then preserves
# proportionally when a real client attaches, so the panel would end up
# owning most of the screen forever. Sizing to a sensible baseline first
# and handing the window back to the client afterwards means the ratios
# scale from something deliberate.
restore_sizing() { tmux set-option -w -t "$SESSION" window-size latest 2>/dev/null; }
trap restore_sizing EXIT
tmux set-option -w -t "$SESSION" window-size manual 2>/dev/null
tmux resize-window -t "$SESSION" -x 220 -y 50 2>/dev/null

# -d so the focus stays on the recorder while we build the rest.
PREVIEW=$(tmux split-window -h -d -t "$REC" -l 52% -P -F '#{pane_id}' \
    -c "$DIR" "exec python3 '$DIR/gcs_preview.py'" 2>/dev/null) || exit 0

# The panel goes under the log, not under the preview. 21 lines is what it
# needs with its separators dropped, and the log is happy with the rest --
# it is a scrolling log, so height costs it far less than it costs a
# picture.
TELEM=$(tmux split-window -v -d -t "$REC" -l 21 -P -F '#{pane_id}' \
    -c "$DIR" "exec bash '$DIR/gcs_telemetry.sh' '$SESSION'" 2>/dev/null) || true

tmux select-pane -t "$REC" -T "recorder  --  keys land here" 2>/dev/null
tmux select-pane -t "$PREVIEW" -T "last snapshot  (ch9 / s)" 2>/dev/null
[[ -n "${TELEM:-}" ]] && tmux select-pane -t "$TELEM" -T "flight data" 2>/dev/null

# Whatever happened above, the recorder ends up selected.
tmux select-pane -t "$REC" 2>/dev/null
