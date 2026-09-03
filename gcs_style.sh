#!/usr/bin/env bash
# Dress the recorder's tmux session as a ground station.
#
#   ./gcs_style.sh [session] [bundle-dir]
#
# Split out of fly.sh so the look does not depend on how the session was
# started: fly.sh calls it after a deploy, and start_recorder.sh calls it
# when it creates the session, so a recorder started either way comes up
# looking the same.
#
# Everything here is a tmux option on one session. Nothing touches the
# recorder, and deleting the whole script changes no behaviour -- it only
# makes the pane plain again.
set -uo pipefail

S="${1:-${FCB_TMUX_SESSION:-fcb}}"
# tmux resolves #() with the server's cwd, which is not necessarily the
# bundle, so the feed script is named absolutely.
DIR="${2:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

# Session options and window options are different namespaces, and
# set-option silently rejects one for the other.
set_() { tmux set-option -t "$S" "$@" 2>/dev/null; }
win_() { tmux set-option -w -t "$S" "$@" 2>/dev/null; }

set_ status on
set_ status-interval 2
# Pane 0 must really be pane 0: the scrapers address the recorder by index.
win_ pane-base-index 0
set_ status-position bottom
set_ status-justify centre
set_ status-style "bg=colour234,fg=colour250"
set_ status-left-length 60
set_ status-right-length 220
set_ status-left "#[fg=colour16,bg=colour214,bold] FCB GCS #[fg=colour214,bg=colour238,nobold] #H #[fg=colour238,bg=colour234] "
set_ status-right "#(bash '$DIR/gcs_status.sh' '$S')"
win_ window-status-current-format "#[fg=colour214,bold]* #W#[nobold]"
win_ window-status-format "#[fg=colour240]  #W"
set_ message-style "bg=colour214,fg=colour16,bold"
win_ mode-style "bg=colour214,fg=colour16"

# The legend belongs on screen, not in a README on the other machine.
win_ pane-border-status top
win_ pane-border-style "fg=colour238"
win_ pane-active-border-style "fg=colour214"
win_ pane-border-format " #[fg=colour214,bold]#{?pane_active,▸ ,}#{pane_title}#[nobold] "

# The panes come last: the options above apply to them as they appear.
[[ -x "$DIR/gcs_layout.sh" ]] && "$DIR/gcs_layout.sh" "$S" "$DIR" >/dev/null 2>&1
