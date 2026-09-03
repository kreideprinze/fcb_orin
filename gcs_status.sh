#!/usr/bin/env bash
# The right-hand side of the GCS status bar in tmux.
#
# It reads the recorder's own live status line straight out of the pane
# rather than having the recorder publish state somewhere. Nothing has to
# be plumbed through the process, the recorder needs no changes to feed
# the bar, and a recorder that has died simply stops updating that line --
# which is precisely the condition the bar should be shouting about.
#
# tmux runs this every `status-interval` seconds, so it stays cheap: one
# capture-pane of the visible pane (not the scrollback) and some pattern
# matching in bash, no subprocesses per field.
SESSION="${1:-fcb}"

BG=colour234                 # the bar's own background
DIM=colour240                # separators
LBL=colour245                # field labels
VAL=colour252                # ordinary values
sep="#[fg=$DIM,bg=$BG,nobold] │ "

# -J joins wrapped lines. The recorder's status line is longer than the
# pane is wide, so without it the capture is split mid-line and every
# field past the wrap -- zoom, imaging mode, altitude -- goes missing.
# The recorder publishes its full status line here. Preferred over
# scraping the pane, which only ever holds the line truncated to the pane
# width -- and the GCS layout makes that pane narrower than the line, so
# heading and groundspeed are simply not in it.
#
# Aged out deliberately: a recorder that died leaves the file behind, and
# showing its last moment as though it were current is worse than showing
# nothing. Three seconds is several status ticks, so a live recorder never
# trips it.
STATE_FILE="${FCB_STATE_FILE:-${XDG_RUNTIME_DIR:-/tmp}/fcb_gcs_state}"
STATE_MAX_AGE=3

status_line() {
    local now age
    if [[ -r "$STATE_FILE" ]]; then
        now=$(date +%s)
        age=$(( now - $(stat -c %Y "$STATE_FILE" 2>/dev/null || echo 0) ))
        if (( age <= STATE_MAX_AGE )); then
            cat "$STATE_FILE" 2>/dev/null
            return
        fi
    fi
    # Fall back to the pane, so this still works against a recorder
    # started without --state-file.
    tmux capture-pane -p -J -t "$1" 2>/dev/null \
        | grep -aE '\| frames [0-9]' | tail -1
}

line="$(status_line "$SESSION:.0")"

out=""

# -- armed / recording -------------------------------------------------
#
# The one field worth reading from across a field. Recording is white on
# red, standby is black on amber, and a recorder that is not talking at
# all is neither -- it must not be mistaken for "armed and fine".
if [[ -z "$line" ]]; then
    printf '%s' "#[fg=colour231,bg=colour88,bold] ✖ NO DATA #[bg=$BG,nobold]"
    printf '%s' "$sep#[fg=$LBL]recorder not reporting "
    printf '%s' "$sep#[fg=$LBL]#[fg=$VAL]$(date +%H:%M:%S) "
    exit 0
fi

if [[ "$line" =~ REC[[:space:]][^[:space:]]+[[:space:]]\(([0-9]+)[[:space:]]fr,[[:space:]]([0-9]+)s\) ]]; then
    secs=${BASH_REMATCH[2]}
    printf -v elapsed '%d:%02d' $((secs / 60)) $((secs % 60))
    out+="#[fg=colour231,bg=colour160,bold] ● REC $elapsed #[bg=$BG,nobold]"
else
    out+="#[fg=colour16,bg=colour214,bold] ARMED #[bg=$BG,nobold]"
fi

# -- zoom --------------------------------------------------------------
if [[ "$line" =~ zoom[[:space:]]([0-9.]+)x\>([0-9.]+)x ]]; then
    # Still travelling to a detent: show where it is heading, not just
    # where it is, or the knob looks like it did nothing.
    out+="$sep#[fg=$LBL]⌕ #[fg=colour123]${BASH_REMATCH[1]}x#[fg=$DIM]→#[fg=colour51,bold]${BASH_REMATCH[2]}x#[nobold]"
elif [[ "$line" =~ zoom[[:space:]]([0-9.]+)x ]]; then
    out+="$sep#[fg=$LBL]⌕ #[fg=colour51]${BASH_REMATCH[1]}x"
fi

# -- imaging mode ------------------------------------------------------
#
# Ordered longest-first: RGB+IR contains RGB, and matching the short one
# first would label the wrong filter position.
for mode in "RGB+IR:colour220" "AUTO:colour141" "RGB:colour84" "IR:colour207"; do
    name=${mode%%:*}; col=${mode##*:}
    if [[ "$line" == *"| $name |"* || "$line" == *"| $name"$'\n'* ]]; then
        out+="$sep#[fg=$col,bold]◉ $name#[nobold]"
        break
    fi
done

# -- frame rate --------------------------------------------------------
#
# Coloured by how far it has fallen: at 59.94 nominal, a sagging rate is
# the first sign of a camera or a CPU in trouble.
if [[ "$line" =~ ([0-9]+\.[0-9])[[:space:]]fps ]]; then
    fps=${BASH_REMATCH[1]}
    whole=${fps%%.*}
    if   (( whole >= 55 )); then col=colour84
    elif (( whole >= 30 )); then col=colour214
    else                         col=colour196
    fi
    out+="$sep#[fg=$col]${fps}#[fg=$LBL]fps"
else
    out+="$sep#[fg=colour196,bold]NO VIDEO#[nobold]"
fi

# -- dropped frames ----------------------------------------------------
if [[ "$line" =~ skipped[[:space:]]([0-9]+) ]]; then
    out+="$sep#[fg=colour214]⚠ ${BASH_REMATCH[1]} drop"
fi

# -- snapshots ---------------------------------------------------------
if [[ "$line" =~ ([0-9]+)[[:space:]]snap ]]; then
    out+="$sep#[fg=$VAL]▣ ${BASH_REMATCH[1]}"
fi

# -- GPS ---------------------------------------------------------------
if [[ "$line" == *"GPS no fix"* ]]; then
    out+="$sep#[fg=colour203]✖ GPS"
elif [[ "$line" == *"GPS "* ]]; then
    out+="$sep#[fg=colour84]✔ GPS"
fi

# -- altitude and heading ----------------------------------------------
if [[ "$line" =~ rel[[:space:]](-?[0-9.]+)m ]]; then
    out+="$sep#[fg=$LBL]alt #[fg=$VAL]${BASH_REMATCH[1]}m"
fi
if [[ "$line" =~ hdg[[:space:]]([0-9]+) ]]; then
    out+="$sep#[fg=$LBL]hdg #[fg=$VAL]${BASH_REMATCH[1]}°"
fi

out+="$sep#[fg=$VAL]$(date +%H:%M:%S) "
printf '%s' "$out"
