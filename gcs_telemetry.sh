#!/usr/bin/env bash
# The flight-data pane: everything the recorder is writing, laid out to be
# read at a glance instead of scanned out of a log line.
#
#   ./gcs_telemetry.sh [session] [refresh-seconds]
#
# Like the status bar, it reads the recorder's own status line out of the
# recorder's pane rather than having the recorder publish state anywhere.
# That keeps this a pure add-on: the recorder needs no knowledge that the
# pane exists, and this pane dying cannot affect a recording.
#
# The screen is repainted by homing the cursor and clearing each line as it
# is written, never by clearing the whole screen first -- a full clear
# every refresh makes the panel flicker on a slow SSH link.
set -uo pipefail

SESSION="${1:-${FCB_TMUX_SESSION:-fcb}}"
REFRESH="${2:-0.5}"

# Pane 0 of the session's window is the recorder; the panes added beside it
# come later in the index. Resolved once per refresh rather than cached,
# so a layout rebuilt underneath us is picked up.
recorder_pane() {
    tmux list-panes -t "$SESSION" -F '#{pane_id}' 2>/dev/null | head -1
}

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

A=$'\033'
DIM="$A[38;5;240m"; LBL="$A[38;5;245m"; VAL="$A[38;5;252m"
AMB="$A[38;5;214m"; GRN="$A[38;5;84m"; RED="$A[38;5;203m"
CYN="$A[38;5;51m";  OFF="$A[0m"; BLD="$A[1m"
EOL="$A[K"

# One row of the panel: label, value, unit, colour.
row() { printf '%s   %s%-9s %s%8s %s%-5s%s%s\n' "$EOL" "$LBL" "$1" "${4:-$VAL}" "$2" "$LBL" "${3:-}" "$OFF" ""; }
head_() { printf '%s  %s%s%s%s\n' "$EOL" "$AMB$BLD" "$1" "$OFF" ""; }
# The panel is 27 lines at full dress and the pane is often shorter. The
# banner at the top is the single most important thing on it, so nothing
# may push it off: the separators and the second legend line are dropped
# first, and only then is anything else at risk.
COMPACT=0
blank() { (( COMPACT )) || printf '%s\n' "$EOL"; }

# A centred needle for roll and pitch: an 11-cell scale with the mark
# placed by value. Reading "+4" off a number is slower than seeing which
# side of centre the needle sits on.
gauge() {
    local v=${1:-0} span=${2:-45} w=11 i out="" pos
    v=${v%%.*}; [[ "$v" =~ ^-?[0-9]+$ ]] || v=0
    pos=$(( (v + span) * (w - 1) / (2 * span) ))
    (( pos < 0 )) && pos=0
    (( pos > w - 1 )) && pos=$(( w - 1 ))
    for (( i = 0; i < w; i++ )); do
        if   (( i == pos ));            then out+="${AMB}◆${DIM}"
        elif (( i == (w - 1) / 2 ));    then out+="${DIM}│"
        else                                 out+="${DIM}·"
        fi
    done
    printf '%s%s' "$out" "$OFF"
}

printf '%s[?25l' "$A"                       # hide the cursor while we repaint
# A trap that only restores the cursor leaves bash to carry on round the
# loop, so the pane would ignore the TERM tmux sends when it closes it.
# The signal traps have to exit as well as tidy up.
show_cursor() { printf '%s[?25h' "$A"; }
trap show_cursor EXIT
trap 'show_cursor; exit 0' INT TERM HUP

while true; do
    line="$(status_line "$(recorder_pane)")"

    rows=$(tput lines 2>/dev/null || echo 24)
    COMPACT=0
    (( rows < 27 )) && COMPACT=1

    printf '%s[H' "$A"

    if [[ -z "$line" ]]; then
        blank
        printf '%s  %s NO DATA %s  the recorder is not reporting%s\n' \
               "$EOL" "$A[48;5;88m$A[38;5;231m$BLD" "$OFF" "$OFF"
        blank
        printf '%s  %sIt may be starting, or it may have died. The log pane%s\n' "$EOL" "$LBL" "$OFF"
        printf '%s  %son the left has the detail.%s\n' "$EOL" "$LBL" "$OFF"
        printf '%s[J' "$A"
        sleep "$REFRESH"
        continue
    fi

    # -- pull the fields out of the one status line --------------------
    roll=""; pitch=""; yaw=""
    [[ "$line" =~ rpy\ (-?[0-9]+)/(-?[0-9]+)/(-?[0-9]+) ]] && {
        roll=${BASH_REMATCH[1]}; pitch=${BASH_REMATCH[2]}; yaw=${BASH_REMATCH[3]}; }
    alt_msl=""; [[ "$line" =~ alt\ (-?[0-9.]+)m ]] && alt_msl=${BASH_REMATCH[1]}
    alt_rel=""; [[ "$line" =~ rel\ (-?[0-9.]+)m ]] && alt_rel=${BASH_REMATCH[1]}
    hdg="";     [[ "$line" =~ hdg\ ([0-9]+) ]] && hdg=${BASH_REMATCH[1]}
    gs="";      [[ "$line" =~ gs\ ([0-9.]+)m/s ]] && gs=${BASH_REMATCH[1]}
    zoom="";    [[ "$line" =~ zoom\ ([0-9.]+)x ]] && zoom=${BASH_REMATCH[1]}
    ztgt="";    [[ "$line" =~ zoom\ [0-9.]+x\>([0-9.]+)x ]] && ztgt=${BASH_REMATCH[1]}
    fps="";     [[ "$line" =~ ([0-9]+\.[0-9])\ fps ]] && fps=${BASH_REMATCH[1]}
    frames="";  [[ "$line" =~ frames\ ([0-9]+) ]] && frames=${BASH_REMATCH[1]}
    drops=0;    [[ "$line" =~ skipped\ ([0-9]+) ]] && drops=${BASH_REMATCH[1]}
    snaps=0;    [[ "$line" =~ ([0-9]+)\ snap ]] && snaps=${BASH_REMATCH[1]}

    mode="--"
    for m in "RGB+IR" "AUTO" "RGB" "IR"; do
        [[ "$line" == *"| $m |"* ]] && { mode=$m; break; }
    done

    recsec=""; recfr=""
    if [[ "$line" =~ REC\ [^\ ]+\ \(([0-9]+)\ fr,\ ([0-9]+)s\) ]]; then
        recfr=${BASH_REMATCH[1]}; recsec=${BASH_REMATCH[2]}
    fi

    # -- draw ----------------------------------------------------------
    blank
    if [[ -n "$recsec" ]]; then
        printf '%s  %s ● RECORDING %s  %s%d:%02d%s   %s%s frames%s\n' "$EOL" \
            "$A[48;5;160m$A[38;5;231m$BLD" "$OFF" \
            "$AMB$BLD" $((recsec / 60)) $((recsec % 60)) "$OFF" \
            "$LBL" "$recfr" "$OFF"
    else
        printf '%s  %s ARMED %s  %sch8 high starts recording%s\n' "$EOL" \
            "$A[48;5;214m$A[38;5;16m$BLD" "$OFF" "$LBL" "$OFF"
    fi
    blank

    head_ "ATTITUDE"
    if [[ -n "$roll" ]]; then
        printf '%s   %sROLL      %s%8s %s°  %s\n' "$EOL" "$LBL" "$VAL" "$roll" "$LBL" "$(gauge "$roll" 45)"
        printf '%s   %sPITCH     %s%8s %s°  %s\n' "$EOL" "$LBL" "$VAL" "$pitch" "$LBL" "$(gauge "$pitch" 45)"
        row "YAW" "$yaw" "°"
    else
        row "ROLL" "--" "°"; row "PITCH" "--" "°"; row "YAW" "--" "°"
    fi
    blank

    head_ "NAVIGATION"
    row "ALT MSL" "${alt_msl:---}" "m"
    row "ALT REL" "${alt_rel:---}" "m"
    row "HEADING" "${hdg:---}" "°"
    row "GND SPD" "${gs:---}" "m/s"
    if [[ "$line" == *"GPS no fix"* ]]; then
        row "GPS" "NO FIX" "" "$RED"
    elif [[ "$line" == *"GPS "* ]]; then
        row "GPS" "FIX" "" "$GRN"
    else
        row "GPS" "--" ""
    fi
    blank

    head_ "CAMERA"
    if [[ -n "$ztgt" ]]; then
        row "ZOOM" "${zoom}→${ztgt}" "x" "$CYN"
    else
        row "ZOOM" "${zoom:---}" "x" "$CYN"
    fi
    case "$mode" in
        RGB)    row "MODE" "$mode" "" "$GRN" ;;
        IR)     row "MODE" "$mode" "" "$A[38;5;207m" ;;
        RGB+IR) row "MODE" "$mode" "" "$AMB" ;;
        AUTO)   row "MODE" "$mode" "" "$A[38;5;141m" ;;
        *)      row "MODE" "--" "" ;;
    esac
    blank

    head_ "VIDEO"
    if [[ -n "$fps" ]]; then
        w=${fps%%.*}
        if   (( w >= 55 )); then c="$GRN"
        elif (( w >= 30 )); then c="$AMB"
        else                     c="$RED"
        fi
        row "RATE" "$fps" "fps" "$c"
    else
        row "RATE" "NO VIDEO" "" "$RED"
    fi
    row "FRAMES" "${frames:---}" ""
    row "DROPPED" "$drops" "" "$( ((drops > 0)) && printf '%s' "$AMB" || printf '%s' "$VAL")"
    row "SNAPS" "$snaps" ""
    blank

    if (( COMPACT )); then
        printf '%s  %s1%s RGB %s2%s IR %s3%s RGB+IR %s4%s AUTO %ss%s snap%s\n' \
               "$EOL" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$OFF"
    else
        printf '%s  %sch7%s zoom  %sch8%s rec  %sch9%s snap  %sch10%s mode%s\n' \
               "$EOL" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$OFF"
        printf '%s  %s1%s RGB  %s2%s IR  %s3%s RGB+IR  %s4%s AUTO  %ss%s snap%s\n' \
               "$EOL" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$VAL" "$DIM" "$OFF"
    fi

    printf '%s[J' "$A"                      # wipe anything below the panel
    sleep "$REFRESH"
done
