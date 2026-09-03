#!/usr/bin/env bash
# Refresh this bundle's copy of the driver package from the git checkout.
#
# ~/fcb_base_driver is the source of truth. The copy here exists only so
# the drone gets one self-contained directory to run from, and it carries
# just the ROS-free modules -- the ROS nodes need rclpy, which the Orin
# does not need installed to record video.
#
# Run this after changing anything in the driver, then deploy.sh.
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${FCB_DRIVER_REPO:-$HOME/fcb_base_driver}"
SRC="$REPO/fcb_base_driver"
DEST="$BUNDLE_DIR/fcb_base_driver"

# Everything fcb_record.py actually imports, directly or transitively.
MODULES=(
    __init__.py
    visca.py
    visca_link.py
    zoom_map.py
    devices.py
    frame_grabber.py
    mavlink_source.py
    rc_source.py
    rc_switch.py
    zoom_servo.py
)

if [[ ! -d "$SRC" ]]; then
    echo "error: driver package not found at $SRC" >&2
    echo "       set FCB_DRIVER_REPO to the fcb_base_driver checkout" >&2
    exit 1
fi

rm -rf "$DEST"
mkdir -p "$DEST"
for module in "${MODULES[@]}"; do
    if [[ ! -f "$SRC/$module" ]]; then
        echo "error: $SRC/$module is missing" >&2
        exit 1
    fi
    cp "$SRC/$module" "$DEST/$module"
done

# A stray ROS import here would only fail on the drone, where rclpy is not
# installed and there is no good time to find out.
if grep -rqE '^\s*(import|from)\s+(rclpy|std_msgs|sensor_msgs|std_srvs)' "$DEST"; then
    echo "error: the bundled driver pulls in ROS modules:" >&2
    grep -rnE '^\s*(import|from)\s+(rclpy|std_msgs|sensor_msgs|std_srvs)' "$DEST" >&2
    exit 1
fi

echo "synced ${#MODULES[@]} modules into $DEST"
