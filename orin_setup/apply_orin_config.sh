#!/usr/bin/env bash
# Re-apply the system settings made on the old Orin (uas@uas-desktop) during
# the Oct 2026 Oppila-board work. Everything else lives in ~/fcb_orin and is
# copied by deploy.sh / fly.sh. Run ON the Orin:  ./apply_orin_config.sh
#
# Idempotent: safe to run again.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAPTOP="${FCB_LAPTOP:-r2d2@10.42.0.1}"

# 1. WiFi: UAS_Council with a static 192.168.0.77, power saving off.
#    (The card is 2.4 GHz-only, so UAS-DTU_5G is out of reach.)
if nmcli -t -f NAME con show | grep -qx UAS_Council; then
    sudo nmcli con mod UAS_Council ipv4.method manual \
        ipv4.addresses 192.168.0.77/24 ipv4.gateway 192.168.0.1 \
        ipv4.dns "192.168.0.1 8.8.8.8" ipv4.route-metric 600 \
        802-11-wireless.powersave 2 connection.autoconnect yes
    echo "UAS_Council: static 192.168.0.77, powersave off"
else
    echo "no UAS_Council profile -- join it once (nmcli dev wifi connect UAS_Council password ...) and re-run"
fi

# 2. The RTL8188FTV driver's own power management off (takes effect on boot).
sudo install -m 0644 "$HERE/rtl8188fu-nopowersave.conf" /etc/modprobe.d/rtl8188fu-nopowersave.conf
echo "installed /etc/modprobe.d/rtl8188fu-nopowersave.conf"

# 3. Recordings in ~/flight_recordings, with ~/fcb_recordings pointing at it.
mkdir -p ~/flight_recordings
if [ -d ~/fcb_recordings ] && [ ! -L ~/fcb_recordings ]; then
    mv -n ~/fcb_recordings/* ~/flight_recordings/ 2>/dev/null || true
    rmdir ~/fcb_recordings && ln -s flight_recordings ~/fcb_recordings
elif [ ! -e ~/fcb_recordings ]; then
    ln -s flight_recordings ~/fcb_recordings
fi
echo "recordings: ~/flight_recordings (~/fcb_recordings -> it)"

# 4. Where fly.sh copies recordings on Ctrl-b d: the laptop's ~/fcb_recordings.
mkdir -p ~/.config/fcb_orin
echo "$LAPTOP:fcb_recordings" > ~/.config/fcb_orin/gcs_dest
echo "copy destination: $LAPTOP:fcb_recordings"

# 5. Key login Orin -> laptop (for the copy and the clock sync). Needs the
#    laptop's password once; skipped if it already works.
[ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
if ssh -o BatchMode=yes -o ConnectTimeout=5 "$LAPTOP" true 2>/dev/null; then
    echo "key login to $LAPTOP: already works"
else
    ssh-keygen -R "${LAPTOP#*@}" >/dev/null 2>&1 || true
    echo "run once (asks for the laptop password):  ssh-copy-id $LAPTOP"
fi
