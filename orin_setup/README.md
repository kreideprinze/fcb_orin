# Changes made on the Orins outside ~/fcb_orin

The code (`fly.sh`, `fcb_record.py`, `fcb_base_driver/`, ...) lives in this
laptop checkout and is copied to the drone by `deploy.sh` / `fly.sh`. This
folder holds what was changed on the Orins *outside* the bundle, so none of
it exists only on a drone.

## Old Orin -- `uas@uas-desktop` (10.42.0.224)

Re-apply all of these with `./apply_orin_config.sh`, run on the Orin.

| What | Where on the Orin | Why |
|---|---|---|
| WiFi `UAS_Council`, static **192.168.0.77**, gateway 192.168.0.1, power saving off | NetworkManager profile `UAS_Council` | the card (RTL8188FTV) is 2.4 GHz only; `UAS-DTU_5G` is out of reach. The `UAS-DTU` profile made earlier was deleted |
| RTL8188FTV driver power management off | `/etc/modprobe.d/rtl8188fu-nopowersave.conf` (copy here) | asleep, it missed ARP and was unreachable over WiFi |
| Recordings folder | `~/flight_recordings`, with `~/fcb_recordings` a symlink to it | all recordings in one new folder; old paths still work |
| Copy-on-detach destination | `~/.config/fcb_orin/gcs_dest` = `r2d2@10.42.0.1:fcb_recordings` | was a leftover `/tmp/fcbtest` |
| Key login Orin -> laptop | Orin `~/.ssh/known_hosts` (laptop key for 10.42.0.1, replacing another machine's); laptop `~/.ssh/authorized_keys` (Orin key `uas@uas-desktop`) | for copy-on-detach and the clock sync in `fly.sh` |
| ROS 2 test build of the Oppila branch | `~/fcb_ros_oppila` (built against `~/fcb-ros/install`) | source is `~/fcb_ros`, branch `oppila-board-support`, on this laptop |
| Diagnostic scripts | `~/stream_probe.py`, `/tmp/enc_bench.py`, `/tmp/visca_integrity.py` | copies in `diagnostics/` here, bugs fixed |

Left alone on purpose: `~/new_fcb` (the separate neohd recorder, which only
exists on this Orin -- copy it here if it matters) and `~/fcb-ros` (your own
ROS workspace with local edits).

## New Orin -- `uasdtu@uasnx` (10.42.0.114, Orin NX on AVerMedia D131, JetPack 7)

Not scripted -- one-off fixes, recorded so they can be redone:

- `UAS-DTU_5G` profile set to static **192.168.1.69/24**, gateway 192.168.1.1,
  route metric 200 (the Alfa card never stayed up long enough to use it).
- An `apt upgrade` (L4T 39.2.0 -> 39.2.1) left `nvidia-l4t-bootloader` half
  configured: its post-install script rejects the D131 board. Fixed with
  `sudo mv /var/lib/dpkg/info/nvidia-l4t-bootloader.postinst /root/...bak`,
  `sudo dpkg --configure -a`, and `sudo apt-mark hold nvidia-l4t-bootloader`.
- Installed: `tmux v4l-utils python3-opencv python3-pip python3-lxml
  nvidia-l4t-gstreamer`, `pip3 install --user --break-system-packages
  pymavlink`; user added to `dialout`.
- The `mt76-usb` DKMS driver (Alfa MT7921AU) rebuilt for the new kernel:
  `sudo dkms build/install mt76-usb/6.8.12 -k $(uname -r) --force`.
- A runtime-only USB quirk (`0e8d:7961:gk`) was tried and is gone after a reboot.

## Orin NX -- `shadow2@uasdtu` (10.42.0.223, JetPack 7 / L4T R39.2.1, Ubuntu 24.04)

Set up 2026-10-07:

- `~/fcb_orin` deployed from this laptop; recordings in `~/flight_recordings`.
- A pip `opencv-python` 5.0.0 (no GStreamer) in `/usr/local` shadows Ubuntu's
  `python3-opencv` 4.6 (with GStreamer). `prefer_cv2.py` makes the bundle load
  the system one, so NVENC works without uninstalling anything.
- Key login Orin -> laptop: Orin key `shadow2@uasdtu` in the laptop's
  `~/.ssh/authorized_keys`; laptop host key for 10.42.0.1 in the Orin's
  `~/.ssh/known_hosts`.
- Copy-on-detach destination: `~/.config/fcb_orin/gcs_dest` =
  `r2d2@10.42.0.1:fcb_recordings` (without the user it tried `shadow2` on the laptop).
- Its clock starts at a stale date after power-off (no RTC battery), like the old Orin.

## Oppila board quirks this all works around

See the "Camera interface boards" section of `../README.md`. In short:
replies held until 32 bytes; ~25% of control bytes lost or corrupted while
video streams; every 4th frame repeated and 30 fps reported for 60; video
does not survive a USB unplug, a stream restart or the Orin booting while
the board stays powered -- only a 12 V power cycle clears it.
`fcb_record.py --board-power-cycle-cmd` is the hook for an Orin-controlled
12 V switch.
