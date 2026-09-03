#!/usr/bin/env bash
# Copy this bundle to the Orin.
#
#   ./deploy.sh orin                     # an ssh host or alias
#   ./deploy.sh nvidia@192.168.1.50
#   ./deploy.sh orin /opt/fcb            # somewhere other than ~/fcb_orin
#
# The driver copy is refreshed from the git checkout first, so what lands
# on the drone is never a stale build of it.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${1:-}"
REMOTE_DIR="${2:-fcb_orin}"

if [[ -z "$TARGET" ]]; then
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
    exit 1
fi

"$HERE/sync_driver.sh"

echo "copying to $TARGET:$REMOTE_DIR ..."
if command -v rsync >/dev/null 2>&1; then
    rsync -av --delete \
        --exclude '__pycache__' --exclude '*.pyc' \
        --exclude '.git' --exclude '.pytest_cache' \
        "$HERE/" "$TARGET:$REMOTE_DIR/"
else
    # rsync is not always on a fresh JetPack image; scp still gets it there.
    echo "rsync not found, falling back to scp"
    ssh "$TARGET" "mkdir -p '$REMOTE_DIR'"
    scp -r "$HERE"/*.py "$HERE"/*.sh "$HERE/fcb_base_driver" \
        "$TARGET:$REMOTE_DIR/"
fi

ssh "$TARGET" "chmod +x '$REMOTE_DIR'/*.sh '$REMOTE_DIR'/fcb_record.py"

cat <<EOF

deployed. Next:

    ssh $TARGET
    cd $REMOTE_DIR
    ./check_deps.sh          # confirm the Orin has what it needs
    ./start_recorder.sh      # run it inside tmux

EOF
