#!/usr/bin/env bash
set -euo pipefail

ROBOTS=(
  "jetson-01@192.168.178.101"
  "jetson-02@192.168.178.102"
  "jetson-03@192.168.178.103"
  "jetson-04@192.168.178.104"
  "jetson-05@192.168.178.105"
)

WS="~/ros_ws"
REPO="$WS/src/Leo_Sct/Leo_sct"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"

run_robot() {
  local R="$1"
  local HOST="${R#*@}"
  local USER="${R%@*}"
  local PADDED="${USER#jetson-}"
  local ROSBRIDGE_SERVICE="jetson-${PADDED}-rosbridge.service"

  echo "========== Updating $R =========="
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$R" "mkdir -p $REPO"

  rsync -az --delete \
    --exclude build \
    --exclude install \
    --exclude log \
    "$LOCAL_REPO/leo_real/" "$R:$REPO/leo_real/"

  rsync -az --delete \
    --exclude build \
    --exclude install \
    --exclude log \
    "$LOCAL_REPO/leo_image/" "$R:$REPO/leo_image/"

  ssh -o BatchMode=yes -o ConnectTimeout=10 "$R" "
    set -e
    cd $REPO
    echo '[update] repo=' \$(pwd)
    find . -type d -name __pycache__ -prune -exec rm -rf {} +

    cd $WS
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select leo_real leo_image

    echo \"Finished on \$(hostname)\"
  "
}

pids=()
for R in "${ROBOTS[@]}"; do
  run_robot "$R" &
  pids+=($!)
done

fail=0
for pid in "${pids[@]}"; do
  wait "$pid" || fail=1
done

if [ "$fail" -ne 0 ]; then
  echo "One or more robots failed."
  exit 1
fi

echo "All robots updated and built successfully."
