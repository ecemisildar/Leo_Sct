#!/usr/bin/env bash
set -u

ROBOTS=(
  "jetson-01@192.168.178.101"
  "jetson-02@192.168.178.102"
  "jetson-03@192.168.178.103"
  "jetson-04@192.168.178.104"
)

WS="~/ros_ws"
REPO="$WS/src/Leo_Sct"
BRANCH="real"

run_robot() {
  local R="$1"
  echo "========== Updating $R =========="
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$R" "
    set -e
    cd $REPO
    git fetch origin
    git reset --hard
    git checkout -f -B $BRANCH origin/$BRANCH
    git reset --hard origin/$BRANCH
    git clean -fd

    cd $WS
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install

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
