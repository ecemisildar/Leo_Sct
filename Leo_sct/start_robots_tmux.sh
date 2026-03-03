#!/usr/bin/env bash
set -euo pipefail

SESSION="leo_robots"

# Format: "robot_name|user@ip|domain_id|robot_ns"
ROBOTS=(
  "Robot1|jetson-01@192.168.178.101|1|"
  "Robot2|jetson-02@192.168.178.102|2|"
  "Robot3|jetson-03@192.168.178.103|3|rob_1"
  "Robot4|jetson-04@192.168.178.104|4|rob_2"
)

usage() {
  cat <<'EOF'
Usage:
  ./start_robots_tmux.sh [--list] [robot_name ...]

Examples:
  ./start_robots_tmux.sh
  ./start_robots_tmux.sh Robot1 Robot3
  ./start_robots_tmux.sh --list
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "${1:-}" == "--list" ]]; then
  for entry in "${ROBOTS[@]}"; do
    IFS='|' read -r NAME _HOST _DID _NS <<<"$entry"
    echo "$NAME"
  done
  exit 0
fi

declare -a SELECTED
if [[ "$#" -eq 0 ]]; then
  SELECTED=("${ROBOTS[@]}")
else
  declare -A WANT
  for arg in "$@"; do
    WANT["${arg,,}"]=1
  done

  for entry in "${ROBOTS[@]}"; do
    IFS='|' read -r NAME _HOST _DID _NS <<<"$entry"
    if [[ -n "${WANT[${NAME,,}]:-}" ]]; then
      SELECTED+=("$entry")
    fi
  done

  if [[ "${#SELECTED[@]}" -eq 0 ]]; then
    echo "No matching robot names."
    echo "Run './start_robots_tmux.sh --list' to see valid names."
    exit 1
  fi
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION"

IDX=0
for entry in "${SELECTED[@]}"; do
  IFS='|' read -r NAME HOST DID NS <<<"$entry"

  if [ "$IDX" -ne 0 ]; then
    tmux split-window -t "$SESSION":0 -v
  fi
  tmux select-layout -t "$SESSION":0 tiled

  # Build launch args (robot_ns optional)
  LAUNCH_ARGS="enable_supervisor:=false"
  if [ -n "${NS:-}" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS robot_ns:=$NS"
  fi

  BRIDGE_PREP=""
  if [ "$DID" = "3" ] || [ "$DID" = "4" ]; then
    # For robots 3 and 4: free rosbridge port if a stale process still owns 9090.
    BRIDGE_PREP="fuser -k 9090/tcp >/dev/null 2>&1 || true; sleep 1; "
  fi

  tmux send-keys -t "$SESSION":0.$IDX \
"echo '[${NAME}] connecting to ${HOST}'; ssh -t $HOST 'source /opt/ros/humble/setup.bash; source ~/ros_ws/install/setup.bash; export ROS_DOMAIN_ID=$DID; export ROS_LOCALHOST_ONLY=0; ${BRIDGE_PREP}ros2 run rosbridge_server rosbridge_websocket --ros-args -p address:=0.0.0.0 -p port:=9090 >/tmp/rosbridge_websocket.log 2>&1 & sleep 1; ros2 launch leo_real leo_real.launch.py $LAUNCH_ARGS'" C-m

  IDX=$((IDX+1))
done

tmux attach -t "$SESSION"
