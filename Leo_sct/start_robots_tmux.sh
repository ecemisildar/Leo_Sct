#!/usr/bin/env bash
set -euo pipefail

SESSION="leo_robots"

# Format: "user@ip domain_id [robot_ns]"
ROBOTS=(
  "jetson-01@192.168.178.101 1"
  "jetson-02@192.168.178.102 2"
  "jetson-03@192.168.178.103 3 rob_1"
  "jetson-04@192.168.178.104 4 rob_2"
)

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION"

IDX=0
for entry in "${ROBOTS[@]}"; do
  HOST=$(awk '{print $1}' <<<"$entry")
  DID=$(awk '{print $2}' <<<"$entry")
  NS=$(awk '{print $3}' <<<"$entry")   # may be empty

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
"ssh -t $HOST 'source /opt/ros/humble/setup.bash; source ~/ros_ws/install/setup.bash; export ROS_DOMAIN_ID=$DID; export ROS_LOCALHOST_ONLY=0; ${BRIDGE_PREP}ros2 run rosbridge_server rosbridge_websocket --ros-args -p address:=0.0.0.0 -p port:=9090 >/tmp/rosbridge_websocket.log 2>&1 & sleep 1; ros2 launch leo_real leo_real.launch.py $LAUNCH_ARGS'" C-m

  IDX=$((IDX+1))
done

tmux attach -t "$SESSION"
