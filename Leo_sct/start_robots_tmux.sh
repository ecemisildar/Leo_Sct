#!/usr/bin/env bash
set -euo pipefail

SESSION="leo_robots"

# Robot list: "user@ip namespace"
ROBOTS=(
  "jetson-03@192.168.178.103 rob_1"
  "jetson-04@192.168.178.104 rob_2"
  # "jetson-05@192.168.178.105 rob_3"
  # "jetson-06@192.168.178.106 rob_4"
  # "jetson-07@192.168.178.107 rob_5"
)

# Kill existing session if already running
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Create new session
tmux new-session -d -s "$SESSION"

IDX=0
for entry in "${ROBOTS[@]}"; do
  HOST=$(awk '{print $1}' <<<"$entry")
  NS=$(awk '{print $2}' <<<"$entry")

  # create a new pane for every robot after the first
  if [ "$IDX" -ne 0 ]; then
    tmux split-window -t "$SESSION":0 -v
  fi

  tmux select-layout -t "$SESSION":0 tiled

  tmux send-keys -t "$SESSION":0.$IDX \
"ssh -t $HOST 'source /opt/ros/humble/setup.bash; source ~/ros_ws/install/setup.bash; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; ros2 launch leo_real leo_real.launch.py enable_supervisor:=false robot_ns:=$NS'" C-m

  IDX=$((IDX+1))
done

tmux attach -t "$SESSION"
