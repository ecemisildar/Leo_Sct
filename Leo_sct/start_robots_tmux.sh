#!/usr/bin/env bash
set -euo pipefail

SESSION="leo_robots"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BEST_RESULTS_DIR="$SCRIPT_DIR/best_results"
REMOTE_REPO="${REMOTE_REPO:-~/ros_ws/src/Leo_Sct/Leo_sct}"
REMOTE_BEST_RESULTS_DIR="$REMOTE_REPO/best_results"

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
  ./start_robots_tmux.sh [--list] [--list-yamls] [--latest | --yaml id|file.yaml] [robot_name ...]

Examples:
  ./start_robots_tmux.sh
  ./start_robots_tmux.sh Robot1 Robot3
  ./start_robots_tmux.sh --list
  ./start_robots_tmux.sh --list-yamls
  ./start_robots_tmux.sh --latest Robot2
  ./start_robots_tmux.sh --yaml 2 Robot3 Robot4
EOF
}

list_yamls() {
  local idx=1
  while IFS= read -r name; do
    printf '%d %s\n' "$idx" "$name"
    idx=$((idx + 1))
  done < <(find "$BEST_RESULTS_DIR" -maxdepth 1 -type f -name '*.yaml' -printf '%f\n' | sort)
}

yaml_names_sorted() {
  find "$BEST_RESULTS_DIR" -maxdepth 1 -type f -name '*.yaml' -printf '%f\n' | sort
}

yaml_name_by_id() {
  local target_id="$1"
  local idx=1
  local name
  while IFS= read -r name; do
    if [[ "$idx" == "$target_id" ]]; then
      printf '%s\n' "$name"
      return 0
    fi
    idx=$((idx + 1))
  done < <(yaml_names_sorted)
  return 1
}

latest_yaml_name() {
  find "$BEST_RESULTS_DIR" -maxdepth 1 -type f -name '*.yaml' -printf '%T@ %f\n' \
    | sort -nr \
    | head -n 1 \
    | cut -d' ' -f2-
}

resolve_yaml_name() {
  local input="$1"

  if [[ "$input" =~ ^[0-9]+$ ]]; then
    if yaml_name_by_id "$input"; then
      return 0
    fi
    echo "YAML id not found: $input" >&2
    exit 1
  fi

  if [[ -f "$BEST_RESULTS_DIR/$input" ]]; then
    basename "$input"
    return 0
  fi

  if [[ -f "$input" ]]; then
    basename "$input"
    return 0
  fi

  echo "YAML not found in best_results: $input" >&2
  exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ ! -d "$BEST_RESULTS_DIR" ]]; then
  echo "best_results directory not found: $BEST_RESULTS_DIR" >&2
  exit 1
fi

YAML_NAME=""
declare -a ROBOT_ARGS=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --list)
      for entry in "${ROBOTS[@]}"; do
        IFS='|' read -r NAME _HOST _DID _NS <<<"$entry"
        echo "$NAME"
      done
      exit 0
      ;;
    --list-yamls)
      list_yamls
      exit 0
      ;;
    --latest)
      if [[ -n "$YAML_NAME" ]]; then
        echo "Only one of --latest or --yaml may be used." >&2
        exit 1
      fi
      YAML_NAME="$(latest_yaml_name)"
      if [[ -z "$YAML_NAME" ]]; then
        echo "No YAML files found in $BEST_RESULTS_DIR" >&2
        exit 1
      fi
      shift
      ;;
    --yaml)
      if [[ -n "$YAML_NAME" ]]; then
        echo "Only one of --latest or --yaml may be used." >&2
        exit 1
      fi
      if [[ "$#" -lt 2 ]]; then
        echo "--yaml requires a file name." >&2
        exit 1
      fi
      YAML_NAME="$(resolve_yaml_name "$2")"
      shift 2
      ;;
    *)
      ROBOT_ARGS+=("$1")
      shift
      ;;
  esac
done

declare -a SELECTED
if [[ "${#ROBOT_ARGS[@]}" -eq 0 ]]; then
  SELECTED=("${ROBOTS[@]}")
else
  declare -A WANT
  for arg in "${ROBOT_ARGS[@]}"; do
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
  if [[ -n "$YAML_NAME" ]]; then
    LAUNCH_ARGS="enable_supervisor:=true supervisor_yaml_path:=$REMOTE_BEST_RESULTS_DIR/$YAML_NAME"
  fi
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
