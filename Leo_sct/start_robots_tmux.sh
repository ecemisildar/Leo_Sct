#!/usr/bin/env bash
set -euo pipefail

SESSION="leo_robots"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BEST_RESULTS_DIR="$SCRIPT_DIR/best_results"
CONFIG_YAML_DIR="$SCRIPT_DIR/leo_real/config"
REMOTE_REPO="${REMOTE_REPO:-}"

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
  ./start_robots_tmux.sh [--list] [--list-yamls] [--list-config-yamls] [--no-stop-service]
                         [--latest | --yaml id|group/id|file.yaml | --config-yaml id|file.yaml]
                         [robot_name ...]

Examples:
  ./start_robots_tmux.sh
  ./start_robots_tmux.sh Robot1 Robot3
  ./start_robots_tmux.sh --list
  ./start_robots_tmux.sh --list-yamls
  ./start_robots_tmux.sh --list-config-yamls
  ./start_robots_tmux.sh --latest Robot2
  ./start_robots_tmux.sh --yaml 2 Robot3 Robot4
  ./start_robots_tmux.sh --yaml 5.4/1 Robot1
  ./start_robots_tmux.sh --config-yaml prompt_8_run_001.yaml Robot1
  ./start_robots_tmux.sh --no-stop-service Robot1
EOF
}

yaml_paths_sorted() {
  find "$BEST_RESULTS_DIR" -mindepth 1 -type f -name '*.yaml' -printf '%P\n' | sort
}

list_yamls() {
  local last_group=""
  local group_idx=0
  local idx=1
  local path=""
  while IFS= read -r path; do
    local group="${path%/*}"
    local name="${path##*/}"
    if [[ "$group" == "$path" ]]; then
      group="."
    fi
    if [[ "$group" != "$last_group" ]]; then
      last_group="$group"
      group_idx=1
    fi
    printf '%d %s/%d %s\n' "$idx" "$group" "$group_idx" "$name"
    idx=$((idx + 1))
    group_idx=$((group_idx + 1))
  done < <(yaml_paths_sorted)
}

yaml_name_by_id() {
  local target_id="$1"
  local idx=1
  local path
  while IFS= read -r path; do
    if [[ "$idx" == "$target_id" ]]; then
      printf '%s\n' "$path"
      return 0
    fi
    idx=$((idx + 1))
  done < <(yaml_paths_sorted)
  return 1
}

yaml_name_by_group_id() {
  local target_group="$1"
  local target_id="$2"
  local idx=1
  local path
  while IFS= read -r path; do
    local group="${path%/*}"
    if [[ "$group" == "$path" ]]; then
      group="."
    fi
    if [[ "$group" == "$target_group" ]]; then
      if [[ "$idx" == "$target_id" ]]; then
        printf '%s\n' "$path"
        return 0
      fi
      idx=$((idx + 1))
    fi
  done < <(yaml_paths_sorted)
  return 1
}

latest_yaml_name() {
  find "$BEST_RESULTS_DIR" -mindepth 1 -type f -name '*.yaml' -printf '%T@ %P\n' \
    | sort -nr \
    | head -n 1 \
    | cut -d' ' -f2-
}

config_yaml_names_sorted() {
  find "$CONFIG_YAML_DIR" -maxdepth 1 -type f -name '*.yaml' ! -name 'real_camera.yaml' -printf '%f\n' | sort
}

list_config_yamls() {
  local idx=1
  local name
  while IFS= read -r name; do
    printf '%d %s\n' "$idx" "$name"
    idx=$((idx + 1))
  done < <(config_yaml_names_sorted)
}

config_yaml_name_by_id() {
  local target_id="$1"
  local idx=1
  local name
  while IFS= read -r name; do
    if [[ "$idx" == "$target_id" ]]; then
      printf '%s\n' "$name"
      return 0
    fi
    idx=$((idx + 1))
  done < <(config_yaml_names_sorted)
  return 1
}

resolve_config_yaml_name() {
  local input="$1"

  if [[ "$input" =~ ^[0-9]+$ ]]; then
    if config_yaml_name_by_id "$input"; then
      return 0
    fi
    echo "Config YAML id not found: $input" >&2
    exit 1
  fi

  if [[ -f "$CONFIG_YAML_DIR/$input" ]]; then
    basename "$input"
    return 0
  fi

  if [[ -f "$input" ]]; then
    basename "$input"
    return 0
  fi

  echo "YAML not found in leo_real/config: $input" >&2
  exit 1
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

  if [[ "$input" =~ ^([^/]+)/([0-9]+)$ ]]; then
    local group="${BASH_REMATCH[1]}"
    local group_id="${BASH_REMATCH[2]}"
    if yaml_name_by_group_id "$group" "$group_id"; then
      return 0
    fi
    echo "YAML id not found in group '$group': $group_id" >&2
    exit 1
  fi

  if [[ -f "$BEST_RESULTS_DIR/$input" ]]; then
    printf '%s\n' "$input"
    return 0
  fi

  if [[ -f "$input" ]]; then
    local rel_path="${input#"$BEST_RESULTS_DIR"/}"
    printf '%s\n' "$rel_path"
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
if [[ ! -d "$CONFIG_YAML_DIR" ]]; then
  echo "config YAML directory not found: $CONFIG_YAML_DIR" >&2
  exit 1
fi

YAML_NAME=""
CONFIG_YAML_NAME=""
STOP_LEO_SERVICE=1
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
    --list-config-yamls)
      list_config_yamls
      exit 0
      ;;
    --no-stop-service)
      STOP_LEO_SERVICE=0
      shift
      ;;
    --latest)
      if [[ -n "$YAML_NAME" || -n "$CONFIG_YAML_NAME" ]]; then
        echo "Only one of --latest, --yaml, or --config-yaml may be used." >&2
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
      if [[ -n "$YAML_NAME" || -n "$CONFIG_YAML_NAME" ]]; then
        echo "Only one of --latest, --yaml, or --config-yaml may be used." >&2
        exit 1
      fi
      if [[ "$#" -lt 2 ]]; then
        echo "--yaml requires a file name." >&2
        exit 1
      fi
      YAML_NAME="$(resolve_yaml_name "$2")"
      shift 2
      ;;
    --config-yaml)
      if [[ -n "$YAML_NAME" || -n "$CONFIG_YAML_NAME" ]]; then
        echo "Only one of --latest, --yaml, or --config-yaml may be used." >&2
        exit 1
      fi
      if [[ "$#" -lt 2 ]]; then
        echo "--config-yaml requires an id or file name." >&2
        exit 1
      fi
      CONFIG_YAML_NAME="$(resolve_config_yaml_name "$2")"
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
  REMOTE_USER="${HOST%@*}"
  REMOTE_REPO_FOR_HOST="${REMOTE_REPO:-/home/$REMOTE_USER/ros_ws/src/Leo_Sct/Leo_sct}"
  REMOTE_BEST_RESULTS_DIR="$REMOTE_REPO_FOR_HOST/best_results"
  REMOTE_CONFIG_YAML_DIR="$REMOTE_REPO_FOR_HOST/leo_real/config"

  if [ "$IDX" -ne 0 ]; then
    tmux split-window -t "$SESSION":0 -v
  fi
  tmux select-layout -t "$SESSION":0 tiled

  # Build launch args (robot_ns optional)
  LAUNCH_ARGS="enable_supervisor:=true"
  if [[ -n "$YAML_NAME" ]]; then
    LAUNCH_ARGS="enable_supervisor:=true supervisor_yaml_path:=$REMOTE_BEST_RESULTS_DIR/$YAML_NAME"
  elif [[ -n "$CONFIG_YAML_NAME" ]]; then
    LAUNCH_ARGS="enable_supervisor:=true supervisor_yaml_path:=$REMOTE_CONFIG_YAML_DIR/$CONFIG_YAML_NAME"
  fi
  if [ -n "${NS:-}" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS robot_ns:=$NS"
  fi

  REMOTE_PREP=""
  if [[ "$STOP_LEO_SERVICE" == "1" ]]; then
    REMOTE_PREP="if systemctl list-unit-files leo-ros.service --no-legend 2>/dev/null | grep -q \"^leo-ros.service\"; then echo \"[${NAME}] stopping leo-ros.service to free rover ports\"; sudo -n systemctl stop leo-ros.service || { echo \"[${NAME}] error: could not stop leo-ros.service with passwordless sudo\"; exit 1; }; fi; "
  fi
  REMOTE_PREP="${REMOTE_PREP}pkill -u \"\$USER\" -f \"ros2 launch leo_real leo_real.launch.py\" 2>/dev/null || true; pkill -u \"\$USER\" -f \"web_video_server\" 2>/dev/null || true; sleep 1; if command -v ss >/dev/null 2>&1 && ss -H -ltn \"sport = :8080\" | grep -q .; then echo \"[${NAME}] error: port 8080 is still in use; stop the process using it and rerun\"; exit 1; fi; "

  tmux send-keys -t "$SESSION":0.$IDX \
"echo '[${NAME}] connecting to ${HOST}'; ssh -t $HOST '${REMOTE_PREP}source /opt/ros/humble/setup.bash; source ~/ros_ws/install/setup.bash; export ROS_DOMAIN_ID=$DID; export ROS_LOCALHOST_ONLY=0; ros2 launch leo_real leo_real.launch.py $LAUNCH_ARGS'" C-m

  IDX=$((IDX+1))
done

tmux attach -t "$SESSION"
