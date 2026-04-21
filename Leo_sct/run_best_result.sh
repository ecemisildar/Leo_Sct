#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BEST_RESULTS_DIR="$SCRIPT_DIR/best_results"

usage() {
  cat <<'EOF'
Usage:
  ./run_best_result.sh [--list]
  ./run_best_result.sh --latest [launch_arg:=value ...]
  ./run_best_result.sh <yaml_name> [launch_arg:=value ...]

Examples:
  ./run_best_result.sh --list
  ./run_best_result.sh --latest
  ./run_best_result.sh explore_sup_gpt_medium_1_run_3_81.yaml robot_ns:=rob_1
EOF
}

list_yamls() {
  find "$BEST_RESULTS_DIR" -maxdepth 1 -type f -name '*.yaml' -printf '%f\n' | sort
}

latest_yaml() {
  find "$BEST_RESULTS_DIR" -maxdepth 1 -type f -name '*.yaml' -printf '%T@ %p\n' \
    | sort -nr \
    | head -n 1 \
    | cut -d' ' -f2-
}

resolve_yaml() {
  local input="$1"
  local path=""

  if [[ -f "$BEST_RESULTS_DIR/$input" ]]; then
    path="$BEST_RESULTS_DIR/$input"
  elif [[ -f "$input" ]]; then
    path="$input"
  fi

  if [[ -z "$path" ]]; then
    echo "YAML not found: $input" >&2
    exit 1
  fi

  realpath "$path"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ ! -d "$BEST_RESULTS_DIR" ]]; then
  echo "best_results directory not found: $BEST_RESULTS_DIR" >&2
  exit 1
fi

if [[ "${1:-}" == "--list" ]]; then
  list_yamls
  exit 0
fi

if [[ "$#" -eq 0 ]]; then
  usage
  exit 1
fi

YAML_PATH=""
if [[ "${1:-}" == "--latest" ]]; then
  YAML_PATH="$(latest_yaml)"
  if [[ -z "$YAML_PATH" ]]; then
    echo "No YAML files found in $BEST_RESULTS_DIR" >&2
    exit 1
  fi
  shift
else
  YAML_PATH="$(resolve_yaml "$1")"
  shift
fi

if [[ ! -f "$YAML_PATH" ]]; then
  echo "Resolved YAML does not exist: $YAML_PATH" >&2
  exit 1
fi

echo "Launching supervisor with $(basename "$YAML_PATH")"

exec ros2 launch leo_real leo_real.launch.py \
  enable_supervisor:=true \
  "supervisor_yaml_path:=$YAML_PATH" \
  "$@"
