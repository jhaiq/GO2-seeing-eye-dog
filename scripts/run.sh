#!/usr/bin/env bash
#
# run.sh, start the canonical stack.
#
# Environment:
#   PERCEPTION        real | none          (default: real)
#   PLANNER           staged | nav2        (default: staged)
#   HARDWARE_ADAPTER  dry_run | unitree_sport  (default: dry_run)
#   DRY_RUN_LOG       path to a JSONL actuation log (default: none)
#   LOG_LEVEL         ROS log level        (default: info)
#
# The default is dry_run: no hardware is touched. Selecting a physical adapter
# is a deliberate act, and this script confirms it before proceeding.
#
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[run] ROS 2 Humble setup not found under /opt/ros/humble." >&2
  exit 1
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
else
  echo "[run] No install space. Build first, or run ./scripts/reproduce.sh." >&2
  exit 1
fi

PERCEPTION="${PERCEPTION:-real}"
PLANNER="${PLANNER:-staged}"
HARDWARE_ADAPTER="${HARDWARE_ADAPTER:-dry_run}"
DRY_RUN_LOG="${DRY_RUN_LOG:-}"
LOG_LEVEL="${LOG_LEVEL:-info}"

if [[ "$HARDWARE_ADAPTER" != "dry_run" ]]; then
  echo
  echo "  ############################################################"
  echo "  #  PHYSICAL HARDWARE ADAPTER SELECTED: $HARDWARE_ADAPTER"
  echo "  #"
  echo "  #  This code path has NEVER been executed against a GO2 by"
  echo "  #  this repository. Every velocity and acceleration limit in"
  echo "  #  config/safety.yaml is an unvalidated desk default."
  echo "  #"
  echo "  #  Have a physical emergency stop within reach."
  echo "  ############################################################"
  echo
  read -r -p "  Type 'yes' to continue: " CONFIRM
  if [[ "$CONFIRM" != "yes" ]]; then
    echo "[run] Aborted."
    exit 1
  fi
fi

echo "[run] perception=${PERCEPTION} planner=${PLANNER} adapter=${HARDWARE_ADAPTER}"

exec ros2 launch go2_bringup system.launch.py \
  perception:="${PERCEPTION}" \
  planner:="${PLANNER}" \
  hardware_adapter:="${HARDWARE_ADAPTER}" \
  dry_run_log_path:="${DRY_RUN_LOG}" \
  log_level:="${LOG_LEVEL}"
