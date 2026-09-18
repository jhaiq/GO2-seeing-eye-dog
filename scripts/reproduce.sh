#!/usr/bin/env bash
#
# reproduce.sh, clean-workspace reproduction of every claim in this repository.
#
#   ./scripts/reproduce.sh
#
# Builds from scratch and runs the full test suite in an isolated ROS domain.
#
# Isolation matters here. These tests assert things like "no command reached
# the actuator", and such an assertion can pass for the wrong reason if a node
# from another session is publishing on the same graph, or fail for the wrong
# reason if another process is competing for the domain. So this script:
#
#   * removes build/, install/ and log/ so nothing stale is linked in;
#   * unsets any inherited ROS workspace overlay;
#   * picks a ROS_DOMAIN_ID from this shell's PID;
#   * refuses to start if a ROS graph is already visible on that domain.
#
# `set -u` is deliberately NOT used: ROS 2 setup files reference unset
# variables (AMENT_TRACE_SETUP_FILES and friends) and abort under it.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: no ROS 2 at $ROS_SETUP" >&2
  echo "Set ROS_DISTRO, or install ROS 2 Humble." >&2
  exit 1
fi

# ── Isolation ───────────────────────────────────────────────────────────────
# Drop any overlay the caller had sourced. A previously-built go2_msgs in
# ~/ros2_ws would otherwise shadow the one built here, and the tests would be
# exercising code that is not in this tree.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_OVERRIDE:-$(( 40 + ($$ % 55) ))}"
export ROS_LOCALHOST_ONLY=1

echo "=============================================================="
echo " GO2 seeing-eye-dog, clean reproduction"
echo "=============================================================="
echo " repo            : $REPO_ROOT"
echo " ROS distro      : $ROS_DISTRO_NAME"
echo " ROS_DOMAIN_ID   : $ROS_DOMAIN_ID (isolated)"
echo " localhost only  : $ROS_LOCALHOST_ONLY"
echo

# shellcheck disable=SC1090
source "$ROS_SETUP"

# ── Refuse to run against a populated graph ─────────────────────────────────
echo "── Checking the domain is empty ─────────────────────────────"
EXISTING="$(timeout 8 ros2 node list 2>/dev/null | grep -v '^$' | grep -v '_ros2cli' || true)"
if [[ -n "$EXISTING" ]]; then
  echo "ERROR: ROS nodes already running on domain $ROS_DOMAIN_ID:" >&2
  echo "$EXISTING" >&2
  echo >&2
  echo "These tests assert on what does and does not reach an actuator, so a" >&2
  echo "foreign graph can make them pass or fail for the wrong reason." >&2
  echo "Stop those nodes, or set ROS_DOMAIN_ID_OVERRIDE to a free domain." >&2
  exit 1
fi
echo "Domain $ROS_DOMAIN_ID is clear."
echo

# ── Clean build ─────────────────────────────────────────────────────────────
echo "── Removing build/, install/, log/ ──────────────────────────"
rm -rf build install log
echo

echo "── Building go2_msgs ────────────────────────────────────────"
colcon build --symlink-install --packages-select go2_msgs
# shellcheck disable=SC1091
source install/setup.bash
echo

# Everything, not just the new packages: go2_bringup declares exec_depends on
# the perception packages, and colcon's ament_python build step verifies those
# are present in the install space. Building a subset leaves it unsatisfiable.
#
# Building the perception packages does not require their runtime dependencies
# (pyaudio, ultralytics, whisper), setup.py does not import them. Only running
# their nodes does, which is why the tests skip them.
echo "── Building the full workspace ──────────────────────────────"
colcon build --symlink-install
# shellcheck disable=SC1091
source install/setup.bash
echo

# ── Verify the built graph, not the source tree ─────────────────────────────
echo "── Verifying installed executables ──────────────────────────"
for exe in \
  "go2_safety_arbiter safety_arbiter_node" \
  "go2_hardware_bridge hardware_bridge_node" \
  "go2_approach_controller approach_controller_node" \
  "go2_lidar_safety lidar_hazard_node"
do
  # shellcheck disable=SC2086
  set -- $exe
  if ros2 pkg executables "$1" 2>/dev/null | grep -q "$2"; then
    echo "  OK   $1 $2"
  else
    echo "  FAIL $1 $2 is not installed as a runnable executable" >&2
    exit 1
  fi
done
echo

echo "── Verifying launch files are installed ─────────────────────"
SHARE="$(ros2 pkg prefix go2_bringup)/share/go2_bringup"
for f in launch/system.launch.py launch/system_dry_run.launch.py \
         launch/motion_authority.launch.py \
         config/safety.yaml config/fusion.yaml config/navigation.yaml; do
  if [[ -e "$SHARE/$f" ]]; then
    echo "  OK   $f"
  else
    echo "  FAIL $f missing from the install space" >&2
    exit 1
  fi
done
echo

# ── Lint ────────────────────────────────────────────────────────────────────
echo "── Lint ─────────────────────────────────────────────────────"
# `ruff check` only, matching scripts/lint.sh and CI. `ruff format` is not
# part of this repository's convention and running it here would report a
# failure for style the project never adopted.
if command -v ruff >/dev/null 2>&1; then
  ruff check . || {
    echo "Lint failed." >&2
    exit 1
  }
else
  echo "  ruff not installed; skipping (CI runs it)"
fi
echo

# ── Tests ───────────────────────────────────────────────────────────────────
echo "── Test suite ───────────────────────────────────────────────"
set +e
python3 -m pytest -v --tb=short
STATUS=$?
set -e
echo

echo "=============================================================="
if [[ $STATUS -eq 0 ]]; then
  echo " PASS, all tests green on isolated domain $ROS_DOMAIN_ID"
  echo
  echo " What this run does and does not establish:"
  echo "   PROVEN : the safety arbiter holds final motion authority, and no"
  echo "            command reached the dry-run actuator without it."
  echo "   NOT PROVEN : anything about physical GO2 hardware. Every actuation"
  echo "            in this run went to DryRunGo2Bridge. No robot was moved."
else
  echo " FAIL, see output above"
fi
echo "=============================================================="
exit $STATUS
