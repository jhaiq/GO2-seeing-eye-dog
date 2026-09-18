#!/usr/bin/env bash
#
# test.sh — run the test suite.
#
# Without ROS on the path this runs the pure-function tests only; the
# node, integration and launch tests skip themselves (see conftest.py's
# `requires_ros`). That is what the ROS-free CI job exercises.
#
# For the full suite including the safety-authority proofs, use
# ./scripts/reproduce.sh, which builds the workspace and isolates the ROS
# domain first.
#
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[test] pytest"
python3 -m pytest "$@"
echo "[test] complete"
