#!/usr/bin/env bash
set -euo pipefail

echo "[inspect] topic list"
agiros topic list

echo "[inspect] confirmed target topic"
agiros topic info /go2/confirmed_target -v || true

echo "[inspect] safety alert topic"
agiros topic info /go2/safety_alert -v || true

echo "[inspect] detected humans rate"
agiros topic hz /go2/detected_humans || true

echo "[inspect] tf tree"
agiros run tf2_tools view_frames || true

echo "[inspect] parameter dump"
agiros param dump /intent_grounding_node || true

echo "[inspect] suggested bag capture"
echo "agiros bag record /go2/detected_humans /go2/confirmed_target /go2/safety_alert /go2/voice_command /tf /tf_static"

