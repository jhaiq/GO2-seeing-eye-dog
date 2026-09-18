#!/usr/bin/env bash
#
# deploy_preflight.sh: read-only go/no-go checks on the robot payload before
# any autonomous-navigation session. Commands NOTHING; publishes NOTHING.
#
#   ./scripts/deploy_preflight.sh            # after sourcing ROS + this workspace
#
# Every check below corresponds to a failure measured on the real robot or in
# closed-loop simulation (docs/go2_field_notes.md, docs/DEPLOYMENT.md). Exit
# status is the number of FAILed checks; WARN does not fail the run.
set -o pipefail

fails=0
pass() { printf '  PASS  %s\n' "$1"; }
fail() { printf '  FAIL  %s\n' "$1"; fails=$((fails + 1)); }
warn() { printf '  WARN  %s\n' "$1"; }

hz() {  # hz <topic> <seconds>: average rate or 0
  timeout "$2" ros2 topic hz "$1" 2>/dev/null | awk '/average rate/ {r=$3} END {print (r=="" ? 0 : r)}'
}

echo "== environment"
[ -n "$ROS_DISTRO" ] && pass "ROS_DISTRO=$ROS_DISTRO" || fail "ROS not sourced"
if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then pass "RMW cyclonedds"; else fail "RMW_IMPLEMENTATION is '$RMW_IMPLEMENTATION' (robot speaks cyclonedds)"; fi
if [ -n "$CYCLONEDDS_URI" ]; then
  # CYCLONEDDS_URI is either a file URI/path or the XML itself (unitree_ros2/setup.sh
  # exports inline XML); reading inline XML as a path reported a false '<none>'.
  case "$CYCLONEDDS_URI" in
    "<"*) dds_cfg=$CYCLONEDDS_URI ;;
    *) dds_cfg=$(cat "${CYCLONEDDS_URI#file://}" 2>/dev/null) ;;
  esac
  iface=$(printf '%s' "$dds_cfg" | grep -o 'NetworkInterface name="[^"]*"' | head -1 | cut -d'"' -f2)
  iface=${iface:-$(printf '%s' "$dds_cfg" | grep -o '<NetworkInterfaceAddress>[^<]*' | head -1 | cut -d'>' -f2)}
  if [ -n "$iface" ] && ip -brief link show "$iface" 2>/dev/null | grep -q UP; then
    pass "CycloneDDS interface '$iface' exists and is UP"
  else
    fail "CycloneDDS interface '${iface:-<none>}' missing or down: every GO2 topic will be advertised with NO data"
  fi
else
  warn "CYCLONEDDS_URI unset: DDS picks an interface itself"
fi
year=$(date +%Y)
if [ "$year" -ge 2026 ]; then pass "system clock year $year"; else fail "system clock reads $year (payload has no RTC); set it before recording evidence"; fi

echo "== packages"
for pkg in nav2_bringup nav2_regulated_pure_pursuit_controller nav2_navfn_planner slam_toolbox \
           pointcloud_to_laserscan unitree_api go2_msgs go2_localization go2_lidar_safety \
           go2_safety_arbiter go2_hardware_bridge go2_bringup; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 && pass "$pkg" || fail "$pkg not found (lab has no internet: stage it before the session)"
done

echo "== robot data flow (topic presence is NOT enough; rates are measured)"
ros2 daemon stop >/dev/null 2>&1
r=$(hz /utlidar/robot_odom 6); awk "BEGIN{exit !($r > 100)}" && pass "/utlidar/robot_odom ${r} Hz" || fail "/utlidar/robot_odom ${r} Hz (expect ~150)"
r=$(hz /utlidar/cloud 6); awk "BEGIN{exit !($r > 10)}" && pass "/utlidar/cloud ${r} Hz" || fail "/utlidar/cloud ${r} Hz (expect ~15)"
r=$(hz /utlidar/cloud_deskewed 6)
if awk "BEGIN{exit !($r > 10)}"; then
  frame=$(timeout 5 ros2 topic echo --once /utlidar/cloud_deskewed --field header.frame_id 2>/dev/null | head -1)
  if [ "$frame" = "odom" ]; then pass "/utlidar/cloud_deskewed ${r} Hz in 'odom' (default relay input is valid)"
  else fail "/utlidar/cloud_deskewed frame is '$frame', not odom: use cloud_in_topic:=/utlidar/cloud with a calibrated extrinsic"; fi
else
  fail "/utlidar/cloud_deskewed ${r} Hz: use cloud_in_topic:=/utlidar/cloud publish_lidar_extrinsic:=true (run calibrate_lidar first)"
fi
subs=$(ros2 topic info /api/sport/request 2>/dev/null | awk '/Subscription count/ {print $3}')
[ "${subs:-0}" -ge 1 ] && pass "/api/sport/request has ${subs} subscriber(s) (sport service up)" || fail "/api/sport/request has no subscriber: sport service not running"
tfpubs=$(ros2 topic info /tf 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "${tfpubs:-0}" -eq 0 ] && pass "no foreign /tf publishers before bring-up" || warn "/tf already has ${tfpubs} publisher(s); a second odom->base_link source would fight the relay"
cmdpubs=$(ros2 topic info /cmd_vel_safe 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "${cmdpubs:-0}" -eq 0 ] && pass "no /cmd_vel_safe publisher before bring-up" || fail "/cmd_vel_safe already has ${cmdpubs} publisher(s): another motion stack is running"

echo "== robot clock (the relay corrects it; this records the evidence)"
python3 - <<'EOF' || true
import time, rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node("preflight_clock"); d = []
n.create_subscription(Odometry, "/utlidar/robot_odom",
    lambda m: d.append(time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), qos_profile_sensor_data)
end = time.time() + 3
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
print(f"  INFO  robot clock offset {sorted(d)[len(d)//2]:.3f} s over {len(d)} odom msgs" if d else "  WARN  no odom to measure clock offset")
EOF

echo
if [ "$fails" -eq 0 ]; then echo "PREFLIGHT: GO ($fails failures)"; else echo "PREFLIGHT: NO-GO ($fails failures)"; fi
exit "$fails"
