"""The production launch (system.launch.py) really starts the Nav2 bridge in the mode
its arguments ask for, read back from the RUNNING node: its parameter and its
/diagnostics attestation (the same gate the send path uses). Runs ros2 launch on a
private localhost domain; needs this package installed (colcon build) and sourced."""

import os
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402

BASE = ["perception:=none", "planner:=staged", "hardware_adapter:=dry_run",
        "localization:=none"]
TOPIC = "/mosaic/motion_authority"


def _launch_and_read(extra, domain):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen(["ros2", "launch", "go2_bringup", "system.launch.py"] + BASE + extra,
                            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    os.environ["ROS_DOMAIN_ID"], os.environ["ROS_LOCALHOST_ONLY"] = str(domain), "1"
    rclpy.init()
    node = rclpy.create_node("authority_launch_probe")
    got = {}

    def on_diag(arr):
        for st in arr.status:
            if st.name == "go2_hardware_bridge: actuation":
                got.update({kv.key: kv.value for kv in st.values})

    node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)
    try:
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline and "authority_enabled" not in got:
            rclpy.spin_once(node, timeout_sec=0.2)
        param = subprocess.run(["ros2", "param", "get", "/hardware_bridge_node",
                                "motion_authority_topic"], env=env, capture_output=True,
                               text=True, timeout=20).stdout.strip()
        return got, param
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(proc.pid, 2)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, 9)


def test_mosaic_arguments_reach_the_running_bridge():
    got, param = _launch_and_read([f"motion_authority_topic:={TOPIC}",
                                   "motion_authority_name:=nav2"], 211)
    assert got.get("authority_enabled") == "True", got
    assert got.get("authority_topic") == TOPIC and got.get("authority_name") == "nav2"
    assert TOPIC in param


def test_default_is_legacy_and_says_so():
    got, _ = _launch_and_read([], 212)
    assert got.get("authority_enabled") == "False" and got.get("authority_topic") == ""


def test_a_misspelled_argument_is_visible_as_legacy_not_silently_gated():
    """ros2 launch ignores an unknown key; the running bridge then attests legacy mode,
    which MOSAIC's preflight refuses (NAV2_LEGACY_MODE)."""
    got, _ = _launch_and_read([f"motion_authorty_topic:={TOPIC}"], 213)
    assert got.get("authority_enabled") == "False", got
