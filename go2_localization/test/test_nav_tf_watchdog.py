"""Stall detection and Nav2 lifecycle recovery for the map->odom stall."""
import threading
import time

import pytest
from go2_localization.nav_tf_watchdog import STALL_MARKER, StallDetector


def test_streak_shorter_than_stall_does_not_fire():
    d = StallDetector(stall_s=3.0, gap_s=1.0)
    assert not any(d.observe(t * 0.05) for t in range(50))  # 2.45 s


def test_continuous_streak_fires_once_then_cools_down():
    d = StallDetector(stall_s=3.0, gap_s=1.0, cooldown_s=30.0)
    fired = [t for t in range(200) if d.observe(t * 0.05)]  # 10 s of errors
    assert len(fired) == 1 and fired[0] * 0.05 >= 3.0


def test_gap_restarts_the_streak():
    d = StallDetector(stall_s=3.0, gap_s=1.0)
    for t in range(40):
        assert not d.observe(t * 0.05)  # 0 .. 1.95 s
    for t in range(40):
        assert not d.observe(5.0 + t * 0.05)  # after a 3 s gap: new streak


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        StallDetector(stall_s=0.0)


def test_node_resets_then_starts_nav2_on_a_persistent_stall():
    rclpy = pytest.importorskip("rclpy")
    nav2_srv = pytest.importorskip("nav2_msgs.srv")
    from rcl_interfaces.msg import Log
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from go2_localization import nav_tf_watchdog

    rclpy.init()
    calls = []
    fake = Node("fake_lifecycle_manager")

    def handle(req, resp):
        calls.append(req.command)
        resp.success = True
        return resp

    fake.create_service(nav2_srv.ManageLifecycleNodes,
                        "/lifecycle_manager_navigation/manage_nodes", handle)
    pub = fake.create_publisher(Log, "/rosout", 100)
    ex = MultiThreadedExecutor()
    ex.add_node(fake)
    threading.Thread(target=ex.spin, daemon=True).start()

    stop = threading.Event()
    watchdog = threading.Thread(
        target=lambda: _run_watchdog(nav_tf_watchdog, stop), daemon=True)
    watchdog.start()
    try:
        time.sleep(1.0)
        end = time.monotonic() + 4.5
        while time.monotonic() < end:
            pub.publish(Log(name="tf_help", level=40, msg=STALL_MARKER))
            time.sleep(0.05)
        deadline = time.monotonic() + 3.0
        while len(calls) < 2 and time.monotonic() < deadline:
            time.sleep(0.05)
        R = nav2_srv.ManageLifecycleNodes.Request
        assert calls[:2] == [R.RESET, R.STARTUP]
    finally:
        stop.set()
        ex.shutdown()
        fake.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_watchdog(module, stop):
    """Run the watchdog node's spin loop until stop is set (context shared)."""
    import rclpy

    orig_init, orig_spin = rclpy.init, rclpy.spin
    try:
        rclpy.init = lambda args=None: None

        def spin_until_stop(node):
            while not stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)

        rclpy.spin = spin_until_stop
        module.main()
    finally:
        rclpy.init, rclpy.spin = orig_init, orig_spin
