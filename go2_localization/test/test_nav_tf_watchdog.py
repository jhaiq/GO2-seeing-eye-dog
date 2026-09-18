"""Controller-stall detection (planner active, controller silent) and Nav2 recovery."""
import threading
import time

import pytest
from go2_localization.nav_tf_watchdog import StallDetector


def _drive(d, seconds, plan_hz=1.0, controller_hz=None, start=0.0, dt=0.05):
    """Simulate topic arrivals; return times at which check() fired."""
    fired = []
    t = start
    next_plan, next_ctrl = start, start
    while t < start + seconds:
        if t >= next_plan:
            d.on_planner_plan(t)
            next_plan += 1.0 / plan_hz
        if controller_hz and t >= next_ctrl:
            d.on_controller_plan(t)
            next_ctrl += 1.0 / controller_hz
        if d.check(t):
            fired.append(t)
        t += dt
    return fired


def test_healthy_navigation_never_fires():
    assert _drive(StallDetector(), 60.0, plan_hz=1.0, controller_hz=20.0) == []


def test_idle_or_recovery_behaviour_never_fires():
    d = StallDetector()
    assert _drive(d, 30.0, plan_hz=0.2) == []  # plans too sparse: not navigating


def test_stall_fires_once_after_the_window_and_cools_down():
    d = StallDetector(stall_s=3.0, cooldown_s=30.0)
    _drive(d, 10.0, plan_hz=1.0, controller_hz=20.0)  # healthy first
    fired = _drive(d, 20.0, plan_hz=1.0, controller_hz=None, start=10.0)  # controller goes silent
    assert len(fired) == 1
    assert 12.9 <= fired[0] <= 14.5  # 3 s after the last controller plan (~9.95 s)


def test_fresh_navigation_gets_a_grace_window():
    d = StallDetector(stall_s=3.0)
    fired = _drive(d, 2.5, plan_hz=1.0, controller_hz=None)
    assert fired == []


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        StallDetector(stall_s=0.0)


def _run_watchdog(module, stop):
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


@pytest.mark.parametrize("controller_alive, expect_reset", [(False, True), (True, False)])
def test_node_resets_nav2_only_when_the_controller_is_silent(controller_alive, expect_reset):
    rclpy = pytest.importorskip("rclpy")
    nav2_srv = pytest.importorskip("nav2_msgs.srv")
    from nav_msgs.msg import Path
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from go2_localization import nav_tf_watchdog

    rclpy.init()
    calls = []
    fake = Node("fake_nav2")

    def handle(req, resp):
        calls.append(req.command)
        resp.success = True
        return resp

    fake.create_service(nav2_srv.ManageLifecycleNodes,
                        "/lifecycle_manager_navigation/manage_nodes", handle)
    plan_pub = fake.create_publisher(Path, "/plan", 10)
    ctrl_pub = fake.create_publisher(Path, "/received_global_plan", 10)
    ex = MultiThreadedExecutor()
    ex.add_node(fake)
    threading.Thread(target=ex.spin, daemon=True).start()
    stop = threading.Event()
    threading.Thread(target=lambda: _run_watchdog(nav_tf_watchdog, stop), daemon=True).start()
    try:
        time.sleep(1.0)
        t0 = time.monotonic()
        last_plan = 0.0
        while time.monotonic() - t0 < 6.0:
            now = time.monotonic()
            if now - last_plan >= 1.0:
                plan_pub.publish(Path())
                last_plan = now
            if controller_alive:
                ctrl_pub.publish(Path())
            time.sleep(0.05)
        time.sleep(1.0)
        R = nav2_srv.ManageLifecycleNodes.Request
        if expect_reset:
            assert calls[:2] == [R.RESET, R.STARTUP]
        else:
            assert calls == []
    finally:
        stop.set()
        ex.shutdown()
        fake.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
