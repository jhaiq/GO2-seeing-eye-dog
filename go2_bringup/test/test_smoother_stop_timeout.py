"""
The velocity smoother must give up on a dead controller quickly.

Nav2's velocity_smoother republishes its last input until ``velocity_timeout``
expires and only then ramps down. The candidate stamper restamps every smoothed
command, so the arbiter and bridge see fresh commands the whole time: if
controller_server crashes mid-goal, the robot keeps its full commanded speed,
blind, for the whole timeout (measured in the full-stack sim: 1.08 s with
velocity_timeout 1.0, then the 0.78 s ramp).

This test runs the real velocity_smoother with the installed nav2_params.yaml,
feeds it 20 Hz commands like controller_server (controller_frequency 20.0),
stops feeding, and measures how long the output stays at full speed.

  * blind hold after the last input <= 0.35 s (velocity_timeout 0.25 plus one
    20 Hz smoother cycle and scheduling slack),
  * a 0.15 s gap in the input (three missed controller cycles) does NOT start
    a deceleration, so ordinary controller jitter does not slow the robot.
"""
import os
import signal
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("lifecycle_msgs.srv")

from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

SPEED = 0.3


def _paths():
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    try:
        exe = os.path.join(get_package_prefix("nav2_velocity_smoother"), "lib",
                           "nav2_velocity_smoother", "velocity_smoother")
        params = os.path.join(get_package_share_directory("go2_navigation"), "config",
                              "nav2_params.yaml")
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"nav2_velocity_smoother or go2_navigation not installed: {exc}")
    return exe, params


def _stop(proc):
    for sig in (signal.SIGINT, signal.SIGKILL):
        if proc.poll() is not None:
            break
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    try:
        os.killpg(proc.pid, 0)
        return False  # something of the group survived
    except ProcessLookupError:
        return True


def test_smoother_stops_following_a_dead_controller_quickly():
    exe, params = _paths()
    domain = 90 + os.getpid() % 10
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node("smoother_timeout_probe", context=ctx)
    out = []
    lock = threading.Lock()

    def on_out(m):
        with lock:
            out.append((time.monotonic(), m.linear.x))
    node.create_subscription(Twist, "/cmd_vel_smoothed", on_out, 50)
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    change = node.create_client(ChangeState, "/velocity_smoother/change_state")
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen([exe, "--ros-args", "--params-file", params], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    try:
        assert change.wait_for_service(timeout_sec=15.0), "smoother never came up"
        for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            req = ChangeState.Request()
            req.transition.id = tid
            fut = change.call_async(req)
            end = time.monotonic() + 10
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.02)
            assert fut.done() and fut.result().success, f"transition {tid} failed"

        cmd = Twist()
        cmd.linear.x = SPEED

        def feed(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                pub.publish(cmd)
                time.sleep(0.05)

        feed(2.5)  # reach SPEED (accel 0.4 m/s^2)
        time.sleep(0.15)  # three missed controller cycles
        t_gap_end = time.monotonic()
        feed(1.0)
        t_last = time.monotonic()
        time.sleep(2.5)
    finally:
        clean = _stop(proc)
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown(context=ctx)
    assert clean, "velocity_smoother process group survived cleanup"
    with lock:
        samples = list(out)
    assert any(abs(v - SPEED) < 1e-3 for _, v in samples), "smoother never reached SPEED"
    gap = [v for t, v in samples if t_gap_end - 0.3 <= t <= t_gap_end + 0.5]
    assert gap and min(gap) >= SPEED - 1e-3, (
        f"a 0.15 s input gap already started a deceleration (min {min(gap):.3f})")
    after = [(t, v) for t, v in samples if t > t_last]
    decel = [t for t, v in after if v < SPEED - 0.01]
    assert decel, "smoother never slowed down after its input stopped"
    hold = decel[0] - t_last
    print(f"blind hold after last input: {hold:.3f} s")
    assert hold <= 0.35, (
        f"smoother kept full speed for {hold:.2f} s after the controller went silent")
    assert after[-1][1] == 0.0, "smoother did not ramp to zero"
