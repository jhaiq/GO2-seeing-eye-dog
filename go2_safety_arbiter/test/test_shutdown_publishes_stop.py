"""The arbiter's shutdown stop must actually leave the process.

``SafetyArbiterNode.destroy_node`` publishes explicit zero-velocity STOPPED
commands (reason SHUTDOWN) so the bridge stops at once instead of waiting for
its watchdog. With rclpy's default signal handling, SIGINT or SIGTERM shut the
rcl context down BEFORE that ran, every publish failed, and the bridge only
stopped the robot when its own hold timer expired (measured in the full-stack
sim: first StopMove 0.19 to 0.22 s after the signal, no SHUTDOWN message).

This runs the installed arbiter, signals it, and records /cmd_vel_safe with an
independent subscriber. Required after SIGINT and after SIGTERM:
  * at least one STOPPED, zero-velocity command with reason SHUTDOWN,
  * a clean exit (code 0, no traceback) within 5 s.
"""
import os
import signal
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
go2_msgs = pytest.importorskip("go2_msgs.msg")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

CONTROL_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)


def _exe():
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix("go2_safety_arbiter"), "lib",
                        "go2_safety_arbiter", "safety_arbiter_node")


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_signal_publishes_shutdown_stop(sig):
    domain = 90 + os.getpid() % 10
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node("arbiter_shutdown_probe", context=ctx)
    got = []
    lock = threading.Lock()

    def on_safe(m):
        with lock:
            got.append((time.monotonic(), m.arbiter_state, list(m.reason_codes),
                        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z))
    node.create_subscription(go2_msgs.SafeVelocityCommand, "/cmd_vel_safe", on_safe,
                             CONTROL_QOS)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen(["python3", _exe()], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True)
    out = ""
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and not got:
            time.sleep(0.05)
        assert got, "arbiter never published on /cmd_vel_safe"
        time.sleep(0.5)
        t_sig = time.monotonic()
        os.kill(proc.pid, sig)
        out = proc.communicate(timeout=5)[0]
        t_exit = time.monotonic()
        time.sleep(0.5)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.communicate(timeout=5)
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown(context=ctx)
    with lock:
        after = [g for g in got if g[0] >= t_sig]
    shutdown = [g for g in after if "SHUTDOWN" in g[2]]
    assert shutdown, f"no SHUTDOWN stop reached /cmd_vel_safe after {sig.name}; {out[-600:]}"
    for _, state, _, vx, vy, wz in shutdown:
        assert state == "STOPPED" and vx == vy == wz == 0.0
    assert proc.returncode == 0, f"exit code {proc.returncode}; {out[-600:]}"
    assert "Traceback" not in out, out
    assert t_exit - t_sig < 5.0
    try:
        os.killpg(proc.pid, 0)
        survivors = True
    except ProcessLookupError:
        survivors = False
    assert not survivors, "arbiter process group survived"
