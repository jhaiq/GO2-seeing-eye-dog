"""Process-level shutdown safety with the REAL Unitree Sport adapter.

The dry-run adapter writes to a file and never touches DDS, so it cannot catch a
stop that fails to leave the process. This test runs the installed bridge with
hardware_adapter:=unitree_sport, feeds it a live 0.2 m/s command stream, signals
it, and records what actually reached /api/sport/request with an independent
subscriber.

Required after SIGINT and after SIGTERM:
  * at least one StopMove (1003) after the signal,
  * no Damp (1001) at any time (Damp drops the robot where it stands),
  * the last request is StopMove, not a Move.

Before the fix, nothing was sent after the signal (rclpy's handler killed the
context first) and the robot kept its last Move; on a crash, StopMove + Damp.
"""

import os
import signal
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
unitree_api = pytest.importorskip("unitree_api.msg")
go2_msgs = pytest.importorskip("go2_msgs.msg")

from rclpy.duration import Duration  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)

MOVE, STOP_MOVE, DAMP = 1008, 1003, 1001
SAFE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)


class Probe(Node):
    """Records every Sport request and streams SAFE_TO_MOVE commands like the arbiter."""

    def __init__(self):
        super().__init__("shutdown_probe")
        self.requests = []
        self.stream = True
        self.seq = 0
        self.create_subscription(unitree_api.Request, "/api/sport/request",
                                 lambda m: self.requests.append(
                                     (time.monotonic(), m.header.identity.api_id)), 200)
        self.pub = self.create_publisher(go2_msgs.SafeVelocityCommand, "cmd_vel_safe", SAFE_QOS)
        self.create_timer(0.05, self._tick)

    def _tick(self):
        if not self.stream:
            return
        self.seq += 1
        now = self.get_clock().now()
        m = go2_msgs.SafeVelocityCommand()
        m.header.stamp = now.to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = 0.2
        m.arbiter_state = "SAFE_TO_MOVE"
        m.sequence = self.seq
        m.authority_token = "test-token-0001"
        m.valid_until = (now + Duration(seconds=0.5)).to_msg()
        self.pub.publish(m)


@pytest.fixture(scope="module")
def ros():
    os.environ["ROS_DOMAIN_ID"] = str(40 + os.getpid() % 50)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    yield
    rclpy.try_shutdown()


def _exe():
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix("go2_hardware_bridge"), "lib",
                        "go2_hardware_bridge", "hardware_bridge_node")


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_signal_sends_stopmove_and_never_damp(ros, sig):
    probe = Probe()
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    threading.Thread(target=ex.spin, daemon=True).start()
    proc = subprocess.Popen(
        ["python3", _exe(), "--ros-args", "-p", "hardware_adapter:=unitree_sport"],
        env=dict(os.environ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and not any(a == MOVE for _, a in probe.requests):
            time.sleep(0.1)
        assert any(a == MOVE for _, a in probe.requests), "bridge never forwarded a Move"
        time.sleep(0.5)
        t_sig = time.monotonic()
        os.killpg(os.getpgid(proc.pid), sig)
        out = proc.communicate(timeout=15)[0]
        probe.stream = False
        time.sleep(1.0)
    finally:
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.communicate(timeout=5)
        ex.shutdown(timeout_sec=1.0)
        probe.destroy_node()
    after = [a for t, a in probe.requests if t >= t_sig]
    assert DAMP not in [a for _, a in probe.requests], "Damp was sent: the robot would drop"
    assert STOP_MOVE in after, f"no StopMove reached the robot after {sig.name}; {out[-400:]}"
    assert after[-1] == STOP_MOVE, f"last request after {sig.name} was {after[-1]}, not StopMove"
    assert "Traceback" not in out, out


def test_adapter_shutdown_is_stopmove_only_even_when_publishing_fails():
    """Covers the crash path too: whatever reaches shutdown(), Damp is never sent."""
    from go2_hardware_bridge.unitree_sport import (API_ID_DAMP, API_ID_STOP_MOVE,
                                                   SHUTDOWN_STOP_REPEATS, UnitreeSportBridge)
    sent = []
    fake = UnitreeSportBridge.__new__(UnitreeSportBridge)
    fake._lock = threading.Lock()
    fake._health = type("H", (), {"state": None, "connected": True})()
    calls = {"n": 0}

    def publish(api_id, params):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("context invalid")
        sent.append(api_id)
        return True
    fake._publish = publish
    UnitreeSportBridge.shutdown(fake)
    assert API_ID_DAMP not in sent
    assert sent == [API_ID_STOP_MOVE] * (SHUTDOWN_STOP_REPEATS - 1)
    assert fake._health.connected is False
