"""ROS tests for go2_state_relay_node against a skewed-clock fake robot."""
from __future__ import annotations

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from go2_localization.state_relay_node import StateRelayNode  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

SKEW = -27_605_481.0


def _wait_for(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not pred():
        time.sleep(0.02)
    return pred()


class FakeRobot:
    def __init__(self, node: Node):
        self.node = node
        self.odom_pub = node.create_publisher(Odometry, "/utlidar/robot_odom", qos_profile_sensor_data)
        self.cloud_pub = node.create_publisher(PointCloud2, "/utlidar/cloud_deskewed", qos_profile_sensor_data)
        self.running = threading.Event()
        self.send_cloud = True
        self.x = 0.0

    def _stamp(self):
        t = self.node.get_clock().now().nanoseconds * 1e-9 + SKEW
        msg = Time(nanoseconds=int(t * 1e9)).to_msg()
        return msg

    def loop(self):
        i = 0
        while self.running.is_set():
            o = Odometry()
            o.header.stamp = self._stamp()
            o.header.frame_id = "odom"
            o.child_frame_id = "base_link"
            self.x += 0.001
            o.pose.pose.position.x = self.x
            o.pose.pose.position.z = 0.32
            o.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(o)
            if self.send_cloud and i % 10 == 0:
                c = PointCloud2()
                c.header.stamp = self._stamp()
                c.header.frame_id = "odom"
                self.cloud_pub.publish(c)
            i += 1
            time.sleep(0.01)


@pytest.fixture
def graph():
    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=4)
    relay = StateRelayNode(parameter_overrides=[Parameter("odom_max_age_s", value=0.3)])
    robot_node = Node("fake_go2")
    probe = Node("relay_probe")
    robot = FakeRobot(robot_node)
    valid = []
    probe.create_subscription(Bool, "/go2/localization_valid", lambda m: valid.append(m.data), 10)
    odoms = []
    probe.create_subscription(Odometry, "/odom", odoms.append, 10)
    buf = Buffer()
    TransformListener(buf, probe)
    for n in (relay, robot_node, probe):
        ex.add_node(n)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    robot.running.set()
    rt = threading.Thread(target=robot.loop, daemon=True)
    rt.start()
    yield relay, robot, probe, valid, odoms, buf
    robot.running.clear()
    rt.join(timeout=1.0)
    ex.shutdown(timeout_sec=2.0)
    for n in (relay, robot_node, probe):
        n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_restamps_to_local_clock_and_publishes_tf(graph):
    relay, robot, probe, valid, odoms, buf = graph
    assert _wait_for(lambda: len(odoms) > 20)
    now = probe.get_clock().now().nanoseconds * 1e-9
    stamp = odoms[-1].header.stamp.sec + odoms[-1].header.stamp.nanosec * 1e-9
    assert abs(now - stamp) < 0.5, "odom not restamped into the local clock"
    assert _wait_for(lambda: buf.can_transform("odom", "base_link", Time()))
    tf = buf.lookup_transform("odom", "base_link", Time())
    assert abs(tf.header.stamp.sec - int(now)) <= 1
    assert tf.transform.translation.z == pytest.approx(0.32)


def test_valid_when_fresh_then_invalid_when_odom_stops(graph):
    relay, robot, probe, valid, odoms, buf = graph
    assert _wait_for(lambda: valid and valid[-1] is True)
    robot.running.clear()
    assert _wait_for(lambda: valid and valid[-1] is False, timeout=3.0)


def test_invalid_when_lidar_stops(graph):
    relay, robot, probe, valid, odoms, buf = graph
    assert _wait_for(lambda: valid and valid[-1] is True)
    robot.send_cloud = False
    assert _wait_for(lambda: valid and valid[-1] is False, timeout=3.0)
    ok, reasons = relay.evaluate()
    assert not ok and any("lidar stale" in r for r in reasons)


def test_require_map_frame_blocks_without_slam(graph):
    relay, robot, probe, valid, odoms, buf = graph
    assert _wait_for(lambda: valid and valid[-1] is True)
    relay.set_parameters([Parameter("require_map_frame", value=True)])
    ok, reasons = relay.evaluate()
    assert not ok and any("map->odom" in r or "map" in r for r in reasons)
