"""
LidarHazardNode against a live ROS graph, and wired to the real SafetyArbiterNode.

The contract under test: CLEAR is published only for a fresh, transformed,
evaluated cloud. Every failure mode (no TF, stale cloud) is silence, and the
arbiter turns silence into zero velocity.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import requires_ros  # noqa: E402

pytestmark = requires_ros

NOSE = 0.40  # HazardParams.body_x_max default


def make_cloud(node, points, frame="base_link"):
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    header = Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame
    return point_cloud2.create_cloud_xyz32(header, np.asarray(points, dtype=np.float32).tolist())


def floor_points():
    xs, ys = np.meshgrid(np.linspace(0.0, 3.0, 20), np.linspace(-1.0, 1.0, 9))
    return np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.32)])


def wall(distance_from_nose, frame_x_sign=1.0):
    ys = np.linspace(-0.1, 0.1, 15)
    return np.column_stack(
        [np.full(15, frame_x_sign * (NOSE + distance_from_nose)), ys, np.full(15, 0.1)]
    )


@pytest.fixture
def hazard_setup(graph):
    from go2_lidar_safety.lidar_hazard_node import LidarHazardNode
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    from go2_msgs.msg import SafetyAlert

    def build(**overrides):
        params = [Parameter(k, value=v) for k, v in overrides.items()]
        node = LidarHazardNode(parameter_overrides=params)
        graph.add(node)
        return node

    source = graph.make_node("fake_lidar")
    cloud_pub = source.create_publisher(PointCloud2, "cloud", 10)

    states, alerts, statuses = [], [], []
    sink = graph.make_node("hazard_sink")
    sink.create_subscription(String, "safety_state", lambda m: states.append(m.data), 10)
    sink.create_subscription(SafetyAlert, "safety_alert", alerts.append, 10)
    sink.create_subscription(
        String, "lidar_safety/status", lambda m: statuses.append(json.loads(m.data)), 10
    )
    return graph, build, source, cloud_pub, states, alerts, statuses


def publisher_at(rate_hz, fn):
    """Return a callable that invokes fn at most rate_hz times per second."""
    last = [0.0]

    def each():
        now = time.monotonic()
        if now - last[0] >= 1.0 / rate_hz:
            last[0] = now
            fn()

    return each


class TestPublishingContract:
    def test_fresh_floor_cloud_publishes_clear(self, hazard_setup):
        graph, build, source, pub, states, alerts, statuses = hazard_setup
        build()
        graph.spin_for(1.0, each=publisher_at(15, lambda: pub.publish(make_cloud(source, floor_points()))))
        assert states, "no hazard decision was published for a fresh cloud"
        assert set(states) == {"CLEAR"}
        assert not alerts
        clear = [s for s in statuses if s["decision"] == "CLEAR"]
        assert clear and clear[-1]["points_evaluated"] == floor_points().shape[0]

    def test_obstacle_publishes_alert_and_state(self, hazard_setup):
        graph, build, source, pub, states, alerts, _statuses = hazard_setup
        build()
        graph.spin_for(1.0, each=publisher_at(15, lambda: pub.publish(make_cloud(source, wall(0.2)))))
        assert states and set(states) == {"EMERGENCY_STOP"}
        assert alerts
        assert alerts[-1].alert_type == "EMERGENCY_STOP"
        assert alerts[-1].distance == pytest.approx(0.2, abs=1e-3)

    def test_slowdown_is_reported(self, hazard_setup):
        graph, build, source, pub, states, alerts, _statuses = hazard_setup
        build()
        graph.spin_for(1.0, each=publisher_at(15, lambda: pub.publish(make_cloud(source, wall(0.6)))))
        assert states and set(states) == {"SLOWDOWN"}
        assert alerts[-1].alert_type == "SLOWDOWN"

    def test_missing_transform_publishes_nothing(self, hazard_setup):
        graph, build, source, pub, states, alerts, statuses = hazard_setup
        build()
        graph.spin_for(
            1.0,
            each=publisher_at(
                15, lambda: pub.publish(make_cloud(source, floor_points(), frame="utlidar_lidar"))
            ),
        )
        assert not states, "CLEAR was published without a transform to base_link"
        assert not alerts
        assert any(s["decision"] == "TF_FAILED" for s in statuses)

    def test_static_extrinsic_is_applied(self, hazard_setup):
        """A sensor yawed 180 deg: its -x is the robot's +x (ahead)."""
        from geometry_msgs.msg import TransformStamped
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        graph, build, source, pub, states, alerts, _statuses = hazard_setup
        broadcaster = StaticTransformBroadcaster(source)
        tf = TransformStamped()
        tf.header.stamp = source.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "utlidar_lidar"
        tf.transform.rotation.z = 1.0
        tf.transform.rotation.w = 0.0
        broadcaster.sendTransform(tf)
        build()
        graph.spin_for(
            1.0,
            each=publisher_at(
                15,
                lambda: pub.publish(
                    make_cloud(source, wall(0.2, frame_x_sign=-1.0), frame="utlidar_lidar")
                ),
            ),
        )
        assert states and states[-1] == "EMERGENCY_STOP"
        assert alerts[-1].distance == pytest.approx(0.2, abs=1e-3)

    def test_stale_cloud_publishes_nothing(self, hazard_setup):
        """A cloud older than max_cloud_age_s at evaluation time is not evaluated."""
        graph, build, source, pub, states, _alerts, statuses = hazard_setup
        build(eval_rate_hz=1.0, max_cloud_age_s=0.2)
        graph.spin_for(0.4)
        pub.publish(make_cloud(source, floor_points()))
        graph.spin_for(1.2)
        assert not states
        assert any(s["decision"] == "STALE" for s in statuses)

    def test_clear_stops_when_the_cloud_stops(self, hazard_setup):
        graph, build, source, pub, states, _alerts, statuses = hazard_setup
        build()
        graph.spin_for(0.6, each=publisher_at(15, lambda: pub.publish(make_cloud(source, floor_points()))))
        # Let the one in-flight cloud (published just before silence) be evaluated.
        graph.spin_for(0.15)
        count = len(states)
        assert count > 0
        graph.spin_for(0.8)
        assert len(states) == count, "CLEAR kept flowing after the LiDAR went silent"
        assert statuses[-1]["decision"] == "NO_DATA"


@pytest.fixture
def chain_setup(graph):
    """lidar_hazard_node -> safety_arbiter_node, with a fake controller and LiDAR."""
    from geometry_msgs.msg import TwistStamped
    from go2_lidar_safety.lidar_hazard_node import LidarHazardNode
    from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    from go2_msgs.msg import SafeVelocityCommand

    graph.add(SafetyArbiterNode())
    graph.add(LidarHazardNode())

    source = graph.make_node("fake_controller_and_lidar")
    control_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    candidate_pub = source.create_publisher(TwistStamped, "cmd_vel_candidate", control_qos)
    cloud_pub = source.create_publisher(PointCloud2, "cloud", 10)

    received = []
    sink = graph.make_node("safe_sink")
    sink.create_subscription(SafeVelocityCommand, "cmd_vel_safe", received.append, control_qos)

    def candidate(vx=0.2):
        msg = TwistStamped()
        msg.header.stamp = source.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        candidate_pub.publish(msg)

    def drive(points):
        send_cloud = publisher_at(15, lambda: cloud_pub.publish(make_cloud(source, points)))
        send_candidate = publisher_at(20, candidate)

        def each():
            send_cloud()
            send_candidate()

        return each

    return graph, received, drive


class TestWithRealArbiter:
    def test_clear_lidar_permits_motion(self, chain_setup):
        graph, received, drive = chain_setup
        graph.spin_for(2.0, each=drive(floor_points()))
        assert received
        assert max(m.twist.linear.x for m in received) == pytest.approx(0.2, abs=1e-3)

    def test_obstacle_ahead_forces_zero(self, chain_setup):
        graph, received, drive = chain_setup
        graph.spin_for(2.0, each=drive(np.vstack([floor_points(), wall(0.2)])))
        assert received
        assert all(m.twist.linear.x == 0.0 for m in received)
        assert any("HAZARD_STOP" in list(m.reason_codes) for m in received)

    def test_slowdown_derates_to_degraded_scale(self, chain_setup):
        graph, received, drive = chain_setup
        graph.spin_for(2.5, each=drive(np.vstack([floor_points(), wall(0.6)])))
        moving = [m.twist.linear.x for m in received if m.twist.linear.x > 0.0]
        assert moving
        # max_vx 0.4 * degraded_scale 0.35 = 0.14
        assert max(moving) == pytest.approx(0.14, abs=1e-3)
        assert any("HAZARD_SLOWDOWN" in list(m.reason_codes) for m in received)

    def test_lidar_silence_stops_the_robot(self, chain_setup):
        graph, received, drive = chain_setup
        graph.spin_for(1.5, each=drive(floor_points()))
        assert any(m.twist.linear.x > 0.0 for m in received)
        received.clear()

        # Candidates continue, the LiDAR goes quiet.
        from geometry_msgs.msg import TwistStamped  # noqa: F401

        graph_nodes = graph.nodes
        source = [n for n in graph_nodes if n.get_name() == "fake_controller_and_lidar"][0]
        candidate_pub = [p for p in source.publishers if p.topic_name.endswith("cmd_vel_candidate")][0]

        def only_candidates():
            msg = TwistStamped()
            msg.header.stamp = source.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = 0.2
            candidate_pub.publish(msg)

        graph.spin_for(2.0, each=publisher_at(20, only_candidates))
        tail = [m for m in received if m.header.stamp.sec > 0][-10:]
        assert tail and all(m.twist.linear.x == 0.0 for m in tail)
        assert any("SAFETY_CONTEXT_STALE" in list(m.reason_codes) for m in received)
