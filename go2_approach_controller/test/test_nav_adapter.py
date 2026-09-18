"""
NavigateToPose adapter tests.

A real action client drives the real ApproachControllerNode through the
adapter, so the Nav2-shaped contract (SUCCEEDED only on REACHED, cancel
confirmed by IDLE, abort when the controller is absent or cannot transform the
goal) is exercised through real action and topic plumbing.
"""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from go2_approach_controller.nav_to_pose_adapter_node import parse_status  # noqa: E402

from conftest import ROS_AVAILABLE, requires_ros  # noqa: E402

if ROS_AVAILABLE:
    try:
        import nav2_msgs.action  # noqa: F401
    except ImportError:  # pragma: no cover
        ROS_AVAILABLE = False

requires_nav2 = pytest.mark.skipif(not ROS_AVAILABLE, reason="needs ROS and nav2_msgs")


class TestParseStatus:
    def test_bare_state(self):
        assert parse_status("IDLE") == ("IDLE", None)

    def test_state_with_range(self):
        state, range_m = parse_status("ACTIVE range=2.35m heading_err=4.0deg")
        assert state == "ACTIVE"
        assert range_m == pytest.approx(2.35)

    def test_nan_range_is_dropped(self):
        assert parse_status("NO_TRANSFORM range=nanm heading_err=nandeg")[1] is None

    def test_empty(self):
        assert parse_status("") == ("", None)


def _spin_until(graph, predicate, timeout_s, each=None):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not predicate():
        graph.spin_for(0.02, each=each)
    return predicate()


def _status_of(result_future):
    return result_future.result().status


@pytest.fixture
def adapter_graph(graph):
    """Adapter plus an action client; the controller is added per test."""
    from go2_approach_controller.nav_to_pose_adapter_node import NavToPoseAdapterNode
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    adapter = graph.add(NavToPoseAdapterNode())
    client_node = graph.make_node("nav_client")
    client = ActionClient(client_node, NavigateToPose, "navigate_to_pose")

    def send(x, y=0.0, frame="map"):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = client_node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        send_future = client.send_goal_async(goal)
        assert _spin_until(graph, send_future.done, 3.0), "goal response timed out"
        return send_future.result()

    assert _spin_until(graph, lambda: client.server_is_ready(), 3.0)
    return graph, adapter, client_node, send


def _add_controller(graph, static_tf=True):
    from geometry_msgs.msg import TransformStamped
    from go2_approach_controller.approach_controller_node import ApproachControllerNode
    from std_msgs.msg import String
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    controller = graph.add(ApproachControllerNode())
    tf_node = graph.make_node("tf_source")
    broadcaster = None
    if static_tf:
        broadcaster = StaticTransformBroadcaster(tf_node)
        tf = TransformStamped()
        tf.header.stamp = tf_node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "map"
        tf.transform.rotation.w = 1.0
        broadcaster.sendTransform([tf])

    statuses = []
    tf_node.create_subscription(
        String, "controller/status", lambda m: statuses.append(m.data), 10
    )
    return controller, tf_node, broadcaster, statuses


@requires_ros
@requires_nav2
class TestAdapter:
    def test_goal_within_tolerance_succeeds(self, adapter_graph):
        from action_msgs.msg import GoalStatus

        graph, _adapter, _client, send = adapter_graph
        _add_controller(graph)
        graph.spin_for(0.3)
        handle = send(0.5)
        assert handle.accepted
        result = handle.get_result_async()
        assert _spin_until(graph, result.done, 5.0)
        assert _status_of(result) == GoalStatus.STATUS_SUCCEEDED

    def test_moving_robot_reaches_goal_with_dynamic_tf(self, adapter_graph):
        """The robot closes on the goal through a live, dynamic map->base_link."""
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import TransformStamped
        from tf2_ros.transform_broadcaster import TransformBroadcaster

        graph, _adapter, _client, send = adapter_graph
        _controller, tf_node, _b, _statuses = _add_controller(graph, static_tf=False)
        broadcaster = TransformBroadcaster(tf_node)
        robot = {"x": 0.0}

        def publish_pose():
            tf = TransformStamped()
            tf.header.stamp = tf_node.get_clock().now().to_msg()
            tf.header.frame_id = "map"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = robot["x"]
            tf.transform.rotation.w = 1.0
            broadcaster.sendTransform(tf)

        graph.spin_for(0.3, each=publish_pose)
        handle = send(3.0)
        assert handle.accepted
        result = handle.get_result_async()

        # Still executing while far away: a latched status must not end it.
        graph.spin_for(0.6, each=publish_pose)
        assert not result.done()

        def advance():
            robot["x"] = min(robot["x"] + 0.02, 2.5)
            publish_pose()

        assert _spin_until(graph, result.done, 10.0, each=advance)
        assert _status_of(result) == GoalStatus.STATUS_SUCCEEDED
        assert robot["x"] >= 3.0 - 0.8 - 0.05

    def test_cancel_stops_the_controller(self, adapter_graph):
        from action_msgs.msg import GoalStatus

        graph, _adapter, _client, send = adapter_graph
        _controller, _tf_node, _b, statuses = _add_controller(graph)
        graph.spin_for(0.3)
        handle = send(3.0)
        graph.spin_for(0.5)
        assert statuses and statuses[-1].startswith("ACTIVE")

        cancel = handle.cancel_goal_async()
        result = handle.get_result_async()
        assert _spin_until(graph, lambda: cancel.done() and result.done(), 4.0)
        assert _status_of(result) == GoalStatus.STATUS_CANCELED
        graph.spin_for(0.2)
        assert statuses[-1] == "IDLE"

    def test_absent_controller_aborts_quickly(self, adapter_graph):
        from action_msgs.msg import GoalStatus

        graph, _adapter, _client, send = adapter_graph
        started = time.monotonic()
        handle = send(3.0)
        assert handle.accepted
        result = handle.get_result_async()
        assert _spin_until(graph, result.done, 5.0)
        assert _status_of(result) == GoalStatus.STATUS_ABORTED
        assert time.monotonic() - started < 3.0

    def test_new_goal_preempts_the_old_one(self, adapter_graph):
        from action_msgs.msg import GoalStatus

        graph, _adapter, _client, send = adapter_graph
        _controller, _tf_node, _b, statuses = _add_controller(graph)
        graph.spin_for(0.3)
        first = send(3.0)
        graph.spin_for(0.4)
        second = send(4.0)
        first_result = first.get_result_async()
        second_result = second.get_result_async()

        assert _spin_until(graph, first_result.done, 3.0)
        assert _status_of(first_result) == GoalStatus.STATUS_ABORTED
        graph.spin_for(0.6)
        assert not second_result.done(), "the new goal must keep executing"
        assert statuses[-1].startswith("ACTIVE range=4.00")

        second.cancel_goal_async()
        assert _spin_until(graph, second_result.done, 3.0)
        assert _status_of(second_result) == GoalStatus.STATUS_CANCELED

    def test_goal_in_unknown_frame_aborts(self, adapter_graph):
        from action_msgs.msg import GoalStatus

        graph, _adapter, _client, send = adapter_graph
        _add_controller(graph)
        graph.spin_for(0.3)
        handle = send(3.0, frame="frame_nobody_publishes")
        assert handle.accepted
        result = handle.get_result_async()
        assert _spin_until(graph, result.done, 5.0)
        assert _status_of(result) == GoalStatus.STATUS_ABORTED

    def test_empty_frame_is_rejected(self, adapter_graph):
        graph, _adapter, _client, send = adapter_graph
        _add_controller(graph)
        graph.spin_for(0.2)
        assert not send(3.0, frame="").accepted
