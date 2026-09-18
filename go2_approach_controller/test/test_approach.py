"""
Staged approach-controller tests.

Covers required Test 3 (goal produces candidate motion) at both the pure
geometry level and the node level, including the property that matters most
architecturally: the controller publishes a CANDIDATE and has no route to the
actuator.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from go2_approach_controller.approach import (  # noqa: E402
    ApproachGains,
    ApproachStatus,
    compute_approach,
    normalize_angle,
)

from conftest import requires_ros  # noqa: E402

GAINS = ApproachGains()


class TestApproachGeometry:
    def test_a_goal_straight_ahead_produces_forward_motion(self):
        command = compute_approach(3.0, 0.0, GAINS)
        assert command.status == ApproachStatus.ACTIVE
        assert command.vx > 0.0
        assert command.wz == pytest.approx(0.0)

    def test_a_goal_to_the_left_produces_a_left_turn(self):
        """REP-103: +y is left, and a left turn is CCW-positive yaw."""
        command = compute_approach(3.0, 1.0, GAINS)
        assert command.wz > 0.0

    def test_a_goal_to_the_right_produces_a_right_turn(self):
        command = compute_approach(3.0, -1.0, GAINS)
        assert command.wz < 0.0

    def test_a_goal_behind_turns_in_place(self):
        """Translating toward a goal behind you means driving through it."""
        command = compute_approach(-3.0, 0.5, GAINS)
        assert command.vx == pytest.approx(0.0)
        assert abs(command.wz) > 0.0

    def test_reaching_the_goal_stops(self):
        command = compute_approach(0.5, 0.0, GAINS)
        assert command.status == ApproachStatus.REACHED
        assert (command.vx, command.vy, command.wz) == (0.0, 0.0, 0.0)

    def test_the_robot_stops_short_of_the_person(self):
        """
        A guide dog that stops on top of its handler is a failure mode.

        The goal tolerance is a social distance, not a numerical convenience.
        """
        assert GAINS.goal_tolerance_m >= 0.5
        assert compute_approach(GAINS.goal_tolerance_m - 0.01, 0.0, GAINS).status == (
            ApproachStatus.REACHED
        )

    def test_speed_decreases_on_approach(self):
        speeds = [compute_approach(d, 0.0, GAINS).vx for d in (4.0, 3.0, 2.0, 1.2, 0.9)]
        for faster, slower in zip(speeds, speeds[1:]):
            assert faster >= slower

    def test_lateral_velocity_is_never_commanded(self):
        """
        The staged controller drives the GO2 as a differential-drive base.

        This is the conservative choice and matches what Nav2's default
        controllers emit, so the Stage 1 to Stage 2 swap does not change the
        shape of the commands the arbiter sees.
        """
        for x in (-3.0, -1.0, 0.5, 2.0, 5.0):
            for y in (-2.0, -0.5, 0.0, 0.5, 2.0):
                assert compute_approach(x, y, GAINS).vy == 0.0

    def test_forward_velocity_is_never_negative(self):
        for x in (-5.0, -1.0, 0.0, 1.0, 5.0):
            for y in (-3.0, 0.0, 3.0):
                assert compute_approach(x, y, GAINS).vx >= 0.0

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_non_finite_goal_produces_a_stop(self, bad):
        command = compute_approach(bad, 0.0, GAINS)
        assert (command.vx, command.vy, command.wz) == (0.0, 0.0, 0.0)
        assert command.status == ApproachStatus.NO_TRANSFORM

    def test_normalize_angle_wraps_correctly(self):
        # +pi and -pi are the same direction, and which one atan2 returns at
        # the boundary depends on the sign of a value that is zero to within
        # rounding. Assert on the magnitude there, and exactly elsewhere.
        assert abs(normalize_angle(3 * math.pi)) == pytest.approx(math.pi)
        assert abs(normalize_angle(-3 * math.pi)) == pytest.approx(math.pi)
        assert normalize_angle(0.5) == pytest.approx(0.5)
        assert normalize_angle(-0.5) == pytest.approx(-0.5)
        assert normalize_angle(2 * math.pi + 0.3) == pytest.approx(0.3)
        assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"k_linear": 0.0},
            {"goal_tolerance_m": -1.0},
            {"slowdown_radius_m": 0.5, "goal_tolerance_m": 0.8},
        ],
    )
    def test_invalid_gains_are_rejected(self, kwargs):
        with pytest.raises(ValueError):
            ApproachGains(**kwargs)


pytestmark_ros = requires_ros


@pytest.fixture
def controller_setup(graph):
    from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
    from go2_approach_controller.approach_controller_node import (
        ApproachControllerNode,
    )
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    node = ApproachControllerNode()
    graph.add(node)

    source = graph.make_node("fake_planner_client")
    broadcaster = StaticTransformBroadcaster(source)
    tf = TransformStamped()
    tf.header.stamp = source.get_clock().now().to_msg()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = "map"
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform([tf])

    goal_pub = source.create_publisher(PoseStamped, "goal_pose", 10)

    candidates = []
    sink = graph.make_node("candidate_sink")
    sink.create_subscription(
        TwistStamped,
        "cmd_vel_candidate",
        candidates.append,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        ),
    )

    def send_goal(x=3.0, y=0.0):
        goal = PoseStamped()
        goal.header.stamp = source.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)

    return graph, node, candidates, send_goal


@requires_ros
class TestControllerNode:
    def test_a_goal_produces_candidate_motion(self, controller_setup):
        """Test 3: a navigation goal becomes a candidate velocity."""
        graph, _node, candidates, send_goal = controller_setup

        graph.spin_for(0.4)
        send_goal(x=3.0, y=0.0)
        graph.spin_for(1.2)

        assert candidates
        assert max(c.twist.linear.x for c in candidates) > 0.0

    def test_candidates_are_stamped_and_framed(self, controller_setup):
        """
        The freshness guarantee depends on a real timestamp and frame.

        A candidate with a zero stamp or the wrong frame is refused by the
        arbiter, so producing them correctly is part of the contract.
        """
        graph, _node, candidates, send_goal = controller_setup

        graph.spin_for(0.3)
        send_goal()
        graph.spin_for(0.8)

        assert candidates
        for candidate in candidates:
            assert candidate.header.frame_id == "base_link"
            assert candidate.header.stamp.sec > 0

    def test_no_goal_means_zero_candidate_not_silence(self, controller_setup):
        """
        With no goal the controller publishes explicit zeros.

        Publishing nothing would be indistinguishable from the controller
        having died, and the two situations warrant different responses.
        """
        graph, _node, candidates, _send_goal = controller_setup
        graph.spin_for(1.0)

        assert candidates, "controller went silent instead of commanding zero"
        assert all(
            c.twist.linear.x == 0.0 and c.twist.angular.z == 0.0 for c in candidates
        )

    def test_a_goal_needing_a_missing_transform_produces_zero(self, controller_setup):
        graph, _node, candidates, _send_goal = controller_setup
        from geometry_msgs.msg import PoseStamped

        source = graph.nodes[1]
        publisher = source.create_publisher(PoseStamped, "goal_pose", 10)
        goal = PoseStamped()
        goal.header.stamp = source.get_clock().now().to_msg()
        goal.header.frame_id = "a_frame_that_does_not_exist"
        goal.pose.position.x = 3.0
        goal.pose.orientation.w = 1.0

        graph.spin_for(0.3)
        publisher.publish(goal)
        graph.spin_for(1.0)

        assert candidates
        assert all(c.twist.linear.x == 0.0 for c in candidates[-5:])

    def test_a_stale_dynamic_transform_produces_zero(self, controller_setup):
        """A dynamic odom chain that stops updating must stop the robot.

        Static chains (zero stamp) never age; a real localization chain does,
        and once it is older than max_transform_age_sec the goal is untrusted.
        """
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from tf2_ros.transform_broadcaster import TransformBroadcaster

        graph, _node, candidates, _send_goal = controller_setup
        source = graph.nodes[1]
        broadcaster = TransformBroadcaster(source)
        tf = TransformStamped()
        tf.header.stamp = source.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "odom_stale"
        tf.transform.rotation.w = 1.0
        broadcaster.sendTransform(tf)

        publisher = source.create_publisher(PoseStamped, "goal_pose", 10)
        goal = PoseStamped()
        goal.header.stamp = source.get_clock().now().to_msg()
        goal.header.frame_id = "odom_stale"
        goal.pose.position.x = 3.0
        goal.pose.orientation.w = 1.0

        graph.spin_for(1.0)  # longer than max_transform_age_sec (0.5 s)
        publisher.publish(goal)
        graph.spin_for(1.0)

        assert candidates
        assert all(c.twist.linear.x == 0.0 for c in candidates[-5:])

    def test_the_controller_cannot_publish_a_safe_command(self, controller_setup):
        """
        The architectural property, asserted against the live node.

        The controller has no publisher of the actuator's message type, so no
        remapping, misconfiguration or copy-paste error can turn its output
        into an authorized command.
        """
        graph, node, _candidates, _send_goal = controller_setup
        graph.spin_for(0.3)

        published_types = {
            pub.msg_type.__name__ for pub in node.publishers
        }
        assert "SafeVelocityCommand" not in published_types

        topics = {pub.topic_name for pub in node.publishers}
        assert not any("cmd_vel_safe" in t for t in topics)

    def test_shutdown_emits_zero_candidates(self, controller_setup):
        graph, node, candidates, send_goal = controller_setup
        graph.spin_for(0.3)
        send_goal()
        graph.spin_for(0.8)
        candidates.clear()

        node.destroy_node()
        graph.executor.remove_node(node)
        graph.nodes.remove(node)
        graph.spin_for(0.3)

        assert candidates
        assert all(c.twist.linear.x == 0.0 for c in candidates)


class TestArrival:
    """Regression: the quadratic slowdown never crossed goal_tolerance_m, so a
    goal was never REACHED (closed-loop sim stopped ~4 cm short for 30 s)."""

    def test_simulated_approach_reaches_within_time(self):
        gains = ApproachGains()
        x, dt, t = 0.0, 0.05, 0.0
        goal = 3.0
        status = None
        while t < 30.0:
            cmd = compute_approach(goal - x, 0.0, gains)
            status = cmd.status
            if status == ApproachStatus.REACHED:
                break
            x += cmd.vx * dt
            t += dt
        assert status == ApproachStatus.REACHED
        assert t < 15.0
        assert goal - x == pytest.approx(gains.goal_tolerance_m, abs=gains.arrival_epsilon_m + 0.01)

    def test_min_speed_never_pushes_inside_the_arrival_band(self):
        gains = ApproachGains()
        cmd = compute_approach(gains.goal_tolerance_m + gains.arrival_epsilon_m / 2, 0.0, gains)
        assert cmd.status == ApproachStatus.REACHED and cmd.vx == 0.0
