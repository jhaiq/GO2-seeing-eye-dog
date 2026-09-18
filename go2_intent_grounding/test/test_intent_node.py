"""
IntentGroundingNode tests against a live ROS graph.

Covers required Test 1 (confirmed caller produces exactly one correct goal)
and Test 2 (no confirmation produces no goal), plus the frame-convention
regression at node level and the observability contract that keeps a blind
user informed.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import (  # noqa: E402
    optical_position_for_body_bearing,
    publish_standard_tf_chain,
    requires_ros,
)

pytestmark = requires_ros


@pytest.fixture
def intent_setup(graph):
    """
    An intent-grounding node with a static TF from the camera frame to map,
    so the goal transform succeeds and Test 1 exercises the whole path.
    """
    from geometry_msgs.msg import PoseStamped
    from go2_intent_grounding.intent_grounding_node import IntentGroundingNode
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Float32, String

    from go2_msgs.msg import DetectedHuman, DetectedHumanArray, GroundingStatus

    node = IntentGroundingNode()
    graph.add(node)

    source = graph.make_node("fake_perception")

    # map -> base_link -> camera_color_optical_frame, with the REAL optical
    # rotation, so a person 3 m in front of the camera is 3 m in front of the
    # robot rather than 3 m above it.
    # Bound to a name deliberately: a StaticTransformBroadcaster that is
    # garbage-collected stops serving its transforms.
    tf_broadcaster = publish_standard_tf_chain(source)

    humans_pub = source.create_publisher(DetectedHumanArray, "detected_humans", 10)
    bearing_pub = source.create_publisher(Float32, "audio_bearing_deg", 10)
    voice_pub = source.create_publisher(String, "voice_command", 10)

    goals = []
    statuses = []
    sink = graph.make_node("goal_sink")
    sink.create_subscription(PoseStamped, "goal_pose", goals.append, 10)
    sink.create_subscription(
        GroundingStatus,
        "grounding_status",
        statuses.append,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        ),
    )

    def say(text):
        msg = String()
        msg.data = text
        voice_pub.publish(msg)

    def hear(bearing_deg):
        msg = Float32()
        msg.data = float(bearing_deg)
        bearing_pub.publish(msg)

    def see(bearing_deg=0.0, distance=3.0, confidence=0.9):
        """
        Publish one detection of a person at ``bearing_deg`` in BODY frame.

        The optical-frame coordinates are computed from the body bearing, so
        the test speaks in the convention a human reasons in (positive =
        left) and the node is responsible for converting correctly.
        """
        px, py, pz = optical_position_for_body_bearing(bearing_deg, distance)
        human = DetectedHuman()
        human.pose.position.x = px
        human.pose.position.y = py
        human.pose.position.z = pz
        human.pose.orientation.w = 1.0
        human.confidence = float(confidence)
        human.track_id = "person-1"

        array = DetectedHumanArray()
        array.header.stamp = source.get_clock().now().to_msg()
        array.header.frame_id = "camera_color_optical_frame"
        array.humans = [human]
        humans_pub.publish(array)

    # tf_broadcaster is returned only to keep it referenced for the duration
    # of the test.
    return graph, node, goals, statuses, say, hear, see, tf_broadcaster


class TestGoalRequiresARequest:
    def test_no_request_means_no_goal(self, intent_setup):
        """
        Test 2: perfect detections with no voice request must produce nothing.

        This is the defect that let the robot set off across a room at a
        person who had not spoken to it.
        """
        graph, _node, goals, statuses, _say, hear, see, _tf = intent_setup

        def perfect_frame():
            hear(0.0)
            see(bearing_deg=0.0, confidence=0.98)

        graph.spin_for(3.0, each=perfect_frame)

        assert goals == [], f"{len(goals)} goal(s) published with no request"
        assert statuses
        assert statuses[-1].state == "IDLE"
        assert statuses[-1].reason == "NO_REQUEST"

    def test_confirmed_caller_produces_exactly_one_goal(self, intent_setup):
        """Test 1: a valid confirmed request produces exactly one correct goal."""
        graph, _node, goals, _statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")

        def frame():
            hear(0.0)
            see(bearing_deg=0.0, confidence=0.9)

        graph.spin_for(3.0, each=frame)

        assert len(goals) == 1, f"expected exactly 1 goal, got {len(goals)}"
        goal = goals[0]
        assert goal.header.frame_id == "map"
        # With the real optical rotation, a person 3 m straight ahead of the
        # camera lands 3 m along +x in map (REP-103 forward), not along +z.
        assert goal.pose.position.x == pytest.approx(3.0, abs=0.01)
        assert goal.pose.position.y == pytest.approx(0.0, abs=0.01)
        assert goal.pose.position.z == pytest.approx(0.0, abs=0.01)

    def test_a_stop_command_prevents_a_goal(self, intent_setup):
        graph, _node, goals, statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.3)
        say("come here")
        graph.spin_for(0.3)
        say("stop")

        def frame():
            hear(0.0)
            see(confidence=0.95)

        graph.spin_for(2.5, each=frame)

        assert goals == []
        assert statuses[-1].state == "STOPPED"
        assert statuses[-1].reason == "USER_STOP"


class TestBearingConventionAtNodeLevel:
    def test_a_caller_on_the_left_is_confirmed(self, intent_setup):
        """
        The frame-sign regression, end to end through the node.

        Person 15 deg to the robot's LEFT, heard 15 deg to the LEFT. Under the
        old uncorrected comparison these read as 30 deg apart and the correct
        caller was rejected.
        """
        graph, _node, goals, _statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")

        def frame():
            hear(15.0)
            see(bearing_deg=15.0, confidence=0.9)

        graph.spin_for(3.0, each=frame)
        assert len(goals) == 1

    def test_a_caller_on_the_right_is_confirmed(self, intent_setup):
        graph, _node, goals, _statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")

        def frame():
            hear(-20.0)
            see(bearing_deg=-20.0, confidence=0.9)

        graph.spin_for(3.0, each=frame)
        assert len(goals) == 1

    def test_the_mirror_image_bystander_is_rejected(self, intent_setup):
        """Heard on the left, seen on the right: not the same person."""
        graph, _node, goals, statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")

        def frame():
            hear(20.0)
            see(bearing_deg=-20.0, confidence=0.98)

        graph.spin_for(3.0, each=frame)
        assert goals == []
        assert statuses[-1].reason == "BEARING_MISMATCH"


class TestCallerMotionAtNodeLevel:
    @pytest.mark.parametrize("bearing", [0.0, 10.0, 18.0, 19.0, 24.0])
    def test_confirmation_holds_across_the_reported_breakpoint(
        self, intent_setup, bearing
    ):
        """
        Test 11 at node level: the reported 18-19 deg cliff is gone, and 25 deg
        remains inside the tolerance the parameter advertises.
        """
        graph, _node, goals, _statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")

        def frame():
            hear(0.0)
            see(bearing_deg=bearing, confidence=0.9)

        graph.spin_for(3.0, each=frame)
        assert len(goals) == 1, f"caller at {bearing} deg was not confirmed"

    def test_beyond_the_gate_is_rejected_with_a_reason(self, intent_setup):
        graph, _node, goals, statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")

        def frame():
            hear(0.0)
            see(bearing_deg=40.0, confidence=0.9)

        graph.spin_for(2.5, each=frame)
        assert goals == []
        assert statuses[-1].reason == "BEARING_MISMATCH"


class TestVisualOnlyAtNodeLevel:
    def test_confirmation_succeeds_with_no_microphone_at_all(self, intent_setup):
        """
        Test 12 at node level: the visual-only fallback is reachable by a
        detection the perception node actually produces.

        Required confidence is 0.44/0.85 = 0.518; the old model needed 0.929.
        """
        graph, _node, goals, _statuses, say, _hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")
        graph.spin_for(3.0, each=lambda: see(bearing_deg=0.0, confidence=0.6))

        assert len(goals) == 1

    def test_a_low_confidence_detection_still_does_not_confirm(self, intent_setup):
        """The fallback is reachable, not permissive."""
        graph, _node, goals, statuses, say, _hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")
        graph.spin_for(2.5, each=lambda: see(bearing_deg=0.0, confidence=0.45))

        assert goals == []
        assert statuses[-1].reason == "LOW_CONFIDENCE"


class TestNoSilentDeadlock:
    def test_a_request_with_no_detections_times_out_and_says_so(self, intent_setup):
        """
        The silent-deadlock defect at node level.

        A blind user asks the robot to come, nothing is detected, and the
        system must report a bounded, explained outcome rather than sitting
        in SEARCHING forever.
        """
        graph, node, goals, statuses, say, _hear, _see, _tf = intent_setup

        node._machine._timeout = 1.0  # keep the test fast

        graph.spin_for(0.4)
        say("come here")
        graph.spin_for(2.5)

        assert goals == []
        assert statuses[-1].state == "TIMED_OUT"
        assert statuses[-1].reason == "CONFIRMATION_TIMEOUT"

    def test_status_is_published_even_when_nothing_is_happening(self, intent_setup):
        """
        Status must be timer-driven, not data-driven. A system that only
        speaks when it has data cannot report having no data.
        """
        graph, _node, _goals, statuses, _say, _hear, _see, _tf = intent_setup
        graph.spin_for(1.5)
        assert len(statuses) >= 3


class TestObservability:
    def test_status_carries_the_scores_and_bearing_delta(self, intent_setup):
        graph, _node, _goals, statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")

        def frame():
            hear(0.0)
            see(bearing_deg=10.0, confidence=0.8)

        graph.spin_for(1.5, each=frame)

        latest = statuses[-1]
        assert latest.visual_score == pytest.approx(0.8, abs=1e-3)
        assert latest.audio_available
        assert latest.bearing_delta_deg == pytest.approx(10.0, abs=0.5)
        assert 0.0 < latest.fused_score <= 1.0
        assert latest.num_detections == 1

    def test_status_reports_progress_toward_confirmation(self, intent_setup):
        graph, node, _goals, statuses, say, hear, see, _tf = intent_setup

        graph.spin_for(0.4)
        say("come here")
        graph.spin_for(0.5, each=lambda: (hear(0.0), see(confidence=0.9)))

        progressing = [s for s in statuses if s.state in ("CANDIDATE", "CONFIRMED")]
        assert progressing
        assert progressing[-1].required_confirmations == 5
