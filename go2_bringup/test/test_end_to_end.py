"""
End-to-end integration: perception inputs to actuation, with nothing faked
between them.

Every node in this test is the real one. Only the two ends are synthetic: the
perception inputs (a bag or a camera would supply them on the robot) and the
hardware adapter (a GO2 would supply that). Everything between — fusion,
caller confirmation, intent grounding, goal emission, the candidate
controller, and the safety arbiter — is production code running in a real ROS
graph.

The test proves two things:

1. The repository owns a complete motion decision path: a spoken request plus
   consistent perception produces an authorized velocity at the actuator.
2. **Candidate command is not actuator authority.** The candidate and the
   authorized command are observed separately and differ, and interfering
   with the candidate stream changes what the actuator receives only through
   the arbiter.
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
def stack(graph, tmp_path):
    """The whole decision stack, wired exactly as motion_authority.launch.py wires it."""
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from go2_approach_controller.approach_controller_node import ApproachControllerNode
    from go2_hardware_bridge.dry_run import DryRunGo2Bridge
    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
    from go2_intent_grounding.intent_grounding_node import IntentGroundingNode
    from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Float32, String

    from go2_msgs.msg import DetectedHuman, DetectedHumanArray, SafeVelocityCommand

    log_path = str(tmp_path / "actuation.jsonl")
    adapter = DryRunGo2Bridge(log_path=log_path)

    grounding = graph.add(IntentGroundingNode())
    controller = graph.add(ApproachControllerNode())
    arbiter = graph.add(SafetyArbiterNode())
    bridge = graph.add(HardwareBridgeNode(adapter=adapter))

    source = graph.make_node("synthetic_perception")

    # Bound to a name deliberately: a StaticTransformBroadcaster that is
    # garbage-collected stops serving its transforms, and the whole test then
    # fails on a TF lookup rather than on the behaviour under test.
    tf_broadcaster = publish_standard_tf_chain(source)

    humans_pub = source.create_publisher(DetectedHumanArray, "detected_humans", 10)
    bearing_pub = source.create_publisher(Float32, "audio_bearing_deg", 10)
    voice_pub = source.create_publisher(String, "voice_command", 10)
    hazard_pub = source.create_publisher(String, "safety_state", 10)

    control_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    observed = {"goals": [], "candidates": [], "safe": []}
    sink = graph.make_node("observer")
    sink.create_subscription(PoseStamped, "goal_pose", observed["goals"].append, 10)
    sink.create_subscription(
        TwistStamped, "cmd_vel_candidate", observed["candidates"].append, control_qos
    )
    sink.create_subscription(
        SafeVelocityCommand, "cmd_vel_safe", observed["safe"].append, control_qos
    )

    def perceive(bearing_deg=0.0, distance=4.0, confidence=0.9, hazard="CLEAR"):
        px, py, pz = optical_position_for_body_bearing(bearing_deg, distance)
        human = DetectedHuman()
        human.pose.position.x = px
        human.pose.position.y = py
        human.pose.position.z = pz
        human.pose.orientation.w = 1.0
        human.confidence = float(confidence)
        human.track_id = "caller"

        array = DetectedHumanArray()
        array.header.stamp = source.get_clock().now().to_msg()
        array.header.frame_id = "camera_color_optical_frame"
        array.humans = [human]
        humans_pub.publish(array)

        bearing = Float32()
        bearing.data = float(bearing_deg)
        bearing_pub.publish(bearing)

        if hazard is not None:
            state = String()
            state.data = hazard
            hazard_pub.publish(state)

    def say(text):
        msg = String()
        msg.data = text
        voice_pub.publish(msg)

    return {
        "tf_broadcaster": tf_broadcaster,  # keep alive for the test's lifetime
        "graph": graph,
        "adapter": adapter,
        "log_path": log_path,
        "observed": observed,
        "perceive": perceive,
        "say": say,
        "nodes": {
            "grounding": grounding,
            "controller": controller,
            "arbiter": arbiter,
            "bridge": bridge,
        },
    }


class TestFullChain:
    def test_a_spoken_request_moves_the_robot(self, stack):
        """
        The headline claim, executed rather than asserted in prose.

        speech -> fusion -> confirmation -> goal -> candidate -> arbiter ->
        bridge -> (dry-run) actuator.
        """
        graph, perceive, say = stack["graph"], stack["perceive"], stack["say"]
        observed, adapter = stack["observed"], stack["adapter"]

        graph.spin_for(0.5, each=lambda: perceive())
        say("come here")
        graph.spin_for(4.0, each=lambda: perceive())

        assert len(observed["goals"]) == 1, "no navigation goal was produced"
        assert observed["candidates"], "no candidate motion was produced"
        assert observed["safe"], "the arbiter published nothing"
        assert adapter.moved(), "no command reached the actuator"

    def test_the_actuation_log_is_machine_readable(self, stack):
        """The dry-run bridge records what reached the actuator, as JSONL."""
        import json

        graph, perceive, say = stack["graph"], stack["perceive"], stack["say"]

        graph.spin_for(0.5, each=lambda: perceive())
        say("come here")
        graph.spin_for(3.5, each=lambda: perceive())

        stack["nodes"]["bridge"]._adapter.shutdown()
        lines = Path(stack["log_path"]).read_text().strip().splitlines()
        assert lines
        records = [json.loads(line) for line in lines]
        assert all(set(("t", "kind", "vx", "vy", "wz", "dry_run")) <= set(r) for r in records)
        assert all(r["dry_run"] is True for r in records)
        assert any(abs(r["vx"]) > 0.0 for r in records)

    def test_no_request_means_no_actuation(self, stack):
        """
        The whole chain, exercised negatively.

        Perfect perception with no spoken request must move nothing at all.
        """
        graph, perceive, adapter = stack["graph"], stack["perceive"], stack["adapter"]
        observed = stack["observed"]

        graph.spin_for(4.0, each=lambda: perceive(confidence=0.98))

        assert observed["goals"] == []
        assert not adapter.moved()


class TestCandidateIsNotAuthority:
    def test_the_authorized_command_differs_from_the_candidate(self, stack):
        """
        Proof that the arbiter is in the path and not a passive observer.

        The controller is deliberately asked for more than the envelope
        permits; what reaches the actuator is the arbiter's clamped value,
        not the controller's request.
        """
        graph, perceive, say = stack["graph"], stack["perceive"], stack["say"]
        observed, adapter = stack["observed"], stack["adapter"]
        controller = stack["nodes"]["controller"]

        # A high gain and a distant goal make the controller demand far more
        # than max_vx = 0.4.
        controller._gains = type(controller._gains)(
            k_linear=8.0,
            k_angular=1.2,
            turn_in_place_rad=0.6,
            goal_tolerance_m=0.8,
            yaw_tolerance_rad=0.25,
            slowdown_radius_m=1.5,
        )

        graph.spin_for(0.5, each=lambda: perceive(distance=8.0))
        say("come here")
        graph.spin_for(4.0, each=lambda: perceive(distance=8.0))

        candidate_max = max(c.twist.linear.x for c in observed["candidates"])
        actuated_max = adapter.max_abs_vx()

        assert candidate_max > 0.4, "the controller did not actually over-command"
        assert actuated_max <= 0.4 + 1e-6, (
            f"an over-limit candidate ({candidate_max:.2f} m/s) reached the "
            f"actuator at {actuated_max:.2f} m/s"
        )
        assert actuated_max < candidate_max

    def test_only_the_arbiter_publishes_the_safe_topic(self, stack):
        """
        Exactly one publisher of the actuator's input topic exists in the
        whole graph, and it is the arbiter.
        """
        graph = stack["graph"]
        graph.spin_for(0.5)

        arbiter = stack["nodes"]["arbiter"]
        infos = arbiter.get_publishers_info_by_topic("/cmd_vel_safe")
        assert len(infos) == 1, (
            f"{len(infos)} publishers on the safe topic: "
            f"{[i.node_name for i in infos]}"
        )
        assert infos[0].node_name == "safety_arbiter_node"

    def test_a_hazard_stops_the_robot_mid_approach(self, stack):
        """
        Safety authority demonstrated dynamically: the goal, the controller
        and the candidate stream are all unchanged, and the robot stops
        anyway because the arbiter says so.
        """
        graph, perceive, say = stack["graph"], stack["perceive"], stack["say"]
        observed, adapter = stack["observed"], stack["adapter"]

        graph.spin_for(0.5, each=lambda: perceive())
        say("come here")
        graph.spin_for(3.5, each=lambda: perceive())
        assert adapter.moved()

        adapter.clear()
        graph.spin_for(2.0, each=lambda: perceive(hazard="STAIRS_DETECTED"))

        # The controller is still commanding motion toward the goal...
        recent_candidates = observed["candidates"][-10:]
        assert any(c.twist.linear.x > 0.0 for c in recent_candidates), (
            "the controller stopped on its own, so this proves nothing about "
            "the arbiter"
        )
        # ...but nothing is reaching the actuator.
        assert not adapter.moved(), "motion continued despite a stop-class hazard"

    def test_killing_the_arbiter_stops_the_robot(self, stack):
        """
        The failure the architecture exists to survive: the safety process
        dies while the robot is moving and the controller keeps commanding.
        """
        graph, perceive, say = stack["graph"], stack["perceive"], stack["say"]
        adapter = stack["adapter"]
        arbiter = stack["nodes"]["arbiter"]

        graph.spin_for(0.5, each=lambda: perceive())
        say("come here")
        graph.spin_for(3.5, each=lambda: perceive())
        assert adapter.moved()

        # Kill the arbiter outright. No shutdown stop, no warning: this is the
        # crash case, not the graceful case.
        arbiter._timer.cancel()
        graph.executor.remove_node(arbiter)
        graph.nodes.remove(arbiter)

        adapter.clear()
        graph.spin_for(2.0, each=lambda: perceive())

        records = adapter.velocity_records()
        assert records, "the bridge stopped acting when the arbiter died"
        assert all(
            abs(r["vx"]) < 1e-9 and abs(r["vy"]) < 1e-9 and abs(r["wz"]) < 1e-9
            for r in records[-20:]
        ), "the robot kept moving after the safety authority died"


class TestInteractionOutcomes:
    def test_a_request_that_finds_nobody_reports_a_timeout(self, stack):
        """A blind user must never be left with an interaction that never ends."""
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        from go2_msgs.msg import GroundingStatus

        graph, say, adapter = stack["graph"], stack["say"], stack["adapter"]
        stack["nodes"]["grounding"]._machine._timeout = 1.5

        statuses = []
        listener = graph.make_node("status_listener")
        listener.create_subscription(
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

        graph.spin_for(0.4)
        say("come here")
        graph.spin_for(3.0)

        assert statuses[-1].state == "TIMED_OUT"
        assert statuses[-1].reason == "CONFIRMATION_TIMEOUT"
        assert not adapter.moved()

    def test_stop_halts_an_approach_in_progress(self, stack):
        graph, perceive, say = stack["graph"], stack["perceive"], stack["say"]
        adapter = stack["adapter"]

        graph.spin_for(0.5, each=lambda: perceive())
        say("come here")
        graph.spin_for(3.5, each=lambda: perceive())
        assert adapter.moved()

        say("stop")
        adapter.clear()
        graph.spin_for(2.5, each=lambda: perceive())

        records = adapter.velocity_records()
        assert records

        # The robot does not stop instantaneously: the arbiter's acceleration
        # limit ramps it down, which is the desired behaviour on a legged
        # platform carrying no cargo but standing next to a person. What is
        # asserted is that it converges to zero and STAYS there.
        assert all(
            abs(r["vx"]) < 1e-9 and abs(r["vy"]) < 1e-9 and abs(r["wz"]) < 1e-9
            for r in records[-30:]
        ), "the robot was still moving well after the user said stop"

        moving = [i for i, r in enumerate(records) if abs(r["vx"]) > 1e-9]
        if moving:
            assert moving[-1] < len(records) // 2, (
                "deceleration took more than half the observation window; the "
                "cancel path may not be wired"
            )
