"""
SafetyArbiterNode tests against a live ROS graph.

Covers required Test 5 (over-limit candidate), the timer-driven watchdog, the
emergency-stop service semantics, and the observability contract.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import requires_ros  # noqa: E402

pytestmark = requires_ros


@pytest.fixture
def arbiter_setup(graph):
    from geometry_msgs.msg import TwistStamped
    from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String

    from go2_msgs.msg import SafeVelocityCommand

    node = SafetyArbiterNode()
    graph.add(node)

    source = graph.make_node("fake_controller")
    control_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    candidate_pub = source.create_publisher(
        TwistStamped, "cmd_vel_candidate", control_qos
    )
    hazard_pub = source.create_publisher(String, "safety_state", 10)

    received = []
    sink = graph.make_node("safe_sink")
    sink.create_subscription(
        SafeVelocityCommand, "cmd_vel_safe", received.append, control_qos
    )

    def send(vx=0.2, vy=0.0, wz=0.0, hazard="CLEAR", stamp_offset=0.0, frame="base_link"):
        now = source.get_clock().now().nanoseconds / 1e9 + stamp_offset
        msg = TwistStamped()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
        msg.header.frame_id = frame
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        candidate_pub.publish(msg)
        if hazard is not None:
            state = String()
            state.data = hazard
            hazard_pub.publish(state)

    return graph, node, received, send


def authorized(received):
    """Velocities the arbiter actually authorized."""
    return [
        (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z) for m in received
    ]


class TestNominal:
    def test_in_limit_candidate_is_authorized(self, arbiter_setup):
        graph, _node, received, send = arbiter_setup
        graph.spin_for(1.5, each=lambda: send(vx=0.2))
        assert received
        assert max(v[0] for v in authorized(received)) == pytest.approx(0.2, abs=1e-3)
        assert received[-1].arbiter_state == "SAFE_TO_MOVE"

    def test_every_output_carries_authority_metadata(self, arbiter_setup):
        graph, _node, received, send = arbiter_setup
        graph.spin_for(0.6, each=lambda: send(vx=0.1))
        assert received
        tokens = {m.authority_token for m in received}
        assert len(tokens) == 1 and tokens != {""}
        sequences = [m.sequence for m in received]
        assert sequences == sorted(sequences)
        assert len(set(sequences)) == len(sequences)


class TestOverLimitCandidate:
    def test_over_limit_candidate_is_clamped(self, arbiter_setup):
        """
        Test 5: a controller demanding more than the envelope is clamped to it.

        The documented policy is clamp-not-reject at the arbiter, because an
        over-eager planner is an expected condition, not a fault. The bridge
        applies reject-not-clamp to the same situation, because there an
        over-limit command means the arbiter itself misbehaved.
        """
        graph, node, received, send = arbiter_setup
        graph.spin_for(2.0, each=lambda: send(vx=9.0, wz=9.0))
        vx_values = [v[0] for v in authorized(received)]
        wz_values = [v[2] for v in authorized(received)]
        assert max(vx_values) == pytest.approx(0.4, abs=1e-3)
        assert max(wz_values) == pytest.approx(0.6, abs=1e-3)
        assert any("SPEED_LIMIT" in list(m.reason_codes) for m in received)

    def test_acceleration_limit_prevents_a_step_change(self, arbiter_setup):
        graph, _node, received, send = arbiter_setup
        graph.spin_for(1.0, each=lambda: send(vx=0.4))
        vx_values = [v[0] for v in authorized(received)]
        moving = [v for v in vx_values if v > 0]
        assert moving, "no motion was authorized at all"
        assert moving[0] < 0.4, "velocity stepped straight to the commanded value"


class TestFailClosed:
    def test_never_receiving_hazard_context_prevents_motion(self, arbiter_setup):
        """
        Hazard input that has NEVER arrived is reported as NOT_INITIALIZED.

        Distinct from SAFETY_CONTEXT_STALE, which means "it arrived and then
        aged out". Only the second is a transient; the first usually means the
        safety monitor was never started, and telling the two apart is the
        difference between waiting and debugging.
        """
        graph, _node, received, send = arbiter_setup
        graph.spin_for(1.0, each=lambda: send(vx=0.3, hazard=None))
        assert received
        assert all(v == (0.0, 0.0, 0.0) for v in authorized(received))
        assert any("NOT_INITIALIZED" in list(m.reason_codes) for m in received)

    def test_hazard_context_going_stale_prevents_motion(self, arbiter_setup):
        """Hazard input that arrived and then stopped is SAFETY_CONTEXT_STALE."""
        graph, node, received, send = arbiter_setup
        node._timing = type(node._timing)(
            candidate_max_age_sec=0.3,
            watchdog_timeout_sec=0.5,
            hazard_max_age_sec=0.2,
            localization_max_age_sec=2.0,
            future_tolerance_sec=0.05,
            safe_command_lifetime_sec=0.3,
        )
        node._core._timing = node._timing

        graph.spin_for(0.6, each=lambda: send(vx=0.3))
        assert max(v[0] for v in authorized(received)) > 0.0

        received.clear()
        graph.spin_for(2.0, each=lambda: send(vx=0.3, hazard=None))
        # Deceleration is slew-limited, so assert on convergence rather than on
        # the whole window.
        assert authorized(received)[-1] == (0.0, 0.0, 0.0)
        assert all(v == (0.0, 0.0, 0.0) for v in authorized(received)[-10:])
        assert any("SAFETY_CONTEXT_STALE" in list(m.reason_codes) for m in received)

    def test_hazard_stop_zeroes_a_moving_robot(self, arbiter_setup):
        graph, _node, received, send = arbiter_setup
        graph.spin_for(1.0, each=lambda: send(vx=0.3))
        assert max(v[0] for v in authorized(received)) > 0.0

        received.clear()
        graph.spin_for(1.5, each=lambda: send(vx=0.3, hazard="STAIRS_DETECTED"))
        assert authorized(received)[-1] == (0.0, 0.0, 0.0)
        assert received[-1].arbiter_state == "STOPPED"

    def test_slowdown_hazard_derates(self, arbiter_setup):
        graph, _node, received, send = arbiter_setup
        graph.spin_for(2.0, each=lambda: send(vx=0.4, hazard="SLOWDOWN"))
        assert received[-1].arbiter_state == "DEGRADED"
        assert max(v[0] for v in authorized(received)) == pytest.approx(
            0.4 * 0.35, abs=1e-2
        )

    def test_watchdog_fires_when_the_controller_goes_quiet(self, arbiter_setup):
        """The arbiter's timer must keep publishing stops after its input dies."""
        graph, _node, received, send = arbiter_setup
        from std_msgs.msg import String

        graph.spin_for(1.0, each=lambda: send(vx=0.3))
        assert max(v[0] for v in authorized(received)) > 0.0

        received.clear()
        source = graph.nodes[1]
        hazard_pub = source.create_publisher(String, "safety_state", 10)

        def hazard_only():
            state = String()
            state.data = "CLEAR"
            hazard_pub.publish(state)

        graph.spin_for(1.5, each=hazard_only)
        assert received, "arbiter stopped publishing instead of publishing a stop"
        assert authorized(received)[-1] == (0.0, 0.0, 0.0)
        assert any(
            "WATCHDOG_TIMEOUT" in list(m.reason_codes) for m in received
        )

    def test_wrong_frame_candidate_is_refused(self, arbiter_setup):
        graph, _node, received, send = arbiter_setup
        graph.spin_for(1.0, each=lambda: send(vx=0.3, frame="odom"))
        assert all(v == (0.0, 0.0, 0.0) for v in authorized(received))

    def test_stale_candidate_is_refused(self, arbiter_setup):
        graph, _node, received, send = arbiter_setup
        graph.spin_for(1.0, each=lambda: send(vx=0.3, stamp_offset=-5.0))
        assert all(v == (0.0, 0.0, 0.0) for v in authorized(received))


class TestEmergencyStop:
    def test_estop_topic_engages_but_cannot_release(self, arbiter_setup):
        """
        A topic may engage an e-stop. It must not be able to release one.

        Otherwise any process on the graph — or a replayed bag — could
        re-enable motion by publishing ``false``.
        """
        graph, node, received, send = arbiter_setup
        from std_msgs.msg import Bool

        source = graph.nodes[1]
        estop_pub = source.create_publisher(Bool, "estop", 10)

        graph.spin_for(0.8, each=lambda: send(vx=0.3))
        assert max(v[0] for v in authorized(received)) > 0.0

        def engage():
            send(vx=0.3)
            estop_pub.publish(Bool(data=True))

        graph.spin_for(0.6, each=engage)
        received.clear()

        def try_release():
            send(vx=0.3)
            estop_pub.publish(Bool(data=False))

        graph.spin_for(1.0, each=try_release)
        assert all(v == (0.0, 0.0, 0.0) for v in authorized(received))
        assert all(m.arbiter_state == "EMERGENCY_STOP" for m in received)

    def test_service_release_restores_motion(self, arbiter_setup):
        graph, node, received, send = arbiter_setup
        from std_srvs.srv import Trigger

        node._engage_estop_cb(Trigger.Request(), Trigger.Response())
        graph.spin_for(0.5, each=lambda: send(vx=0.3))
        assert all(v == (0.0, 0.0, 0.0) for v in authorized(received))

        node._release_estop_cb(Trigger.Request(), Trigger.Response())
        received.clear()
        graph.spin_for(1.5, each=lambda: send(vx=0.3))
        assert max(v[0] for v in authorized(received)) > 0.0


class TestShutdown:
    def test_shutdown_emits_an_explicit_stop(self, arbiter_setup):
        """
        Test 14: the arbiter announces a stop rather than merely going quiet.

        Exercises the real teardown path — ``destroy_node`` — and then spins
        only the remaining nodes to collect what was emitted. The arbiter's
        own timer is gone by then, so anything received afterwards is the
        shutdown stop and nothing else.
        """
        graph, node, received, send = arbiter_setup
        graph.spin_for(0.8, each=lambda: send(vx=0.3))
        assert max(v[0] for v in authorized(received)) > 0.0
        received.clear()

        node.destroy_node()
        graph.executor.remove_node(node)
        graph.nodes.remove(node)

        graph.spin_for(0.4)

        assert received, "arbiter went silent on shutdown instead of stopping"
        assert all(v == (0.0, 0.0, 0.0) for v in authorized(received))
        assert all("SHUTDOWN" in list(m.reason_codes) for m in received)
        assert all(m.arbiter_state == "STOPPED" for m in received)


class TestObservability:
    def test_status_topic_reports_state_and_reasons(self, arbiter_setup):
        graph, _node, _received, send = arbiter_setup
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        from go2_msgs.msg import SafetyStatus

        status = []
        listener = graph.make_node("status_listener")
        listener.create_subscription(
            SafetyStatus,
            "safety/status",
            status.append,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        graph.spin_for(1.5, each=lambda: send(vx=9.0))

        assert status
        latest = status[-1]
        assert latest.state in ("SAFE_TO_MOVE", "DEGRADED", "STOPPED", "EMERGENCY_STOP")
        assert latest.decision_count > 0
        assert latest.intervention_count > 0
        assert "SPEED_LIMIT" in list(latest.last_intervention_reasons)
        assert latest.hazard_context_valid

    def test_diagnostics_are_published(self, arbiter_setup):
        graph, _node, _received, send = arbiter_setup
        from diagnostic_msgs.msg import DiagnosticArray

        diags = []
        listener = graph.make_node("diag_listener")
        listener.create_subscription(DiagnosticArray, "/diagnostics", diags.append, 10)
        graph.spin_for(1.0, each=lambda: send(vx=0.2))

        assert diags
        names = {s.name for d in diags for s in d.status}
        assert any("safety_arbiter" in n for n in names)
