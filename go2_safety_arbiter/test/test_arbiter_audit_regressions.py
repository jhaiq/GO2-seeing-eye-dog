"""
Regressions for defects found by the adversarial safety audit.

Each class names the finding it closes. These are the tests that would have
caught the bugs, written after the fact, which is the honest description, and
the reason they are in their own file rather than mixed into the suite that
did not catch them.

The findings that remain OPEN (the trust-boundary ones: any process on the DDS
domain can publish a `SafeVelocityCommand` or call `release_estop`) have no
tests here, because they are not fixed. They are documented in
`docs/safety_architecture_audit.md` and bounded in
`docs/research_system_claims.md`.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from go2_safety_arbiter.limits import (  # noqa: E402
    MAX_ALLOWED_COMMAND_AGE_SEC,
    MAX_ALLOWED_VX,
    MAX_ALLOWED_WATCHDOG_SEC,
    LimitConfigError,
    TimingPolicy,
    VelocityLimits,
)

from conftest import requires_ros  # noqa: E402


class TestH2ParametersAreBounded:
    """
    H2: every safety bound was unbounded above.

    `-p max_vx:=100.0` was accepted and the arbiter dutifully clamped to
    100 m/s. A layer that enforces whatever it is told to enforce is not a
    safety layer. `-p watchdog_timeout_sec:=100000` disabled the watchdog
    entirely, and motion continued 27 s after the producer died.
    """

    @pytest.mark.parametrize(
        "field,bad",
        [
            ("max_vx", 100.0),
            ("max_vy", 50.0),
            ("max_wz", 99.0),
            ("max_accel_linear", 1000.0),
            ("max_accel_angular", 1000.0),
        ],
    )
    def test_velocity_limits_have_hard_ceilings(self, field, bad):
        with pytest.raises(LimitConfigError) as excinfo:
            VelocityLimits(**{field: bad})
        assert "ceiling" in str(excinfo.value)

    @pytest.mark.parametrize(
        "field,bad",
        [
            ("candidate_max_age_sec", 100000.0),
            ("watchdog_timeout_sec", 100000.0),
            ("hazard_max_age_sec", 100000.0),
            ("safe_command_lifetime_sec", 1e12),
            ("future_tolerance_sec", 60.0),
        ],
    )
    def test_timing_policy_has_hard_ceilings(self, field, bad):
        with pytest.raises(LimitConfigError):
            TimingPolicy(**{field: bad})

    def test_the_ceilings_are_not_themselves_configurable(self):
        """
        The ceilings live in source so they cannot be raised from a params
        file or a command line. Raising one must be a code change and a review.
        """
        assert MAX_ALLOWED_VX <= 1.5
        assert MAX_ALLOWED_WATCHDOG_SEC <= 2.0
        assert MAX_ALLOWED_COMMAND_AGE_SEC <= 1.0

    def test_the_shipped_defaults_are_well_inside_the_ceilings(self):
        """A default sitting at its ceiling would leave no margin to reduce."""
        limits = VelocityLimits()
        assert limits.max_vx < MAX_ALLOWED_VX / 2
        timing = TimingPolicy()
        assert timing.watchdog_timeout_sec < MAX_ALLOWED_WATCHDOG_SEC / 2

    def test_a_lifetime_that_would_overflow_the_wire_format_is_rejected(self):
        """
        M2: `safe_command_lifetime_sec` large enough that `now + lifetime`
        exceeded int32 raised inside the publish path and killed the arbiter,
        taking the stop command with it.
        """
        with pytest.raises(LimitConfigError):
            TimingPolicy(safe_command_lifetime_sec=2**31)


@requires_ros
class TestC5HazardChannelsAreResolvedBySeverity:
    """
    C5: the two hazard channels shared one variable, so the last message to
    arrive decided the state. A `std_msgs/String` reading "CLEAR" cancelled an
    EMERGENCY_STOP that the depth pipeline was asserting.
    """

    @pytest.fixture
    def setup(self, graph):
        from geometry_msgs.msg import TwistStamped
        from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String

        from go2_msgs.msg import SafetyAlert, SafeVelocityCommand

        node = SafetyArbiterNode()
        graph.add(node)
        source = graph.make_node("hazard_source")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        candidate_pub = source.create_publisher(TwistStamped, "cmd_vel_candidate", qos)
        alert_pub = source.create_publisher(SafetyAlert, "safety_alert", 10)
        state_pub = source.create_publisher(String, "safety_state", 10)

        received = []
        sink = graph.make_node("safe_sink")
        sink.create_subscription(
            SafeVelocityCommand, "cmd_vel_safe", received.append, qos
        )

        def drive(alert=None, state=None, vx=0.3):
            now = source.get_clock().now()
            msg = TwistStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = vx
            candidate_pub.publish(msg)
            if alert is not None:
                a = SafetyAlert()
                a.header.stamp = now.to_msg()
                a.alert_type = alert
                alert_pub.publish(a)
            if state is not None:
                state_pub.publish(String(data=state))

        return graph, node, received, drive

    def _velocities(self, received):
        return [m.twist.linear.x for m in received]

    def test_a_clear_string_cannot_cancel_an_emergency_alert(self, setup):
        """The exploit, executed: 'CLEAR' spam against a genuine hazard."""
        graph, _node, received, drive = setup

        graph.spin_for(2.0, each=lambda: drive(alert="EMERGENCY_STOP", state="CLEAR"))

        assert received
        assert all(v == 0.0 for v in self._velocities(received)), (
            "a std_msgs/String cancelled a depth-detected emergency stop"
        )

    def test_the_most_restrictive_channel_wins(self, setup):
        graph, _node, received, drive = setup
        graph.spin_for(2.0, each=lambda: drive(alert="SLOWDOWN", state="CLEAR"))
        assert received[-1].arbiter_state in ("DEGRADED", "STOPPED")

    def test_a_stop_hazard_latches_across_a_dropped_frame(self, setup):
        """
        A single missing alert frame must not re-permit motion. Depth pipelines
        drop frames; a safety state that flickers with them is not a safety
        state.
        """
        graph, _node, received, drive = setup

        graph.spin_for(1.0, each=lambda: drive(alert="STAIRS_DETECTED"))
        received.clear()
        # Hazard channel goes quiet but "CLEAR" keeps arriving on the other.
        graph.spin_for(0.3, each=lambda: drive(state="CLEAR"))

        assert all(v == 0.0 for v in self._velocities(received))

    def test_an_unrecognised_alert_is_not_outvoted_by_clear(self, setup):
        graph, _node, received, drive = setup
        graph.spin_for(2.0, each=lambda: drive(alert="SOMETHING_NEW", state="CLEAR"))
        assert all(v == 0.0 for v in self._velocities(received))


@requires_ros
class TestM4InitialInputsAreRequired:
    """
    M4: `require_initial_inputs` was documented, declared, and never read.
    `Reason.NOT_INITIALIZED` was defined and never emitted.
    """

    def test_never_receiving_hazard_context_reports_not_initialized(self, graph):
        from geometry_msgs.msg import TwistStamped
        from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        from go2_msgs.msg import SafeVelocityCommand

        node = SafetyArbiterNode()
        graph.add(node)
        source = graph.make_node("candidate_only")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        pub = source.create_publisher(TwistStamped, "cmd_vel_candidate", qos)
        received = []
        sink = graph.make_node("sink")
        sink.create_subscription(SafeVelocityCommand, "cmd_vel_safe", received.append, qos)

        def drive():
            msg = TwistStamped()
            msg.header.stamp = source.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = 0.3
            pub.publish(msg)

        graph.spin_for(1.5, each=drive)

        assert received
        assert all(m.twist.linear.x == 0.0 for m in received)
        assert any("NOT_INITIALIZED" in list(m.reason_codes) for m in received)


@requires_ros
class TestM2ATickExceptionStillStops:
    """
    M2: an exception in the timer killed the arbiter process outright. The
    core guarded individual rules, but that guarantee was worth nothing if the
    surrounding tick raised, the stop was never published either.
    """

    def test_a_raising_tick_publishes_a_stop_instead_of_dying(self, graph):
        from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        from go2_msgs.msg import SafeVelocityCommand

        node = SafetyArbiterNode()
        graph.add(node)

        received = []
        sink = graph.make_node("sink")
        sink.create_subscription(
            SafeVelocityCommand,
            "cmd_vel_safe",
            received.append,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        def explode():
            raise RuntimeError("simulated fault in the decision path")

        node._tick_inner = explode

        graph.spin_for(1.0)

        assert received, "the arbiter went silent instead of publishing a stop"
        assert all(m.twist.linear.x == 0.0 for m in received)
        assert all(m.arbiter_state == "STOPPED" for m in received)
        assert any(
            "RULE_EVALUATION_FAILED" in list(m.reason_codes) for m in received
        )


@requires_ros
class TestC3WatchdogsUseSteadyTime:
    """
    C3: with `use_sim_time` enabled, a stalled `/clock` froze every watchdog.
    Ages computed as zero, so the watchdog could not fire, the fault was in
    the watchdog's own notion of elapsed time. A robot moving when the clock
    stalled kept moving; the audit measured 14.9 s of wall time at 0.4 m/s
    with every watchdog asleep.
    """

    def test_the_arbiter_watchdog_clock_is_not_the_node_clock(self, graph):
        from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode
        from rclpy.clock import ClockType

        node = graph.add(SafetyArbiterNode())
        assert node._steady_clock.clock_type == ClockType.STEADY_TIME

    def test_the_bridge_watchdog_clock_is_not_the_node_clock(self, graph):
        from go2_hardware_bridge.dry_run import DryRunGo2Bridge
        from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
        from rclpy.clock import ClockType

        node = graph.add(HardwareBridgeNode(adapter=DryRunGo2Bridge()))
        assert node._steady_clock.clock_type == ClockType.STEADY_TIME

    def test_steady_time_advances_independently_of_the_node_clock(self, graph):
        """
        The property that matters: steady time keeps moving whatever the rest
        of the graph believes about time.
        """
        import time

        from go2_safety_arbiter.safety_arbiter_node import SafetyArbiterNode

        node = graph.add(SafetyArbiterNode())
        first = node._steady_now()
        time.sleep(0.25)
        assert node._steady_now() - first >= 0.2


def test_unknown_hazard_outranks_every_motion_permitting_type():
    """Regression: an unrecognised hazard ranked below SLOWDOWN, so SLOWDOWN
    on the other channel outvoted it and motion continued."""
    pytest.importorskip("rclpy")
    from go2_safety_arbiter.safety_arbiter_node import _hazard_severity

    for permitting in ("CLEAR", "SLOWDOWN", "NARROW_PASSAGE", "RESTRICT:F", "RESTRICT:FW"):
        assert _hazard_severity("SOMETHING_NEW") > _hazard_severity(permitting)
    assert _hazard_severity("RESTRICT:FB") > _hazard_severity("RESTRICT:F") > _hazard_severity("SLOWDOWN")
