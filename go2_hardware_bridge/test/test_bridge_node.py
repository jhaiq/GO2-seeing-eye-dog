"""
Hardware-bridge node tests: fail-closed behaviour under real ROS messaging.

These correspond to the required runtime tests:
  Test 4  nominal safe command actuates
  Test 6  stale safe command -> zero at the actuator
  Test 7  safety node death -> bridge fails closed
  Test 8  malformed command -> refused
  Test 9  emergency stop latches
  Test 10 no bypass: the candidate topic cannot reach the bridge
  Test 13 lifecycle startup: zero before inputs are ready
  Test 14 shutdown safety
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import requires_ros  # noqa: E402

pytestmark = requires_ros


@pytest.fixture
def bridge_setup(graph):
    """A bridge node with a recording adapter, plus a publisher of safe commands."""
    from go2_hardware_bridge.dry_run import DryRunGo2Bridge
    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    from go2_msgs.msg import SafeVelocityCommand

    adapter = DryRunGo2Bridge()
    node = HardwareBridgeNode(adapter=adapter)
    graph.add(node)

    source = graph.make_node("fake_arbiter")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    publisher = source.create_publisher(SafeVelocityCommand, "cmd_vel_safe", qos)

    state = {"sequence": 0}

    def send(
        vx=0.2,
        vy=0.0,
        wz=0.0,
        token="test-authority",
        arbiter_state="SAFE_TO_MOVE",
        stamp_offset=0.0,
        lifetime=0.3,
        sequence=None,
    ):
        now = source.get_clock().now().nanoseconds / 1e9
        stamp = now + stamp_offset
        msg = SafeVelocityCommand()
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        msg.arbiter_state = arbiter_state
        msg.authority_token = token
        if sequence is None:
            state["sequence"] += 1
            msg.sequence = state["sequence"]
        else:
            msg.sequence = sequence
        expiry = now + lifetime
        msg.valid_until.sec = int(expiry)
        msg.valid_until.nanosec = int((expiry - int(expiry)) * 1e9)
        publisher.publish(msg)

    return graph, node, adapter, send


class TestNominalActuation:
    def test_valid_command_reaches_the_actuator(self, bridge_setup):
        """Test 4: a nominal authorized command actuates."""
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.6, each=lambda: send(vx=0.2))
        assert adapter.moved()
        assert adapter.max_abs_vx() == pytest.approx(0.2, abs=1e-6)

    def test_transmitted_value_equals_the_authorized_value(self, bridge_setup):
        """The bridge is transport, not policy: it must not alter the value."""
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.15, wz=0.25))
        moving = [
            r for r in adapter.velocity_records() if abs(r["vx"]) > 1e-9
        ]
        assert moving
        assert all(r["vx"] == pytest.approx(0.15) for r in moving)
        assert all(r["wz"] == pytest.approx(0.25) for r in moving)


class TestStaleAndWatchdog:
    def test_stale_command_produces_zero(self, bridge_setup):
        """Test 6: a command older than the watchdog must not actuate."""
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.3, stamp_offset=-5.0))
        assert not adapter.moved()

    def test_expired_command_produces_zero(self, bridge_setup):
        """valid_until is enforced by the bridge independently of the arbiter."""
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.3, lifetime=-1.0))
        assert not adapter.moved()

    def test_silence_after_motion_converges_to_zero(self, bridge_setup):
        """
        Test 7: the arbiter dying must stop the robot.

        Modelled exactly as the failure looks to the bridge: commands were
        arriving, then they stop. No notification, no shutdown message.
        """
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.3))
        assert adapter.moved()

        adapter.clear()
        t_window = adapter._clock()
        graph.spin_for(1.0)  # arbiter is "dead": nothing is published

        records = adapter.velocity_records()
        assert records, "bridge must keep acting on its own timer"

        # A message already in the depth-1 queue when the arbiter died may
        # still be delivered and, while it remains inside its validity window,
        # honoured. That is correct: it was a legitimately authorized command.
        # What must NOT happen is motion continuing past the watchdog. Assert
        # on the tail, and on the fact that motion ceased and stayed ceased.
        # The bridge stops on its own timer and then re-asserts StopMove at a bounded
        # rate (not one per tick), so assert in time rather than by record count:
        # the bridge ends stopped, and any residual motion is confined to the first
        # half of the window, i.e. bounded by the watchdog.
        assert records[-1]["kind"] == "zero", (
            "bridge continued to actuate after its command source disappeared"
        )
        moving = [r for r in records
                  if abs(r["vx"]) > 1e-9 or abs(r["vy"]) > 1e-9 or abs(r["wz"]) > 1e-9]
        if moving:
            assert moving[-1]["t"] - t_window < 0.5, (
                "motion persisted well beyond the watchdog window"
            )
            assert any(r["kind"] == "zero" and r["t"] > moving[-1]["t"] for r in records)


class TestMalformedCommands:
    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_command_is_refused(self, bridge_setup, bad):
        """Test 8: NaN/inf must never actuate."""
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=bad))
        assert not adapter.moved()

    def test_over_limit_command_is_refused_not_clamped(self, bridge_setup):
        """
        An over-limit command means the arbiter malfunctioned.

        The bridge refuses rather than clamping, so the fault surfaces instead
        of being silently absorbed by a second layer of clamping.
        """
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=9.0))
        assert not adapter.moved()

    def test_zero_timestamp_is_refused(self, bridge_setup):
        graph, node, adapter, send = bridge_setup
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        from go2_msgs.msg import SafeVelocityCommand

        source = graph.make_node("zero_stamp_source")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        publisher = source.create_publisher(SafeVelocityCommand, "cmd_vel_safe", qos)

        def send_unstamped():
            msg = SafeVelocityCommand()
            msg.twist.linear.x = 0.3
            msg.arbiter_state = "SAFE_TO_MOVE"
            msg.authority_token = "test-authority"
            msg.sequence = 999
            publisher.publish(msg)

        graph.spin_for(0.5, each=send_unstamped)
        assert not adapter.moved()

    def test_self_contradictory_command_is_refused(self, bridge_setup):
        """A command claiming STOPPED while carrying velocity is a fault."""
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.3, arbiter_state="STOPPED"))
        assert not adapter.moved()

    def test_unknown_arbiter_state_is_refused(self, bridge_setup):
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.3, arbiter_state="PROBABLY_FINE"))
        assert not adapter.moved()


class TestAuthority:
    def test_second_arbiter_cannot_take_over_a_moving_robot(self, bridge_setup):
        """
        Two arbiters running at once must not both drive the robot.

        The bridge latches the first authority token it accepts and refuses a
        different one until it has been stopped for the handover quiet period.
        """
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.2, token="arbiter-A"))
        assert adapter.moved()

        adapter.clear()
        graph.spin_for(0.6, each=lambda: send(vx=0.35, token="arbiter-B"))
        assert not adapter.moved(), "an impostor arbiter took control"

    def test_replayed_sequence_is_refused(self, bridge_setup):
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.4, each=lambda: send(vx=0.2, sequence=5))
        adapter.clear()
        # Replay the same sequence number forever: the bridge must not honour it.
        graph.spin_for(0.6, each=lambda: send(vx=0.3, sequence=5))
        assert not adapter.moved()

    def test_missing_authority_token_is_refused(self, bridge_setup):
        graph, _node, adapter, send = bridge_setup
        graph.spin_for(0.5, each=lambda: send(vx=0.3, token=""))
        assert not adapter.moved()


class TestEmergencyStop:
    def test_estop_service_latches_until_restart(self, bridge_setup):
        """Test 9: actuation stays stopped after an e-stop, despite valid commands."""
        graph, node, adapter, send = bridge_setup
        graph.spin_for(0.4, each=lambda: send(vx=0.2))
        assert adapter.moved()

        from std_srvs.srv import Trigger

        node._estop_cb(Trigger.Request(), Trigger.Response())

        adapter.clear()
        graph.spin_for(1.0, each=lambda: send(vx=0.3))
        assert not adapter.moved(), "motion resumed while e-stop was engaged"


class TestNoBypass:
    def test_bridge_has_no_twist_subscription(self, bridge_setup):
        """
        Test 10, part 1: the bridge exposes no geometry_msgs/Twist inlet.

        Enumerated from the live node rather than by reading source, so adding
        such a subscription in future breaks this test.
        """
        graph, node, _adapter, _send = bridge_setup
        graph.spin_for(0.2)
        types = {
            sub.msg_type.__name__ for sub in node.subscriptions
        }
        assert "Twist" not in types
        assert "TwistStamped" not in types
        assert types == {"SafeVelocityCommand"}, f"unexpected motion inlets: {types}"

    def test_publishing_the_candidate_topic_does_not_actuate(self, bridge_setup):
        """
        Test 10, part 2: a controller shouting on the candidate topic is inert.

        This is the accidental-bypass scenario the architecture exists to
        prevent, executed for real against a live bridge.
        """
        graph, _node, adapter, _send = bridge_setup
        from geometry_msgs.msg import TwistStamped

        rogue = graph.make_node("rogue_controller")
        publisher = rogue.create_publisher(TwistStamped, "cmd_vel_candidate", 10)

        def shout():
            msg = TwistStamped()
            msg.header.stamp = rogue.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = 5.0
            publisher.publish(msg)

        graph.spin_for(1.0, each=shout)
        assert not adapter.moved()

    def test_a_twist_publisher_cannot_even_be_created_on_the_safe_topic(
        self, bridge_setup
    ):
        """
        Test 10, part 3: even naming the safe topic correctly is not enough.

        This is the strongest form of the guarantee, and stronger than
        originally expected: the middleware does not merely decline to match a
        ``geometry_msgs/Twist`` publisher to the bridge's
        ``SafeVelocityCommand`` subscription, it refuses to CREATE the
        publisher at all, because the topic already exists with an
        incompatible type.

        So a controller, teleop node or ``ros2 topic pub`` aimed at
        ``cmd_vel_safe`` fails loudly at construction rather than publishing
        into the void. The separation is enforced by the type system and the
        RMW, not by naming discipline.
        """
        graph, node, adapter, _send = bridge_setup
        from geometry_msgs.msg import Twist

        graph.spin_for(0.2)  # let the bridge's subscription be discovered
        rogue = graph.make_node("rogue_twist")

        with pytest.raises(Exception) as excinfo:
            rogue.create_publisher(Twist, "cmd_vel_safe", 10)
        assert "incompatible type" in str(excinfo.value).lower()
        assert not adapter.moved()


class TestLifecycle:
    def test_no_actuation_before_any_command_arrives(self, bridge_setup):
        """Test 13: before inputs are ready the output must be zero."""
        graph, _node, adapter, _send = bridge_setup
        graph.spin_for(0.6)
        assert not adapter.moved()

    def test_shutdown_stops_the_robot(self, bridge_setup):
        """Test 14: node shutdown must leave the adapter stopped."""
        graph, node, adapter, send = bridge_setup
        graph.spin_for(0.4, each=lambda: send(vx=0.25))
        assert adapter.moved()

        node.destroy_node()
        graph.nodes.remove(node)

        last = adapter.records[-1]
        assert last["kind"] in ("zero", "shutdown")
        assert adapter.health().state == "STOPPED"


class TestObservability:
    def test_status_reports_dry_run_honestly(self, bridge_setup):
        """A dry-run bridge must never present itself as hardware."""
        graph, _node, adapter, send = bridge_setup
        from go2_msgs.msg import BridgeStatus

        received = []
        listener = graph.make_node("status_listener")
        listener.create_subscription(
            BridgeStatus, "bridge/status", received.append, 10
        )
        graph.spin_for(0.6, each=lambda: send(vx=0.2))

        assert received
        assert all(msg.dry_run for msg in received)
        assert all(msg.adapter_name == "DryRunGo2Bridge" for msg in received)

    def test_rejections_are_counted_and_explained(self, bridge_setup):
        graph, node, adapter, send = bridge_setup
        graph.spin_for(0.6, each=lambda: send(vx=9.0))
        assert node._rejected_count > 0
        assert node._last_reject_reasons


class TestProcessLevelShutdown:
    """
    A SIGTERM must run the teardown path, not raise out of ``spin``.

    This is how launch stops a node and how Ctrl-C reaches one. Before
    ``ExternalShutdownException`` was handled, SIGTERM invalidated the rcl
    context underneath the executor and ``spin`` raised
    ``RCLError: failed to initialize wait set``, *before* the ``finally``
    block could call ``destroy_node``. The adapter was therefore never told to
    stop, which on a physical robot means coasting on the last command until
    the onboard controller times out.

    Run as a real subprocess, because the bug only exists at process level:
    an in-process test never sees the signal.
    """

    def test_sigterm_leaves_the_adapter_stopped(self, tmp_path):
        import json
        import os
        import signal
        import subprocess
        import time

        log_path = tmp_path / "actuation.jsonl"
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(30 + (os.getpid() % 60))
        env["ROS_LOCALHOST_ONLY"] = "1"

        # start_new_session puts the node in its own process group, so the
        # signal can be delivered to the GROUP. `ros2 run` is a wrapper that
        # does not forward SIGTERM to the executable it spawns, so signalling
        # the wrapper alone leaves the node running and proves nothing.
        process = subprocess.Popen(
            [
                "ros2", "run", "go2_hardware_bridge", "hardware_bridge_node",
                "--ros-args",
                "-p", f"dry_run_log_path:={log_path}",
            ],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            deadline = time.time() + 15.0
            while time.time() < deadline and not log_path.exists():
                time.sleep(0.2)
            assert log_path.exists(), "bridge never started"
            time.sleep(1.0)

            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            output = process.communicate(timeout=20)[0]
        finally:
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.communicate(timeout=5)

        assert "Traceback" not in output, f"unclean shutdown:\n{output}"

        records = [
            json.loads(line)
            for line in log_path.read_text().strip().splitlines()
            if line
        ]
        assert records, "no actuation records were written"
        assert records[-1]["kind"] == "shutdown", (
            f"teardown did not run; last record was {records[-1]}"
        )
        assert all(
            abs(r["vx"]) < 1e-9 and abs(r["vy"]) < 1e-9 and abs(r["wz"]) < 1e-9
            for r in records
        ), "a nonzero command was left standing at shutdown"
