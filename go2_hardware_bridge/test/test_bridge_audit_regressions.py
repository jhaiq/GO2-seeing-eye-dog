"""
Bridge-side regressions for defects found by the adversarial safety audit.

Closes H3 (authority handover was dead code), M1 (malformed commands were
accepted), the uint64 sequence lockout, and the latching-command hazard in the
physical adapter.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import requires_ros  # noqa: E402

pytestmark = requires_ros


@pytest.fixture
def bridge(graph):
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
    node = graph.add(HardwareBridgeNode(adapter=adapter))
    source = graph.make_node("arbiter_stub")
    publisher = source.create_publisher(
        SafeVelocityCommand,
        "cmd_vel_safe",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        ),
    )
    counters = {"seq": 0}

    def send(
        vx=0.2,
        token="arbiter-A",
        state="SAFE_TO_MOVE",
        frame="base_link",
        lifetime=0.3,
        sequence=None,
        reason_codes=None,
    ):
        now = source.get_clock().now().nanoseconds / 1e9
        msg = SafeVelocityCommand()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
        msg.header.frame_id = frame
        msg.twist.linear.x = float(vx)
        msg.arbiter_state = state
        msg.authority_token = token
        msg.reason_codes = list(reason_codes or [])
        if sequence is None:
            counters["seq"] += 1
            msg.sequence = counters["seq"]
        else:
            msg.sequence = sequence
        expiry = now + lifetime
        msg.valid_until.sec = int(expiry)
        msg.valid_until.nanosec = int((expiry - int(expiry)) * 1e9)
        publisher.publish(msg)

    return graph, node, adapter, send


class TestH3AuthorityHandoverIsReachable:
    """
    H3: the handover branch was unreachable.

    Every idle tick called `_stop`, `_stop` refreshed the quiet timestamp, and
    at 50 Hz the measured quiet period never exceeded 0.02 s against a 1.0 s
    requirement. A legitimate arbiter that crashed and respawned could never
    reclaim authority: the robot was immobilised until the bridge itself was
    restarted. One hostile message could brick motion permanently.
    """

    def test_a_restarted_arbiter_can_reclaim_authority(self, bridge):
        graph, _node, adapter, send = bridge

        graph.spin_for(0.6, each=lambda: send(vx=0.2, token="arbiter-A"))
        assert adapter.moved()

        # Arbiter A dies. Silence long enough to exceed the quiet window.
        adapter.clear()
        graph.spin_for(1.6)

        # Arbiter B (the restart) comes up with a fresh token.
        adapter.clear()
        graph.spin_for(1.0, each=lambda: send(vx=0.2, token="arbiter-B-restarted"))

        assert adapter.moved(), (
            "a restarted arbiter could not reclaim authority; the bridge is "
            "permanently wedged after any arbiter crash"
        )

    def test_handover_is_refused_while_the_robot_is_actually_moving(self, bridge):
        """
        The fix must not reopen C2's window: a second token cannot take over a
        robot that is currently in motion.

        Scoped to well under `authority_handover_quiet_sec` on purpose. Beyond
        that window the bridge is *designed* to hand over, and there is a known
        residual documented in docs/safety_architecture_audit.md: an impostor
        publishing at the same rate displaces the legitimate arbiter in the
        depth-1 queue, which stops the robot and eventually satisfies the quiet
        condition. That chain needs transport-level authentication (SROS2) to
        close, not a tighter timer, and pretending otherwise here would be a
        test that asserts a guarantee the code does not provide.
        """
        graph, node, adapter, send = bridge
        assert node._handover_quiet >= 1.0

        graph.spin_for(0.6, each=lambda: send(vx=0.2, token="arbiter-A"))
        assert adapter.moved()

        adapter.clear()

        def both():
            send(vx=0.2, token="arbiter-A")
            send(vx=0.4, token="impostor-B")

        graph.spin_for(0.5, each=both)

        assert adapter.max_abs_vx() <= 0.2 + 1e-9, (
            "the impostor's command reached the actuator while the legitimate "
            "arbiter was still driving"
        )

    def test_a_near_uint64_sequence_cannot_wedge_the_bridge(self, bridge):
        """
        A single message with `sequence = 2**64 - 1` latched a value nothing
        could exceed, permanently locking the bridge into SEQUENCE_REGRESSION.
        One packet, permanent motion lockout.
        """
        graph, _node, adapter, send = bridge

        graph.spin_for(0.4, each=lambda: send(vx=0.3, sequence=2**64 - 1))
        assert not adapter.moved(), "an implausible sequence number was honoured"

        adapter.clear()
        graph.spin_for(1.0, each=lambda: send(vx=0.2, sequence=None))
        assert adapter.moved(), "the bridge stayed wedged after the hostile message"


class TestM1MalformedCommandsAreRefused:
    """
    M1: 201 of 201 structurally malformed commands were accepted, empty
    `frame_id`, a garbage reason-code vocabulary, `EMERGENCY_STOP` in the
    reasons alongside `SAFE_TO_MOVE`, and a `valid_until` in the year 2038.
    """

    def test_an_empty_frame_id_is_refused(self, bridge):
        graph, _node, adapter, send = bridge
        graph.spin_for(0.8, each=lambda: send(vx=0.3, frame=""))
        assert not adapter.moved()

    def test_an_unexpected_frame_id_is_refused(self, bridge):
        graph, _node, adapter, send = bridge
        graph.spin_for(0.8, each=lambda: send(vx=0.3, frame="odom"))
        assert not adapter.moved()

    def test_a_far_future_validity_horizon_is_refused(self, bridge):
        """
        A command claiming validity into 2038 is not a licence to move until
        2038. An arbiter that stamped one is malfunctioning; a message carrying
        one may not have come from an arbiter at all.
        """
        graph, _node, adapter, send = bridge
        graph.spin_for(0.8, each=lambda: send(vx=0.3, lifetime=100_000.0))
        assert not adapter.moved()

    def test_unknown_reason_codes_are_refused(self, bridge):
        graph, _node, adapter, send = bridge
        graph.spin_for(
            0.8,
            each=lambda: send(vx=0.3, reason_codes=["\x00GARBAGE", "NOT_A_REAL_CODE"]),
        )
        assert not adapter.moved()

    def test_a_stopping_reason_alongside_a_permissive_state_is_refused(self, bridge):
        """An internally inconsistent command is not a trustworthy one."""
        graph, _node, adapter, send = bridge
        graph.spin_for(
            0.8,
            each=lambda: send(
                vx=0.3, state="SAFE_TO_MOVE", reason_codes=["EMERGENCY_STOP"]
            ),
        )
        assert not adapter.moved()

    def test_a_benign_reason_code_still_passes(self, bridge):
        """The vocabulary check must not reject legitimate annotated commands."""
        graph, _node, adapter, send = bridge
        graph.spin_for(1.0, each=lambda: send(vx=0.2, reason_codes=["SPEED_LIMIT"]))
        assert adapter.moved()


class TestC4LatchingAdapterHazard:
    """
    C4: the Sport API's `Move` LATCHES on the robot. A SIGKILLed bridge left
    `Move(0.4, 0, 0)` as the last thing the GO2 heard, and it walks away.

    `DryRunGo2Bridge` is a pure recorder with no latch, so no dry-run test can
    detect this. What CAN be asserted is that the contract carries the hook the
    physical adapter needs, and that the bridge drives it every cycle.
    """

    def test_the_adapter_contract_has_a_tick_hook(self):
        from go2_hardware_bridge.interface import HardwareBridgeInterface

        assert hasattr(HardwareBridgeInterface, "tick")

    def test_the_bridge_ticks_the_adapter_every_cycle(self, bridge):
        graph, _node, adapter, _send = bridge

        ticks = {"n": 0}
        original = adapter.tick

        def counting_tick():
            ticks["n"] += 1
            return original()

        adapter.tick = counting_tick
        graph.spin_for(0.6)

        assert ticks["n"] > 5, (
            "the bridge does not drive the adapter's own watchdog, so a "
            "latching transport would hold its last command indefinitely"
        )

    def test_the_unitree_adapter_stops_when_a_command_is_not_renewed(self):
        """
        The command-hold timeout, tested against a fake `unitree_api` so the
        logic is exercised without hardware.
        """
        import sys
        import time
        import types
        from unittest.mock import MagicMock

        published = []

        class FakeRequest:
            def __init__(self):
                self.header = types.SimpleNamespace(
                    identity=types.SimpleNamespace(api_id=0, id=0),
                    lease=types.SimpleNamespace(id=0),
                    policy=types.SimpleNamespace(priority=0, noreply=False),
                )
                self.parameter = ""
                self.binary = []

        fake_module = types.ModuleType("unitree_api.msg")
        fake_module.Request = FakeRequest
        parent = types.ModuleType("unitree_api")
        parent.msg = fake_module
        sys.modules["unitree_api"] = parent
        sys.modules["unitree_api.msg"] = fake_module
        try:
            from go2_hardware_bridge.unitree_sport import (
                API_ID_MOVE,
                API_ID_STOP_MOVE,
                UnitreeSportBridge,
            )

            publisher = MagicMock()
            publisher.get_subscription_count.return_value = 1
            publisher.publish.side_effect = lambda m: published.append(
                m.header.identity.api_id
            )
            node = MagicMock()
            node.create_publisher.return_value = publisher

            adapter = UnitreeSportBridge(node, command_hold_sec=0.05)
            adapter.send_velocity(0.3, 0.0, 0.0)
            assert published[-1] == API_ID_MOVE

            adapter.tick()
            assert published[-1] == API_ID_MOVE, "stopped too eagerly"

            time.sleep(0.08)
            adapter.tick()
            assert published[-1] == API_ID_STOP_MOVE, (
                "an un-renewed Move was left latched on the robot"
            )
        finally:
            sys.modules.pop("unitree_api", None)
            sys.modules.pop("unitree_api.msg", None)

    def test_the_unitree_adapter_reports_failure_with_no_subscriber(self):
        """
        M3: DDS publish() does not raise when nobody listens, so
        `send_velocity` returned True unconditionally and the transmit-failure
        counter could never trip. The bridge believed it had stopped a robot it
        never reached.
        """
        import sys
        import types
        from unittest.mock import MagicMock

        class FakeRequest:
            def __init__(self):
                self.header = types.SimpleNamespace(
                    identity=types.SimpleNamespace(api_id=0, id=0),
                    lease=types.SimpleNamespace(id=0),
                    policy=types.SimpleNamespace(priority=0, noreply=False),
                )
                self.parameter = ""
                self.binary = []

        fake_module = types.ModuleType("unitree_api.msg")
        fake_module.Request = FakeRequest
        parent = types.ModuleType("unitree_api")
        parent.msg = fake_module
        sys.modules["unitree_api"] = parent
        sys.modules["unitree_api.msg"] = fake_module
        try:
            from go2_hardware_bridge.interface import HardwareBridgeError
            from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

            publisher = MagicMock()
            publisher.get_subscription_count.return_value = 0
            node = MagicMock()
            node.create_publisher.return_value = publisher

            adapter = UnitreeSportBridge(node)
            assert adapter.send_velocity(0.3, 0.0, 0.0) is False
            assert adapter.health().transmit_failures >= 1

            with pytest.raises(HardwareBridgeError):
                adapter.connect()
        finally:
            sys.modules.pop("unitree_api", None)
            sys.modules.pop("unitree_api.msg", None)
