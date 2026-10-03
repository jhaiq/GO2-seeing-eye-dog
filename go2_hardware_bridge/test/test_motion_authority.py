"""AuthorityGate unit tests (pure logic) and bridge-level authority gating."""
import json
import sys
from pathlib import Path

import pytest

from go2_hardware_bridge.motion_authority import ACQUIRED, REVOKED, AuthorityGate

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def grant(owner="nav2", epoch=1, guardian="g1", **kw):
    d = {"v": 1, "owner": owner, "epoch": epoch, "guardian": guardian,
         "state": "RUN", "mode": "auto"}
    d.update(kw)
    return json.dumps(d)


class TestGate:
    def test_owned_when_fresh_and_named(self):
        g = AuthorityGate("nav2", 0.3)
        assert not g.owned(0.0)
        g.on_grant(grant(), 1.0)
        assert g.owned(1.2)
        assert g.update(1.2) == ACQUIRED
        assert g.update(1.25) is None

    def test_stale_grant_revokes_once(self):
        g = AuthorityGate("nav2", 0.3)
        g.on_grant(grant(), 1.0)
        assert g.update(1.1) == ACQUIRED
        assert not g.owned(1.31)
        assert g.update(1.31) == REVOKED
        assert g.update(1.4) is None

    @pytest.mark.parametrize("owner", ["come_here", None])
    def test_other_owner_or_null_revokes_once(self, owner):
        g = AuthorityGate("nav2", 0.3)
        g.on_grant(grant(), 1.0)
        g.update(1.0)
        g.on_grant(grant(owner=owner, epoch=2), 1.1)
        assert not g.owned(1.1)
        assert g.update(1.1) == REVOKED
        g.on_grant(grant(owner=owner, epoch=3), 1.2)
        assert g.update(1.2) is None

    def test_epoch_or_guardian_change_while_owned_reacquires(self):
        g = AuthorityGate("nav2", 0.3)
        g.on_grant(grant(), 1.0)
        assert g.update(1.0) == ACQUIRED
        g.on_grant(grant(epoch=2), 1.1)
        assert g.update(1.1) == ACQUIRED
        assert g.update(1.15) is None
        g.on_grant(grant(epoch=2, guardian="g2"), 1.2)
        assert g.update(1.2) == ACQUIRED

    @pytest.mark.parametrize("raw", [
        "not json", "[]", "null", b"\xff\xfe", 5, None,
        grant(v=2), grant(v="1"), grant(v=True), grant(owner=3),
        grant(epoch="1"), grant(epoch=True), grant(guardian=1),
        json.dumps({"v": 1, "owner": "nav2", "epoch": 1}),
    ])
    def test_malformed_ignored(self, raw):
        g = AuthorityGate("nav2", 0.3)
        assert g.on_grant(raw, 1.0) is False
        assert not g.owned(1.0)
        # and it does not refresh a valid grant
        g.on_grant(grant(), 1.0)
        g.on_grant(raw, 1.25)
        assert not g.owned(1.4)

    def test_legacy_always_owned(self):
        g = AuthorityGate("nav2", 0.3, enabled=False)
        assert g.owned(0.0) and g.owned(1e6)
        assert g.update(5.0) is None
        g.on_grant(grant(owner="come_here"), 1.0)
        assert g.owned(1.0)


# ── Bridge-level ────────────────────────────────────────────────────────
from conftest import requires_ros  # noqa: E402


@pytest.fixture
def authority_setup(graph):
    from go2_hardware_bridge.dry_run import DryRunGo2Bridge
    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String

    from go2_msgs.msg import SafeVelocityCommand

    adapter = DryRunGo2Bridge()
    node = HardwareBridgeNode(adapter=adapter, parameter_overrides=[
        Parameter("motion_authority_topic", Parameter.Type.STRING, "/test/grant"),
        Parameter("motion_authority_name", Parameter.Type.STRING, "nav2"),
        Parameter("grant_timeout_s", Parameter.Type.DOUBLE, 0.3),
    ])
    graph.add(node)
    src = graph.make_node("fake_src")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST)
    cmd_pub = src.create_publisher(SafeVelocityCommand, "cmd_vel_safe", qos)
    grant_pub = src.create_publisher(String, "/test/grant", qos)
    st = {"seq": 0}

    def send(vx=0.2):
        now = src.get_clock().now().nanoseconds / 1e9
        m = SafeVelocityCommand()
        m.header.stamp.sec = int(now)
        m.header.stamp.nanosec = int((now - int(now)) * 1e9)
        m.header.frame_id = "base_link"
        m.twist.linear.x = float(vx)
        m.arbiter_state = "SAFE_TO_MOVE" if vx else "STOPPED"
        m.authority_token = "t"
        st["seq"] += 1
        m.sequence = st["seq"]
        e = now + 0.3
        m.valid_until.sec = int(e)
        m.valid_until.nanosec = int((e - int(e)) * 1e9)
        cmd_pub.publish(m)

    def give(owner="nav2", epoch=1):
        grant_pub.publish(String(data=grant(owner=owner, epoch=epoch)))

    def both(vx=0.2, owner="nav2", epoch=1):
        give(owner, epoch)
        send(vx)

    return graph, node, adapter, send, give, both


def kinds(adapter):
    return [r["kind"] for r in adapter.records]


@requires_ros
class TestBridgeGating:
    def test_nonzero_move_dropped_when_not_owned_zero_still_sent(self, authority_setup):
        graph, node, adapter, send, give, both = authority_setup
        graph.spin_for(0.6, each=lambda: both(0.2, owner="come_here"))
        assert not adapter.moved()
        assert node._authority_dropped > 0
        adapter.clear()
        graph.spin_for(1.2, each=lambda: both(0.0, owner="come_here"))  # > reassert period
        assert "zero" in kinds(adapter)
        assert not adapter.moved()

    def test_move_forwarded_when_owned(self, authority_setup):
        graph, node, adapter, send, give, both = authority_setup
        graph.spin_for(0.6, each=lambda: both(0.2))
        assert adapter.moved()

    def test_revoked_one_stopmove_then_no_move_until_acquired_and_fresh(self, authority_setup):
        graph, node, adapter, send, give, both = authority_setup
        graph.spin_for(0.5, each=lambda: both(0.2))
        assert adapter.moved()
        # Grants stop and commands stop: authority goes stale.
        graph.spin_for(0.5)
        # Grant returns (ACQUIRED) but no fresh command: nothing may move.
        adapter.clear()
        graph.spin_for(0.4, each=give)
        assert not adapter.moved()
        # A fresh command after ACQUIRED moves.
        graph.spin_for(0.3, each=lambda: both(0.2))
        assert adapter.moved()

    def test_revoke_issues_single_stopmove_and_drops_held_command(self, authority_setup):
        graph, node, adapter, send, give, both = authority_setup
        graph.spin_for(0.5, each=lambda: both(0.2))
        adapter.clear()
        # Grant flips to another owner while commands keep flowing.
        graph.spin_for(0.5, each=lambda: both(0.2, owner="come_here"))
        assert not adapter.moved()
        # First post-revoke action is a zero.
        assert kinds(adapter)[0] == "zero"


@requires_ros
class TestBoundedZero:
    """StopMove is a transition plus a bounded reassert, never one per incoming message."""

    def test_unowned_nonzero_stream_does_not_flood_stopmove(self, authority_setup):
        graph, node, adapter, send, give, both = authority_setup
        graph.spin_for(2.0, each=lambda: both(0.2, owner="come_here"))
        assert not adapter.moved()
        assert kinds(adapter).count("zero") <= 3   # ~1 per s, not one per command

    def test_zero_command_stream_is_bounded(self, authority_setup):
        graph, node, adapter, send, give, both = authority_setup
        graph.spin_for(0.5, each=lambda: both(0.2))
        assert adapter.moved()
        adapter.clear()
        graph.spin_for(2.0, each=lambda: both(0.0))
        z = kinds(adapter).count("zero")
        assert 1 <= z <= 3   # transition StopMove + <= 1/s reassert
        assert kinds(adapter)[0] == "zero"
