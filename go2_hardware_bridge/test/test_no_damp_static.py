"""Damp (1001) must never be sent by the bridge: behavioural and static checks."""
import ast
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
PKG = Path(__file__).resolve().parents[1] / "go2_hardware_bridge"


class FakePub:
    def __init__(self):
        self.sent = []

    def get_subscription_count(self):
        return 1

    def publish(self, msg):
        self.sent.append(msg.header.identity.api_id)


def make_adapter():
    pytest.importorskip("unitree_api.msg")
    from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

    pub = FakePub()
    node = SimpleNamespace(create_publisher=lambda *a, **k: pub,
                           get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None))
    return UnitreeSportBridge(node, require_subscriber=False, discovery_timeout_sec=0.0), pub


def test_emergency_stop_publishes_no_damp():
    adapter, pub = make_adapter()
    adapter.emergency_stop()
    assert 1001 not in pub.sent
    assert pub.sent == [1003]
    assert "STOPPED" in str(adapter.health().state).upper()


def test_publish_refuses_damp():
    adapter, pub = make_adapter()
    before = adapter.health().transmit_failures
    assert adapter._publish(1001, None) is False
    assert pub.sent == []
    assert adapter.health().transmit_failures == before + 1


def _damp_sites(path):
    tree = ast.parse(path.read_text())
    hits = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and getattr(n.func, "attr", getattr(n.func, "id", "")) == "_publish" and n.args:
            a = n.args[0]
            if (isinstance(a, ast.Name) and a.id == "API_ID_DAMP") or (
                isinstance(a, ast.Constant) and a.value == 1001
            ):
                hits.append(n.lineno)
    return hits


def test_no_module_passes_damp_to_publish():
    files = sorted(PKG.glob("*.py"))
    assert files
    bad = {f.name: _damp_sites(f) for f in files if _damp_sites(f)}
    assert not bad, bad


def test_damp_constant_not_used_outside_adapter():
    for f in PKG.glob("*.py"):
        if f.name == "unitree_sport.py":
            continue
        assert "API_ID_DAMP" not in f.read_text(), f.name


from conftest import requires_ros  # noqa: E402


@requires_ros
def test_latched_estop_bounded_stopmove_rate(graph):
    from go2_hardware_bridge.dry_run import DryRunGo2Bridge
    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
    from std_srvs.srv import Trigger

    adapter = DryRunGo2Bridge()
    node = HardwareBridgeNode(adapter=adapter)
    graph.add(node)
    client = graph.make_node("estop_client").create_client(Trigger, "/hardware_bridge_node/emergency_stop")
    assert client.wait_for_service(timeout_sec=3.0)
    client.call_async(Trigger.Request())
    graph.spin_for(3.0)
    n = sum(1 for r in adapter.records if r["kind"] == "emergency_stop")
    assert 1 <= n <= 4, n
