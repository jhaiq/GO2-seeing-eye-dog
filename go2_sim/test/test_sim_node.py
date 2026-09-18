"""
ROS-level tests: the simulator against real unitree_api messages and against
the REAL UnitreeSportBridge adapter class from go2_hardware_bridge.

Skipped when unitree_api (unitree_ros2) is not importable, e.g. in CI.
"""
import json
import sys
from pathlib import Path

import pytest

_root = Path(__file__).resolve().parents[2]
for _p in (_root, _root / "go2_sim", _root / "go2_hardware_bridge"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from go2_sim.sim_core import World, world_from_dict  # noqa: E402

from conftest import requires_ros  # noqa: E402

try:
    import unitree_api.msg  # noqa: F401

    _HAS_UNITREE = True
except ImportError:
    _HAS_UNITREE = False

# A skipif marker, not a module-level importorskip: under the launch_testing
# pytest plugin a module-level skip here also swallowed test_sim_core.py.
pytestmark = [
    requires_ros,
    pytest.mark.skipif(not _HAS_UNITREE, reason="unitree_api (unitree_ros2) not importable"),
]

OPEN_WORLD = World(start_x=0.0, start_y=0.0, start_yaw=0.0)


def _make_sim(graph, world=OPEN_WORLD):
    from go2_sim.sim_node import Go2KinematicSimNode

    return graph.add(Go2KinematicSimNode(world=world))


def _request(api_id, params=None, ident=7):
    from unitree_api.msg import Request

    msg = Request()
    msg.header.identity.id = ident
    msg.header.identity.api_id = api_id
    msg.parameter = json.dumps(params) if params is not None else ""
    return msg


def test_move_request_drives_odom_and_stopmove_stops(graph):
    from nav_msgs.msg import Odometry
    from unitree_api.msg import Request, Response

    sim = _make_sim(graph)
    client = graph.make_node("sport_client")
    pub = client.create_publisher(Request, "/api/sport/request", 10)
    odoms, responses = [], []
    client.create_subscription(Odometry, "/utlidar/robot_odom", odoms.append, 10)
    client.create_subscription(Response, "/api/sport/response", responses.append, 10)

    graph.spin_for(0.5)
    pub.publish(_request(1008, {"x": 0.4, "y": 0.0, "z": 0.0}))
    graph.spin_for(1.5)
    assert odoms and odoms[-1].pose.pose.position.x > 0.3
    assert odoms[-1].header.frame_id == "odom" and odoms[-1].child_frame_id == "base_link"
    # Robot clock skew: stamps sit about 320 days behind wall time.
    now_s = client.get_clock().now().nanoseconds / 1e9
    stamp_s = odoms[-1].header.stamp.sec + odoms[-1].header.stamp.nanosec / 1e9
    assert now_s - stamp_s == pytest.approx(27605481.0, abs=1.0)
    assert any(r.header.identity.id == 7 and r.header.status.code == 0 for r in responses)

    pub.publish(_request(1003))
    graph.spin_for(1.0)
    x_stopped = sim.state.x
    graph.spin_for(0.5)
    assert sim.state.x == pytest.approx(x_stopped, abs=1e-3)


def test_move_latches_without_renewal(graph):
    """The real robot keeps executing Move until told otherwise; so must the sim."""
    from unitree_api.msg import Request

    sim = _make_sim(graph)
    pub = graph.make_node("sport_client").create_publisher(Request, "/api/sport/request", 10)
    graph.spin_for(0.5)
    pub.publish(_request(1008, {"x": 0.3, "y": 0.0, "z": 0.0}))
    graph.spin_for(0.5)
    x1 = sim.state.x
    graph.spin_for(1.0)
    assert sim.state.x - x1 == pytest.approx(0.3, abs=0.05)


def test_damp_ignores_later_moves(graph):
    from std_msgs.msg import String
    from unitree_api.msg import Request

    sim = _make_sim(graph)
    client = graph.make_node("sport_client")
    pub = client.create_publisher(Request, "/api/sport/request", 10)
    states = []
    client.create_subscription(String, "/go2_sim/state", states.append, 10)
    graph.spin_for(0.5)
    pub.publish(_request(1001))
    graph.spin_for(0.3)
    pub.publish(_request(1008, {"x": 0.4, "y": 0.0, "z": 0.0}))
    graph.spin_for(1.0)
    assert sim.damped
    assert sim.state.x == pytest.approx(0.0, abs=1e-6)
    assert states and states[-1].data == "DAMPED"


def test_collision_blocks_motion_and_counts(graph):
    world = world_from_dict(
        {
            "start_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "obstacles": [{"x": 1.0, "y": 0.0, "size_x": 0.1, "size_y": 2.0, "height": 1.0}],
        }
    )
    from unitree_api.msg import Request

    sim = _make_sim(graph, world)
    pub = graph.make_node("sport_client").create_publisher(Request, "/api/sport/request", 10)
    graph.spin_for(0.5)
    pub.publish(_request(1008, {"x": 0.5, "y": 0.0, "z": 0.0}))
    graph.spin_for(3.0)
    # Front edge (x + 0.35) must stay short of the wall face at 0.95.
    assert sim.state.x + 0.35 <= 0.95 + 1e-6
    assert sim.collisions == 1


def test_clouds_are_published_in_both_frames(graph):
    from sensor_msgs.msg import PointCloud2

    _make_sim(graph)
    client = graph.make_node("cloud_client")
    raw, deskewed = [], []
    client.create_subscription(PointCloud2, "/utlidar/cloud", raw.append, 10)
    client.create_subscription(PointCloud2, "/utlidar/cloud_deskewed", deskewed.append, 10)
    graph.spin_for(1.0)
    assert raw and deskewed
    assert raw[-1].header.frame_id == "utlidar_lidar"
    assert deskewed[-1].header.frame_id == "odom"
    assert raw[-1].width > 100 and raw[-1].point_step == 16


class TestRealUnitreeAdapter:
    """The REAL UnitreeSportBridge class, never run against hardware, run against the sim."""

    def _adapter(self, graph):
        from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

        node = graph.make_node("bridge_host")
        adapter = UnitreeSportBridge(node, command_hold_sec=0.2, require_subscriber=True)
        graph.spin_for(0.5)
        assert adapter.connect()
        return adapter

    def test_send_velocity_moves_and_send_zero_stops(self, graph):
        sim = _make_sim(graph)
        adapter = self._adapter(graph)
        graph.spin_for(1.5, each=lambda: adapter.send_velocity(0.3, 0.0, 0.0), step=0.05)
        assert sim.state.x > 0.25
        adapter.send_zero()
        graph.spin_for(1.0)
        x = sim.state.x
        graph.spin_for(0.5)
        assert sim.state.x == pytest.approx(x, abs=1e-3)

    def test_tick_watchdog_stops_a_latched_move(self, graph):
        """Stop renewing: tick() must issue StopMove, or the latched Move walks away."""
        sim = _make_sim(graph)
        adapter = self._adapter(graph)
        graph.spin_for(0.5, each=lambda: adapter.send_velocity(0.3, 0.0, 0.0), step=0.05)
        graph.spin_for(1.0, each=adapter.tick, step=0.02)
        x = sim.state.x
        graph.spin_for(0.5, each=adapter.tick, step=0.02)
        assert sim.state.x == pytest.approx(x, abs=1e-3)

    def test_emergency_stop_damps(self, graph):
        sim = _make_sim(graph)
        adapter = self._adapter(graph)
        graph.spin_for(0.5, each=lambda: adapter.send_velocity(0.3, 0.0, 0.0), step=0.05)
        adapter.emergency_stop()
        graph.spin_for(0.3)
        assert sim.damped
        x = sim.state.x
        graph.spin_for(0.5, each=lambda: adapter.send_velocity(0.3, 0.0, 0.0), step=0.05)
        assert sim.state.x == pytest.approx(x, abs=1e-3)

    def test_requests_use_the_hardware_verified_header(self, graph):
        """Unique identity.id per request and noreply=false (field notes s4), so
        every request gets a matchable /api/sport/response."""
        from unitree_api.msg import Request, Response

        _make_sim(graph)
        adapter = self._adapter(graph)
        probe = graph.make_node("header_probe")
        requests, responses = [], []
        probe.create_subscription(Request, "/api/sport/request", requests.append, 50)
        probe.create_subscription(Response, "/api/sport/response", responses.append, 50)
        graph.spin_for(0.3)
        import time as _time

        last = [0.0]

        def send_at_20hz():
            # spin_for calls `each` on every spin_once, which returns early when
            # work is pending; throttle to a bridge-like 20 Hz.
            now = _time.monotonic()
            if now - last[0] >= 0.05:
                last[0] = now
                adapter.send_velocity(0.2, 0.0, 0.0)

        graph.spin_for(0.5, each=send_at_20hz, step=0.01)
        adapter.send_zero()
        graph.spin_for(0.5)
        ids = [r.header.identity.id for r in requests]
        assert len(ids) >= 5
        assert len(set(ids)) == len(ids), "request ids must be unique"
        assert all(r.header.policy.noreply is False for r in requests)
        answered = {r.header.identity.id for r in responses} & set(ids)
        # At a bridge-like rate every request is answered and matchable by id.
        assert len(answered) == len(ids)
        assert ids[-1] in answered

    def test_connect_waits_for_a_late_sport_service(self, graph):
        """Regression: the bridge exited at launch when DDS discovery of the
        sport service lagged construction (2 of 8 closed-loop runs)."""
        import threading
        import time as _time

        from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

        node = graph.make_node("late_bridge_host")
        adapter = UnitreeSportBridge(node, require_subscriber=True, discovery_timeout_sec=5.0)
        started = threading.Timer(1.5, lambda: _make_sim(graph))
        started.start()
        t0 = _time.monotonic()
        assert adapter.connect()
        waited = _time.monotonic() - t0
        started.join()
        assert 1.0 < waited < 5.0

    def test_connect_still_fails_closed_without_a_sport_service(self, graph):
        from go2_hardware_bridge.interface import HardwareBridgeError
        from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

        node = graph.make_node("lonely_bridge_host")
        adapter = UnitreeSportBridge(node, require_subscriber=True, discovery_timeout_sec=0.5)
        with pytest.raises(HardwareBridgeError):
            adapter.connect()
