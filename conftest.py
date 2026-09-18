"""
pytest configuration and shared ROS 2 test fixtures.

Two kinds of test live in this repository and they have different needs:

* **Pure-function tests** import a module directly and need only the source
  tree on ``sys.path``.  They run in CI without ROS installed.
* **Node and integration tests** need a real ``rclpy`` context, real message
  types, and isolation from any ROS graph that happens to be running on the
  developer's machine.  They are skipped automatically when ROS is absent.

The isolation matters.  A test that passes because a node from a previous run
is still alive on the default domain is not evidence of anything.  Every
ROS-level test here runs in its own ``rclpy`` context on a domain ID derived
from the process ID, so concurrent runs and stray graphs cannot influence the
result.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

_root = Path(__file__).parent

# Source trees, so pure-function tests can import without a colcon install.
for _pkg in [
    "go2_audio_perception",
    "go2_intent_grounding",
    "go2_perception",
    "go2_safety_monitor",
    "go2_voice_commander",
    "go2_safety_arbiter",
    "go2_hardware_bridge",
    "go2_approach_controller",
    "go2_sim",
]:
    _src = _root / _pkg
    if _src.is_dir() and str(_src) not in sys.path:
        sys.path.insert(0, str(_src))

if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))


def _ros_available() -> bool:
    """True when rclpy AND this repository's generated messages are importable."""
    try:
        import rclpy  # noqa: F401

        from go2_msgs.msg import SafeVelocityCommand  # noqa: F401
    except Exception:  # noqa: BLE001
        return False
    return True


ROS_AVAILABLE = _ros_available()

requires_ros = pytest.mark.skipif(
    not ROS_AVAILABLE,
    reason=(
        "ROS 2 and go2_msgs are not importable. Build the workspace first: "
        "./scripts/reproduce.sh"
    ),
)


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "ros: test requires a live rclpy context and go2_msgs"
    )


@pytest.fixture(scope="function")
def ros_context():
    """
    An initialised rclpy context, isolated by domain ID.

    The GLOBAL context is used deliberately. Node classes under test call
    ``super().__init__(name)`` without a context argument, exactly as they do
    in production, so testing them against a private context would be testing
    a construction path that never runs on the robot.

    Isolation instead comes from ``ROS_DOMAIN_ID``, pinned to a value derived
    from the process ID. A test therefore cannot discover a node belonging to
    another process, another test run, or a graph the developer left running —
    which is the failure mode where a test passes for the wrong reason.
    """
    if not ROS_AVAILABLE:
        pytest.skip("ROS 2 unavailable")
    import rclpy

    # Domain IDs must be in [0, 101] for the default DDS configuration.
    domain = 30 + (os.getpid() % 60)
    previous = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(domain)

    if rclpy.ok():
        # A previous test failed to clean up. Do not silently share its
        # context: that is precisely the cross-contamination being avoided.
        rclpy.shutdown()
    rclpy.init()
    try:
        yield rclpy.get_global_executor()._context
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        if previous is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous


class GraphHarness:
    """
    Runs a set of nodes on one executor and lets a test advance time.

    ``spin_for`` is the workhorse: it pumps the executor for a wall-clock
    duration while optionally invoking a callback each iteration, which is how
    these tests drive a publisher at a realistic rate instead of dumping a
    burst of messages and hoping.
    """

    def __init__(self, context):
        import rclpy.executors

        self.context = context
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.nodes = []

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def make_node(self, name: str):
        from rclpy.node import Node

        return self.add(Node(name))

    def spin_for(self, seconds: float, each=None, step: float = 0.01):
        import time

        deadline = time.time() + seconds
        while time.time() < deadline:
            if each is not None:
                each()
            self.executor.spin_once(timeout_sec=step)

    def shutdown(self):
        for node in self.nodes:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        self.nodes.clear()


@pytest.fixture(scope="function")
def graph(ros_context):
    harness = GraphHarness(ros_context)
    try:
        yield harness
    finally:
        harness.shutdown()


# ── Frame helpers for node tests ──────────────────────────────────────────

#: Rotation from ``base_link`` (REP-103: x forward, y left, z up) to a camera
#: optical frame (REP-105: x right, y down, z forward), as (x, y, z, w).
#: Equivalent to roll=-pi/2, pitch=0, yaw=-pi/2 — the standard ROS
#: camera_link -> camera_optical_frame rotation.
#:
#: Tests use the real rotation rather than identity on purpose. With an
#: identity transform a detection 4 m in front of the camera arrives in
#: base_link as (0, 0, 4) — four metres straight UP — and a controller that
#: reads x and y sees a goal at the origin and reports "goal reached" without
#: moving. That would make an end-to-end test pass while proving nothing.
OPTICAL_FROM_BODY_QUAT = (-0.5, 0.5, -0.5, 0.5)


def publish_standard_tf_chain(node):
    """
    Broadcast map -> base_link -> camera_color_optical_frame.

    ``map -> base_link`` is identity (the robot sits at the map origin, which
    keeps goal arithmetic readable). ``base_link -> camera_color_optical_frame``
    carries the real optical rotation, so a person 3 m in front of the camera
    is 3 m in front of the ROBOT, not 3 m above it.

    Returns the broadcaster, which the caller must keep alive.
    """
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    broadcaster = StaticTransformBroadcaster(node)
    stamp = node.get_clock().now().to_msg()

    map_to_base = TransformStamped()
    map_to_base.header.stamp = stamp
    map_to_base.header.frame_id = "map"
    map_to_base.child_frame_id = "base_link"
    map_to_base.transform.rotation.w = 1.0

    base_to_optical = TransformStamped()
    base_to_optical.header.stamp = stamp
    base_to_optical.header.frame_id = "base_link"
    base_to_optical.child_frame_id = "camera_color_optical_frame"
    qx, qy, qz, qw = OPTICAL_FROM_BODY_QUAT
    base_to_optical.transform.rotation.x = qx
    base_to_optical.transform.rotation.y = qy
    base_to_optical.transform.rotation.z = qz
    base_to_optical.transform.rotation.w = qw

    broadcaster.sendTransform([map_to_base, base_to_optical])
    return broadcaster


def optical_position_for_body_bearing(bearing_deg, distance):
    """
    Camera-optical (x, y, z) for a person at ``bearing_deg`` in BODY frame.

    Positive bearing means to the robot's left, which is NEGATIVE x in the
    optical frame. Tests express scenarios the way a person reasons about
    them; the code under test is responsible for the conversion.
    """
    import math

    angle = math.radians(bearing_deg)
    return (-math.sin(angle) * distance, 0.0, math.cos(angle) * distance)


def mock_missing_modules(names):
    """
    Install ``MagicMock`` stand-ins ONLY for modules that are genuinely absent.

    The pure-function tests for the audio and voice nodes need those modules'
    parent packages importable so the node module can be imported at all, but
    they do not exercise ROS or audio hardware. Previously they installed
    mocks unconditionally via ``sys.modules.setdefault``.

    That poisoned the interpreter for every ROS test that ran afterwards in
    the same session: ``std_msgs`` and ``geometry_msgs`` were left as mocks,
    so real message construction failed deep inside ``tf2_msgs`` with
    ``TypeError: isinstance() arg 2 must be a type``. Collection order decided
    whether the suite passed, which is precisely the "green for the wrong
    reason" failure this repository is trying to eliminate.

    Mocking only what is actually missing keeps the ROS-free CI job working
    while leaving a real ROS installation untouched.
    """
    import importlib
    from unittest.mock import MagicMock

    for name in names:
        if name in sys.modules:
            continue
        try:
            importlib.import_module(name)
        except Exception:  # noqa: BLE001 — any import failure means "absent"
            sys.modules[name] = MagicMock()
