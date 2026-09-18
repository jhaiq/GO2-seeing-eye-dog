"""
go2_kinematic_sim_node: a stand-in for the GO2's DDS surface.

Consumes the same Sport API requests the real robot does and publishes the
same odometry and lidar topics, with the same frame ids and the same robot
clock skew, so the real motion chain (arbiter -> bridge with the
``unitree_sport`` adapter) can be exercised closed loop.

  subscribes  /api/sport/request        unitree_api/msg/Request
  publishes   /api/sport/response       unitree_api/msg/Response
              /utlidar/robot_odom       nav_msgs/Odometry      odom -> base_link, skewed stamps
              /utlidar/cloud            sensor_msgs/PointCloud2 utlidar_lidar, skewed stamps
              /utlidar/cloud_deskewed   sensor_msgs/PointCloud2 odom, skewed stamps
              /go2_sim/ground_truth     geometry_msgs/PoseStamped odom, unskewed stamps
              /go2_sim/state            std_msgs/String  MOVING | IDLE | DAMPED
              /go2_sim/collisions       std_msgs/UInt32  collision events so far
              /go2_sim/objects          visualization_msgs/MarkerArray (latched)

Like the real robot it publishes NO /tf. Move is LATCHED: the last velocity
keeps executing until another request arrives, exactly the property the
bridge's command-starvation watchdog exists to handle.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, UInt32
from visualization_msgs.msg import Marker, MarkerArray

from go2_sim.sim_core import (
    API_ID_DAMP,
    BASE_HEIGHT_M,
    DEFAULT_CLOCK_SKEW_SEC,
    LidarExtrinsic,
    RobotState,
    World,
    beam_directions,
    footprint_collides,
    load_world,
    parse_sport_request,
    simulate_scan,
    skewed_stamp_ns,
    step_kinematics,
    yaw_to_quat,
)

try:
    from unitree_api.msg import Request, Response
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "go2_sim needs unitree_api (unitree_ros2). Source its install space first."
    ) from exc


def _stamp(msg_stamp, ns: int) -> None:
    msg_stamp.sec = ns // 1_000_000_000
    msg_stamp.nanosec = ns % 1_000_000_000


def make_cloud(points: np.ndarray, frame_id: str, stamp_ns: int) -> PointCloud2:
    """x, y, z, intensity float32 cloud built directly from a numpy buffer."""
    n = len(points)
    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, :3] = points
    buf[:, 3] = 100.0
    msg = PointCloud2()
    _stamp(msg.header.stamp, stamp_ns)
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(("x", "y", "z", "intensity"))
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = buf.tobytes()
    return msg


class Go2KinematicSimNode(Node):
    def __init__(self, world: World | None = None, **kwargs) -> None:
        super().__init__("go2_kinematic_sim_node", **kwargs)
        p = self.declare_parameter
        p("world_file", "")
        p("clock_skew_sec", DEFAULT_CLOCK_SKEW_SEC)
        p("tau_sec", 0.15)
        p("physics_rate_hz", 100.0)
        p("odom_rate_hz", 150.0)
        p("cloud_rate_hz", 15.0)
        p("footprint_length_m", 0.70)
        p("footprint_width_m", 0.31)
        p("lidar_xyz", [0.29, 0.0, -0.05])
        p("lidar_rpy", [0.0, 2.8782, 0.0])
        p("lidar_n_azimuth", 120)
        p("lidar_n_elevation", 16)
        p("lidar_max_range_m", 10.0)
        p("lidar_noise_std_m", 0.01)
        p("random_seed", 0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        if world is None:
            path = str(g("world_file"))
            world = load_world(path) if path else World()
        self._world = world
        self._skew = float(g("clock_skew_sec"))
        self._tau = float(g("tau_sec"))
        self._fp = (float(g("footprint_length_m")), float(g("footprint_width_m")))
        self._extrinsic = LidarExtrinsic(
            xyz=tuple(float(v) for v in g("lidar_xyz")),
            rpy=tuple(float(v) for v in g("lidar_rpy")),
        )
        self._dirs = beam_directions(int(g("lidar_n_azimuth")), int(g("lidar_n_elevation")))
        self._max_range = float(g("lidar_max_range_m"))
        self._noise = float(g("lidar_noise_std_m"))
        self._rng = np.random.default_rng(int(g("random_seed")))

        self._lock = threading.Lock()
        self._state = RobotState(world.start_x, world.start_y, world.start_yaw)
        self._cmd = (0.0, 0.0, 0.0)
        self._damped = False
        self._collisions = 0
        self._blocked = False
        self._last_step = time.monotonic()
        self.cloud_times_ms: list[float] = []

        self._pub_odom = self.create_publisher(Odometry, "/utlidar/robot_odom", 10)
        self._pub_cloud = self.create_publisher(PointCloud2, "/utlidar/cloud", 5)
        self._pub_cloud_odom = self.create_publisher(PointCloud2, "/utlidar/cloud_deskewed", 5)
        self._pub_resp = self.create_publisher(Response, "/api/sport/response", 10)
        self._pub_truth = self.create_publisher(PoseStamped, "/go2_sim/ground_truth", 10)
        self._pub_state = self.create_publisher(String, "/go2_sim/state", 10)
        self._pub_coll = self.create_publisher(UInt32, "/go2_sim/collisions", 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_objects = self.create_publisher(MarkerArray, "/go2_sim/objects", latched)
        self.create_subscription(Request, "/api/sport/request", self._on_request, 10)

        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / float(g("physics_rate_hz")), self._physics, clock=steady)
        self.create_timer(1.0 / float(g("odom_rate_hz")), self._publish_odom, clock=steady)
        self.create_timer(1.0 / float(g("cloud_rate_hz")), self._publish_cloud, clock=steady)
        self.create_timer(0.1, self._publish_status, clock=steady)
        self._publish_objects()
        self.get_logger().info(
            f"GO2 kinematic sim: {len(world.obstacles)} obstacles, {len(world.objects)} objects, "
            f"clock skew {self._skew:.1f}s, start ({world.start_x:.2f}, {world.start_y:.2f})"
        )

    # ── Sport API ──────────────────────────────────────────────────────

    def _on_request(self, msg: Request) -> None:
        api_id = int(msg.header.identity.api_id)
        command = parse_sport_request(api_id, msg.parameter)
        code = 0
        with self._lock:
            if command.error:
                code = -1
                self.get_logger().warn(f"rejected request: {command.error}")
            elif api_id == API_ID_DAMP:
                self._damped = True
                self._cmd = (0.0, 0.0, 0.0)
                self._state.vx = self._state.vy = self._state.wz = 0.0
            elif self._damped and any(abs(v) > 0.0 for v in command.velocity):
                code = -2  # damped robots do not walk
            else:
                if command.velocity != self._cmd:
                    # A new command is a new attempt: a later contact counts again.
                    self._blocked = False
                self._cmd = command.velocity
        resp = Response()
        resp.header.identity.id = msg.header.identity.id
        resp.header.identity.api_id = api_id
        resp.header.status.code = code
        self._pub_resp.publish(resp)

    # ── Physics ────────────────────────────────────────────────────────

    def _physics(self) -> None:
        now = time.monotonic()
        with self._lock:
            dt = min(now - self._last_step, 0.1)
            self._last_step = now
            if dt <= 0.0:
                return
            nxt = step_kinematics(self._state, self._cmd, dt, self._tau)
            hit = footprint_collides(nxt.x, nxt.y, nxt.yaw, *self._fp, self._world.boxes)
            if hit is None:
                self._state = nxt
            else:
                # Counted once per command: a latched Move pressing into a wall
                # re-touches it every few steps as the lag rebuilds velocity.
                if not self._blocked:
                    self._collisions += 1
                    self.get_logger().warn(f"collision with {hit.label or 'obstacle'}")
                self._blocked = True
                self._state.vx = self._state.vy = self._state.wz = 0.0

    def _snapshot(self) -> RobotState:
        with self._lock:
            s = self._state
            return RobotState(s.x, s.y, s.yaw, s.vx, s.vy, s.wz)

    def _publish_odom(self) -> None:
        s = self._snapshot()
        now_ns = self.get_clock().now().nanoseconds
        odom = Odometry()
        _stamp(odom.header.stamp, skewed_stamp_ns(now_ns, self._skew))
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = s.x
        odom.pose.pose.position.y = s.y
        odom.pose.pose.position.z = BASE_HEIGHT_M
        q = yaw_to_quat(s.yaw)
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        odom.twist.twist.linear.x = s.vx
        odom.twist.twist.linear.y = s.vy
        odom.twist.twist.angular.z = s.wz
        self._pub_odom.publish(odom)

        truth = PoseStamped()
        _stamp(truth.header.stamp, now_ns)
        truth.header.frame_id = "odom"
        truth.pose = odom.pose.pose
        self._pub_truth.publish(truth)

    def _publish_cloud(self) -> None:
        s = self._snapshot()
        t0 = time.perf_counter()
        pts_sensor, pts_odom = simulate_scan(
            s, self._world, self._extrinsic, self._dirs, self._max_range, self._noise, self._rng
        )
        stamp_ns = skewed_stamp_ns(self.get_clock().now().nanoseconds, self._skew)
        self._pub_cloud.publish(make_cloud(pts_sensor, "utlidar_lidar", stamp_ns))
        self._pub_cloud_odom.publish(make_cloud(pts_odom, "odom", stamp_ns))
        self.cloud_times_ms.append((time.perf_counter() - t0) * 1e3)
        if len(self.cloud_times_ms) > 1000:
            del self.cloud_times_ms[:500]

    def _publish_status(self) -> None:
        with self._lock:
            damped, cmd, s, coll = self._damped, self._cmd, self._state, self._collisions
        moving = any(abs(v) > 1e-3 for v in (*cmd, s.vx, s.vy, s.wz))
        self._pub_state.publish(String(data="DAMPED" if damped else ("MOVING" if moving else "IDLE")))
        self._pub_coll.publish(UInt32(data=coll))

    def _publish_objects(self) -> None:
        arr = MarkerArray()
        for i, box in enumerate(self._world.objects):
            m = Marker()
            m.header.frame_id = "odom"
            m.ns = box.label
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = box.x
            m.pose.position.y = box.y
            m.pose.position.z = (box.z_min + box.z_max) / 2.0
            q = yaw_to_quat(box.yaw)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x, m.scale.y, m.scale.z = box.size_x, box.size_y, box.z_max - box.z_min
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.8
            m.text = box.label
            arr.markers.append(m)
        self._pub_objects.publish(arr)

    # Test and tooling helpers.
    @property
    def state(self) -> RobotState:
        return self._snapshot()

    @property
    def damped(self) -> bool:
        with self._lock:
            return self._damped

    @property
    def collisions(self) -> int:
        with self._lock:
            return self._collisions


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2KinematicSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["Go2KinematicSimNode", "make_cloud", "main", "math"]
