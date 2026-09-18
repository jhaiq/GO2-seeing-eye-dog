"""
go2_state_relay_node: turn the stock GO2 sensor surface into a navigable frame tree.

A stock GO2 publishes no /tf, no map frame, and stamps every message in a
robot clock that was measured 27,605,481 s behind the payload (see
docs/go2_field_notes.md). This node:

    /utlidar/robot_odom (robot clock)  ->  /odom + TF odom->base_link (local clock)
    <lidar cloud> (robot clock)        ->  /go2/lidar/points (local clock, same frame)
    params                             ->  static TF base_link->utlidar_lidar (raw cloud only)
    freshness of all of the above      ->  /go2/localization_valid (std_msgs/Bool, 10 Hz)

Freshness is judged by RECEIPT time on a steady clock, never by message
stamps, so a stalled or skewed robot clock cannot make stale data look fresh.

localization_valid is published continuously and is False unless every
required input is fresh. It is the arbiter's require_localization input, so
losing odometry, the LiDAR, or (when require_map_frame) the SLAM map->odom
transform stops the robot.
"""
from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from go2_localization.clock_offset import ClockOffsetEstimator, split_stamp

RELIABLE_10 = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class StateRelayNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("go2_state_relay_node", parameter_overrides=parameter_overrides or [])

        self.declare_parameter("odom_in_topic", "/utlidar/robot_odom")
        self.declare_parameter("cloud_in_topic", "/utlidar/cloud_deskewed")
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("cloud_out_topic", "/go2/lidar/points")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        # offset: learn robot->local clock offset from odom (default).
        # receipt: stamp with local receive time. none: pass robot stamps through.
        self.declare_parameter("restamp_mode", "offset")
        self.declare_parameter("clock_jump_threshold_s", 1.0)
        # Static extrinsic for a sensor-frame cloud. Only published when true;
        # a deskewed cloud is already in odom and needs none.
        self.declare_parameter("publish_lidar_extrinsic", False)
        self.declare_parameter("lidar_frame", "utlidar_lidar")
        self.declare_parameter("lidar_xyz", [0.28945, 0.0, -0.046825])
        self.declare_parameter("lidar_rpy", [0.0, 2.8782, 0.0])
        self.declare_parameter("odom_max_age_s", 0.2)
        self.declare_parameter("cloud_max_age_s", 0.5)
        self.declare_parameter("require_cloud", True)
        self.declare_parameter("require_map_frame", False)
        self.declare_parameter("map_tf_max_age_s", 1.0)
        self.declare_parameter("validity_rate_hz", 10.0)
        # Never forward a cloud stamped later than (newest odom pose - margin).
        # Humble tf2_ros (0.25.23, current) has a lock-order-inversion deadlock
        # between TransformListener::subscription_callback ->
        # testTransformableRequests and MessageFilter -> waitForTransform ->
        # addTransformableRequest, which is only reachable while a transform
        # request is PENDING. It froze Nav2's controller TF buffer for good in
        # about 3 in 10 closed-loop sim trials (gdb stacks of the frozen
        # controller_server). A cloud whose pose is already buffered never
        # creates a pending request in p2l, slam_toolbox or the costmaps.
        # Cost: at most margin of stamp shift (7 mm at 0.35 m/s at 0.02 s).
        self.declare_parameter("cloud_stamp_margin_s", 0.02)

        p = self.get_parameter
        self._odom_frame = str(p("odom_frame").value)
        self._base_frame = str(p("base_frame").value)
        self._map_frame = str(p("map_frame").value)
        self._restamp_mode = str(p("restamp_mode").value)
        if self._restamp_mode not in ("offset", "receipt", "none"):
            raise ValueError(f"restamp_mode must be offset|receipt|none, got {self._restamp_mode!r}")
        self._clock_est = ClockOffsetEstimator(
            jump_threshold_s=float(p("clock_jump_threshold_s").value)
        )

        self._odom_rx: Optional[float] = None
        self._latest_odom_stamp_s: Optional[float] = None
        self._clouds_clamped = 0
        self._cloud_rx: Optional[float] = None
        self._odom_count = 0
        self._cloud_count = 0
        self._cloud_dropped_unready = 0

        self._tf = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_pub = self.create_publisher(Odometry, str(p("odom_out_topic").value), RELIABLE_10)
        self._cloud_pub = self.create_publisher(
            PointCloud2, str(p("cloud_out_topic").value), qos_profile_sensor_data
        )
        self._valid_pub = self.create_publisher(Bool, "/go2/localization_valid", RELIABLE_10)
        self._status_pub = self.create_publisher(String, "/go2/localization/status", RELIABLE_10)

        # Sensor QoS (best effort) matches both reliable and best-effort publishers.
        self.create_subscription(
            Odometry, str(p("odom_in_topic").value), self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, str(p("cloud_in_topic").value), self._on_cloud, qos_profile_sensor_data
        )

        if bool(p("publish_lidar_extrinsic").value):
            self._publish_extrinsic()

        rate = max(1.0, float(p("validity_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_validity)
        self.get_logger().info(
            f"relay: {p('odom_in_topic').value} -> {p('odom_out_topic').value} + TF "
            f"{self._odom_frame}->{self._base_frame}; {p('cloud_in_topic').value} -> "
            f"{p('cloud_out_topic').value}; restamp={self._restamp_mode}"
        )

    # ------------------------------------------------------------------ time
    @staticmethod
    def _steady() -> float:
        return time.monotonic()

    def _local_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _restamp(self, stamp, learn: bool):
        """Return a corrected builtin_interfaces/Time, or None if not yet possible."""
        stamp_s = stamp.sec + stamp.nanosec * 1e-9
        now_s = self._local_now_s()
        if self._restamp_mode == "none":
            return stamp
        if self._restamp_mode == "receipt":
            corrected = now_s
        else:
            if learn:
                self._clock_est.add(stamp_s, now_s)
            corrected = self._clock_est.correct(stamp_s)
            if corrected is None:
                return None
        out = type(stamp)()
        out.sec, out.nanosec = split_stamp(corrected)
        return out

    # -------------------------------------------------------------- inputs
    def _on_odom(self, msg: Odometry) -> None:
        stamp = self._restamp(msg.header.stamp, learn=True)
        if stamp is None:
            return
        self._odom_rx = self._steady()
        self._odom_count += 1
        self._latest_odom_stamp_s = stamp.sec + stamp.nanosec * 1e-9

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self._odom_pub.publish(out)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(tf)

    def _on_cloud(self, msg: PointCloud2) -> None:
        # Clouds never train the estimator: their stamp-to-receipt gap includes
        # sweep duration, which would bias the minimum.
        stamp = self._restamp(msg.header.stamp, learn=False)
        if stamp is None or self._latest_odom_stamp_s is None:
            self._cloud_dropped_unready += 1
            return
        limit = self._latest_odom_stamp_s - float(self.get_parameter("cloud_stamp_margin_s").value)
        if stamp.sec + stamp.nanosec * 1e-9 > limit:
            stamp = type(stamp)()
            stamp.sec, stamp.nanosec = split_stamp(limit)
            self._clouds_clamped += 1
        self._cloud_rx = self._steady()
        self._cloud_count += 1
        msg.header.stamp = stamp
        self._cloud_pub.publish(msg)

    def _publish_extrinsic(self) -> None:
        xyz = [float(v) for v in self.get_parameter("lidar_xyz").value]
        rpy = [float(v) for v in self.get_parameter("lidar_rpy").value]
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError("lidar_xyz and lidar_rpy must have 3 elements")
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._base_frame
        tf.child_frame_id = str(self.get_parameter("lidar_frame").value)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        q = quaternion_from_rpy(*rpy)
        (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ) = q
        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(tf)

    # ------------------------------------------------------------ validity
    def _age(self, rx: Optional[float]) -> Optional[float]:
        return None if rx is None else self._steady() - rx

    def _map_tf_error(self) -> Optional[str]:
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._odom_frame, Time())
        except TransformException as exc:
            return f"no {self._map_frame}->{self._odom_frame}: {exc}"
        stamp_s = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if stamp_s > 0.0:
            # slam_toolbox future-dates map->odom by its publish period, so a
            # negative age is normal; only an old transform is a failure.
            age = self._local_now_s() - stamp_s
            limit = float(self.get_parameter("map_tf_max_age_s").value)
            if age > limit:
                return f"{self._map_frame}->{self._odom_frame} is {age:.2f}s old"
        return None

    def evaluate(self) -> tuple[bool, list[str]]:
        reasons = []
        odom_age = self._age(self._odom_rx)
        if odom_age is None:
            reasons.append("odom never received" if self._clock_est.ready or self._restamp_mode != "offset"
                           else "odom never received (clock offset not learned)")
        elif odom_age > float(self.get_parameter("odom_max_age_s").value):
            reasons.append(f"odom stale {odom_age:.2f}s")
        if bool(self.get_parameter("require_cloud").value):
            cloud_age = self._age(self._cloud_rx)
            if cloud_age is None:
                reasons.append("lidar never received")
            elif cloud_age > float(self.get_parameter("cloud_max_age_s").value):
                reasons.append(f"lidar stale {cloud_age:.2f}s")
        if bool(self.get_parameter("require_map_frame").value):
            err = self._map_tf_error()
            if err:
                reasons.append(err)
        return (not reasons), reasons

    def _publish_validity(self) -> None:
        ok, reasons = self.evaluate()
        self._valid_pub.publish(Bool(data=ok))
        status = {
            "valid": ok,
            "reasons": reasons,
            "restamp_mode": self._restamp_mode,
            "clock_offset_s": self._clock_est.offset_s,
            "clock_resets": self._clock_est.resets,
            "odom_msgs": self._odom_count,
            "cloud_msgs": self._cloud_count,
            "cloud_dropped_before_offset": self._cloud_dropped_unready,
            "clouds_stamp_clamped": self._clouds_clamped,
        }
        self._status_pub.publish(String(data=json.dumps(status)))
        if not ok:
            self.get_logger().warn(
                "localization INVALID: " + "; ".join(reasons), throttle_duration_sec=5.0
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._valid_pub.publish(Bool(data=False))
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

