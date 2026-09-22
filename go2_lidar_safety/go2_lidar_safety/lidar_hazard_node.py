"""
lidar_hazard_node: camera-free hazard context for the safety arbiter.

    subscribes  cloud (PointCloud2, any frame; remapped to /go2/lidar/points)
    publishes   safety_state          std_msgs/String       CLEAR or the hazard type
                safety_alert          go2_msgs/SafetyAlert  only when a hazard is asserted
                lidar_safety/status   std_msgs/String       JSON diagnostics

Publishing contract, identical to go2_safety_monitor so the arbiter treats the
two sources the same way:

* A hazard publishes SafetyAlert AND the hazard type on safety_state.
* No hazard publishes "CLEAR" on safety_state.
* CLEAR is published ONLY after a fresh cloud was actually transformed and
  evaluated. On a missing transform, a stale cloud, or no cloud at all, the
  node publishes nothing on either safety topic. The arbiter then sees its
  hazard context age past hazard_max_age_sec and stops. Silence is the
  fail-closed signal; a CLEAR that was not earned would be the unsafe one.

Freshness is judged by RECEIPT time on a steady (monotonic) clock, so a
stalled /clock or a skewed robot clock cannot make an old cloud look new.
Evaluation runs on a timer: a cloud that arrives and then is not followed by
another stops producing decisions after max_cloud_age_s.
"""
from __future__ import annotations

import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from go2_lidar_safety.hazard_core import (
    CLEAR,
    HazardParams,
    evaluate,
    quaternion_matrix,
    transform_points,
)
from go2_msgs.msg import SafetyAlert

_PARAM_DEFAULTS = HazardParams()


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """(N, 3) float64 array from any PointCloud2 with x, y, z fields."""
    if msg.width * msg.height == 0:
        return np.zeros((0, 3))
    structured = point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=False
    )
    return np.column_stack(
        [np.asarray(structured[name], dtype=np.float64).reshape(-1) for name in ("x", "y", "z")]
    )


class LidarHazardNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("lidar_hazard_node", **kwargs)

        self.declare_parameter("cloud_topic", "cloud")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eval_rate_hz", 20.0)
        self.declare_parameter("max_cloud_age_s", 0.5)
        # Header stamp sanity against the node clock. 0 disables. Only useful
        # once stamps are corrected to the local clock (the odom relay does).
        self.declare_parameter("max_stamp_age_s", 0.0)
        self.declare_parameter("tf_timeout_s", 0.05)
        for name, value in vars(_PARAM_DEFAULTS).items():
            self.declare_parameter(name, value)

        self._params = HazardParams(
            **{name: self.get_parameter(name).value for name in vars(_PARAM_DEFAULTS)}
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._max_cloud_age = float(self.get_parameter("max_cloud_age_s").value)
        self._max_stamp_age = float(self.get_parameter("max_stamp_age_s").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_s").value)
        rate = float(self.get_parameter("eval_rate_hz").value)
        if not (math.isfinite(rate) and rate > 0.0):
            raise ValueError("eval_rate_hz must be > 0")
        if not (math.isfinite(self._max_cloud_age) and self._max_cloud_age > 0.0):
            raise ValueError("max_cloud_age_s must be > 0")

        self._tf_buffer = tf2_ros.Buffer()
        # Own node + thread: this node spins single-threaded, so a blocking
        # lookup with a timeout could never receive the transform it waits for
        # (a cloud stamped 1 ms after the latest odom TF failed as "future
        # extrapolation" in closed-loop sim). A separate node because a node may
        # only be in one executor. Created here, not via node=None: tf2_ros
        # 0.25.20 (the payload's Humble) dereferences a None node.
        self._tf_node = rclpy.create_node(
            f"{self.get_name()}_tf_listener", namespace=self.get_namespace()
        )
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True
        )

        self._pending: Optional[PointCloud2] = None
        self._pending_steady: float = 0.0

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._cloud_cb,
            qos_profile_sensor_data,
        )
        self._state_pub = self.create_publisher(String, "safety_state", 10)
        self._alert_pub = self.create_publisher(SafetyAlert, "safety_alert", 10)
        self._status_pub = self.create_publisher(String, "lidar_safety/status", 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"LidarHazardNode active. base={self._base_frame} "
            f"stop<{self._params.stop_distance_m:.2f}m slow<{self._params.slowdown_distance_m:.2f}m "
            f"corridor=+/-{self._params.corridor_half_width:.2f}m "
            f"drop_check={'on' if self._params.drop_check_enabled else 'off'}"
        )

    # ── inputs ───────────────────────────────────────────────────────────
    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._pending = msg
        self._pending_steady = time.monotonic()

    # ── evaluation ───────────────────────────────────────────────────────
    def _status(self, decision: str, **fields) -> None:
        payload = {"decision": decision}
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False, default=str)
        self._status_pub.publish(msg)

    def _tick(self) -> None:
        msg = self._pending
        if msg is None:
            self._status("NO_DATA")
            return
        self._pending = None  # each cloud is evaluated at most once
        age = time.monotonic() - self._pending_steady
        if age > self._max_cloud_age:
            self._status("STALE", cloud_age_s=round(age, 3))
            return

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._max_stamp_age > 0.0 and stamp_s > 0.0:
            skew = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
            if abs(skew) > self._max_stamp_age:
                self._status("STALE_STAMP", stamp_skew_s=round(skew, 3))
                return

        try:
            points = cloud_to_xyz(msg)
            frame = msg.header.frame_id
            if frame and frame != self._base_frame:
                tf = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    frame,
                    Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=self._tf_timeout),
                )
                r = tf.transform.rotation
                t = tf.transform.translation
                points = transform_points(
                    points, quaternion_matrix(r.x, r.y, r.z, r.w), (t.x, t.y, t.z)
                )
            elif not frame:
                raise ValueError("cloud has an empty frame_id")
        except Exception as exc:  # noqa: BLE001 - any failure is silence, never CLEAR
            self.get_logger().warn(f"cloud not evaluated: {exc}", throttle_duration_sec=2.0)
            self._status("TF_FAILED", error=str(exc)[:200])
            return

        result = evaluate(points, self._params)
        state = String()
        state.data = result.decision
        if result.decision != CLEAR:
            alert = SafetyAlert()
            alert.header.stamp = self.get_clock().now().to_msg()
            alert.header.frame_id = self._base_frame
            alert.alert_type = result.decision
            alert.distance = float(result.distance) if math.isfinite(result.distance) else -1.0
            alert.description = result.description
            self._alert_pub.publish(alert)
        self._state_pub.publish(state)

        def finite(v):
            return round(v, 3) if math.isfinite(v) else None

        self._status(
            result.decision,
            nearest_ahead_m=finite(result.nearest_ahead_m),
            nearest_surround_m=finite(result.nearest_surround_m),
            floor_points_ahead=result.floor_points_ahead,
            points_evaluated=result.points_evaluated,
            cloud_age_s=round(age, 3),
        )


    def destroy_node(self) -> None:
        self._tf_node.destroy_node()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarHazardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT from launch shuts the context down under the listener thread;
        # that is a normal exit, not a crash.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
