#!/usr/bin/env python3
"""
ApproachControllerNode, STAGE 1 candidate-motion producer.

*** THIS IS NOT A PLANNER. ***

It performs no path planning, no obstacle avoidance and no costmap
reasoning.  It drives straight at the goal.  It exists because this
repository does not yet provide the map, localization, odometry, TF tree or
laser scan that Nav2's planner and controller servers require (see
``docs/target_runtime_architecture.md`` §"Nav2 honesty"), and shipping a
non-functional Nav2 bring-up would have been worse than shipping an honest
stand-in.

What makes it a *staged* component rather than a permanent hack is that its
runtime contract is byte-identical to what Nav2 will provide:

    subscribes  /goal_pose            geometry_msgs/PoseStamped
    publishes   /cmd_vel_candidate    geometry_msgs/TwistStamped

Nav2's ``controller_server`` consumes the same goal and produces the same
velocity semantics.  Swapping to Nav2 is a launch-file change
(``system.launch.py planner:=nav2``), not an architecture change, and
crucially the safety and actuation half of the stack is untouched by that
swap.

Obstacle avoidance in Stage 1 comes from the SafetyArbiter's hazard policy
(depth-based stop and slowdown), which is a stop-and-refuse behaviour, not a
go-around.  That limitation is recorded in
``docs/research_system_claims.md`` and must not be described as navigation.
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
import tf2_geometry_msgs  # noqa: F401, registers PoseStamped transform support
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from go2_approach_controller.approach import (
    ApproachGains,
    ApproachStatus,
    compute_approach,
)

CONTROL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class ApproachControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("approach_controller_node")

        self.declare_parameter("control_frequency_hz", 20.0)
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("k_linear", 0.6)
        self.declare_parameter("k_angular", 1.2)
        self.declare_parameter("turn_in_place_rad", 0.6)
        self.declare_parameter("goal_tolerance_m", 0.8)
        self.declare_parameter("yaw_tolerance_rad", 0.25)
        self.declare_parameter("slowdown_radius_m", 1.5)
        self.declare_parameter("arrival_epsilon_m", 0.05)
        self.declare_parameter("min_linear_speed", 0.08)
        self.declare_parameter("goal_lifetime_sec", 30.0)
        self.declare_parameter("tf_timeout_sec", 0.1)
        # Oldest transform this loop will act on. Looking up at "latest
        # available" is what a control loop wants, but "latest available"
        # never expires: if the state source dies, tf2 keeps returning the
        # last transform for ever and the controller would keep steering
        # against a frozen pose. Bounding the age turns a dead state source
        # into an explicit NO_TRANSFORM stop. Matched to the arbiter's own
        # 0.5 s watchdog so the two agree on what "stale" means.
        self.declare_parameter("max_transform_age_sec", 0.5)

        freq = float(self.get_parameter("control_frequency_hz").value)
        if not math.isfinite(freq) or freq <= 0.0:
            raise ValueError(f"control_frequency_hz must be finite and > 0, got {freq}")
        self._period = 1.0 / freq
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._max_tf_age = float(self.get_parameter("max_transform_age_sec").value)
        self._goal_lifetime = float(self.get_parameter("goal_lifetime_sec").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self._gains = ApproachGains(
            k_linear=float(self.get_parameter("k_linear").value),
            k_angular=float(self.get_parameter("k_angular").value),
            turn_in_place_rad=float(self.get_parameter("turn_in_place_rad").value),
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
            yaw_tolerance_rad=float(self.get_parameter("yaw_tolerance_rad").value),
            slowdown_radius_m=float(self.get_parameter("slowdown_radius_m").value),
            arrival_epsilon_m=float(self.get_parameter("arrival_epsilon_m").value),
            min_linear_speed=float(self.get_parameter("min_linear_speed").value),
        )

        self._goal: Optional[PoseStamped] = None
        self._goal_time: Optional[float] = None
        self._status = ApproachStatus.IDLE

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(PoseStamped, "goal_pose", self._goal_cb, 10)
        self.create_subscription(String, "cancel_goal", self._cancel_cb, 10)

        self._cmd_pub = self.create_publisher(
            TwistStamped, "cmd_vel_candidate", CONTROL_QOS
        )
        self._status_pub = self.create_publisher(String, "controller/status", STATUS_QOS)

        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().warn(
            "ApproachControllerNode is a STAGED stand-in for Nav2: straight-line "
            "approach, no planning, no obstacle avoidance. Hazard response is the "
            "SafetyArbiter's stop policy only."
        )

    # ── Callbacks ─────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _goal_cb(self, msg: PoseStamped) -> None:
        self._goal = msg
        self._goal_time = self._now()
        self._status = ApproachStatus.ACTIVE
        self.get_logger().info(
            f"New goal in frame '{msg.header.frame_id}': "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    def _cancel_cb(self, _msg: String) -> None:
        self._goal = None
        self._goal_time = None
        self._status = ApproachStatus.IDLE
        self.get_logger().info("Goal cancelled")

    # ── Control loop ──────────────────────────────────────────────────

    def _tick(self) -> None:
        now = self._now()

        if self._goal is None:
            self._status = ApproachStatus.IDLE
            self._publish(0.0, 0.0, 0.0)
            self._publish_status()
            return

        if self._goal_time is not None and (now - self._goal_time) > self._goal_lifetime:
            self.get_logger().warn("Goal exceeded goal_lifetime_sec; abandoning")
            self._goal = None
            self._goal_time = None
            self._status = ApproachStatus.GOAL_STALE
            self._publish(0.0, 0.0, 0.0)
            self._publish_status()
            return

        # Transform the goal using the LATEST available transform, not the one
        # that was current when the goal arrived.  `Buffer.transform` looks up
        # at `header.stamp`, so passing the stored goal through unchanged pins
        # the lookup to the goal's own timestamp: the robot's motion after that
        # instant becomes invisible, the range and heading error freeze at
        # their values at goal time, and the loop runs open-loop until the goal
        # stamp ages out of the buffer and every tick fails with NO_TRANSFORM.
        # A zero stamp is tf2's "latest available", which is what a control
        # loop wants for a goal that is static in its own frame.
        try:
            latest = self._tf_buffer.lookup_transform(
                self._robot_frame,
                self._goal.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
            # tf2 returns a zero stamp only when every link in the chain is
            # static (or source == target). Such a chain cannot go stale, so
            # the age limit applies to dynamic chains only.
            stamp_s = latest.header.stamp.sec + latest.header.stamp.nanosec / 1e9
            age = now - stamp_s
            if stamp_s > 0.0 and age > self._max_tf_age:
                raise tf2_ros.ExtrapolationException(
                    f"latest {self._goal.header.frame_id} -> {self._robot_frame} "
                    f"transform is {age:.2f}s old (limit {self._max_tf_age:.2f}s)"
                )
            goal_now = PoseStamped()
            goal_now.header.frame_id = self._goal.header.frame_id
            goal_now.header.stamp = latest.header.stamp
            goal_now.pose = self._goal.pose
            goal_in_robot = tf2_geometry_msgs.do_transform_pose_stamped(goal_now, latest)
        except (tf2_ros.TransformException, Exception) as exc:  # noqa: BLE001
            # A TF failure must produce a stop, not a guess and not silence.
            # Publishing an explicit zero (rather than publishing nothing) also
            # keeps the arbiter's candidate stream alive, so the operator sees
            # "controller says stop" rather than "controller died".
            self.get_logger().warn(
                f"TF {self._goal.header.frame_id} -> {self._robot_frame} failed: {exc}",
                throttle_duration_sec=2.0,
            )
            self._status = ApproachStatus.NO_TRANSFORM
            self._publish(0.0, 0.0, 0.0)
            self._publish_status()
            return

        command = compute_approach(
            goal_in_robot.pose.position.x,
            goal_in_robot.pose.position.y,
            self._gains,
        )
        self._status = command.status
        self._publish(command.vx, command.vy, command.wz)
        self._publish_status(command)

        if command.status == ApproachStatus.REACHED and self._goal is not None:
            self.get_logger().info(f"Goal reached (range {command.range_m:.2f} m)")

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._robot_frame
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _publish_status(self, command=None) -> None:
        msg = String()
        if command is None:
            msg.data = self._status
        else:
            msg.data = (
                f"{command.status} range={command.range_m:.2f}m "
                f"heading_err={math.degrees(command.heading_error_rad):.1f}deg"
            )
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            for _ in range(3):
                self._publish(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    """
    Entry point.

    ``ExternalShutdownException`` is handled explicitly. Without it, a SIGTERM
    during ``spin`` (which is how launch stops a node, and how Ctrl-C reaches
    one) invalidates the rcl context underneath the executor and raises
    ``RCLError: failed to initialize wait set`` from inside ``spin``, before
    the ``finally`` block can run ``destroy_node``. For the controller, that
    would mean the final zero candidates never being published.

    Catching it here means the teardown path runs on every ordinary stop.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = ApproachControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # Which exception surfaces on SIGTERM is a race between the signal
        # handler invalidating the rcl context and `spin` building its next
        # wait set. Losing that race raises RCLError("failed to initialize
        # wait set") instead of ExternalShutdownException, and the difference
        # is load-dependent: it passed in isolation and failed under a full
        # test run. Treat any exception raised AFTER the context is already
        # down as the ordinary shutdown it is, and re-raise anything else so a
        # genuine fault is still loud.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
