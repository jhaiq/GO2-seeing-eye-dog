#!/usr/bin/env python3
"""
NavToPoseAdapterNode: a nav2_msgs/NavigateToPose server over the staged
approach controller.

Clients written against Nav2 (the semantic grounding node in particular) send
a NavigateToPose goal and treat SUCCEEDED as "the robot arrived". This node
lets them drive ``approach_controller_node`` through exactly that contract::

    NavigateToPose goal  ->  goal_pose (PoseStamped)  ->  approach controller
    controller/status    ->  succeed / abort / feedback
    cancel               ->  cancel_goal (String), confirmed by IDLE

It adds NO motion capability. The controller still publishes only a candidate
velocity, which the SafetyArbiter owns. The adapter is not a planner either:
the controller drives in a straight line and stops for hazards, it does not
route around obstacles.

Status contract (``controller/status``, std_msgs/String, published every
control tick, TRANSIENT_LOCAL depth 1):

* ``IDLE``                          no goal
* ``ACTIVE range=R m heading_err=..``  approaching
* ``REACHED range=R m ...``         within tolerance, holding
* ``NO_TRANSFORM``                  goal cannot be expressed in the robot frame
* ``GOAL_STALE``                    goal exceeded goal_lifetime_sec, dropped

The status carries no goal identity, so a latched REACHED or IDLE from the
PREVIOUS goal can arrive right after a new goal is sent. Statuses received
within ``status_settle_s`` of sending a goal are therefore ignored, and a
REACHED is only trusted once the settle window has passed.
"""
from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import String

from go2_approach_controller.approach import ApproachStatus

STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

_RANGE_RE = re.compile(r"range=([-+0-9.eEnaif]+)m")

SUCCEEDED = "SUCCEEDED"
ABORTED = "ABORTED"
CANCELED = "CANCELED"


def parse_status(text: str) -> tuple[str, Optional[float]]:
    """Split a controller status string into (state, range_m or None)."""
    state = text.split(" ", 1)[0].strip() if text else ""
    match = _RANGE_RE.search(text or "")
    range_m = None
    if match:
        try:
            value = float(match.group(1))
            range_m = value if math.isfinite(value) else None
        except ValueError:
            range_m = None
    return state, range_m


@dataclass
class _Tracked:
    """Bookkeeping for the goal currently driving the controller."""

    handle: object
    pose: PoseStamped
    done: Future
    sent_at: float
    seen_active: bool = False
    no_transform_since: Optional[float] = None
    cancel_sent_at: Optional[float] = None
    last_range: Optional[float] = None
    outcome: Optional[str] = None
    message: str = ""
    extra: dict = field(default_factory=dict)


class NavToPoseAdapterNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("nav_to_pose_adapter_node", **kwargs)

        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("monitor_rate_hz", 20.0)
        self.declare_parameter("status_timeout_s", 1.0)
        self.declare_parameter("status_settle_s", 0.2)
        self.declare_parameter("no_transform_timeout_s", 1.0)
        self.declare_parameter("goal_ack_timeout_s", 1.0)
        self.declare_parameter("cancel_confirm_timeout_s", 1.0)
        self.declare_parameter("feedback_period_s", 0.2)
        # The approach controller stops goal_tolerance_m SHORT of any goal (a
        # social distance for "come here"). A NavigateToPose caller asks for a
        # pose, so the forwarded goal is pushed that far further along the
        # approach line and the robot stops at the requested pose.
        self.declare_parameter("compensate_controller_tolerance", True)
        self.declare_parameter("controller_goal_tolerance_m", 0.8)

        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._status_timeout = float(self.get_parameter("status_timeout_s").value)
        self._settle = float(self.get_parameter("status_settle_s").value)
        self._no_tf_timeout = float(self.get_parameter("no_transform_timeout_s").value)
        self._ack_timeout = float(self.get_parameter("goal_ack_timeout_s").value)
        self._cancel_timeout = float(self.get_parameter("cancel_confirm_timeout_s").value)
        self._feedback_period = float(self.get_parameter("feedback_period_s").value)
        rate = float(self.get_parameter("monitor_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError(f"monitor_rate_hz must be finite and > 0, got {rate}")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self._cancel_pub = self.create_publisher(String, "cancel_goal", 10)
        self.create_subscription(String, "controller/status", self._status_cb, STATUS_QOS)

        self._status_state: Optional[str] = None
        self._status_range: Optional[float] = None
        self._status_at: Optional[float] = None
        self._active: Optional[_Tracked] = None
        self._last_feedback = 0.0

        self._server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
        )
        self._timer = self.create_timer(1.0 / rate, self._monitor)
        self.get_logger().info(
            "NavigateToPose adapter over the STAGED approach controller: "
            "straight-line approach, no planning, no obstacle avoidance."
        )

    # ── helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _mono() -> float:
        return time.monotonic()

    def _status_cb(self, msg: String) -> None:
        state, range_m = parse_status(msg.data)
        self._status_state = state
        self._status_range = range_m
        self._status_at = self._mono()

    def _controller_present(self) -> bool:
        return self._goal_pub.get_subscription_count() > 0

    def _tf_distance(self, pose: PoseStamped) -> Optional[float]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._robot_frame, pose.header.frame_id, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        import tf2_geometry_msgs  # noqa: F401, PLC0415 registers PoseStamped support

        probe = PoseStamped()
        probe.header.frame_id = pose.header.frame_id
        probe.header.stamp = tf.header.stamp
        probe.pose = pose.pose
        goal = tf2_geometry_msgs.do_transform_pose_stamped(probe, tf)
        return math.hypot(goal.pose.position.x, goal.pose.position.y)

    def _current_pose(self, frame: str) -> Optional[PoseStamped]:
        try:
            tf = self._tf_buffer.lookup_transform(
                frame, self._robot_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = tf.header.stamp
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _compensated(self, pose: PoseStamped) -> PoseStamped:
        """Extend the goal along robot->goal by the controller's stop tolerance."""
        if not bool(self.get_parameter("compensate_controller_tolerance").value):
            return pose
        tol = float(self.get_parameter("controller_goal_tolerance_m").value)
        robot = self._current_pose(pose.header.frame_id)
        if robot is None or tol <= 0.0:
            self.get_logger().warn(
                "no robot pose in goal frame; forwarding goal uncompensated "
                f"(robot will stop {tol:.2f} m short)"
            )
            return pose
        dx = pose.pose.position.x - robot.pose.position.x
        dy = pose.pose.position.y - robot.pose.position.y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return pose
        out = PoseStamped()
        out.header = pose.header
        out.pose.orientation = pose.pose.orientation
        out.pose.position.x = pose.pose.position.x + tol * dx / dist
        out.pose.position.y = pose.pose.position.y + tol * dy / dist
        out.pose.position.z = pose.pose.position.z
        return out

    def _finish(self, tracked: _Tracked, outcome: str, message: str) -> None:
        if tracked.done.done():
            return
        tracked.outcome = outcome
        tracked.message = message
        if self._active is tracked:
            self._active = None
        tracked.done.set_result(outcome)

    async def _sleep(self, seconds: float) -> None:
        """Yield to the executor for ``seconds`` without blocking a thread."""
        waiter = Future()
        timer = self.create_timer(seconds, lambda: waiter.done() or waiter.set_result(True))
        try:
            await waiter
        finally:
            self.destroy_timer(timer)

    def _send_cancel(self) -> None:
        msg = String()
        msg.data = "cancel"
        self._cancel_pub.publish(msg)

    # ── action callbacks ──────────────────────────────────────────────

    def _on_goal(self, request) -> GoalResponse:
        if not request.pose.header.frame_id:
            self.get_logger().warn("goal rejected: empty frame_id")
            return GoalResponse.REJECT
        p = request.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.get_logger().warn("goal rejected: non-finite position")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle) -> None:
        # A new goal preempts the one in progress. The old goal is aborted,
        # NOT canceled on the controller: the new goal_pose replaces it, and a
        # cancel_goal here would race with and wipe out the new goal.
        previous = self._active
        if previous is not None:
            self._finish(previous, ABORTED, "preempted by a newer goal")
        goal_handle.execute()

    async def _execute(self, goal_handle):
        request = goal_handle.request
        tracked = _Tracked(
            handle=goal_handle,
            pose=request.pose,
            done=Future(),
            sent_at=self._mono(),
        )

        if not self._controller_present():
            deadline = self._mono() + self._status_timeout
            while not self._controller_present() and self._mono() < deadline:
                await self._sleep(0.02)
        if not self._controller_present():
            self._finish(tracked, ABORTED, "approach controller not running (no goal_pose subscriber)")
        else:
            self._active = tracked
            tracked.sent_at = self._mono()
            self._goal_pub.publish(self._compensated(request.pose))
            self.get_logger().info(
                f"goal forwarded in '{request.pose.header.frame_id}': "
                f"({request.pose.pose.position.x:.2f}, {request.pose.pose.position.y:.2f})"
            )
            await tracked.done

        result = NavigateToPose.Result()
        if tracked.outcome == SUCCEEDED:
            goal_handle.succeed()
        elif tracked.outcome == CANCELED:
            goal_handle.canceled()
        else:
            self.get_logger().warn(f"goal aborted: {tracked.message}")
            goal_handle.abort()
        return result

    # ── monitor ───────────────────────────────────────────────────────

    def _monitor(self) -> None:
        tracked = self._active
        if tracked is None:
            return
        now = self._mono()
        handle = tracked.handle

        # Cancel path.
        if handle.is_cancel_requested:
            if tracked.cancel_sent_at is None:
                tracked.cancel_sent_at = now
                self._send_cancel()
                return
            confirmed = (
                self._status_state == ApproachStatus.IDLE
                and self._status_at is not None
                and self._status_at > tracked.cancel_sent_at
            )
            if confirmed:
                self._finish(tracked, CANCELED, "controller confirmed IDLE")
            elif now - tracked.cancel_sent_at > self._cancel_timeout:
                self.get_logger().error(
                    "cancel NOT confirmed: controller did not report IDLE within "
                    f"{self._cancel_timeout:.1f}s; the robot may still be commanded"
                )
                self._finish(tracked, CANCELED, "cancel unconfirmed")
            return

        # Controller liveness. Measured from the later of goal send and last
        # status, so a status latched long ago does not count as alive.
        last_seen = max(tracked.sent_at, self._status_at or 0.0)
        if now - last_seen > self._status_timeout:
            self._finish(tracked, ABORTED, "no controller status (controller dead?)")
            return

        fresh = self._status_at is not None and self._status_at > tracked.sent_at + self._settle
        if fresh:
            state = self._status_state
            if self._status_range is not None:
                tracked.last_range = self._status_range
            if state == ApproachStatus.ACTIVE:
                tracked.seen_active = True
                tracked.no_transform_since = None
            elif state == ApproachStatus.REACHED:
                self._finish(tracked, SUCCEEDED, "controller reported REACHED")
                return
            elif state == ApproachStatus.GOAL_STALE:
                self._finish(tracked, ABORTED, "controller dropped the goal (GOAL_STALE)")
                return
            elif state == ApproachStatus.NO_TRANSFORM:
                if tracked.no_transform_since is None:
                    tracked.no_transform_since = now
                elif now - tracked.no_transform_since > self._no_tf_timeout:
                    self._finish(tracked, ABORTED, "goal frame not transformable (NO_TRANSFORM)")
                    return
            elif state == ApproachStatus.IDLE:
                if tracked.seen_active:
                    self._finish(tracked, ABORTED, "controller went IDLE before REACHED")
                    return
                if now - tracked.sent_at > self._ack_timeout:
                    self._finish(tracked, ABORTED, "controller never accepted the goal")
                    return
        elif now - tracked.sent_at > self._ack_timeout + self._settle:
            self._finish(tracked, ABORTED, "no status after goal was sent")
            return

        if now - self._last_feedback >= self._feedback_period:
            self._last_feedback = now
            self._publish_feedback(tracked, now)

    def _publish_feedback(self, tracked: _Tracked, now: float) -> None:
        feedback = NavigateToPose.Feedback()
        distance = tracked.last_range
        if distance is None:
            distance = self._tf_distance(tracked.pose)
        feedback.distance_remaining = float(distance) if distance is not None else float("nan")
        feedback.navigation_time = Duration(seconds=now - tracked.sent_at).to_msg()
        current = self._current_pose(tracked.pose.header.frame_id)
        if current is not None:
            feedback.current_pose = current
        tracked.handle.publish_feedback(feedback)

    def destroy_node(self) -> bool:
        try:
            if self._active is not None:
                self._send_cancel()
                self._finish(self._active, ABORTED, "adapter shutting down")
            else:
                self._send_cancel()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._server.destroy()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NavToPoseAdapterNode()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
