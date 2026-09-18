#!/usr/bin/env python3
"""
IntentGroundingNode — multimodal caller confirmation and goal emission.

Chain position::

    /go2/detected_humans  ─┐
    /go2/audio/bearing_deg ├─> [fusion] ─> [state machine] ─> /goal_pose
    /go2/voice_command    ─┘                              └─> /go2/grounding_status

Three defects in the previous implementation are fixed here; each is
documented at the point of the fix and covered by a regression test:

1. A navigation goal could be published with **no voice request ever
   received** — the voice callback only reset state, it never armed anything.
   The state machine now ignores detections in ``IDLE``.
2. Visual and acoustic bearings were compared **across frames with opposite
   sign conventions**, so a caller on the robot's left was scored against a
   bearing pointing right.  Conversion is now explicit
   (``go2_intent_grounding.bearings``).
3. ``SEARCHING`` could persist forever with no output.  Status is now
   published on a timer with an explicit reason, and the request times out.

This node does not command motion.  It publishes a goal; a controller turns
that into a candidate velocity; the SafetyArbiter decides whether any of it
reaches the robot.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
import rclpy.duration
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from go2_intent_grounding.bearings import body_yaw_from_optical_position
from go2_intent_grounding.fusion import FusionParams, FusionReason, FusionResult, fuse
from go2_intent_grounding.grounding_state import (
    GroundingReason,
    GroundingState,
    GroundingStateMachine,
)
from go2_msgs.msg import ConfirmedTarget, DetectedHumanArray, GroundingStatus

STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

REQUEST_PHRASES = ("come here", "come", "here", "over here")
STOP_PHRASES = ("stop", "wait", "stay")


class IntentGroundingNode(Node):
    def __init__(self) -> None:
        super().__init__("intent_grounding_node")

        # ── Fusion parameters (see config/fusion.yaml) ────────────────
        self.declare_parameter("audio_weight", 0.4)
        self.declare_parameter("visual_weight", 0.6)
        self.declare_parameter("bearing_gate_deg", 25.0)
        self.declare_parameter("bearing_soft_deg", 25.0)
        self.declare_parameter("audio_absent_factor", 0.85)
        self.declare_parameter("min_confidence_threshold", 0.44)
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        # ── Interaction parameters ────────────────────────────────────
        self.declare_parameter("confirmation_frames", 5)
        self.declare_parameter("audio_timeout_sec", 2.0)
        self.declare_parameter("request_timeout_sec", 15.0)
        self.declare_parameter("status_rate_hz", 5.0)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.1)

        self._params = FusionParams(
            bearing_gate_deg=float(self.get_parameter("bearing_gate_deg").value),
            bearing_soft_deg=float(self.get_parameter("bearing_soft_deg").value),
            audio_weight=float(self.get_parameter("audio_weight").value),
            visual_weight=float(self.get_parameter("visual_weight").value),
            audio_absent_factor=float(self.get_parameter("audio_absent_factor").value),
            min_confidence=float(self.get_parameter("min_confidence_threshold").value),
        )
        self._camera_yaw_offset = math.radians(
            float(self.get_parameter("camera_yaw_offset_deg").value)
        )
        self._audio_timeout = float(self.get_parameter("audio_timeout_sec").value)
        self._goal_frame = str(self.get_parameter("goal_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self._machine = GroundingStateMachine(
            required_confirmations=int(self.get_parameter("confirmation_frames").value),
            request_timeout_sec=float(self.get_parameter("request_timeout_sec").value),
        )

        # ── State ─────────────────────────────────────────────────────
        self._bearing_rad: Optional[float] = None
        self._bearing_stamp: Optional[float] = None
        self._last_result: Optional[FusionResult] = None
        self._last_num_detections = 0
        self._locked_pose: Optional[PoseStamped] = None
        self._locked_bearing: Optional[float] = None
        #: (DetectedHumanArray, DetectedHuman, FusionResult) captured on the
        #: frame that achieved a lock, consumed once by _emit_confirmation.
        self._pending_target: Optional[Tuple] = None

        # ── Interfaces ────────────────────────────────────────────────
        self.create_subscription(Float32, "audio_bearing_deg", self._bearing_cb, 10)
        self.create_subscription(
            DetectedHumanArray, "detected_humans", self._humans_cb, 10
        )
        self.create_subscription(String, "voice_command", self._voice_cb, 10)

        self._target_pub = self.create_publisher(ConfirmedTarget, "confirmed_target", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        # A user saying "stop" must actually stop the robot, which means the
        # controller has to drop its goal. Without this the grounding node
        # would enter STOPPED while the controller happily kept driving at the
        # last goal it was given.
        self._cancel_pub = self.create_publisher(String, "cancel_goal", 10)
        self._status_pub = self.create_publisher(
            GroundingStatus, "grounding_status", STATUS_QOS
        )
        # Retained for backwards compatibility with existing tooling that
        # watches the old string topic. The structured topic is authoritative.
        self._legacy_state_pub = self.create_publisher(String, "grounding_state", 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        rate = float(self.get_parameter("status_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError(f"status_rate_hz must be finite and > 0, got {rate}")
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            "IntentGroundingNode ready. "
            f"gate={self._params.bearing_gate_deg}deg "
            f"threshold={self._params.min_confidence} "
            f"(visual-only needs conf >= {self._params.min_visual_for_visual_only:.3f}, "
            f"gate-edge needs conf >= {self._params.min_visual_at_gate_edge:.3f}). "
            "A voice request is REQUIRED before any goal is emitted."
        )

    # ── Callbacks ─────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _bearing_cb(self, msg: Float32) -> None:
        if not math.isfinite(msg.data):
            self.get_logger().warn("Non-finite acoustic bearing discarded")
            return
        # audio_perception_node publishes degrees in the body convention
        # (REP-103, CCW-positive). No sign change here; the conversion that
        # was missing is on the *visual* side, in bearings.py.
        self._bearing_rad = math.radians(msg.data)
        self._bearing_stamp = self._now()

    def _voice_cb(self, msg: String) -> None:
        command = msg.data.lower().strip()
        if command in REQUEST_PHRASES:
            self.get_logger().info(f"Request received: '{command}' — arming search")
            self._machine.on_request(self._now())
            self._locked_pose = None
            self._locked_bearing = None
        elif command in STOP_PHRASES:
            self.get_logger().info(f"Stop command: '{command}' — cancelling goal")
            self._machine.on_stop()
            self._locked_pose = None
            self._locked_bearing = None
            self._pending_target = None
            cancel = String()
            cancel.data = command
            self._cancel_pub.publish(cancel)

    def _audio_available(self, now: float) -> bool:
        if self._bearing_rad is None or self._bearing_stamp is None:
            return False
        return (now - self._bearing_stamp) < self._audio_timeout

    def _humans_cb(self, msg: DetectedHumanArray) -> None:
        now = self._now()
        audio_ok = self._audio_available(now)
        humans = list(msg.humans)
        self._last_num_detections = len(humans)

        best: Optional[Tuple[FusionResult, object]] = None
        for human in humans:
            body_yaw = body_yaw_from_optical_position(
                human.pose.position.x,
                human.pose.position.z,
                self._camera_yaw_offset,
            )
            result = fuse(
                visual_score=human.confidence,
                candidate_bearing_rad=body_yaw,
                acoustic_bearing_rad=self._bearing_rad if audio_ok else None,
                audio_available=audio_ok,
                params=self._params,
            )
            # Rank accepted candidates above rejected ones, then by score, so
            # the reported reason describes the *best* candidate rather than
            # an arbitrary one.
            key = (result.accepted, result.fused_score)
            if best is None or key > (best[0].accepted, best[0].fused_score):
                best = (result, human)

        if best is None:
            self._last_result = None
            self._machine.on_detection_frame(now, 0, False, GroundingReason.NO_DETECTIONS)
            return

        result, human = best
        self._last_result = result

        # A locked target that has walked outside the association gate must be
        # re-acquired rather than silently pursued.
        if self._machine.state == GroundingState.CONFIRMED and result.reason == (
            FusionReason.BEARING_MISMATCH
        ):
            self._machine.on_target_moved(now)
            return

        self._machine.on_detection_frame(
            now,
            len(humans),
            result.accepted,
            self._map_reason(result.reason),
        )

        if self._machine.state == GroundingState.CONFIRMED and result.accepted:
            self._locked_pose = self._pose_stamped(msg, human)
            self._locked_bearing = body_yaw_from_optical_position(
                human.pose.position.x, human.pose.position.z, self._camera_yaw_offset
            )
            self._pending_target = (msg, human, result)

    @staticmethod
    def _map_reason(fusion_reason: str) -> str:
        return {
            FusionReason.BEARING_MISMATCH: GroundingReason.BEARING_MISMATCH,
            FusionReason.LOW_CONFIDENCE: GroundingReason.LOW_CONFIDENCE,
            FusionReason.INVALID_DETECTION: GroundingReason.LOW_CONFIDENCE,
            FusionReason.ACCEPTED: GroundingReason.ACCUMULATING,
        }.get(fusion_reason, GroundingReason.LOW_CONFIDENCE)

    @staticmethod
    def _pose_stamped(msg: DetectedHumanArray, human) -> PoseStamped:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = human.pose
        return pose

    # ── Timer ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = self._now()
        snapshot = self._machine.tick(now)

        if snapshot.just_confirmed:
            self._emit_confirmation()

        self._publish_status(snapshot, now)

    def _emit_confirmation(self) -> None:
        pending = self._pending_target
        self._pending_target = None
        if pending is None or self._locked_pose is None:
            self.get_logger().error(
                "Confirmed with no pending target; no goal emitted. This is a bug "
                "and is reported rather than papered over."
            )
            return
        msg, human, result = pending

        confirmed = ConfirmedTarget()
        confirmed.header = msg.header
        confirmed.pose = human.pose
        confirmed.confidence = float(result.fused_score)
        confirmed.target_id = human.track_id
        self._target_pub.publish(confirmed)

        try:
            goal = self._tf_buffer.transform(
                self._locked_pose,
                self._goal_frame,
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except Exception as exc:  # noqa: BLE001 — tf2 raises several types
            # No goal is published. The status message carries the state, so a
            # feedback layer can tell the user the robot cannot localize the
            # request rather than leaving them waiting in silence.
            self.get_logger().warn(
                f"TF {self._locked_pose.header.frame_id} -> {self._goal_frame} "
                f"failed: {exc}. No goal published."
            )
            return

        self._goal_pub.publish(goal)
        self.get_logger().info(
            f"Caller confirmed (fused={result.fused_score:.3f}, "
            f"visual={result.visual_score:.3f}, audio={result.audio_score:.3f}, "
            f"delta={result.bearing_delta_deg:.1f}deg). Goal published in "
            f"'{self._goal_frame}'."
        )

    def _publish_status(self, snapshot, now: float) -> None:
        msg = GroundingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = snapshot.state
        msg.reason = snapshot.reason
        msg.consecutive_confirmations = int(snapshot.consecutive_confirmations)
        msg.required_confirmations = int(snapshot.required_confirmations)
        msg.request_time_remaining_sec = float(snapshot.request_time_remaining_sec)
        msg.num_detections = int(self._last_num_detections)
        msg.audio_available = self._audio_available(now)

        result = self._last_result
        if result is None:
            msg.visual_score = -1.0
            msg.audio_score = -1.0
            msg.fused_score = -1.0
            msg.bearing_delta_deg = float("nan")
        else:
            msg.visual_score = float(result.visual_score)
            msg.audio_score = float(result.audio_score)
            msg.fused_score = float(result.fused_score)
            msg.bearing_delta_deg = float(result.bearing_delta_deg)

        self._status_pub.publish(msg)

        legacy = String()
        legacy.data = snapshot.state
        self._legacy_state_pub.publish(legacy)


def main(args=None) -> None:
    """
    Entry point.

    ``ExternalShutdownException`` is handled explicitly. Without it, a SIGTERM
    during ``spin`` (which is how launch stops a node, and how Ctrl-C reaches
    one) invalidates the rcl context underneath the executor and raises
    ``RCLError: failed to initialize wait set`` from inside ``spin`` — before
    the ``finally`` block can run ``destroy_node``. For the grounding node, that
    would mean an unclean teardown.

    Catching it here means the teardown path runs on every ordinary stop.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = IntentGroundingNode()
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
