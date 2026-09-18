#!/usr/bin/env python3
"""
SafetyArbiterNode — the single node with final authority over GO2 motion.

Contract
--------
Inputs
  /cmd_vel_candidate            geometry_msgs/TwistStamped   (canonical)
  /cmd_vel_candidate_unstamped  geometry_msgs/Twist          (compatibility)
  /go2/safety_alert             go2_msgs/SafetyAlert
  /go2/safety_state             std_msgs/String
  /go2/estop                    std_msgs/Bool                (engage only)
  ~/release_estop               std_srvs/Trigger             (release only)

Output
  /cmd_vel_safe                 go2_msgs/SafeVelocityCommand
  /safety/status                go2_msgs/SafetyStatus
  /diagnostics                  diagnostic_msgs/DiagnosticArray

Why the output type is not Twist
--------------------------------
``SafeVelocityCommand`` is a custom type.  ROS 2 will not connect a
``geometry_msgs/Twist`` publisher to a ``SafeVelocityCommand`` subscriber, so
a controller, a teleop node or an ad-hoc ``ros2 topic pub`` physically cannot
deliver a message to the hardware bridge.  The type system, not developer
discipline, enforces Invariant A.

Timer-driven, not callback-driven
---------------------------------
The decision runs on a fixed-rate timer, never in the candidate callback.
That is what makes the watchdog work: when candidates stop arriving the timer
keeps firing, observes no fresh candidate, and publishes a stop.  A purely
callback-driven design would simply go quiet, and going quiet is not a stop.
"""
from __future__ import annotations

import math
import uuid
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from go2_msgs.msg import SafetyAlert, SafetyStatus, SafeVelocityCommand
from go2_safety_arbiter.core import (
    HAZARD_CLEAR_TYPES,
    HAZARD_SLOWDOWN_TYPES,
    HAZARD_STOP_TYPES,
    Candidate,
    SafetyArbiterCore,
    SafetyContext,
    Velocity,
)
from go2_safety_arbiter.limits import (
    LimitConfigError,
    RequirementPolicy,
    TimingPolicy,
    VelocityLimits,
)
from go2_safety_arbiter.reasons import Reason, SafetyState


#: Control commands are reliable and shallow: we always want the newest one
#: and we never want the middleware to replay a backlog of stale velocities.
def _hazard_severity(hazard: str) -> int:
    """Rank hazard types so the most restrictive channel wins a disagreement."""
    if hazard in HAZARD_STOP_TYPES:
        return 3
    if hazard in HAZARD_SLOWDOWN_TYPES:
        return 2
    if hazard in HAZARD_CLEAR_TYPES:
        return 0
    # Unrecognised. Ranked ABOVE clear, so an unknown alert type can never be
    # outvoted by a "CLEAR" on the other channel. The core turns it into a stop.
    return 1


CONTROL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)

#: Status topics are latched so a late-joining diagnostic consumer sees the
#: current state immediately rather than waiting for the next tick.
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class SafetyArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_arbiter_node")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("control_frequency_hz", 20.0)
        self.declare_parameter("max_vx", 0.4)
        self.declare_parameter("max_vy", 0.2)
        self.declare_parameter("max_wz", 0.6)
        self.declare_parameter("max_accel_linear", 0.5)
        self.declare_parameter("max_accel_angular", 1.0)
        self.declare_parameter("degraded_scale", 0.35)
        self.declare_parameter("candidate_max_age_sec", 0.30)
        self.declare_parameter("watchdog_timeout_sec", 0.50)
        self.declare_parameter("hazard_max_age_sec", 1.00)
        self.declare_parameter("localization_max_age_sec", 2.00)
        self.declare_parameter("future_tolerance_sec", 0.05)
        self.declare_parameter("safe_command_lifetime_sec", 0.30)
        self.declare_parameter("hazard_clear_hold_sec", 0.5)
        self.declare_parameter("require_hazard_context", True)
        self.declare_parameter("require_localization", False)
        self.declare_parameter("expected_frame_id", "base_link")
        self.declare_parameter("accept_unstamped_candidate", True)

        freq = float(self.get_parameter("control_frequency_hz").value)
        if not math.isfinite(freq) or freq <= 0.0:
            raise ValueError(f"control_frequency_hz must be finite and > 0, got {freq}")
        self._period = 1.0 / freq

        try:
            limits = VelocityLimits(
                max_vx=float(self.get_parameter("max_vx").value),
                max_vy=float(self.get_parameter("max_vy").value),
                max_wz=float(self.get_parameter("max_wz").value),
                max_accel_linear=float(self.get_parameter("max_accel_linear").value),
                max_accel_angular=float(self.get_parameter("max_accel_angular").value),
                degraded_scale=float(self.get_parameter("degraded_scale").value),
            )
            timing = TimingPolicy(
                candidate_max_age_sec=float(self.get_parameter("candidate_max_age_sec").value),
                watchdog_timeout_sec=float(self.get_parameter("watchdog_timeout_sec").value),
                hazard_max_age_sec=float(self.get_parameter("hazard_max_age_sec").value),
                localization_max_age_sec=float(
                    self.get_parameter("localization_max_age_sec").value
                ),
                future_tolerance_sec=float(self.get_parameter("future_tolerance_sec").value),
                safe_command_lifetime_sec=float(
                    self.get_parameter("safe_command_lifetime_sec").value
                ),
            )
        except LimitConfigError as exc:
            # An unusable safety configuration must prevent the node from
            # starting. Starting with silently-corrected limits would be the
            # worst possible outcome.
            self.get_logger().fatal(f"Refusing to start with invalid safety config: {exc}")
            raise

        self.declare_parameter("require_initial_inputs", True)
        requirements = RequirementPolicy(
            require_hazard_context=bool(self.get_parameter("require_hazard_context").value),
            require_localization=bool(self.get_parameter("require_localization").value),
            require_initial_inputs=bool(self.get_parameter("require_initial_inputs").value),
        )
        self._hazard_clear_hold = float(
            self.get_parameter("hazard_clear_hold_sec").value
        )
        if not math.isfinite(self._hazard_clear_hold) or not (
            0.0 <= self._hazard_clear_hold <= 5.0
        ):
            raise ValueError(
                "hazard_clear_hold_sec must be finite and in [0, 5], got "
                f"{self._hazard_clear_hold}"
            )
        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._accept_unstamped = bool(self.get_parameter("accept_unstamped_candidate").value)

        self._timing = timing
        self._core = SafetyArbiterCore(limits, timing, requirements, self._period)
        self._requirements = requirements

        # Every freshness and watchdog decision uses a STEADY clock, never the
        # node clock.
        #
        # With use_sim_time enabled, the node clock comes from /clock. If /clock
        # stops — a paused simulator, a crashed clock publisher, a bag that ran
        # out — then `now` stops advancing, every age computes as zero, and the
        # watchdog can never fire. It is not that the watchdog fails to notice
        # the fault; it is that the watchdog's own notion of elapsed time is
        # the thing that broke. A robot moving at the moment the clock stalls
        # keeps moving.
        #
        # Steady time is monotonic, independent of /clock and of wall-clock
        # adjustments, so "0.5 s have passed" means it regardless of what the
        # rest of the graph believes about time.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        if self.get_parameter_or("use_sim_time", None) is not None and bool(
            self.get_parameter("use_sim_time").value
        ):
            self.get_logger().warn(
                "use_sim_time is true. Safety timing uses a STEADY clock "
                "regardless, so watchdogs remain live if /clock stalls. Message "
                "timestamps are still compared against simulated time."
            )

        # ── Runtime state ─────────────────────────────────────────────
        self._authority_token = uuid.uuid4().hex
        self._sequence = 0
        self._candidate: Optional[Candidate] = None
        self._candidate_unstamped_used = False
        # The two hazard channels are tracked SEPARATELY and resolved by
        # most-restrictive-wins.
        #
        # They used to share one variable, so whichever message arrived last
        # decided the hazard state. A `std_msgs/String` reading "CLEAR" on
        # /go2/safety_state would therefore cancel an EMERGENCY_STOP that the
        # depth pipeline was asserting on /go2/safety_alert — a lower-evidence
        # channel silently overriding a higher-evidence one purely by timing.
        self._alert_type: Optional[str] = None
        self._alert_stamp: Optional[float] = None
        self._state_type: Optional[str] = None
        self._state_stamp: Optional[float] = None
        #: Stop-class hazards latch, and clear only after they have been absent
        #: from BOTH channels for hazard_clear_hold_sec. Without the hold, a
        #: single dropped or flapping alert frame re-permits motion.
        self._hazard_latched_until: Optional[float] = None
        #: Inputs observed at least once since startup (Invariant B, the
        #: "has this ever arrived?" question).
        self._seen_hazard_input = False
        self._seen_localization_input = False
        self._localization_ok = False
        self._localization_stamp: Optional[float] = None
        self._estop_topic_engaged = False
        self._last_decision = None

        # ── Interfaces ────────────────────────────────────────────────
        self.create_subscription(
            TwistStamped, "cmd_vel_candidate", self._candidate_cb, CONTROL_QOS
        )
        if self._accept_unstamped:
            self.create_subscription(
                Twist, "cmd_vel_candidate_unstamped", self._candidate_unstamped_cb, CONTROL_QOS
            )
        self.create_subscription(SafetyAlert, "safety_alert", self._alert_cb, 10)
        self.create_subscription(String, "safety_state", self._safety_state_cb, 10)
        self.create_subscription(Bool, "estop", self._estop_cb, 10)
        self.create_subscription(Bool, "localization_valid", self._localization_cb, 10)

        self._safe_pub = self.create_publisher(
            SafeVelocityCommand, "cmd_vel_safe", CONTROL_QOS
        )
        self._status_pub = self.create_publisher(SafetyStatus, "safety/status", STATUS_QOS)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        # Release is a service, never a topic: releasing an emergency stop
        # must be an addressed, acknowledged request, not a fire-and-forget
        # message that any process can broadcast.
        self.create_service(Trigger, "~/release_estop", self._release_estop_cb)
        self.create_service(Trigger, "~/engage_estop", self._engage_estop_cb)

        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"SafetyArbiterNode active. token={self._authority_token[:8]} "
            f"rate={freq:.1f}Hz limits=(vx={limits.max_vx} vy={limits.max_vy} "
            f"wz={limits.max_wz}) require_hazard={requirements.require_hazard_context} "
            f"require_localization={requirements.require_localization}"
        )
        if self._accept_unstamped:
            self.get_logger().warn(
                "accept_unstamped_candidate=true: commands on "
                "cmd_vel_candidate_unstamped are stamped at RECEIPT, so "
                "candidate age excludes producer-side latency. Reduced assurance."
            )

    # ── Callbacks ─────────────────────────────────────────────────────

    def _now(self) -> float:
        """
        Message-time reference, used to interpret incoming header stamps.

        Producers stamp with their node clock, so ages must be computed in the
        same base. See :meth:`_steady_now` for the base used by the watchdogs.
        """
        return self.get_clock().now().nanoseconds / 1e9

    def _steady_now(self) -> float:
        """
        Monotonic reference for watchdog and freshness decisions.

        Never derived from /clock, so a stalled simulation clock cannot freeze
        a watchdog. See the constructor for why this matters.
        """
        return self._steady_clock.now().nanoseconds / 1e9

    def _candidate_cb(self, msg: TwistStamped) -> None:
        # Header stamps come from the producer's node clock, while the watchdog
        # runs on steady time. Convert once, here, by measuring how old the
        # message is in the producer's own base and re-expressing that age on
        # the steady timeline. A stamp is then comparable with _steady_now()
        # without inheriting /clock's failure modes.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp <= 0.0:
            # An unset stamp is a defect in the producer. Do not silently
            # substitute "now" — that would convert a bug into permission.
            self.get_logger().warn(
                "Candidate with zero timestamp rejected at intake", throttle_duration_sec=2.0
            )
            self._candidate = Candidate(
                Velocity(0.0, 0.0, 0.0), 0.0, msg.header.frame_id
            )
            return
        message_age = self._now() - stamp
        self._candidate = Candidate(
            velocity=Velocity(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z),
            stamp=self._steady_now() - message_age,
            frame_id=msg.header.frame_id or self._expected_frame,
        )
        self._candidate_unstamped_used = False

    def _candidate_unstamped_cb(self, msg: Twist) -> None:
        self._candidate = Candidate(
            velocity=Velocity(msg.linear.x, msg.linear.y, msg.angular.z),
            stamp=self._steady_now(),
            frame_id=self._expected_frame,
        )
        self._candidate_unstamped_used = True

    def _alert_cb(self, msg: SafetyAlert) -> None:
        self._alert_type = msg.alert_type
        self._alert_stamp = self._steady_now()
        self._seen_hazard_input = True

    def _safety_state_cb(self, msg: String) -> None:
        # safety_monitor_node publishes "CLEAR" when nothing is wrong. Without
        # this the arbiter would treat a clear scene as missing hazard context
        # and refuse to move, because SafetyAlert is only published on alerts.
        self._state_type = msg.data
        self._state_stamp = self._steady_now()
        self._seen_hazard_input = True

    def _resolve_hazard(self, now: float):
        """
        Combine the two hazard channels, most restrictive wins.

        Returns ``(hazard_type, stamp)`` for the core, or ``(None, None)`` when
        no fresh information exists on either channel — which the core treats
        as a stop.
        """
        candidates = []
        for hazard, stamp in (
            (self._alert_type, self._alert_stamp),
            (self._state_type, self._state_stamp),
        ):
            if hazard is None or stamp is None:
                continue
            if (now - stamp) > self._timing.hazard_max_age_sec:
                continue
            candidates.append((hazard.strip().upper(), stamp))

        if not candidates:
            return None, None

        # Latch stop-class hazards for a hold period, so a single dropped or
        # flapping frame cannot re-permit motion.
        for hazard, _stamp in candidates:
            if hazard in HAZARD_STOP_TYPES:
                self._hazard_latched_until = now + self._hazard_clear_hold
        if self._hazard_latched_until is not None:
            if now < self._hazard_latched_until:
                return "EMERGENCY_STOP", now
            self._hazard_latched_until = None

        severity = {h: _hazard_severity(h) for h, _ in candidates}
        worst = max(candidates, key=lambda c: severity[c[0]])
        return worst[0], worst[1]

    def _estop_cb(self, msg: Bool) -> None:
        # A topic may only ENGAGE the e-stop, never release it. Releasing
        # requires the service. This asymmetry is deliberate.
        if msg.data:
            self._estop_topic_engaged = True
            self._core.engage_estop()
            self.get_logger().warn("EMERGENCY STOP engaged via /go2/estop")

    def _localization_cb(self, msg: Bool) -> None:
        self._localization_ok = bool(msg.data)
        self._localization_stamp = self._steady_now()
        self._seen_localization_input = True

    def _engage_estop_cb(self, _req, resp):
        self._core.engage_estop()
        self.get_logger().warn("EMERGENCY STOP engaged via service")
        resp.success = True
        resp.message = "emergency stop engaged"
        return resp

    def _release_estop_cb(self, _req, resp):
        self._estop_topic_engaged = False
        self._core.release_estop()
        self.get_logger().warn("Emergency stop RELEASED via service")
        resp.success = True
        resp.message = "emergency stop released"
        return resp

    # ── Control loop ──────────────────────────────────────────────────

    def _tick(self) -> None:
        """
        One control decision.

        Wrapped so that an unexpected exception anywhere in the decision or
        publish path still results in a published STOP. ``core.evaluate``
        already guards individual safety rules, but that guarantee was worth
        nothing if the surrounding tick raised: the process died and took the
        stop command with it, leaving the bridge to discover the loss only via
        its own watchdog, and leaving a restarted arbiter unable to reclaim
        authority. An exception here means the arbiter is broken, and a broken
        arbiter must say "stop" rather than say nothing.
        """
        try:
            self._tick_inner()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Safety tick failed: {exc}. Publishing STOP.",
                throttle_duration_sec=1.0,
            )
            try:
                self._publish_stop(
                    [Reason.RULE_EVALUATION_FAILED], self._steady_now()
                )
            except Exception:  # noqa: BLE001 — nothing further can be done here
                pass

    def _tick_inner(self) -> None:
        now = self._steady_now()

        candidate = self._candidate
        # The watchdog lives here rather than in the core so that the core sees
        # a clean "no candidate" input. Anything older than the watchdog window
        # is discarded outright and forgotten, so it can never be re-evaluated
        # on a later tick.
        if candidate is not None and (now - candidate.stamp) > self._timing.watchdog_timeout_sec:
            self._candidate = None
            candidate = None

        # "Has this ever arrived?", asked separately from "is it recent?".
        # A node that has never heard from the safety monitor is in a different
        # situation from one whose last hazard message aged out, and only the
        # second is a transient worth describing as staleness.
        if self._requirements.require_initial_inputs:
            missing = []
            if self._requirements.require_hazard_context and not self._seen_hazard_input:
                missing.append("hazard")
            if self._requirements.require_localization and not self._seen_localization_input:
                missing.append("localization")
            if missing:
                self.get_logger().warn(
                    f"Not initialized: never received {', '.join(missing)}. "
                    "Commanding stop.",
                    throttle_duration_sec=5.0,
                )
                self._publish_stop([Reason.NOT_INITIALIZED], now)
                return

        hazard_type, hazard_stamp = self._resolve_hazard(now)

        context = SafetyContext(
            hazard_type=hazard_type,
            hazard_stamp=hazard_stamp,
            localization_ok=self._localization_ok,
            localization_stamp=self._localization_stamp,
            estop_engaged=self._estop_topic_engaged,
            expected_frame_id=self._expected_frame,
        )

        decision = self._core.evaluate(candidate, context, now)
        self._last_decision = decision
        self._publish_safe(decision, now)
        self._publish_status(decision, now, candidate)
        self._publish_diagnostics(decision, now)

    def _publish_stop(self, reasons, now: float) -> None:
        """Publish an unconditional zero-velocity command with the given reasons."""
        from go2_safety_arbiter.core import Decision

        decision = Decision(
            velocity=Velocity(0.0, 0.0, 0.0),
            state=SafetyState.STOPPED,
            reason_codes=tuple(reasons),
            intervened=True,
            candidate_age_sec=-1.0,
        )
        self._last_decision = decision
        self._publish_safe(decision, now)
        self._publish_status(decision, now, None)
        self._publish_diagnostics(decision, now)

    def _publish_safe(self, decision, now: float) -> None:
        """
        Publish an authorized command.

        ``now`` is steady time, used only for bookkeeping. The wire fields
        ``header.stamp`` and ``valid_until`` are both derived from the NODE
        clock, because that is the base the bridge compares them against.
        Mixing the two bases here would make every command look expired or
        eternally valid, depending on the offset between the clocks.
        """
        self._sequence += 1
        msg = SafeVelocityCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._expected_frame
        msg.twist.linear.x = float(decision.velocity.vx)
        msg.twist.linear.y = float(decision.velocity.vy)
        msg.twist.angular.z = float(decision.velocity.wz)
        msg.arbiter_state = decision.state
        msg.reason_codes = list(decision.reason_codes)
        msg.sequence = self._sequence
        msg.authority_token = self._authority_token

        expiry = self._now() + self._timing.safe_command_lifetime_sec
        # safe_command_lifetime_sec is ceiling-bounded in limits.py, so this
        # cannot overflow the int32 that builtin_interfaces/Time.sec requires.
        # It previously could, and the resulting assertion killed the arbiter
        # from inside the publish path, taking the stop command with it.
        msg.valid_until.sec = int(expiry)
        msg.valid_until.nanosec = int((expiry - int(expiry)) * 1e9)
        self._safe_pub.publish(msg)

    def _publish_status(self, decision, now: float, candidate) -> None:
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._expected_frame
        msg.state = decision.state
        msg.reason_codes = list(decision.reason_codes)
        msg.candidate_age_sec = float(decision.candidate_age_sec)
        msg.last_safe_command.linear.x = float(decision.velocity.vx)
        msg.last_safe_command.linear.y = float(decision.velocity.vy)
        msg.last_safe_command.angular.z = float(decision.velocity.wz)
        if candidate is not None:
            msg.last_candidate_command.linear.x = float(candidate.velocity.vx)
            msg.last_candidate_command.linear.y = float(candidate.velocity.vy)
            msg.last_candidate_command.angular.z = float(candidate.velocity.wz)
        msg.intervention_count = self._core.intervention_count
        msg.decision_count = self._core.decision_count
        msg.last_intervention_reasons = list(self._core.last_intervention_reasons)
        msg.estop_engaged = self._core.estop_latched
        hazard_type, hazard_stamp = self._resolve_hazard(now)
        msg.hazard_context_valid = hazard_stamp is not None
        msg.localization_valid = (
            self._localization_ok
            and self._localization_stamp is not None
            and (now - self._localization_stamp) <= self._timing.localization_max_age_sec
        )
        self._status_pub.publish(msg)

    def _publish_diagnostics(self, decision, now: float) -> None:
        status = DiagnosticStatus()
        status.name = "go2_safety_arbiter: motion authority"
        status.hardware_id = self._authority_token[:8]
        if decision.state == SafetyState.SAFE_TO_MOVE:
            status.level = DiagnosticStatus.OK
        elif decision.state == SafetyState.DEGRADED:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.ERROR
        status.message = f"{decision.state} " + (
            ",".join(decision.reason_codes) if decision.reason_codes else "nominal"
        )
        status.values = [
            KeyValue(key="state", value=decision.state),
            KeyValue(key="reason_codes", value=",".join(decision.reason_codes)),
            KeyValue(key="candidate_age_sec", value=f"{decision.candidate_age_sec:.3f}"),
            KeyValue(key="safe_vx", value=f"{decision.velocity.vx:.3f}"),
            KeyValue(key="safe_vy", value=f"{decision.velocity.vy:.3f}"),
            KeyValue(key="safe_wz", value=f"{decision.velocity.wz:.3f}"),
            KeyValue(key="intervention_count", value=str(self._core.intervention_count)),
            KeyValue(key="decision_count", value=str(self._core.decision_count)),
            KeyValue(
                key="last_intervention_reasons",
                value=",".join(self._core.last_intervention_reasons),
            ),
            KeyValue(key="estop_engaged", value=str(self._core.estop_latched)),
            KeyValue(key="authority_token", value=self._authority_token),
            KeyValue(key="unstamped_candidate_in_use", value=str(self._candidate_unstamped_used)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    # ── Shutdown ──────────────────────────────────────────────────────

    def emit_shutdown_stop(self, repeats: int = 3) -> None:
        """
        Publish explicit zero-velocity STOPPED commands before dying.

        Repeated because the safe topic is depth-1 volatile: a single message
        can be lost if the bridge's executor is momentarily busy, and the cost
        of a redundant stop is nothing.
        """
        from go2_safety_arbiter.core import Decision

        stop = Decision(
            velocity=Velocity(0.0, 0.0, 0.0),
            state=SafetyState.STOPPED,
            reason_codes=(Reason.SHUTDOWN,),
            intervened=True,
            candidate_age_sec=-1.0,
        )
        for _ in range(repeats):
            self._publish_safe(stop, self._steady_now())

    def destroy_node(self) -> bool:
        try:
            self.emit_shutdown_stop()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to emit shutdown stop: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    """
    Entry point.

    ``ExternalShutdownException`` is handled explicitly. Without it, a SIGTERM
    during ``spin`` (which is how launch stops a node, and how Ctrl-C reaches
    one) invalidates the rcl context underneath the executor and raises
    ``RCLError: failed to initialize wait set`` from inside ``spin`` — before
    the ``finally`` block can run ``destroy_node``. For the arbiter, that
    would mean the explicit zero-velocity STOPPED commands never being published, so
    the bridge learns of the stop only when its watchdog expires.

    Catching it here means the teardown path runs on every ordinary stop.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyArbiterNode()
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
