#!/usr/bin/env python3
"""
HardwareBridgeNode — the only process permitted to actuate the GO2.

Contract
--------
Input   /cmd_vel_safe   go2_msgs/SafeVelocityCommand   (ONLY motion input)
Output  /bridge/status  go2_msgs/BridgeStatus
        /diagnostics    diagnostic_msgs/DiagnosticArray
Service ~/emergency_stop  std_srvs/Trigger

This node subscribes to exactly one motion topic and it is a custom type that
no controller can produce.  There is no parameter, remap or code path here
that accepts ``geometry_msgs/Twist``.

Defence in depth
----------------
The bridge does not trust the arbiter.  It re-checks everything the arbiter
already checked, because the failure being defended against is *the arbiter
being wrong, absent, restarted, or duplicated*:

* **Independent watchdog.**  A timer running at the bridge's own rate stops
  the robot when no accepted command has arrived within
  ``watchdog_timeout_sec``.  This fires whether the arbiter crashed, hung, was
  killed, or lost network — the bridge never needs to be told.
* **Independent expiry.**  Each command carries ``valid_until``.  A command
  is refused at or after that time even if it arrived a moment ago.
* **Authority latching.**  The bridge latches the first ``authority_token``
  it sees and refuses commands bearing a different one, so two concurrently
  running arbiters cannot both drive the robot.  A token change is only
  accepted after the bridge has been quiet (stopped) for
  ``authority_handover_quiet_sec``, which makes a legitimate arbiter restart
  work while an overlapping duplicate does not.
* **Sequence monotonicity.**  A repeated or regressing sequence number from
  the same token is refused, which catches replayed or reordered commands.
* **Value re-validation.**  NaN, infinity and over-limit values are refused
  here too, using the bridge's own limit parameters.

Any refusal produces a stop, never a pass-through.
"""
from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from go2_safety_arbiter.reasons import Reason, SafetyState
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from go2_hardware_bridge.dry_run import DryRunGo2Bridge
from go2_hardware_bridge.interface import (
    BridgeHealth,
    BridgeState,
    HardwareBridgeError,
    HardwareBridgeInterface,
)
from go2_msgs.msg import BridgeStatus, SafeVelocityCommand

#: A sequence number this large cannot have been reached by counting at any
#: plausible control rate, so it is a sign of a malformed or hostile sender
#: rather than a long-running arbiter. Accepting one would latch a value
#: nothing can exceed, wedging the bridge permanently.
MAX_PLAUSIBLE_SEQUENCE = 2**60

#: The reason-code vocabulary a well-formed command may carry.
KNOWN_REASON_CODES = frozenset(
    value
    for name, value in vars(Reason).items()
    if not name.startswith("_") and isinstance(value, str)
)

#: Reason codes that are only consistent with a zero-velocity state.
STOPPING_REASON_CODES = frozenset(
    {
        Reason.EMERGENCY_STOP,
        Reason.HAZARD_STOP,
        Reason.WATCHDOG_TIMEOUT,
        Reason.STALE_COMMAND,
        Reason.INVALID_COMMAND,
        Reason.INVALID_TIMESTAMP,
        Reason.NO_LOCALIZATION,
        Reason.SAFETY_CONTEXT_STALE,
        Reason.NOT_INITIALIZED,
        Reason.RULE_EVALUATION_FAILED,
        Reason.SHUTDOWN,
    }
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


def build_adapter(node: Node, kind: str, log_path: str) -> HardwareBridgeInterface:
    """
    Construct the requested adapter, or fail.

    There is deliberately no fallback from ``unitree_sport`` to ``dry_run``.
    An operator who asked for hardware and silently got a simulator would
    believe the robot was under command when it was not.
    """
    kind = (kind or "").strip().lower()
    if kind == "dry_run":
        return DryRunGo2Bridge(log_path=log_path or None)
    if kind == "unitree_sport":
        from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

        return UnitreeSportBridge(
            node,
            command_hold_sec=float(
                node.get_parameter("adapter_command_hold_sec").value
            ),
            require_subscriber=bool(
                node.get_parameter("adapter_require_subscriber").value
            ),
        )
    raise HardwareBridgeError(
        f"Unknown hardware_adapter {kind!r}. Valid values: dry_run, unitree_sport."
    )


class HardwareBridgeNode(Node):
    def __init__(self, adapter: Optional[HardwareBridgeInterface] = None) -> None:
        super().__init__("hardware_bridge_node")

        self.declare_parameter("hardware_adapter", "dry_run")
        self.declare_parameter("dry_run_log_path", "")
        self.declare_parameter("watchdog_timeout_sec", 0.30)
        self.declare_parameter("control_frequency_hz", 50.0)
        self.declare_parameter("max_vx", 0.4)
        self.declare_parameter("max_vy", 0.2)
        self.declare_parameter("max_wz", 0.6)
        self.declare_parameter("authority_handover_quiet_sec", 1.0)
        self.declare_parameter("expected_frame_id", "base_link")
        self.declare_parameter("max_command_lifetime_sec", 2.0)
        self.declare_parameter("adapter_command_hold_sec", 0.2)
        self.declare_parameter("adapter_require_subscriber", True)
        self.declare_parameter("max_consecutive_transmit_failures", 3)

        self._watchdog = float(self.get_parameter("watchdog_timeout_sec").value)
        freq = float(self.get_parameter("control_frequency_hz").value)
        if not math.isfinite(freq) or freq <= 0.0:
            raise ValueError(f"control_frequency_hz must be finite and > 0, got {freq}")
        self._period = 1.0 / freq
        self._max_vx = abs(float(self.get_parameter("max_vx").value))
        self._max_vy = abs(float(self.get_parameter("max_vy").value))
        self._max_wz = abs(float(self.get_parameter("max_wz").value))
        self._handover_quiet = float(self.get_parameter("authority_handover_quiet_sec").value)
        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._max_lifetime = float(self.get_parameter("max_command_lifetime_sec").value)
        if not math.isfinite(self._max_lifetime) or not (0.0 < self._max_lifetime <= 5.0):
            raise ValueError(
                f"max_command_lifetime_sec must be in (0, 5], got {self._max_lifetime}"
            )
        self._max_tx_failures = int(
            self.get_parameter("max_consecutive_transmit_failures").value
        )

        # Watchdog timing uses a STEADY clock, never the node clock. With
        # use_sim_time enabled the node clock comes from /clock, and a stalled
        # /clock freezes the watchdog itself: ages compute as zero and it can
        # never fire, so a robot moving when the clock stalled keeps moving.
        # This is the bridge's last line of defence and it must not depend on
        # anything the rest of the graph can stop publishing.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self._adapter = adapter or build_adapter(
            self,
            str(self.get_parameter("hardware_adapter").value),
            str(self.get_parameter("dry_run_log_path").value),
        )
        # A failed connect is not a warning. An adapter that cannot reach its
        # transport but starts anyway reports healthy while publishing into the
        # void, and the operator believes the robot is under command.
        if not self._adapter.connect() and not self._adapter.dry_run:
            raise HardwareBridgeError(
                f"{self._adapter.name}.connect() failed; refusing to start. "
                f"Adapter reports: {self._adapter.health().detail}"
            )

        # ── State ─────────────────────────────────────────────────────
        self._authority_token: Optional[str] = None
        self._last_sequence: int = 0
        self._last_accept_time: Optional[float] = None
        #: When the bridge last transmitted a NON-ZERO velocity. Used for the
        #: authority-handover quiet window.
        #:
        #: This is deliberately not the same as "when we last called _stop".
        #: It used to be, and the result was that the handover branch could
        #: never be reached: every idle tick calls _stop, _stop refreshed the
        #: timestamp, and at 50 Hz the measured quiet period never exceeded
        #: 0.02 s against a 1.0 s requirement. A legitimate arbiter that
        #: crashed and respawned could therefore never reclaim authority, and
        #: the robot was immobilised until the bridge itself was restarted.
        self._last_nonzero_time: Optional[float] = None
        self._commanded_nonzero = False
        self._last_reject_reasons: List[str] = []
        self._rejected_count = 0
        self._consecutive_tx_failures = 0
        self._estop_engaged = False
        self._last_tx = (0.0, 0.0, 0.0)
        self._pending: Optional[SafeVelocityCommand] = None

        self.create_subscription(
            SafeVelocityCommand, "cmd_vel_safe", self._safe_cb, CONTROL_QOS
        )
        self._status_pub = self.create_publisher(BridgeStatus, "bridge/status", STATUS_QOS)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_service(Trigger, "~/emergency_stop", self._estop_cb)

        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"HardwareBridgeNode active. adapter={self._adapter.name} "
            f"dry_run={self._adapter.dry_run} watchdog={self._watchdog}s "
            f"rate={freq:.0f}Hz"
        )
        if not self._adapter.dry_run:
            self.get_logger().warn(
                "PHYSICAL ADAPTER SELECTED. This code path has never been "
                "executed against a GO2 by this repository."
            )

    # ── Time ──────────────────────────────────────────────────────────

    def _now(self) -> float:
        """Node-clock reference, used to interpret arbiter-supplied stamps."""
        return self.get_clock().now().nanoseconds / 1e9

    def _steady_now(self) -> float:
        """Monotonic reference for the bridge's own watchdog and quiet timer."""
        return self._steady_clock.now().nanoseconds / 1e9

    # ── Intake ────────────────────────────────────────────────────────

    def _safe_cb(self, msg: SafeVelocityCommand) -> None:
        # Store only. All acceptance logic runs on the timer so that a flood
        # of messages cannot starve the watchdog, and so that acceptance and
        # actuation happen at a single, known rate.
        self._pending = msg

    def _validate(self, msg: SafeVelocityCommand, now: float) -> List[str]:
        """Return reject reasons. Empty list means the command is acceptable."""
        reasons: List[str] = []

        # Authority
        token = msg.authority_token or ""
        if not token:
            reasons.append(Reason.AUTHORITY_MISMATCH)
        elif self._authority_token is None:
            # First command ever, or first after a quiet period: latch.
            self._authority_token = token
            self._last_sequence = 0
        elif token != self._authority_token:
            # Quiet means "has not actually moved recently", not "has called
            # _stop recently".
            if self._last_nonzero_time is None:
                quiet_for = float("inf")
            else:
                quiet_for = now - self._last_nonzero_time
            if not self._commanded_nonzero and quiet_for >= self._handover_quiet:
                self.get_logger().warn(
                    f"Authority handover: {self._authority_token[:8]} -> {token[:8]} "
                    f"after {quiet_for:.2f}s stopped"
                )
                self._authority_token = token
                self._last_sequence = 0
            else:
                reasons.append(Reason.AUTHORITY_MISMATCH)

        # Sequence monotonicity (only meaningful once a token is latched)
        if Reason.AUTHORITY_MISMATCH not in reasons:
            if msg.sequence <= self._last_sequence:
                reasons.append(Reason.SEQUENCE_REGRESSION)
            elif msg.sequence > MAX_PLAUSIBLE_SEQUENCE:
                # A sequence near the uint64 ceiling cannot have been reached by
                # counting: at 20 Hz it would take longer than the age of the
                # universe. Accepting one would latch a value that nothing can
                # ever exceed, permanently wedging the bridge into
                # SEQUENCE_REGRESSION on every subsequent command.
                reasons.append(Reason.SEQUENCE_REGRESSION)

        # Expiry and freshness are evaluated against the NODE clock, because
        # that is the base the arbiter stamped them in. The watchdog in _tick
        # uses steady time; these two checks are about the message, that one is
        # about the passage of time.
        node_now = self._now()
        valid_until = msg.valid_until.sec + msg.valid_until.nanosec / 1e9
        if valid_until <= 0.0 or node_now >= valid_until:
            reasons.append(Reason.COMMAND_EXPIRED)
        elif (valid_until - node_now) > self._max_lifetime:
            # A command claiming to be valid far into the future is not a
            # licence to move for that long. An arbiter that stamped one is
            # malfunctioning, and a message that arrived with one may not have
            # come from an arbiter at all.
            reasons.append(Reason.COMMAND_EXPIRED)

        # Header freshness — independent of valid_until.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp <= 0.0:
            reasons.append(Reason.COMMAND_EXPIRED)
        elif (node_now - stamp) > self._watchdog:
            reasons.append(Reason.BRIDGE_WATCHDOG_TIMEOUT)

        # Frame. A command expressed in a frame the bridge does not expect is
        # not interpretable as a body velocity, whatever its numbers say.
        if msg.header.frame_id != self._expected_frame:
            reasons.append(Reason.INVALID_COMMAND)

        # Value validity
        vx, vy, wz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            reasons.append(Reason.INVALID_COMMAND)
        elif abs(vx) > self._max_vx or abs(vy) > self._max_vy or abs(wz) > self._max_wz:
            # The arbiter should already have clamped. If it did not, the
            # arbiter is malfunctioning: refuse rather than clamp, so the
            # fault is visible instead of silently absorbed.
            reasons.append(Reason.SPEED_LIMIT)

        # A command authored in a stopping state must be zero. If it is not,
        # the arbiter contradicted itself: refuse.
        if msg.arbiter_state in SafetyState.ZERO_STATES and any(
            abs(v) > 1e-9 for v in (vx, vy, wz) if math.isfinite(v)
        ):
            reasons.append(Reason.INVALID_COMMAND)

        if msg.arbiter_state not in SafetyState.ALL:
            reasons.append(Reason.INVALID_COMMAND)

        # Reason codes must come from the known vocabulary, and must not
        # contradict the state they arrive with. A command asserting
        # SAFE_TO_MOVE while carrying EMERGENCY_STOP in its reasons is
        # internally inconsistent, and the safe reading of an inconsistent
        # command is that it is not trustworthy.
        for code in msg.reason_codes:
            if code not in KNOWN_REASON_CODES:
                reasons.append(Reason.INVALID_COMMAND)
                break
            if code in STOPPING_REASON_CODES and msg.arbiter_state not in (
                SafetyState.ZERO_STATES
            ):
                reasons.append(Reason.INVALID_COMMAND)
                break

        return reasons

    # ── Control loop ──────────────────────────────────────────────────

    def _tick(self) -> None:
        try:
            self._tick_inner()
        except Exception as exc:  # noqa: BLE001
            # A bridge whose control loop raises must still stop the robot.
            self.get_logger().error(
                f"Bridge tick failed: {exc}. Forcing stop.",
                throttle_duration_sec=1.0,
            )
            try:
                self._adapter.send_zero()
            except Exception:  # noqa: BLE001
                pass

    def _tick_inner(self) -> None:
        now = self._steady_now()

        # Give the adapter its own cycle. Adapters with latching transports use
        # this to enforce a command-hold timeout underneath the bridge's
        # watchdog; the dry-run adapter ignores it.
        self._adapter.tick()

        if self._estop_engaged:
            self._stop(now, [Reason.EMERGENCY_STOP], emergency=True)
            self._publish_status(now)
            return

        msg = self._pending
        self._pending = None

        if msg is None:
            # No new command this tick. The watchdog decides whether the last
            # accepted command may still stand.
            if (
                self._last_accept_time is None
                or (now - self._last_accept_time) > self._watchdog
            ):
                self._stop(now, [Reason.BRIDGE_WATCHDOG_TIMEOUT])
            # Between ticks and within the watchdog window we simply do not
            # retransmit: the adapter holds the last commanded velocity for at
            # most watchdog_timeout_sec before this branch zeroes it.
            self._publish_status(now)
            return

        reasons = self._validate(msg, now)
        if reasons:
            self._rejected_count += 1
            self._last_reject_reasons = reasons
            self.get_logger().warn(
                f"Rejected safe command seq={msg.sequence}: {','.join(reasons)}",
                throttle_duration_sec=1.0,
            )
            self._stop(now, reasons)
            self._publish_status(now)
            return

        self._last_sequence = msg.sequence
        self._last_accept_time = now
        self._last_reject_reasons = []

        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        wz = float(msg.twist.angular.z)

        if abs(vx) <= 1e-9 and abs(vy) <= 1e-9 and abs(wz) <= 1e-9:
            self._stop(now, list(msg.reason_codes))
        else:
            ok = self._adapter.send_velocity(vx, vy, wz)
            self._last_tx = (vx, vy, wz)
            self._commanded_nonzero = True
            self._last_nonzero_time = now
            if ok:
                self._consecutive_tx_failures = 0
            else:
                self._consecutive_tx_failures += 1
                self._last_reject_reasons = [Reason.TRANSMIT_FAILED]
                if self._consecutive_tx_failures >= self._max_tx_failures:
                    self.get_logger().error(
                        f"{self._consecutive_tx_failures} consecutive transmit "
                        "failures; forcing stop"
                    )
                    self._stop(now, [Reason.TRANSMIT_FAILED])

        self._publish_status(now)

    def _stop(self, now: float, reasons: List[str], emergency: bool = False) -> None:
        if emergency:
            self._adapter.emergency_stop()
        else:
            self._adapter.send_zero()
        self._last_tx = (0.0, 0.0, 0.0)
        if self._commanded_nonzero:
            self._commanded_nonzero = False
        if reasons:
            self._last_reject_reasons = reasons

    # ── Services ──────────────────────────────────────────────────────

    def _estop_cb(self, _req, resp):
        self._estop_engaged = True
        self._stop(self._steady_now(), [Reason.EMERGENCY_STOP], emergency=True)
        self.get_logger().warn("Bridge emergency stop engaged (latched)")
        resp.success = True
        resp.message = "bridge emergency stop engaged; restart bridge to clear"
        return resp

    # ── Observability ─────────────────────────────────────────────────

    def _publish_status(self, now: float) -> None:
        health: BridgeHealth = self._adapter.health()
        msg = BridgeStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.bridge_state = BridgeState.STOPPED if self._estop_engaged else health.state
        msg.adapter_name = self._adapter.name
        msg.dry_run = bool(self._adapter.dry_run)
        msg.connected = bool(health.connected)
        msg.last_safe_command_age_sec = (
            -1.0 if self._last_accept_time is None else float(now - self._last_accept_time)
        )
        msg.last_transmit_ok = bool(health.last_transmit_ok)
        msg.transmit_attempts = int(health.transmit_attempts)
        msg.transmit_failures = int(health.transmit_failures)
        msg.rejected_count = int(self._rejected_count)
        msg.last_reject_reasons = list(self._last_reject_reasons)
        msg.last_transmitted.linear.x = self._last_tx[0]
        msg.last_transmitted.linear.y = self._last_tx[1]
        msg.last_transmitted.angular.z = self._last_tx[2]
        self._status_pub.publish(msg)

        status = DiagnosticStatus()
        status.name = "go2_hardware_bridge: actuation"
        status.hardware_id = self._adapter.name
        moving = any(abs(v) > 1e-9 for v in self._last_tx)
        if self._estop_engaged or health.state == BridgeState.FAULT:
            status.level = DiagnosticStatus.ERROR
        elif not health.connected or self._last_reject_reasons:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = f"{msg.bridge_state} {'MOVING' if moving else 'STOPPED'}"
        status.values = [
            KeyValue(key="adapter", value=msg.adapter_name),
            KeyValue(key="dry_run", value=str(msg.dry_run)),
            KeyValue(key="connected", value=str(msg.connected)),
            KeyValue(key="last_safe_command_age_sec", value=f"{msg.last_safe_command_age_sec:.3f}"),
            KeyValue(key="authority_token", value=self._authority_token or ""),
            KeyValue(key="last_sequence", value=str(self._last_sequence)),
            KeyValue(key="rejected_count", value=str(msg.rejected_count)),
            KeyValue(key="last_reject_reasons", value=",".join(msg.last_reject_reasons)),
            KeyValue(key="transmit_failures", value=str(msg.transmit_failures)),
            KeyValue(key="last_vx", value=f"{self._last_tx[0]:.3f}"),
            KeyValue(key="last_vy", value=f"{self._last_tx[1]:.3f}"),
            KeyValue(key="last_wz", value=f"{self._last_tx[2]:.3f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    # ── Shutdown ──────────────────────────────────────────────────────

    def destroy_node(self) -> bool:
        try:
            self._adapter.shutdown()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Adapter shutdown failed: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    """
    Entry point.

    ``ExternalShutdownException`` is handled explicitly. Without it, a SIGTERM
    during ``spin`` (which is how launch stops a node, and how Ctrl-C reaches
    one) invalidates the rcl context underneath the executor and raises
    ``RCLError: failed to initialize wait set`` from inside ``spin`` — before
    the ``finally`` block can run ``destroy_node``. For the bridge, that
    would mean the adapter never being told to stop, leaving a physical robot to
    coast on its last command until its own controller times out.

    Catching it here means the teardown path runs on every ordinary stop.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareBridgeNode()
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
