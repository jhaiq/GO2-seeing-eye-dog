"""
UnitreeSportBridge, velocity transport to a physical GO2 via the Sport API.

STATUS: IMPLEMENTED, NEVER EXECUTED AGAINST HARDWARE BY THIS REPOSITORY.

Everything in this file is written from the Unitree ROS 2 SDK message
definitions and the documented Sport API request format.  No line of it has
been run against a GO2.  It is provided so the adapter contract is real
rather than hypothetical, and so the eventual hardware bring-up is a
bring-up, not a rewrite.  ``docs/research_system_claims.md`` lists this under
"implemented but not physically validated" and it must stay there until a
logged hardware session says otherwise.

The command-starvation property
-------------------------------
The Sport API's ``Move`` is a LATCHING command: the onboard controller keeps
executing the last velocity it was given until it receives another one or a
``StopMove``. That makes the obvious implementation dangerous. If this bridge
is SIGKILLed, or its host loses power, or the process is OOM-killed, no
shutdown handler runs, and the last thing the robot heard was
``Move(0.4, 0, 0)``. It walks away.

So this adapter does **not** treat ``send_velocity`` as "transmit when the
value changes". Every call transmits, the bridge calls it on every control
tick, and the adapter additionally refuses to keep asserting a velocity it has
not been re-given within ``command_hold_sec``. Bridge death therefore becomes
command starvation at the robot, which is a condition the robot can act on,
rather than a silent handover of control to the last message that happened to
arrive.

This is the one place where the dry-run adapter and the physical adapter differ
in kind rather than in destination: ``DryRunGo2Bridge`` is a pure recorder with
no latching semantics, so **no dry-run test can detect this class of bug**.
That asymmetry is why it is written down here.

Interface choice
----------------
The Sport API (high-level) is used rather than ``/lowcmd`` (low-level).
Reasons:

* The Sport API's ``Move`` request (api_id 1008) takes exactly the body
  velocity ``(vx, vy, wz)`` this architecture produces.  Low-level control
  would require this repository to own balance and gait generation for a
  quadruped, which it does not and should not.
* Unitree's onboard controller keeps the robot balanced.  A bug in this stack
  produces a bad velocity; a bug in a low-level path produces a fall.
* ``/lowcmd`` additionally requires a correct CRC over the packed command
  struct and requires the onboard sport service to be stopped first.  The
  pre-existing ``go2_gait_controller/scripts/hw_bridge.py`` did neither, which
  is one reason it is superseded here.
"""
from __future__ import annotations

import json
import threading
import time
from typing import Any, Optional

from go2_hardware_bridge.interface import (
    BridgeHealth,
    BridgeState,
    HardwareBridgeError,
    HardwareBridgeInterface,
)

#: Unitree Sport API identifiers.
API_ID_DAMP = 1001
API_ID_STOP_MOVE = 1003
API_ID_MOVE = 1008
API_ID_STAND_UP = 1010

#: Topic the GO2 sport service listens on.
SPORT_REQUEST_TOPIC = "/api/sport/request"


class UnitreeSportBridge(HardwareBridgeInterface):
    """
    Publishes ``unitree_api/msg/Request`` Move commands to the GO2.

    Construction fails loudly if ``unitree_api`` is not importable, rather
    than degrading to a no-op.  A hardware adapter that silently does nothing
    is more dangerous than one that refuses to start, because the operator
    believes commands are being delivered.
    """

    dry_run = False

    def __init__(
        self,
        node: Any,
        topic: str = SPORT_REQUEST_TOPIC,
        qos_depth: int = 1,
        command_hold_sec: float = 0.2,
        require_subscriber: bool = True,
        discovery_timeout_sec: float = 10.0,
    ) -> None:
        try:
            from unitree_api.msg import Request  # noqa: PLC0415, optional dependency
        except ImportError as exc:  # pragma: no cover, requires GO2 SDK
            raise HardwareBridgeError(
                "unitree_api is not importable, so UnitreeSportBridge cannot be "
                "constructed. Install the Unitree ROS 2 SDK "
                "(https://github.com/unitreerobotics/unitree_ros2) on the robot's "
                "onboard computer, or run with hardware_adapter:=dry_run."
            ) from exc

        self._Request = Request
        self._node = node
        self._lock = threading.Lock()
        self._health = BridgeHealth(state=BridgeState.UNINITIALIZED, connected=False)
        self._pub = node.create_publisher(Request, topic, qos_depth)
        self._topic = topic
        self._command_hold = float(command_hold_sec)
        self._require_subscriber = bool(require_subscriber)
        self._discovery_timeout = max(0.0, float(discovery_timeout_sec))
        self._last_move_time: Optional[float] = None
        self._holding_nonzero = False

    # ── Contract ──────────────────────────────────────────────────────

    def connect(self) -> bool:
        # There is no handshake on the Sport API request topic: it is a
        # fire-and-forget publisher. "Connected" here means the publisher
        # exists and at least one subscriber (the sport service) is visible.
        # This is an honest, weak liveness signal and is reported as such.
        #
        # DDS discovery is not instantaneous. Checking once at construction
        # raced the sport service in closed-loop sim: the bridge exited at
        # launch in 2 of 8 runs and every authorized command went nowhere. The
        # graph cache updates without spinning, so poll for a bounded time.
        deadline = time.monotonic() + self._discovery_timeout
        while self._pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        with self._lock:
            subscribers = self._pub.get_subscription_count()
            connected = subscribers > 0
            self._health = BridgeHealth(
                state=BridgeState.CONNECTED if connected else BridgeState.DISCONNECTED,
                connected=connected,
                detail=(
                    f"{subscribers} subscriber(s) on {self._topic}; "
                    "subscriber presence is not proof the robot will move"
                ),
            )
        if not connected and self._require_subscriber:
            raise HardwareBridgeError(
                f"No subscriber on {self._topic} after {self._discovery_timeout:.1f}s. "
                "The GO2 sport service does not "
                "appear to be running or reachable. Starting anyway would mean "
                "publishing commands into the void while reporting healthy; set "
                "require_subscriber:=false only if you understand that."
            )
        return connected

    def send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """
        Transmit a Move request. Every call transmits; nothing is deduplicated.

        Re-sending an unchanged velocity is deliberate. See the module
        docstring: Move latches on the robot, so a bridge that only publishes
        on change hands control to its last message if it dies.
        """
        ok = self._publish(API_ID_MOVE, {"x": float(vx), "y": float(vy), "z": float(wz)})
        with self._lock:
            self._last_move_time = time.monotonic()
            self._holding_nonzero = any(
                abs(v) > 1e-9 for v in (vx, vy, wz)
            )
        return ok

    def tick(self) -> None:
        """
        Called by the bridge on every control cycle, whether or not a command
        arrived.

        If a non-zero velocity has been asserted and not renewed within
        ``command_hold_sec``, this issues ``StopMove``. It is a second,
        adapter-local watchdog sitting underneath the bridge's own, and it
        exists because the failure it guards against is the bridge's control
        loop stalling rather than the arbiter's.
        """
        with self._lock:
            holding = self._holding_nonzero
            last = self._last_move_time
        if not holding or last is None:
            return
        if (time.monotonic() - last) > self._command_hold:
            self._node.get_logger().warn(
                "Move command not renewed within command_hold_sec; issuing StopMove"
            )
            self.send_zero()

    def send_zero(self) -> bool:
        # StopMove (1003) is preferred over Move(0,0,0): it tells the onboard
        # controller to halt rather than to track a zero velocity, which
        # settles the robot rather than leaving it actively balancing a
        # commanded zero.
        ok = self._publish(API_ID_STOP_MOVE, None)
        with self._lock:
            self._holding_nonzero = False
        return ok

    def emergency_stop(self) -> bool:
        # Damp (1001) drops the joints to a damped state. It is the strongest
        # stop reachable over the Sport API without cutting power.
        ok_stop = self._publish(API_ID_STOP_MOVE, None)
        ok_damp = self._publish(API_ID_DAMP, None)
        with self._lock:
            self._health.state = BridgeState.STOPPED
            self._health.detail = "emergency stop: StopMove + Damp issued"
        return ok_stop and ok_damp

    def health(self) -> BridgeHealth:
        with self._lock:
            snapshot = BridgeHealth(
                state=self._health.state,
                connected=self._health.connected,
                detail=self._health.detail,
                transmit_attempts=self._health.transmit_attempts,
                transmit_failures=self._health.transmit_failures,
                last_transmit_ok=self._health.last_transmit_ok,
            )
        try:
            snapshot.connected = self._pub.get_subscription_count() > 0
        except Exception:  # noqa: BLE001, health() must not raise
            snapshot.connected = False
        return snapshot

    def shutdown(self) -> None:
        try:
            self.send_zero()
            self.emergency_stop()
        except Exception:  # noqa: BLE001
            pass
        with self._lock:
            self._health.state = BridgeState.STOPPED
            self._health.connected = False

    # ── Internals ─────────────────────────────────────────────────────

    def _publish(self, api_id: int, params: Optional[dict]) -> bool:
        # DDS publish() on a fire-and-forget topic does not raise when nobody
        # is subscribed, so without this check send_velocity would return True
        # unconditionally and the bridge's transmit-failure counter could never
        # trip. A bridge that believes it stopped a robot it never reached is
        # worse than one that reports a fault.
        try:
            if self._pub.get_subscription_count() == 0:
                with self._lock:
                    self._health.transmit_attempts += 1
                    self._health.transmit_failures += 1
                    self._health.last_transmit_ok = False
                    self._health.connected = False
                    self._health.state = BridgeState.DISCONNECTED
                    self._health.detail = f"no subscriber on {self._topic}"
                return False
        except Exception:  # noqa: BLE001, fall through to the publish attempt
            pass

        msg = self._Request()
        msg.header.identity.api_id = api_id
        # Field notes (docs/go2_field_notes.md s4): the request format verified
        # on the robot uses a unique identity.id per request and noreply=false,
        # under which the sport service answers on /api/sport/response with the
        # matching id. id=0 on every request made replies unmatchable, and
        # noreply=true was never exercised on hardware.
        self._request_seq = (getattr(self, "_request_seq", 0) + 1) % (2**63)
        msg.header.identity.id = int(time.time_ns() // 1000) * 1000 + self._request_seq % 1000
        msg.header.lease.id = 0
        msg.header.policy.priority = 0
        msg.header.policy.noreply = False
        msg.parameter = json.dumps(params) if params is not None else ""
        msg.binary = []
        try:
            self._pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._health.transmit_attempts += 1
                self._health.transmit_failures += 1
                self._health.last_transmit_ok = False
                self._health.state = BridgeState.FAULT
                self._health.detail = f"publish failed: {exc}"
            return False
        with self._lock:
            self._health.transmit_attempts += 1
            self._health.last_transmit_ok = True
        return True
