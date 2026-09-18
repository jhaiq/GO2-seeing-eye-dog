"""
nav_tf_watchdog: detect and recover Nav2's controller losing map->odom.

Observed in closed-loop sim (about 1 in 10 trials, under both FastDDS and
CycloneDDS): Nav2's controller_server stops seeing new map->odom transforms
while slam_toolbox keeps publishing them and every other reader (the relay,
the behaviour server, a fresh tf2_echo) keeps receiving them. From then on the
controller rejects every path with

    [tf_help]: Transform data too old when converting from map to odom

and navigation never succeeds again. The robot fails safe (no candidates,
the arbiter commands zero), but the stack is dead until restarted. The cause
inside controller_server was NOT isolated (docs/DEPLOYMENT.md).

This node watches /rosout for that error persisting continuously for
stall_s and then resets and restarts the Nav2 lifecycle through the supported
manage_nodes service, which rebuilds the controller's costmap and TF buffer.
The goal in progress aborts; callers see ABORTED and may retry.
"""
from __future__ import annotations

import json
import time
from typing import Optional

STALL_MARKER = "Transform data too old when converting from map to odom"


class StallDetector:
    """Pure logic: a continuous streak of marker messages longer than stall_s."""

    def __init__(self, stall_s: float = 3.0, gap_s: float = 1.0, cooldown_s: float = 30.0) -> None:
        if stall_s <= 0 or gap_s <= 0 or cooldown_s < 0:
            raise ValueError("stall_s and gap_s must be > 0, cooldown_s >= 0")
        self.stall_s, self.gap_s, self.cooldown_s = stall_s, gap_s, cooldown_s
        self._streak_start: Optional[float] = None
        self._last_seen: Optional[float] = None
        self._last_recovery: Optional[float] = None

    def observe(self, now: float) -> bool:
        """Record one marker message at time now. True if recovery should fire."""
        if self._last_seen is None or now - self._last_seen > self.gap_s:
            self._streak_start = now
        self._last_seen = now
        if now - self._streak_start < self.stall_s:
            return False
        if self._last_recovery is not None and now - self._last_recovery < self.cooldown_s:
            return False
        self._last_recovery = now
        self._streak_start = now  # a new streak must build up again
        return True


def main(args=None) -> None:
    import rclpy
    from nav2_msgs.srv import ManageLifecycleNodes
    from rcl_interfaces.msg import Log
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class NavTfWatchdog(Node):
        def __init__(self) -> None:
            super().__init__("nav_tf_watchdog")
            self.declare_parameter("stall_s", 3.0)
            self.declare_parameter("cooldown_s", 30.0)
            self.declare_parameter("manage_nodes_service", "/lifecycle_manager_navigation/manage_nodes")
            self._detector = StallDetector(
                stall_s=float(self.get_parameter("stall_s").value),
                cooldown_s=float(self.get_parameter("cooldown_s").value),
            )
            self._client = self.create_client(
                ManageLifecycleNodes, str(self.get_parameter("manage_nodes_service").value)
            )
            self._recoveries = 0
            self._busy = False
            self._status = self.create_publisher(String, "/go2/nav_health", 10)
            qos = QoSProfile(depth=100)
            qos.reliability = ReliabilityPolicy.RELIABLE
            self.create_subscription(Log, "/rosout", self._on_log, qos)
            self.create_timer(1.0, self._publish_status)

        def _on_log(self, msg: Log) -> None:
            if STALL_MARKER not in msg.msg:
                return
            if self._detector.observe(time.monotonic()) and not self._busy:
                self._recover()

        def _recover(self) -> None:
            if not self._client.service_is_ready():
                self.get_logger().error("Nav2 map->odom stall detected but manage_nodes is unavailable")
                return
            self._busy = True
            self._recoveries += 1
            self.get_logger().error(
                f"Nav2 controller lost map->odom (stall > {self._detector.stall_s:.1f}s); "
                f"resetting Nav2 lifecycle (recovery #{self._recoveries})"
            )
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.RESET
            self._client.call_async(req).add_done_callback(self._after_reset)

        def _after_reset(self, future) -> None:
            ok = future.result() is not None and future.result().success
            if not ok:
                self.get_logger().error("Nav2 RESET failed; attempting STARTUP anyway")
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.STARTUP
            self._client.call_async(req).add_done_callback(self._after_startup)

        def _after_startup(self, future) -> None:
            ok = future.result() is not None and future.result().success
            self._busy = False
            (self.get_logger().info if ok else self.get_logger().error)(
                f"Nav2 STARTUP after recovery: {'ok' if ok else 'FAILED'}"
            )

        def _publish_status(self) -> None:
            self._status.publish(String(data=json.dumps(
                {"map_odom_stall_recoveries": self._recoveries, "recovering": self._busy}
            )))

    rclpy.init(args=args)
    node = NavTfWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
