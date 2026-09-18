"""
nav_tf_watchdog: detect and recover Nav2's controller losing map->odom.

Observed in closed-loop sim (about 1 in 10 trials, under both FastDDS and
CycloneDDS): Nav2's controller_server stops seeing new map->odom transforms
while slam_toolbox keeps publishing them and other readers keep receiving
them. The controller then rejects every path ("Transform data too old when
converting from map to odom") and navigation is dead for the rest of the run.
The robot fails safe (no candidates, the arbiter commands zero). The cause
inside controller_server was NOT isolated (docs/DEPLOYMENT.md).

Detection. The error text is logged by a stand-alone "tf_help" logger that
never reaches /rosout, so it cannot be observed from outside the process
(the first version of this node watched /rosout and never fired). Instead,
Regulated Pure Pursuit publishes the transformed plan on
/received_global_plan on every control cycle in which the map->odom
transform succeeds (20 Hz). A stall is: the planner is actively publishing
/plan (the behaviour tree is navigating) while /received_global_plan has
been silent for stall_s.

Recovery. Reset and restart the Nav2 lifecycle through manage_nodes, which
rebuilds the controller's costmap and TF buffer. The goal in progress aborts;
callers see ABORTED and may retry.
"""
from __future__ import annotations

import json
from collections import deque
from typing import Optional


class StallDetector:
    """Pure logic over observation times (seconds, monotonic)."""

    def __init__(self, stall_s: float = 3.0, min_plans: int = 2, cooldown_s: float = 30.0) -> None:
        if stall_s <= 0 or min_plans < 1 or cooldown_s < 0:
            raise ValueError("stall_s > 0, min_plans >= 1, cooldown_s >= 0 required")
        self.stall_s, self.min_plans, self.cooldown_s = stall_s, min_plans, cooldown_s
        self._plans: deque[float] = deque(maxlen=64)
        self._nav_since: Optional[float] = None
        self._last_controller_plan: Optional[float] = None
        self._last_recovery: Optional[float] = None

    def on_planner_plan(self, now: float) -> None:
        # A gap longer than the window starts a new navigation episode.
        if not self._plans or now - self._plans[-1] > self.stall_s:
            self._nav_since = now
        self._plans.append(now)

    def on_controller_plan(self, now: float) -> None:
        self._last_controller_plan = now

    def check(self, now: float) -> bool:
        """True when recovery should fire now."""
        recent_plans = sum(1 for t in self._plans if now - t <= self.stall_s)
        if recent_plans < self.min_plans or self._nav_since is None:
            return False  # not navigating (idle, or in a recovery behaviour)
        # Silent since the later of: navigation start, last controller plan.
        ref = self._nav_since
        if self._last_controller_plan is not None:
            ref = max(ref, self._last_controller_plan)
        if now - ref <= self.stall_s:
            return False
        if self._last_recovery is not None and now - self._last_recovery < self.cooldown_s:
            return False
        self._last_recovery = now
        self._plans.clear()
        self._nav_since = None
        return True


def main(args=None) -> None:
    import time

    import rclpy
    from nav2_msgs.srv import ManageLifecycleNodes
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from std_msgs.msg import String

    class NavTfWatchdog(Node):
        def __init__(self) -> None:
            super().__init__("nav_tf_watchdog")
            self.declare_parameter("stall_s", 3.0)
            self.declare_parameter("cooldown_s", 30.0)
            self.declare_parameter("planner_plan_topic", "/plan")
            self.declare_parameter("controller_plan_topic", "/received_global_plan")
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
            self.create_subscription(
                Path, str(self.get_parameter("planner_plan_topic").value),
                lambda _m: self._detector.on_planner_plan(time.monotonic()), 10)
            self.create_subscription(
                Path, str(self.get_parameter("controller_plan_topic").value),
                lambda _m: self._detector.on_controller_plan(time.monotonic()), 10)
            self.create_timer(0.5, self._tick)
            self.create_timer(1.0, self._publish_status)

        def _tick(self) -> None:
            if not self._busy and self._detector.check(time.monotonic()):
                self._recover()

        def _recover(self) -> None:
            if not self._client.service_is_ready():
                self.get_logger().error("Nav2 controller stall detected but manage_nodes is unavailable")
                return
            self._busy = True
            self._recoveries += 1
            self.get_logger().error(
                "Nav2 controller stopped transforming plans while the planner is active "
                f"(> {self._detector.stall_s:.1f}s); resetting Nav2 lifecycle "
                f"(recovery #{self._recoveries})"
            )
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.RESET
            self._client.call_async(req).add_done_callback(self._after_reset)

        def _after_reset(self, future) -> None:
            if future.result() is None or not future.result().success:
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
                {"controller_stall_recoveries": self._recoveries, "recovering": self._busy}
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
