#!/usr/bin/env python3
"""
CandidateStamperNode — adapts an unstamped controller to the stamped contract.

Nav2 on ROS 2 Humble publishes ``geometry_msgs/Twist`` on ``cmd_vel``;
``TwistStamped`` only became the Nav2 default in later distributions.  The
safety contract in this repository requires a timestamp so that command
freshness (Invariant C) is enforceable.

This node bridges the two, and it is deliberately honest about what it can
and cannot know:

    in   cmd_vel_in   geometry_msgs/Twist        (Nav2's raw output)
    out  cmd_vel_candidate  geometry_msgs/TwistStamped

The stamp it applies is the **reception** time, not the generation time.
That means the age the arbiter computes excludes any latency inside the
producer and in the middleware before arrival.  It is a lower bound on true
age.  This is recorded here, in the launch file, and in the architecture doc,
because a freshness guarantee built on an unverifiable timestamp is worth
less than one built on a real one — but it is still worth more than nothing,
since it catches the dominant failure mode: the producer stopping.

When Nav2 is upgraded to a distribution that publishes TwistStamped, delete
this node and point Nav2 straight at ``cmd_vel_candidate``.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

CONTROL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class CandidateStamperNode(Node):
    def __init__(self) -> None:
        super().__init__("candidate_stamper_node")
        self.declare_parameter("frame_id", "base_link")
        self._frame = str(self.get_parameter("frame_id").value)

        self.create_subscription(Twist, "cmd_vel_in", self._cb, CONTROL_QOS)
        self._pub = self.create_publisher(TwistStamped, "cmd_vel_candidate", CONTROL_QOS)

        self.get_logger().warn(
            "CandidateStamperNode: stamping unstamped velocities at RECEIPT time. "
            "Candidate age therefore excludes producer-side latency."
        )

    def _cb(self, msg: Twist) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        out.twist = msg
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CandidateStamperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
