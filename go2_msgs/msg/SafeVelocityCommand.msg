# go2_msgs/msg/SafeVelocityCommand.msg
#
# The ONLY message type accepted by the GO2 hardware bridge.
#
# This type exists for a mechanical safety reason, not for convenience.
# A planner, controller, teleop tool or `ros2 topic pub` emitting
# geometry_msgs/Twist CANNOT connect to a subscriber of this type: ROS 2
# refuses to match publishers and subscribers with different type hashes.
# Therefore the only process that can physically deliver a motion command to
# the hardware bridge is a process that deliberately constructs this message.
# By convention and by launch wiring that process is the SafetyArbiter.
#
# See docs/target_runtime_architecture.md (Invariant A, Invariant D).

std_msgs/Header header

# Authorized body velocity in base_link (REP-103: x forward, y left, z up,
# yaw CCW-positive). Already validated, clamped and rate-limited.
geometry_msgs/Twist twist

# Arbiter state that authorized this command.
# One of: SAFE_TO_MOVE | DEGRADED | STOPPED | EMERGENCY_STOP
string arbiter_state

# Machine-readable reasons the candidate was modified or zeroed.
# Empty when the candidate passed through unmodified.
string[] reason_codes

# Monotonic counter from the authoring arbiter. Lets the bridge detect a
# second, competing arbiter (sequence goes backwards / duplicates) and a
# restarted arbiter (sequence resets).
uint64 sequence

# Random identity generated once per arbiter process instance. The bridge
# latches the first token it sees and rejects commands carrying a different
# token, so two simultaneously running arbiters cannot both drive the robot.
string authority_token

# Hard expiry. The bridge MUST NOT actuate this command at or after this
# time, independent of its own watchdog.
builtin_interfaces/Time valid_until
