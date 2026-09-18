# `go2_gait_controller` — superseded actuation path

## Status: the hardware bridge in this package has been REMOVED

`scripts/hw_bridge.py` and `launch/gait_hw_launch.py` are deleted. They were
the repository's only other route to GO2 actuation, and leaving them in place
would have contradicted the invariant the rest of the stack now enforces:

> No motion command may reach the physical GO2 without passing through the
> deterministic safety authority.

Actuation now goes through `go2_hardware_bridge`, which consumes only
`go2_msgs/SafeVelocityCommand` published by `go2_safety_arbiter`. See
`docs/target_runtime_architecture.md`.

## Why the old bridge was not worth keeping

Independent of the architecture, the removed code could not have worked:

* **It was never installed as an executable.** `CMakeLists.txt` installed
  `scripts/` to `share/${PROJECT_NAME}`, but `ros2 run` and
  `launch_ros.actions.Node(executable=...)` resolve executables from
  `lib/${PROJECT_NAME}`. `install(DIRECTORY)` without `USE_SOURCE_PERMISSIONS`
  also stripped the executable bit. `gait_hw_launch.py` would have raised
  `ExecutableNotFound` at launch-description build time, aborting the whole
  launch — including the gait controller listed beside it.

* **Its `/lowcmd` path would have been rejected by the robot.** It built a
  `unitree_go/msg/LowCmd` with a default-initialised header: `head` left as
  `(0, 0)` instead of `(0xFE, 0xEF)`, `level_flag` left as `0` instead of
  `0xFF`, and — decisively — `crc` left as `0`. Unitree's own reference
  implementation recomputes the CRC over the packed struct before every
  publish. There is no CRC computation anywhere in this repository.

* **It never released the onboard sport service.** Low-level control requires
  stopping the GO2's motion-control service first (Unitree's examples call
  `ServiceSwitch("sport_mode", 0)` / `MotionSwitcherClient.ReleaseMode()`).
  This code touched no service at all, so a well-formed `LowCmd` would have
  been fighting the running sport controller.

* **Its Sport API path had no velocity route.** `walk` and `trot` logged a
  warning saying velocity "should come from `/cmd_vel`" and did nothing.
  Nothing in the repository published `/cmd_vel`, and nothing subscribed to it.

* **It had no watchdog, no command freshness check, and no safety state.** A
  single `/go2/gait_command` message would have started motion that nothing
  was arranged to stop.

## What remains in this package

The C++ gait state machine (`src/gait_controller_node.cpp`) and its simulation
launch files are retained: they are joint-trajectory generation for Gazebo, not
an actuation path to the physical robot.

Note that `auto_activate` still defaults to `true`, so constructing the node
configures and activates it immediately and begins publishing STAND joint
trajectories at 50 Hz. That is acceptable only because no hardware bridge in
this repository consumes `/joint_group_effort_controller/joint_trajectory` any
more. Before this package is ever pointed at real hardware again, that default
must be inverted and the resulting path must terminate at the safety arbiter,
not at a motor.
