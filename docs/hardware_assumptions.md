# Hardware assumptions

Everything this repository assumes about the physical robot, and how much of
it has been checked. **None of it has been verified on a GO2 by this
repository.**

## Platform

* Unitree GO2 EDU with an onboard Linux compute module, ROS 2 Humble.
* Audio requires GO2 EDU or Pro. The Air has no microphone hardware.

## Actuation interface

**Assumed:** the Unitree Sport API, over `unitree_api/msg/Request` published to
`/api/sport/request`, with the onboard sport service running and subscribed.

| API id | Use |
|---|---|
| 1008 `Move` | Body velocity `(vx, vy, wz)` — the normal command path |
| 1003 `StopMove` | Zero velocity. Preferred over `Move(0,0,0)`: it halts the controller rather than making it track a commanded zero. |
| 1001 `Damp` | Emergency stop. The strongest stop reachable over the Sport API without cutting power. |

**Deliberately not assumed:** `/lowcmd` low-level control. It would require
this repository to own balance and gait generation for a quadruped, plus a CRC
over the packed command struct and an explicit release of the onboard sport
service. See `go2_gait_controller/DEPRECATED.md` for what went wrong when a
previous implementation attempted it without either.

**Unverified:** whether the GO2 accepts these requests as constructed, how it
behaves at the commanded velocities, and what the real end-to-end latency is.

## Velocity envelope

Every limit in `go2_bringup/config/safety.yaml` is a **conservative desk
default**, chosen to be slower than the platform's documented capability, and
marked `hardware_validated: false`.

| Parameter | Value | Basis |
|---|---|---|
| `max_vx` | 0.4 m/s | Guess. A deliberate derate for a robot moving near someone who cannot see it. |
| `max_vy` | 0.2 m/s | Guess |
| `max_wz` | 0.6 rad/s | Guess (~34°/s) |
| `max_accel_linear` | 0.5 m/s² | Guess. At 20 Hz this is 0.025 m/s per tick. |
| `max_accel_angular` | 1.0 rad/s² | Guess |

**What a hardware session must establish before these can be called validated:**
actual achievable velocity and acceleration; stopping distance from `max_vx`
under `StopMove` and under `Damp`; whether the commanded velocity is tracked or
merely requested; and behaviour at the low end, where a quadruped may refuse to
move at all rather than walking slowly.

## Timing

Watchdog periods and command lifetimes were chosen as multiples of the control
period, not measured.

| Parameter | Value |
|---|---|
| Arbiter rate | 20 Hz |
| Bridge rate | 50 Hz |
| `candidate_max_age_sec` | 0.30 s |
| Arbiter `watchdog_timeout_sec` | 0.50 s |
| Bridge `watchdog_timeout_sec` | 0.30 s |

The bridge's watchdog is deliberately tighter than the arbiter's, so that a
dead arbiter is caught by the bridge rather than waited on.

**Unverified:** real DDS latency on the GO2's onboard computer, over its
network, under load. If it exceeds `candidate_max_age_sec`, every command will
be refused as stale and the robot will simply never move — a safe failure, but
one to recognise rather than debug blindly.

## Sensors

**Microphone array.** Four channels, 5 cm linear spacing, both hardcoded in
`audio_perception_node.py`. The mapping from GCC-PHAT's `tau` sign to
left/right depends on the physical microphone ordering, which is not documented
in code. The node asserts REP-103 (positive = left).

*If that assertion is wrong, the fusion association gate will systematically
reject the correct caller* — the same class of failure that was just fixed one
layer downstream. A bench test with a known source direction is owed. See
`docs/debugging.md`.

**RealSense D435i or similar.** The camera-to-`base_link` extrinsic is
**unmodelled**. `camera_yaw_offset_deg` in `config/fusion.yaml` is an explicit
placeholder defaulting to 0.0, assuming a boresighted camera. The correct value
must come from TF once a robot description exists.

Two further known issues:

* `realsense2_camera` 4.58.x publishes under `/camera/camera/...`, while the
  perception nodes subscribe to `/camera/...`.
* `perception_node` consumes *unaligned* depth while back-projecting with
  *color* intrinsics, biasing every 3D pose by the depth-to-color extrinsic,
  even when the launch file enables `align_depth`.

## Frames and localization

`base_link` follows REP-103 (x forward, y left, z up, yaw CCW-positive).
Camera optical frames follow REP-105 (x right, y down, z forward). The
conversion is explicit in `go2_intent_grounding/bearings.py`.

**There is no robot description, no odometry source, no map and no
localization.** `require_localization` is `false` in `safety.yaml` for exactly
this reason: enabling it with nothing publishing `/go2/localization_valid`
would correctly, but unhelpfully, stop the robot permanently.

This is also why Nav2 is not the default planner. See
`docs/target_runtime_architecture.md`.

## Network

DDS discovery, multicast and interface behaviour are environment-dependent and
have not been characterised on the robot's network. Note that the bridge's
authority-token latch assumes the arbiter and the bridge can see each other; a
partitioned graph presents as `BRIDGE_WATCHDOG_TIMEOUT`, which stops the robot.

## Before any hardware session

1. A physical emergency stop within reach. The software e-stop is not a
   substitute for cutting power.
2. Space in every direction: the staged controller does not avoid obstacles.
3. `ros2 topic echo /go2/bridge/status` open, confirming `dry_run: false` and
   watching `last_transmitted`.
4. `ros2 topic info /cmd_vel_safe --verbose` showing exactly one publisher.
5. Limits in `safety.yaml` reduced further than the defaults for the first run,
   not raised.
