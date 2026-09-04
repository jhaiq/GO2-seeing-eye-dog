# GO2 Field Notes (measured 2026-09-01 / 2026-09-02)

Everything in this file was **measured on a live Unitree Go2 EDU with a Jetson Orin
NX 16 GB payload**, not inferred from documentation. Anything not measured is marked
NOT VERIFIED. Numbers come from two lab sessions; treat them as one robot, one room,
one floor until a second session reproduces them.

Platform: Jetson Orin NX 16 GB, Ubuntu 22.04.5, kernel 5.15.148-tegra, aarch64,
ROS 2 Humble, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## 1. The failure mode that costs you a session

CycloneDDS is bound to a named network interface. If that name does not exist on the
host, **every Go2 topic is advertised and carries no data**. Discovery still works, so
`ros2 topic list` looks completely healthy and `ros2 topic info` reports publishers.
Only `ros2 topic hz` / `echo` reveals it.

On the Orin NX payload the robot-facing NIC is `enP8p1s0`. A `CYCLONEDDS_URI` inherited
from a desktop control PC (`enp0s31f6` and similar) silently produces this state.

```bash
grep -o 'NetworkInterface name="[^"]*"' "${CYCLONEDDS_URI#file://}"
ip -brief link show enP8p1s0     # must exist, must be UP
```

A detector reading a dead topic is indistinguishable from a detector reading a safe
world. Assert data flow, not topic presence.

## 2. `ros2 topic list` lies after any environment change

A stale `ros2` daemon holds the discovery context from the previous environment. Symptom
is a list containing only `/rosout` and `/parameter_events`. Always:

```bash
ros2 daemon stop     # then re-run
```

`ros2 topic echo` accepts `--no-daemon`; `ros2 topic hz` does **not**.

## 3. Measured topic surface

Full API surface after a clean daemon restart: **109 topics**.

| Topic | Type | Measured rate |
|---|---|---|
| `/lowstate` | `unitree_go/msg/LowState` | 500.0 Hz |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 294.9 Hz |
| `/utlidar/imu` | `sensor_msgs/Imu` | 248.1 Hz |
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | 151.0 Hz |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | 15.4 Hz |
| `/frontvideostream` | compressed video | 28.6 Hz |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | **silent unless the remote is powered on** |

Also verified absent on a stock robot: **no `/tf`, no `map` frame, no `/cmd_vel`
publisher**. `/utlidar/robot_odom` is `header.frame_id = "odom"`,
`child_frame_id = "base_link"`, origin at boot pose. Any node that needs a global frame
has to supply it; do not assume TF exists.

## 4. Commanding the robot without the SDK

`unitree_sdk2py` is **not installed on the payload and is not required**. A hand-rolled
`unitree_api/msg/Request` published to `/api/sport/request` is accepted by the robot:

- `header.identity.id` = any unique int, `header.identity.api_id` = the command id
- `header.lease.id = 0`, `header.policy.priority = 0`, `header.policy.noreply = false`
- `parameter` = a JSON string

Verified api ids: **1001 CheckMode**, **1003 StopMove**, **1008 Move**
(`{"x": <m/s>, "y": <m/s>, "z": <rad/s>}`). The robot replies on `/api/sport/response`
with `status.code = 0` and the matching request id.

`CheckMode` returns `{"form":"0","name":"mcf"}`. **Only ever `SelectMode('mcf')` or
`'ai'`.** `SelectMode('normal')` wedges the robot and needs a power cycle.

**`/api/sport/request` has 9 publishers on a stock robot.** Any preflight asserting that
the topic has exactly one publisher is wrong and will hard-fail on real hardware. Count
*your own* node's publishers instead, and assert the platform total is `>= 1`.

Order of operations that isolates faults: send `StopMove` (1003) first. It is a real,
accepted request that carries zero motion risk, so a failure there means the header
format is wrong, not that the robot refused to move.

Measured envelope from the first commanded-motion runs: commanded `x = 0.15 m/s` for 2 s
produced a peak `|vx|` of 0.159 - 0.161 m/s over two reproducible runs. Fail-closed
latency with a 250 ms lease timeout was 4 `Move` messages before `StopMove` took over,
on hardware and in dry-run alike. Stop *distance* is still **NOT MEASURED**.

Gait envelope (prior sessions): a clean trot needs `vx >= 0.5`; CCW yaw needs
`>= 1.0 rad/s`; combined translation and yaw degrades (this is the stock `mcf` RL policy
ceiling, not a bug in your controller).

## 5. LiDAR extrinsics are pose dependent

Measured from `/utlidar/imu` orientation, 4000 samples per pose, same room, same floor,
minutes apart:

| Pose | `body_height` | roll | pitch |
|---|---|---|---|
| Standing | 0.323 m | **-131.56 deg** | -2.24 deg (sd 0.043) |
| Sitting | 0.072 m | **-108.30 deg** | +0.17 deg (sd 0.030) |

A 23.3 deg roll difference between poses, stable, not noise. Consequence, measured by
replaying both bags through a live ground/drop-off detector at true 15.4 Hz:

| Bag | Sweeps | False alarms |
|---|---|---|
| Standing | 1220 | **445 (36%)** |
| Sitting | 1143 | **0** |

**Any ground-plane, drop-off, or obstacle-height logic calibrated on a sitting or
low-pose capture will false-alarm at normal standing height.** Calibrate against a
standing capture, and make the extrinsics pose-aware rather than raising the threshold
until it goes quiet. The mechanism is consistent with uncompensated pose-dependent
extrinsics but has **NOT** been proven: a pitch-only model does not account for the
observed 0.118 m drop across a 0.367 m range gap, so roll and the ground-plane fit are
likely involved too.

## 6. Clocks are wrong by default

- The **robot's own** message stamps read 2025-10-17 during a 2026-09 session. Robot-side
  clock skew is present and was not corrected.
- The **payload Jetson has no working RTC** (RTC reads 1970-01-01) and NTP is inactive.
  It boots with a stale clock and has rebooted mid-session.

Set the clock before recording anything you intend to use as evidence, and record which
clock a bag's stamps came from. A bag captured before a manual `date -s` is misdated.

## 7. The lab network has no internet egress

No apt, no pip, no git from the robot network. Anything that must be installed on the
payload has to be **staged in advance as aarch64 wheels** and copied over. Plan installs
before the session, not during it.

## 8. Sensors are not guaranteed to be attached

A recent session found **no RealSense D435i and no ReSpeaker on the payload's USB bus**
(`lsusb` showed only the WiFi dongle). Every node that opens a camera or a mic needs a
camera-less / mic-less configuration path that degrades instead of crashing, and the
preflight has to report the absence loudly rather than blocking the whole session.

---

## 9. What this changes for this stack

`docs/hardware_assumptions.md` should be read against this file; where they disagree,
these measurements win.

- **`/go2/safety_alert` consumers** (RiskGraph among them) need a pose to be useful, and
  the only pose source is `/utlidar/robot_odom` in the `odom` frame. There is no `map`
  frame and no `/tf` (section 3), so nothing downstream can transform between frames.
- **The gait controller must not assume `/cmd_vel` has a publisher.** A stock robot has
  none; whichever node owns the safety envelope must be the only one that ever does.
- **Audio and vision hardware are not guaranteed** (section 8). The most recent session
  found no mic array and no depth camera on the payload's USB bus. Nodes that open one
  must degrade loudly rather than run blind.
- **`mcf` mode only** (section 4). `SelectMode('normal')` wedges the robot.
