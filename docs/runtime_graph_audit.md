# Runtime graph audit — the state of this repository before the upgrade

Audited at commit `55d1d40`, before any change described in
`docs/END_TO_END_UPGRADE_REPORT.md`. Every statement here comes from
executable code, package manifests and the launch graph. README and
documentation claims were read only to locate assertions to check, and no
finding rests on them. Findings were produced independently twice — once
directly and once by a separate reviewer with no access to the first pass —
and reconciled.

`build/`, `install/` and `log/` were excluded as stale colcon artefacts.

---

## The ten questions, answered

### 1. Who publishes `/goal_pose`?

`go2_intent_grounding/go2_intent_grounding/intent_grounding_node.py:65`
declares the publisher; it publishes at `:148` inside `humans_callback`.
It is the only publisher in the repository.

### 2. Who consumes `/goal_pose`?

**Nothing in this repository.** A search over every tracked source finds no
subscription.

Externally, `nav2_bt_navigator` would subscribe to it, and
`go2_bringup/launch/go2_full.launch.py:175` does include
`nav2_bringup/navigation_launch.py`. But see question 3: that stack could not
come up.

### 3. Is Nav2 currently launched anywhere?

It is *referenced* in a launch file and it does not *work*. Four independent
faults, any one of which is fatal:

**a. A behaviour-tree plugin library that does not exist.**
`go2_navigation/config/nav2_params.yaml:31` lists
`nav2_reactive_fallback_bt_node` among `bt_navigator`'s plugin libraries. No
`libnav2_reactive_fallback_bt_node.so` exists in `/opt/ros/humble/lib` — all
thirteen other entries do. `ReactiveFallback` is a BT.CPP built-in, not a nav2
plugin library. `bt_navigator` calls `registerFromPlugin` on each name, so
`on_configure` fails and `lifecycle_manager` cannot bring the stack up. This
alone blocks Nav2 regardless of maps or TF.

**b. The repository's own behaviour tree was silently ignored.**
`go2_full.launch.py:188` passes `default_nav_to_pose_bt_xml` to the include.
`nav2_bringup/launch/navigation_launch.py` on Humble does not declare that
argument, and launch silently sets unmatched arguments as launch
configurations rather than erroring. `nav2_params.yaml` does not set it
either. So `bt_navigator` loaded nav2's stock tree while a preflight check at
`go2_full.launch.py:46-52` verified the repository's own tree file existed on
disk. The check passed; the file was never used.

**c. The behaviour tree targets the wrong BT.CPP major version.**
`navigate_to_pose_recovery.xml:1` declares `BTCPP_format="4"`. Humble ships
`behaviortree_cpp_v3`.

**d. The recovery configuration block is dead.**
`nav2_params.yaml:183-203` is keyed `recoveries_server` with plugins
`nav2_recoveries/Spin|BackUp|Wait`. Humble's node is `behavior_server` and the
plugin classes are `nav2_behaviors/...`. The block is never read. It also
contains a duplicate `use_sim_time` key (`:185` and `:199`).

**Runtime dependencies `nav2_params.yaml` implies that the repository does not
provide:**

| Requirement | Implied at | Provided? |
|---|---|---|
| `map` frame, `map -> odom` TF | `:12`, `:150` | No. No AMCL, no SLAM, no static publisher. Only a TF *listener* exists. |
| `/map` `OccupancyGrid` | `:156-158` | No. `map_server`'s `yaml_filename` is `""` and `navigation_launch.py` does not start it. |
| `odom` frame, `odom -> base_link` TF | `:108`, `:196` | No. |
| `/odom` `nav_msgs/Odometry` | `:14` | No. |
| `base_link` frame | `:13`, `:109`, `:151` | No. Nothing publishes robot TF on the hardware path. |
| `/scan` `LaserScan` | `:126-131`, `:162-168` | No lidar driver is launched. |
| `/camera/depth/color/points` | `:132-137` | No. `go2_full.launch.py` never enables `pointcloud.enable`. |
| A `cmd_vel` consumer | implied | No. See question 5. |

**Verdict:** the launch process starts; Nav2 lifecycle bring-up fails at
`bt_navigator` configure; even without that fault, planner and costmap
activation would block on the missing map, odometry, TF and scan.

### 4. Who publishes `/cmd_vel`?

**Nobody.** An exhaustive search over Python, C++, YAML, XML and launch files
finds zero publishers, zero subscribers and zero remappings. The only textual
occurrences are three comments in `go2_gait_controller/scripts/hw_bridge.py`
(`:134`, `:135`, `:137`) describing an integration that was never written.

Had Nav2 come up, `nav2_velocity_smoother` would have published `/cmd_vel` —
into a topic with no subscriber.

### 5. Who consumes `/cmd_vel`?

**Nobody.** The velocity output terminates at an unsubscribed topic.

### 6. Does any safety component sit inline between controller output and hardware?

**No.** `go2_safety_monitor/safety_monitor_node.py` analyses depth images and
publishes `/go2/safety_alert` and `/go2/safety_state`. Nothing in the
repository subscribes to either topic, and no Nav2 behaviour-tree node or
plugin consumes them. The node's own docstring (`:10`) says it publishes
"alerts that the Nav2 behavior tree responds to"; that response does not
exist. `docs/architecture.md:31` concedes the point.

It was a **monitor** in the strict sense: it observed and annotated. It had no
authority over anything.

### 7. Can any component bypass safety and directly command the GO2?

**Yes — every actuation path bypassed it, because there was nothing inline to
bypass.** Two paths existed:

**Path A, Sport API.** `ros2 topic pub /go2/gait_command` →
`hw_bridge.py:75` → `_handle_sport_command` `:125` → `unitree_api/msg/Request`
on `/api/sport/request` `:99`. Handled `stand` (1010), `idle` (1009) and
`estop` (1001). `walk` and `trot` produced no actuation at all (`:132-138`).

**Path B, low-level joints.** `gait_controller_node.cpp` control loop (50 Hz,
`:84-87`) → `JointTrajectory` on
`/joint_group_effort_controller/joint_trajectory` → `hw_bridge.py:160` →
`unitree_go/msg/LowCmd` on `/lowcmd` `:202`.

Path B could be triggered by merely running the node: `auto_activate` defaults
to `true` (`gait_controller_node.cpp:31`) and the constructor calls
`on_configure` and `on_activate` directly (`:35-45`), bypassing the lifecycle
manager, entering `STAND` and starting the 50 Hz publisher. Two of the three
launch files leave that default on.

Neither path was reachable from `go2_full.launch.py`, whose
`LaunchDescription` (`:204-216`) contains no gait controller and no bridge.
And neither was reachable from its own launch file either — see question 8.

### 8. What does `hw_bridge.py` actually do today?

Four things, three of which are broken.

**It is not installed as a runnable executable.**
`go2_gait_controller/CMakeLists.txt:68-70` installs `scripts/` to
`share/${PROJECT_NAME}`. `ros2 run` and `launch_ros.actions.Node(executable=)`
resolve executables from `lib/${PROJECT_NAME}`; `share/` is never searched.
There is no `install(PROGRAMS ...)`. `install(DIRECTORY)` without
`USE_SOURCE_PERMISSIONS` also strips the executable bit.
`gait_hw_launch.py:62-64` would raise `ExecutableNotFound` at
launch-description build time, aborting the entire launch including the gait
controller beside it. **Both actuation paths above were therefore unreachable
through any launch file as committed.**

**Its `/lowcmd` output would have been rejected by the robot.**
`unitree_go/msg/LowCmd` requires `head = (0xFE, 0xEF)`, `level_flag = 0xFF`,
and a `crc` recomputed over the packed struct before every publish — Unitree's
own `go2_stand_example.cpp` does exactly this. `hw_bridge.py:181-202`
default-initialises the message (`head` zero, `level_flag` zero, `crc` zero)
and fills only twelve of twenty `motor_cmd` entries. There is no CRC
computation anywhere in the repository (`grep -i crc`: zero hits). It also
published at the 50 Hz trajectory rate rather than the 500 Hz the low-level
loop expects.

**It never released the onboard sport service.** Low-level control requires
stopping the GO2's motion-control service first. `_setup_lowlevel` (`:147-169`)
creates a subscriber and a publisher and touches no service. A well-formed
`LowCmd` would have been fighting the running sport controller.

**It failed silently when the SDK was absent.** `main`'s
`except (ImportError, ValueError): pass` (`:221-222`) means the process exits
with status 0 when `unitree_api` or `unitree_go` is missing.

**It had no watchdog, no freshness check and no safety state.** One
`/go2/gait_command` message started motion that nothing was arranged to stop.

### 9. What physical Unitree interface is currently intended?

Both, inconsistently: `sport_api` mode (high-level, `/api/sport/request`) was
the default, with `lowlevel` mode (`/lowcmd`) as an option. The Sport API path
was the only one that could ever have worked, and it had no velocity route.

The upgraded stack commits to the **Sport API** and says why in
`go2_hardware_bridge/go2_hardware_bridge/unitree_sport.py`.

### 10. Does the repository already contain code that should be reused?

Yes, and it was reused rather than rewritten:

* **`go2_safety_monitor`'s depth analysis** — stair, drop, narrow-passage and
  proximity detection. Sound perception work. It was promoted from an ignored
  observer into the arbiter's hazard input.
* **`go2_perception`'s YOLO detection and back-projection**, and
  **`go2_audio_perception`'s GCC-PHAT bearing estimation** — kept unchanged.
* **`go2_msgs`** — extended, not replaced.
* **`go2_intent_grounding`'s confirmation-frames concept** — kept; the scoring
  underneath it was replaced.
* **`go2_gait_controller`'s C++ gait state machine** — retained for
  simulation. Only its hardware bridge was removed.

---

## Node inventory (pre-upgrade)

Legend: "HW deps" are module-level imports that fail without the listed
driver, model or device.

### `audio_perception_node`
* **Package/exec** `go2_audio_perception` / `audio_perception_node`
* **Source** `go2_audio_perception/go2_audio_perception/audio_perception_node.py`
* **Subscribes** none
* **Publishes** `/go2/audio/bearing_deg` `Float32`; `/go2/audio/sound_source`
  `Vector3Stamped`; `/go2/audio/mono_raw` `Int16MultiArray`
* **Services/actions** none · **Timer** 10 Hz → `process_audio` · **TF** none
* **Parameters** `sample_rate`, `chunk_duration_ms`, `n_mics`,
  `publish_rate_hz`, `energy_threshold`
* **Launched by** `go2_full.launch.py:79`; `nemo_integration.launch.py`
* **HW deps** `pyaudio` + a live 4-channel input device, opened in
  `__init__` — the constructor throws with no microphone
* **Tests** `test_gcc_phat.py`, pure-function only (`rclpy`/`pyaudio` mocked)

### `nemo_asr_node`
* **Package/exec** `go2_audio_perception` / `nemo_asr_node`
* **Subscribes** `/go2/audio/mono_raw` · **Publishes** `/go2/audio/transcript`
  — **no subscriber anywhere**
* **Launched by** `nemo_integration.launch.py` only; not in `go2_full`
* **HW deps** `nemo.collections.asr`, `torch`; downloads a checkpoint at construct
* **Tests** none
* **Defect** a worker thread calls `rclpy.spin_once` while `main` also spins
  the node — a concurrent-spin hazard

### `perception_node`
* **Package/exec** `go2_perception` / `perception_node`
* **Subscribes** `/camera/color/image_raw`, `/camera/depth/image_rect_raw`
  (BEST_EFFORT), `/camera/color/camera_info`
* **Publishes** `/go2/detected_humans` `DetectedHumanArray`;
  `/go2/perception/visualization`
* **HW deps** `ultralytics` (weights loaded in `__init__`), `cv2`, `cv_bridge`
* **Tests** none; `go2_perception/test` does not exist
* **Defect (topic namespace)** `realsense2_camera` 4.58.3 publishes under
  `/camera/camera/...`; these subscriptions would receive nothing as launched
* **Defect (intrinsics)** consumes *unaligned* depth while back-projecting
  with *color* intrinsics, biasing every 3D pose by the depth-to-color
  extrinsic — even though the launch file enables `align_depth`

### `safety_monitor_node`
* **Package/exec** `go2_safety_monitor` / `safety_monitor_node`
* **Subscribes** `/camera/depth/image_rect_raw`, `/camera/depth/camera_info`
  (same namespace defect)
* **Publishes** `/go2/safety_alert`, `/go2/safety_state`,
  `/go2/safety/visualization` — **no in-repo subscriber for any**
* **Tests** none
* **Defect** claims Nav2 responds to its alerts; nothing does

### `intent_grounding_node`
* **Package/exec** `go2_intent_grounding` / `intent_grounding_node`
* **Subscribes** `/go2/audio/bearing_deg`, `/go2/detected_humans`,
  `/go2/voice_command`
* **Publishes** `/go2/confirmed_target`, `/goal_pose`, `/go2/grounding_state`
* **TF** listener only; `camera_color_optical_frame -> map` at `:143`, failure
  caught and the goal silently dropped. No TF broadcaster in the repository
  supplies that chain.
* **Tests** `test_fusion.py`, pure-function only. The state machine and TF path
  were entirely untested.
* **Defects** see the dedicated section below

### `voice_commander_node`
* **Package/exec** `go2_voice_commander` / `voice_commander_node`
* **Publishes** `/go2/voice_command`, `/go2/voice_raw_transcript`
* **HW deps** `whisper` (model loaded in `__init__`), `pyaudio`
* **Tests** `test_voice_commander.py`, pure-function only
* **Note** publishes any parsed command from any speaker; no identity gate

### `go2_gait_controller` (C++ lifecycle node)
* **Package/exec** `go2_gait_controller` / `go2_gait_controller_node`
  (correctly installed to `lib/`)
* **Subscribes** `/go2/gait_command`, `/joint_states`
* **Publishes** `/joint_group_effort_controller/joint_trajectory`
* **Timer** 50 Hz, created in `on_activate` · **TF** none
* **Tests** none; `CMakeLists.txt` registers only `ament_lint_auto`
* **Defect** `auto_activate` defaults true and the constructor bypasses the
  lifecycle manager, immediately entering `STAND` with a live publisher

### `go2_hw_bridge`
* **Package** `go2_gait_controller`; **not installed as an executable**
* See question 8. Removed in this upgrade; see
  `go2_gait_controller/DEPRECATED.md`.

---

## Multimodal defects found in `intent_grounding_node`

Three, all reproduced analytically and then at node level. Regression tests
for each live in `go2_intent_grounding/test/`.

### A goal could be published with no voice request

`voice_callback` (`:78-88`) was the only consumer of `/go2/voice_command`, and
it only ever *reset* state — `target_locked = False`,
`consecutive_confirmations = 0`. It set no enabling flag and gated nothing.

`humans_callback` incremented the confirmation count purely on detections and
published a goal at `:128` when the count reached five. Both conditions are
reachable from the cold-start state with no voice message ever received.

Since audio was optional too, the **complete** precondition for the robot to
set off across a room was: five consecutive detection frames with YOLO
confidence ≥ 0.9286, plus a successful TF lookup. No microphone, no wake word,
no speaker. A person who had not spoken to the robot could be approached by it.

The secondary effect: because `target_locked` was cleared *only* by
`voice_callback`, after the first autonomous goal no further goals were
published until a voice command arrived. The voice channel was a re-arm, never
an arm.

### The two bearings were compared across incompatible frames

`perception_node` produces poses in `camera_color_optical_frame`
(REP-105: +x right, +y down, +z forward), so
`math.atan2(pose.position.x, pose.position.z)` at `:104` is
**clockwise-positive**.

`audio_perception_node` asserts the REP-103 body convention for its bearing —
it publishes the matching unit vector as `x=cos(az), y=sin(az)` in
`base_link` — which is **counter-clockwise-positive**.

`compute_audio_score` took a bare scalar difference between them. A caller
15° to the robot's **left** reads as −15° visually and +15° acoustically: 30°
of apparent disagreement for a perfectly agreeing pair, beyond the 25° gate.
The correct caller was rejected and their mirror image was accepted.

Neither existing test caught it, because both fed hand-chosen angles rather
than either producer's convention.

Two further frame problems remain partly open and are recorded in
`docs/research_system_claims.md`: the `Float32` bearing message carries no
header and therefore no frame id, and the camera-to-body extrinsic is
unmodelled (now an explicit `camera_yaw_offset_deg` parameter defaulting to
zero, rather than an implicit assumption).

### The advertised tolerance was not the effective one

With `score = 0.4·audio + 0.6·visual`, `audio = 1 − Δ/25°` and a 0.65
threshold, a 0.9-confidence caller stopped being confirmable at

```
0.4·(1 − Δ/25) + 0.6·0.9 ≥ 0.65   ⟹   Δ ≤ 18.125°
```

matching the reported 18–19° transition and the 0.940 / 0.540 endpoint scores
exactly. The effective tolerance was a function of the detector's confidence,
which is not something a parameter named `bearing_tolerance_deg` can honestly
describe.

The same arithmetic made the **visual-only fallback** require
`0.65 / 0.7 = 0.9286` detection confidence — against a detector configured to
accept 0.5. The fallback almost never fell back.

And it made **disagreeing audio worse than absent audio**: with audio fresh
but inconsistent the ceiling was `0.6 · visual = 0.6 < 0.65`, so confirmation
was *impossible at any confidence*, while merely losing the microphone gave
`0.7 · visual`. Combined with the sign error, the audio channel actively
opposed the correct answer.

### Silent, unbounded `SEARCHING`

Nothing ran on a timer. State was published only from `humans_callback`, so
with no detections arriving there was no output at all. A request could leave
the system in `SEARCHING` indefinitely with no way for a blind user to learn
that nothing was happening or why.

---

## Test-suite classification (pre-upgrade)

Four files, 28 tests, **all pure-function**.

| File | Tests | Class |
|---|---|---|
| `go2_audio_perception/test/test_gcc_phat.py` | 6 | Pure. `rclpy`, `pyaudio`, `scipy`, `std_msgs`, `geometry_msgs` all `MagicMock`ed. |
| `go2_intent_grounding/test/test_fusion.py` | 10 | Pure. |
| `go2_voice_commander/test/test_voice_commander.py` | 4 | Pure. `rclpy`/`pyaudio`/`whisper` mocked. |
| `evaluation/tests/test_eer.py` | 8 | Pure. No ROS. |

`rclpy.init` appeared in **zero** test files. There was no node test, no
`launch_testing` test, no state-machine test, no message round-trip test and no
integration test. Untested entirely: the confirmation state machine and its TF
path, `perception_node`, `safety_monitor_node`, the C++ gait state machine, and
`hw_bridge.py`'s `LowCmd` construction — that is, every actuation-relevant and
every safety-relevant code path.

The mocking had a second cost discovered during the upgrade: because those
files installed mocks into `sys.modules` unconditionally, they corrupted the
real message types for any ROS test running later in the same session, making
collection order decide whether a suite passed. Fixed in `conftest.py`'s
`mock_missing_modules`.

---

## Edge status summary

| Edge | Status |
|---|---|
| audio input → acoustic bearing | IMPLEMENTED_AND_EXECUTABLE (needs a 4-channel device) |
| audio input → speech → voice command | IMPLEMENTED_AND_EXECUTABLE (needs a microphone) |
| speech → speaker verification | **MISSING** (`evaluation/eval_speaker_id.py` is an offline CLI; no node imports it) |
| camera → visual detection | IMPLEMENTED_AND_EXECUTABLE (topic-namespace defect; needs a RealSense) |
| bearing + detection → fusion | IMPLEMENTED_BUT_DEFECTIVE (frame sign error; tolerance mismatch) |
| fusion → caller confirmation | IMPLEMENTED_BUT_DEFECTIVE (no request required; silent deadlock) |
| confirmation → intent grounding | IMPLEMENTED_AND_EXECUTABLE |
| intent grounding → `/goal_pose` | IMPLEMENTED_AND_EXECUTABLE (needs a TF chain nothing publishes) |
| `/goal_pose` → planner/controller | **DOCUMENTATION_ONLY** (Nav2 cannot complete bring-up) |
| planner/controller → `/cmd_vel` | **MISSING** (no publisher, no subscriber) |
| `/cmd_vel` → safety | **MISSING** (nothing inline) |
| safety → actuation | **MISSING** (safety monitor owns no output) |
| actuation → GO2 | **PLACEHOLDER** (not installed as an executable; malformed `LowCmd`; no CRC; sport service never released) |
| depth → hazard alerts | IMPLEMENTED_AND_EXECUTABLE (published to nobody) |

The chain was **complete from microphone to `/goal_pose`, and absent from
`/goal_pose` to the robot.**

What replaced it is described in `docs/target_runtime_architecture.md`.
