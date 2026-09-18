# ROS graph

The runtime topic graph. `docs/target_runtime_architecture.md` explains why it
is shaped this way; this file is the reference.

## Motion path (the one that matters)

```
/goal_pose  ──> approach_controller_node ──> /cmd_vel_candidate
                                                    │
                                                    v
                                          safety_arbiter_node
                                                    │
                                            /cmd_vel_safe
                                                    │
                                                    v
                                          hardware_bridge_node ──> adapter ──> GO2
```

`/cmd_vel_candidate` carries `geometry_msgs/TwistStamped`.
`/cmd_vel_safe` carries `go2_msgs/SafeVelocityCommand`, a type no planner or
controller produces. That difference is what prevents a bypass; see Invariant A.

## Topics

### Perception

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/go2/audio/bearing_deg` | `std_msgs/Float32` | `audio_perception_node` | `intent_grounding_node` |
| `/go2/audio/sound_source` | `geometry_msgs/Vector3Stamped` | `audio_perception_node` | — |
| `/go2/audio/mono_raw` | `std_msgs/Int16MultiArray` | `audio_perception_node` | `nemo_asr_node` (optional) |
| `/go2/voice_command` | `std_msgs/String` | `voice_commander_node` | `intent_grounding_node` |
| `/go2/detected_humans` | `go2_msgs/DetectedHumanArray` | `perception_node` | `intent_grounding_node` |
| `/go2/safety_state` | `std_msgs/String` | `safety_monitor_node` | **`safety_arbiter_node`** |
| `/go2/safety_alert` | `go2_msgs/SafetyAlert` | `safety_monitor_node` | **`safety_arbiter_node`** |

The last two rows are the change that turned the safety monitor from an
ignored observer into an input with consequences. Before this upgrade nothing
subscribed to either topic.

### Intent and navigation

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/go2/confirmed_target` | `go2_msgs/ConfirmedTarget` | `intent_grounding_node` | — (for logging and analysis) |
| `/go2/grounding_status` | `go2_msgs/GroundingStatus` | `intent_grounding_node` | feedback layer (not yet implemented) |
| `/go2/grounding_state` | `std_msgs/String` | `intent_grounding_node` | — (legacy, retained for tooling) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `intent_grounding_node` | `approach_controller_node`, or Nav2 |
| `/go2/cancel_goal` | `std_msgs/String` | `intent_grounding_node` | `approach_controller_node` |
| `/go2/controller/status` | `std_msgs/String` | `approach_controller_node` | — |

`/go2/cancel_goal` is new. Without it, a user saying "stop" moved the
grounding node into `STOPPED` while the controller kept driving toward the
last goal it was given.

### Motion authority

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/cmd_vel_candidate` | `geometry_msgs/TwistStamped` | controller | `safety_arbiter_node` |
| `/cmd_vel_candidate_unstamped` | `geometry_msgs/Twist` | Nav2 (via remap) | `safety_arbiter_node` |
| **`/cmd_vel_safe`** | **`go2_msgs/SafeVelocityCommand`** | **`safety_arbiter_node` ONLY** | **`hardware_bridge_node` ONLY** |
| `/go2/estop` | `std_msgs/Bool` | any (engage only) | `safety_arbiter_node` |
| `/go2/localization_valid` | `std_msgs/Bool` | localization source (none yet) | `safety_arbiter_node` |

### Observability

| Topic | Type | Publisher |
|---|---|---|
| `/go2/safety/status` | `go2_msgs/SafetyStatus` | `safety_arbiter_node` |
| `/go2/bridge/status` | `go2_msgs/BridgeStatus` | `hardware_bridge_node` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | arbiter and bridge |

## Services

| Service | Type | Server | Effect |
|---|---|---|---|
| `/safety_arbiter_node/engage_estop` | `std_srvs/Trigger` | arbiter | Latch emergency stop |
| `/safety_arbiter_node/release_estop` | `std_srvs/Trigger` | arbiter | Release the latch. **The only way to release it.** |
| `/hardware_bridge_node/emergency_stop` | `std_srvs/Trigger` | bridge | Latch at the bridge; cleared only by restarting it |

Engaging an e-stop is possible over a topic. Releasing one is not: a stray
message or a replayed bag must not be able to re-enable motion.

## QoS

| Kind | Profile | Why |
|---|---|---|
| Control (`/cmd_vel_candidate`, `/cmd_vel_safe`) | `RELIABLE`, `VOLATILE`, `KEEP_LAST`, depth 1 | Always the newest command; never a replayed backlog of stale velocities |
| Status (`/go2/safety/status`, `/go2/bridge/status`, `/go2/grounding_status`) | `RELIABLE`, `TRANSIENT_LOCAL`, depth 1 | A late-joining diagnostic tool sees current state immediately |
| Sensor (camera topics) | `BEST_EFFORT`, depth 1 | Standard for high-rate image streams |

Control topics are deliberately `VOLATILE`. A `TRANSIENT_LOCAL` control topic
would let a late-joining bridge receive a stale, possibly nonzero, command from
before it started.

## Frames

| Frame | Convention | Notes |
|---|---|---|
| `map` | REP-105 | Goal frame. **No localization source exists yet.** |
| `odom` | REP-105 | Required by Nav2. Nothing publishes it. |
| `base_link` | REP-103: +x forward, +y left, +z up, yaw CCW-positive | Velocity command frame |
| `camera_color_optical_frame` | REP-105: +x right, +y down, +z forward | Detection frame |

The conversion between the last two is explicit in
`go2_intent_grounding/bearings.py`. It was previously implicit and inverted;
see `docs/runtime_graph_audit.md`.

## Inspecting a running system

```bash
ros2 topic echo /go2/safety/status        # what the arbiter is deciding, and why
ros2 topic echo /go2/bridge/status        # what reached the actuator
ros2 topic echo /go2/grounding_status     # what the interaction is doing
ros2 topic hz   /cmd_vel_safe             # the arbiter should publish at 20 Hz always
ros2 topic info /cmd_vel_safe --verbose   # MUST show exactly one publisher
```

That last command is the quickest check that the architecture is intact. More
than one publisher on `/cmd_vel_safe` is a defect.
