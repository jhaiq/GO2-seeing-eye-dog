# Architecture

A short orientation. The authoritative documents are:

* `docs/target_runtime_architecture.md` — the architecture, its invariants and
  the reasoning behind them
* `docs/ros_graph.md` — the topic, service and frame reference
* `docs/safety_architecture_audit.md` — adversarial review of the safety design
* `docs/research_system_claims.md` — what may and may not be claimed
* `docs/runtime_graph_audit.md` — what this repository looked like before, and why it changed

## The shape of the system

```
perception  ->  caller confirmation  ->  intent grounding  ->  navigation goal
                                                                     |
                                                                     v
                                                          candidate motion command
                                                                     |
                                                                     v
                                                    DETERMINISTIC SAFETY ARBITER
                                                                     |
                                                              safe command
                                                                     |
                                                                     v
                                                            hardware adapter -> GO2
```

The rule the whole design serves:

> No motion command may reach the physical GO2 without passing through the
> deterministic safety authority.

The safety layer does not observe, score, annotate or recommend. It owns the
actuator output. Everything upstream of it produces *candidates*; only the
arbiter produces commands.

## Packages

| Package | Role |
|---|---|
| `go2_audio_perception` | GCC-PHAT acoustic bearing; optional NeMo ASR |
| `go2_voice_commander` | Whisper transcription and command parsing |
| `go2_perception` | YOLOv8 human detection with depth back-projection |
| `go2_safety_monitor` | Depth-based hazard detection (stairs, drops, narrow passages, proximity) |
| `go2_intent_grounding` | Audio-visual fusion, caller confirmation state machine, goal emission |
| `go2_approach_controller` | **Staged** candidate-motion producer; Nav2 compatibility stamper |
| `go2_safety_arbiter` | **Final authority over all motion** |
| `go2_hardware_bridge` | Adapter contract; dry-run and Unitree Sport API adapters |
| `go2_navigation` | Nav2 parameters and behaviour tree (Stage 2) |
| `go2_gait_controller` | C++ gait state machine, simulation only; its hardware bridge was removed |
| `go2_bringup` | Canonical launch graph and versioned configuration |
| `go2_msgs` | Message definitions |

## Running it

```bash
ros2 launch go2_bringup system_dry_run.launch.py   # no hardware
ros2 launch go2_bringup system.launch.py           # real perception, dry-run actuation
```

`hardware_adapter:=dry_run` is the default everywhere. Selecting a physical
adapter is a deliberate act, and that adapter has never been executed against a
GO2.

## Honesty boundaries

* `/goal_pose` emission is not autonomous approach.
* A safety monitor is not an arbiter unless it owns the actuator output.
* A dry-run execution is not hardware validation.

`docs/research_system_claims.md` states what the evidence supports.
