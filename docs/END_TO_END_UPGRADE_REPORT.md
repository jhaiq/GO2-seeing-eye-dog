# End-to-end upgrade report

What this repository was, what it is now, and what remains untrue about it.

---

## 1. What architecture existed before?

A complete perception front end connected to nothing.

Microphone → GCC-PHAT bearing. Microphone → Whisper → command string. Camera →
YOLOv8 → 3D person poses. All three into a fusion node, which published a
`PoseStamped` on `/goal_pose`.

And there it stopped. `/goal_pose` had **no consumer in the repository**.

The launch file included `nav2_bringup`, but that stack could not complete
lifecycle bring-up: `nav2_params.yaml` listed `nav2_reactive_fallback_bt_node`,
a behaviour-tree plugin library that does not exist in ROS 2 Humble, so
`bt_navigator` failed `on_configure`. The repository's own behaviour tree was
never loaded either — the launch passed `default_nav_to_pose_bt_xml` to a
launch file that does not declare that argument, and launch silently ignores
unmatched arguments. A preflight check verified the tree file existed on disk
while nothing used it.

Nothing published `/cmd_vel`. Nothing subscribed to `/cmd_vel`.

The safety monitor analysed depth images and published alerts that **no node in
the repository subscribed to**. Its docstring said Nav2's behaviour tree
responded to them. It did not.

Two actuation paths existed in `go2_gait_controller`, and neither was reachable
from any launch file: `hw_bridge.py` was installed to `share/` rather than
`lib/`, so `ros2 run` and `launch_ros` could not find it, and
`install(DIRECTORY)` had stripped its executable bit.

Full detail: [`docs/runtime_graph_audit.md`](runtime_graph_audit.md).

## 2. What was missing?

Everything between the navigation goal and the robot.

* No controller consuming `/goal_pose`.
* No velocity command anywhere in the system.
* No safety component inline between any controller and any actuator.
* No working hardware bridge — and the one that existed would have been
  rejected by the robot: it built a `unitree_go/msg/LowCmd` with a zero frame
  header, `level_flag` unset, and **no CRC**, which Unitree's own reference
  implementation recomputes before every publish. It also never released the
  onboard sport service, so a well-formed message would have been fighting the
  running controller.
* No node-level tests. The suite was 28 pure-function tests. `rclpy.init`
  appeared in zero test files. Every actuation-relevant and safety-relevant
  code path was untested.

## 3. What architecture exists now?

```
perception → caller confirmation → intent grounding → /goal_pose
                                                          ↓
                                              candidate controller
                                                          ↓
                                              /cmd_vel_candidate
                                            geometry_msgs/TwistStamped
                                                          ↓
                                    ╔═══════════════════════════════╗
                                    ║      safety_arbiter_node      ║
                                    ║   FINAL MOTION AUTHORITY      ║
                                    ╚═══════════════════════════════╝
                                                          ↓
                                                 /cmd_vel_safe
                                          go2_msgs/SafeVelocityCommand
                                                          ↓
                                              hardware_bridge_node
                                                          ↓
                                    DryRunGo2Bridge | UnitreeSportBridge
```

Three new packages: `go2_safety_arbiter`, `go2_hardware_bridge`,
`go2_approach_controller`. One canonical entrypoint,
`ros2 launch go2_bringup system.launch.py`, whose actuation half is defined in
exactly one file that every variant includes unchanged.

Full detail: [`docs/target_runtime_architecture.md`](target_runtime_architecture.md).

## 4. What node has final motion authority?

**`safety_arbiter_node`**, from `go2_safety_arbiter`.

It is the only publisher of `go2_msgs/SafeVelocityCommand` in the workspace,
enforced three ways: statically by `scripts/repo_doctor.py`, which fails CI if
any other package constructs such a publisher; in the launch graph, where
`motion_authority.launch.py` is the only file that starts an actuator; and at
runtime, where the bridge latches an authority token and refuses commands
bearing a different one.

## 5. Can Nav2 or raw controller output reach hardware without safety?

**No**, and the reason is structural rather than conventional.

The bridge subscribes to `go2_msgs/SafeVelocityCommand`. A controller publishes
`geometry_msgs/TwistStamped`. ROS 2 will not connect publishers and subscribers
whose type hashes differ.

The guarantee is stronger than "the connection is refused". The RMW refuses to
*create* a `Twist` publisher on `/cmd_vel_safe` at all:

```
RCLError: Failed to create publisher: create_publisher() called for existing
topic name rt/cmd_vel_safe with incompatible type geometry_msgs::msg::dds_::Twist_
```

So the mistake fails loudly at construction rather than publishing into the
void. Verified against a live bridge in
`test_bridge_node.py::TestNoBypass`, and independently by the adversarial audit
(86 messages, zero adapter records).

**Bounded by one assumption:** a trusted DDS domain. Any process that can reach
the domain and construct the correct type can publish it. See §11.

## 6. What happens if the safety node dies?

The robot stops, without being told.

The bridge runs its own watchdog at 50 Hz on a **monotonic clock**, tighter
(0.30 s) than the arbiter's own (0.50 s), so the bridge stops first rather than
waiting on a process that may be gone. It does not need a notification, a
shutdown message, or a healthy arbiter to do this.

Verified for the crash case — no shutdown message, no warning — in
`test_end_to_end.py::test_killing_the_arbiter_stops_the_robot`, which kills the
arbiter mid-motion while the controller keeps commanding.

On a graceful stop the arbiter additionally publishes explicit zero-velocity
`STOPPED` commands before dying, and a SIGTERM now runs that path rather than
raising out of `spin`
(`test_bridge_node.py::TestProcessLevelShutdown`, a real subprocess).

## 7. What happens if commands become stale?

Zero velocity, by three independent mechanisms:

* **Age.** A candidate older than `candidate_max_age_sec` (0.30 s) is refused
  with `STALE_COMMAND`.
* **Silence.** No candidate within `watchdog_timeout_sec` gives
  `WATCHDOG_TIMEOUT`. Both the arbiter and the bridge run on timers, not
  callbacks, precisely so that silence produces a stop rather than more
  silence.
* **Expiry.** Every `SafeVelocityCommand` carries `valid_until`, enforced by
  the bridge without consulting the arbiter.

Nothing continues the last command. The previous authorized velocity is used
only to enforce the acceleration limit, never as a fallback output —
`test_arbiter_core.py::test_last_command_is_never_carried_forward`.

## 8. Is the physical GO2 bridge real or still dry-run?

**The default is dry-run, and no code in this repository has ever moved a
physical robot.**

`UnitreeSportBridge` is fully implemented against the Sport API and has **never
been executed against a GO2**. It uses `Move` (1008) for velocity, `StopMove`
(1003) for zero, and `StopMove` + `Damp` (1001) for emergency stop.

Choosing it is deliberate: `hardware_adapter:=unitree_sport`, with no fallback.
Asking for hardware without the Unitree SDK is a hard failure, not a silent
downgrade to a simulator — an operator who believed the robot was under command
when it was not would be in a worse position than one whose launch failed.

## 9. What fusion bugs were confirmed?

Both reported defects reproduced exactly. A third, more serious one was found
during the audit.

**Caller movement (confirmed).** The reported ~18.1° breakpoint was real. With
`score = 0.4·audio + 0.6·visual`, `audio = 1 − Δ/25°` and a 0.65 threshold, a
0.9-confidence caller stopped being confirmable at

```
0.4·(1 − Δ/25) + 0.6·0.9 ≥ 0.65  ⟹  Δ ≤ 18.125°
```

matching the observed 0.940 and 0.540 endpoints. The effective tolerance was a
function of the *detector's confidence*, which a parameter named
`bearing_tolerance_deg` cannot honestly describe.

**Visual-only fallback (confirmed).** `visual × 0.7 ≥ 0.65` required detection
confidence ≥ **0.928571…**, against a detector configured to accept 0.5. The
fallback almost never fell back.

**Frame sign inversion (found here, and the worst of the three).**
`perception_node` produces poses in the camera optical frame (+x right), so
`atan2(x, z)` is clockwise-positive. `audio_perception_node` asserts the
REP-103 body convention, counter-clockwise-positive. The two were compared with
a bare subtraction. A caller 15° to the robot's **left** read as −15° visually
and +15° acoustically: **30° of apparent disagreement for a perfectly agreeing
pair**, beyond the 25° gate. The correct caller was rejected and their mirror
image was accepted.

Neither existing test caught it, because both fed hand-chosen angles rather
than either producer's convention.

**A fourth, unreported:** a goal could be published with **no voice request ever
received**. The voice callback only *reset* state; it never armed anything. The
complete precondition for the robot to set off across a room was five detection
frames above 0.9286 confidence. No microphone, no wake word, no speaker.

## 10. What was fixed?

### Architecture

* Built the entire goal → controller → arbiter → bridge → actuator chain.
* Removed the old bypass: `hw_bridge.py` and `gait_hw_launch.py` deleted,
  with `go2_gait_controller/DEPRECATED.md` recording why. `repo_doctor.py`
  fails CI if they return.
* Deprecated `go2_full.launch.py`, which now raises with a pointer to the
  replacement rather than starting a graph that could not work.
* Moved every safety-critical constant into versioned YAML with unit, meaning,
  source, and hardware-validation status per parameter.

### Fusion

* Replaced additive scoring with **visual confidence modulated by acoustic
  corroboration**, separating association (a hard gate at `bearing_gate_deg`)
  from confidence. The advertised tolerance is now the effective one, and it no
  longer moves with detector confidence — verified by bisecting the actual
  acceptance boundary at four confidence levels.
* Threshold **derived** from a stated contract, not tuned: a 0.75-confidence
  caller must be confirmable anywhere inside the gate and with no microphone.
  Visual-only now needs 0.518 confidence rather than 0.929.
* Made the frame conversion explicit in `bearings.py`, with the sign inversion
  fixed and the missing camera extrinsic named as a parameter rather than
  assumed silently.
* Fixed an ordering defect neither report mentioned: *disagreeing* audio used
  to score worse than *absent* audio, making confirmation impossible at any
  confidence while the microphone disagreed. `FusionParams` now rejects any
  configuration that would reintroduce it.
* Added a real state machine: a request is required, requests time out with
  `CONFIRMATION_TIMEOUT`, a moved target reports `TARGET_MOVED`, and status is
  published on a timer so "nothing is happening" is itself reported.
* Wired "stop" through to goal cancellation. Previously the grounding node
  entered `STOPPED` while the controller kept driving.

### Safety, after the adversarial audit

The audit found the original claim **false**: five paths delivered non-zero
velocity to the adapter without arbiter authorization. Three of the five
critical findings needed **no attacker at all**.

| Finding | Fix |
|---|---|
| A stalled `/clock` froze every watchdog (14.9 s at 0.4 m/s measured) | Watchdogs moved to a monotonic `STEADY_TIME` clock |
| SIGKILL left the GO2 walking on a latched Sport API `Move` | `tick()` added to the adapter contract; the Unitree adapter now issues `StopMove` if a command is not renewed |
| A `std_msgs/String` "CLEAR" cancelled a real `EMERGENCY_STOP` | Channels separated, most-restrictive-wins, stop-class hazards latch |
| Every limit unbounded above (`max_vx:=100` accepted) | Hard ceilings compiled into `limits.py`, not configurable |
| Authority handover was dead code; one packet bricked motion permanently | Quiet timer decoupled from `_stop`; implausible sequence numbers refused |
| Malformed commands accepted (201/201) | Frame, validity horizon and reason-code vocabulary validated |
| An exception in the timer killed the arbiter | `_tick` publishes a stop on any exception |
| The physical adapter could never report a transmit failure | Subscriber-count gating; `connect()` failure is now fatal |
| `require_initial_inputs` documented but never implemented | Implemented, with `NOT_INITIALIZED` distinct from `SAFETY_CONTEXT_STALE` |

Also corrected: `docs/target_runtime_architecture.md` had claimed "the most
permissive reachable configuration still clamps, rate-limits, and enforces both
watchdogs." It clamped at 100 m/s and enforced watchdogs at 10⁵ s. **That
sentence was false**; both the sentence and the code were fixed.

Full detail: [`docs/safety_architecture_audit.md`](safety_architecture_audit.md).

### Test infrastructure

Fixed a defect that made the previous suite unreliable in a way nobody had
noticed: the pure-function tests installed `MagicMock` into `sys.modules`
unconditionally, corrupting the real message types for any ROS test running
later in the same session. **Collection order decided whether the suite
passed.** Mocks are now installed only for modules genuinely absent.

## 11. What remains unvalidated?

**No code in this repository has moved a physical robot.** Every actuation it
has performed went to a recording dry-run adapter.

* `UnitreeSportBridge`: implemented, never executed against a GO2.
* Every velocity, acceleration and timing constant: conservative desk values,
  marked `hardware_validated: false`, with CI failing if that marking is
  removed.
* The acoustic bearing's left/right sign convention. `audio_perception_node`
  asserts REP-103, but whether GCC-PHAT's `tau` sign actually maps that way
  depends on undocumented microphone ordering. **If it is inverted, the fusion
  gate will systematically reject the correct caller** — the same class of
  failure just fixed, one layer upstream. A bench test is owed.
* The camera-to-body extrinsic (`camera_yaw_offset_deg`, defaulting to 0.0).
* Nav2. Wired and never successfully launched; it needs a map, localization,
  odometry, TF and a laser scan, none of which this repository provides.

**Open audit findings.** The system has **no authentication**. Any process that
can reach the DDS domain and import `go2_msgs` can publish a
`SafeVelocityCommand` or call `release_estop`; the audit demonstrated both. The
custom message type is an anti-footgun, not an access control. Closing this
requires SROS2 with an access-control policy, plus a physical e-stop the
software cannot clear. That is a deployment change and it is not done.

There is also a residual found while writing the fix for the handover deadlock:
an impostor publishing at the same rate displaces the legitimate arbiter in the
depth-1 queue, which stops the robot, which eventually satisfies the quiet
condition and permits takeover. Denial of service converts into takeover. It is
recorded rather than papered over, and the regression test is deliberately
scoped to the guarantee that does hold.

## 12. Exact test totals

**298 tests, all passing.**

| Tests | File | Kind |
|---:|---|---|
| 59 | `go2_safety_arbiter/test/test_arbiter_core.py` | Pure, deterministic safety |
| 46 | `go2_intent_grounding/test/test_fusion.py` | Pure, fusion regressions |
| 30 | `go2_intent_grounding/test/test_grounding_state.py` | Pure, state machine |
| 24 | `go2_hardware_bridge/test/test_bridge_node.py` | ROS node + subprocess |
| 22 | `go2_safety_arbiter/test/test_arbiter_audit_regressions.py` | Audit regressions |
| 21 | `go2_approach_controller/test/test_approach.py` | Pure + ROS node |
| 18 | `go2_intent_grounding/test/test_intent_node.py` | ROS node |
| 18 | `go2_bringup/test/test_launch_wiring.py` | Launch graph + config |
| 16 | `go2_safety_arbiter/test/test_arbiter_node.py` | ROS node |
| 13 | `go2_hardware_bridge/test/test_bridge_audit_regressions.py` | Audit regressions |
| 10 | `go2_audio_perception/test/test_gcc_phat.py` | Pure |
| 9 | `go2_bringup/test/test_end_to_end.py` | Full-chain integration |
| 8 | `evaluation/tests/test_eer.py` | Pure |
| 4 | `go2_voice_commander/test/test_voice_commander.py` | Pure |
| **298** | | |

Before this upgrade: **28 tests, all pure-function**, with `rclpy.init`
appearing in zero test files.

By kind now: 157 pure-function, 106 ROS node-level, 18 launch/config, 9
end-to-end integration, 1 subprocess-level, 7 spanning categories.

The required coverage from the brief:

| Required | Where |
|---|---|
| 1. Intent → goal | `test_intent_node.py::test_confirmed_caller_produces_exactly_one_goal` |
| 2. No confirmation → no goal | `test_intent_node.py::test_no_request_means_no_goal` |
| 3. Goal → candidate motion | `test_approach.py::TestControllerNode` |
| 4. Candidate → safe command | `test_bridge_node.py::TestNominalActuation` |
| 5. Over-limit candidate | `test_arbiter_node.py::TestOverLimitCandidate` |
| 6. Stale candidate | `test_bridge_node.py::TestStaleAndWatchdog` |
| 7. Safety node failure | `test_end_to_end.py::test_killing_the_arbiter_stops_the_robot` |
| 8. Malformed command | `test_bridge_node.py::TestMalformedCommands`, `test_bridge_audit_regressions.py::TestM1...` |
| 9. Emergency stop | `test_arbiter_node.py::TestEmergencyStop`, `test_bridge_node.py::TestEmergencyStop` |
| 10. No bypass | `test_bridge_node.py::TestNoBypass` + `test_launch_wiring.py` |
| 11. Moving caller | `test_fusion.py::TestCallerMotionBoundary`, `test_intent_node.py::TestCallerMotionAtNodeLevel` |
| 12. Visual-only fallback | `test_fusion.py::TestVisualOnlyFallback` |
| 13. Lifecycle startup | `test_bridge_node.py::TestLifecycle`, `TestM4InitialInputsAreRequired` |
| 14. Shutdown safety | `test_arbiter_node.py::TestShutdown`, `test_bridge_node.py::TestProcessLevelShutdown` |

## 13. Exact reproduction command

```bash
cd ~/workspace/GO2-seeing-eye-dog
./scripts/reproduce.sh
```

It removes `build/`, `install/` and `log/`; unsets any inherited ROS overlay
(a previously-built `go2_msgs` in another workspace would otherwise shadow the
one being tested); picks a `ROS_DOMAIN_ID` from the shell PID; **refuses to
start if any ROS node is already visible on that domain**; builds messages then
packages; verifies every executable and launch file is actually installed; runs
ruff; then runs the suite.

The domain check matters. These tests assert on what does and does not reach an
actuator, and a foreign graph can make such an assertion pass or fail for the
wrong reason.

## 14. What is now honestly claimable

**Claimable:**

> A safety-authoritative architecture for assistive quadruped navigation, in
> which a deterministic fail-closed arbiter holds exclusive authority over
> actuation on a trusted control network. Authority is enforced structurally —
> the actuator's input is a message type no planner can produce — rather than
> by convention, and is demonstrated by runtime integration tests showing
> planner output bounded, hazard-stopped, and cut off entirely when the safety
> process dies. An independent adversarial audit is reported alongside it,
> including the findings that remain open.

**Not claimable:** anything about hardware. Nav2-based navigation. Obstacle
avoidance (the system stops for hazards; it does not route around them).
Speaker verification. A *secure* architecture — it is deterministic and
fail-closed on a trusted domain, and an audit demonstrated takeover from an
unprivileged process.

The full breakdown, with the evidence for each claim and the exact wording that
would be false, is in
[`docs/research_system_claims.md`](research_system_claims.md).

Reporting the open audit findings alongside the closed ones is not optional. An
architecture paper that presents only the attacks its design survives is
advertising, not evidence.
