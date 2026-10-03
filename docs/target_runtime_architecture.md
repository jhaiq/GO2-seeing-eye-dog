# Target runtime architecture

The architecture this repository now implements, the invariants it enforces,
and the reasoning behind the choices that are not obvious.

The single rule everything else serves:

> **No motion command may reach the physical GO2 without passing through the
> deterministic safety authority.**

Not "should not". *Cannot*, enforced by the type system, the launch graph and
the middleware, and demonstrated by runtime tests rather than asserted in prose.

**Bounded by one assumption, stated up front:** a trusted DDS domain. ROS 2
without SROS2 has no authentication, so any process that can reach the domain
and import `go2_msgs` has the same privileges as the arbiter. The mechanisms
here reliably stop a misconfigured controller, a stray `ros2 topic pub`, a
crashed process, a stalled clock and a mistyped parameter. They do not stop an
adversary already on the robot's network, and nothing in-band could. See
`docs/safety_architecture_audit.md`, findings C1 and H1.

---

## The graph

```
  microphone ──> audio_perception_node ──────> /go2/audio/bearing_deg
                                                        │
  microphone ──> voice_commander_node ───────> /go2/voice_command
                                                        │
  RealSense ───> perception_node ────────────> /go2/detected_humans
                                                        │
                                                        v
                                         ┌──────────────────────────┐
                                         │  intent_grounding_node   │
                                         │  fusion + confirmation   │
                                         │  state machine           │
                                         └──────────────────────────┘
                                              │              │
                              /goal_pose ─────┘              └──> /go2/grounding_status
                                    │
                                    v
                   ┌────────────────────────────────┐
                   │  CANDIDATE MOTION PRODUCER     │
                   │  staged: approach_controller   │
                   │  or:     Nav2 + stamper        │
                   └────────────────────────────────┘
                                    │
                          /cmd_vel_candidate
                        geometry_msgs/TwistStamped
                                    │
  RealSense ──> safety_monitor_node ─┼──> /go2/safety_state, /go2/safety_alert
                                    │              │
                                    v              v
             ╔══════════════════════════════════════════════════╗
             ║           safety_arbiter_node                    ║
             ║   *** FINAL AUTHORITY OVER ALL MOTION ***        ║
             ║                                                  ║
             ║   validity -> limits -> hazards -> arbitration   ║
             ║   -> rate limiting -> authorization              ║
             ║                                                  ║
             ║   allow  |  derate  |  stop  |  emergency stop   ║
             ╚══════════════════════════════════════════════════╝
                                    │
                            /cmd_vel_safe
                     go2_msgs/SafeVelocityCommand   <-- custom type
                                    │
                                    v
                   ┌────────────────────────────────┐
                   │     hardware_bridge_node       │
                   │  independent re-validation     │
                   │  independent watchdog          │
                   │  authority-token latch         │
                   └────────────────────────────────┘
                                    │
                     HardwareBridgeInterface
                          ╱              ╲
              DryRunGo2Bridge        UnitreeSportBridge
              (JSONL recorder)       (Sport API, never run
                                      against hardware)
                                             │
                                             v
                                        physical GO2
```

---

## The invariants, and how each is enforced

### Invariant A, candidate and safe commands are separate

The controller publishes `geometry_msgs/TwistStamped` on
`/cmd_vel_candidate`. The bridge subscribes to `go2_msgs/SafeVelocityCommand`
on `/cmd_vel_safe`. They are different topics carrying different types.

**Enforcement is structural, not conventional.** ROS 2 will not connect
publishers and subscribers whose type hashes differ, so a controller, a teleop
node or an ad-hoc `ros2 topic pub` cannot deliver a message to the bridge no
matter how it is remapped.

This turns out to be stronger than merely "the connection is refused". The RMW
refuses to *create* a `Twist` publisher on `/cmd_vel_safe` at all once the
bridge's subscription exists:

```
RCLError: Failed to create publisher: create_publisher() called for existing
topic name rt/cmd_vel_safe with incompatible type geometry_msgs::msg::dds_::Twist_
```

So the mistake fails loudly at construction rather than publishing into the
void. Asserted in
`go2_hardware_bridge/test/test_bridge_node.py::TestNoBypass`.

The custom type carries what a bare `Twist` cannot: the authoring arbiter's
identity token, a monotonic sequence number, the state that authorized the
command, the reason codes behind it, and a hard `valid_until` expiry.

### Invariant B, fail closed

Every path that cannot positively establish permission returns zero velocity.
In `go2_safety_arbiter/core.py` there is no `else: allow`; the default of
every branch is stop.

| Condition | Result | Reason code |
|---|---|---|
| No candidate received | STOP | `WATCHDOG_TIMEOUT` |
| Candidate older than `candidate_max_age_sec` | STOP | `STALE_COMMAND` |
| NaN or infinity in any component | STOP | `INVALID_COMMAND` |
| Zero, negative or far-future timestamp | STOP | `INVALID_TIMESTAMP` |
| Frame id other than `expected_frame_id` | STOP | `INVALID_COMMAND` |
| Hazard information missing or stale | STOP | `SAFETY_CONTEXT_STALE` |
| Stop-class hazard asserted | STOP | `HAZARD_STOP` |
| Unrecognised hazard type | STOP | `HAZARD_STOP` |
| Localization required and absent | STOP | `NO_LOCALIZATION` |
| Emergency stop latched | STOP | `EMERGENCY_STOP` |
| A safety rule raises an exception | STOP | `RULE_EVALUATION_FAILED` |

Two properties are worth stating explicitly because they are what most
"safety layers" get wrong:

**An unrecognised hazard type is a stop, not a pass.** If
`safety_monitor_node` gains a new alert type tomorrow and the arbiter has not
been taught about it, the robot stops. The alternative, treating unknown as
benign, means every future perception improvement is a potential silent
regression in safety.

**A crashing rule is a stop.** `evaluate` catches every exception and converts
it to a zero-velocity decision. A bug in a safety check must never be the
reason a robot keeps moving.

### Invariant C, bounded command lifetime

Three independent timers, deliberately not one:

* `candidate_max_age_sec` (0.30 s) bounds an individual command's age. A very
  old message is refused even if it arrived this instant.
* `watchdog_timeout_sec` (0.50 s at the arbiter) bounds *silence*. A stream of
  individually-fresh commands still stops the robot if it dries up.
* `valid_until`, stamped into every `SafeVelocityCommand` and enforced by the
  bridge without consulting the arbiter.

Nothing anywhere continues the last command. In `core.py` the previous
authorized velocity is used *only* to enforce the acceleration limit, never as
a fallback output. That distinction is the whole point:
"continue last command" is how a robot keeps walking after its planner dies.

Both the arbiter and the bridge run their decisions on **timers, not
callbacks**. A callback-driven design goes quiet when its input stops, and
going quiet is not a stop.

### Invariant D, explicit authority

`safety_arbiter_node` is the only publisher of `SafeVelocityCommand` anywhere
in the workspace. This is enforced at three levels:

1. **Statically**, `scripts/repo_doctor.py` fails CI if any package outside
   `go2_safety_arbiter` constructs such a publisher, or if any launch file
   outside `motion_authority.launch.py` starts a hardware bridge.
2. **In the launch graph**, `motion_authority.launch.py` is the single file
   that defines the actuation path, and every entrypoint includes it unchanged.
3. **At runtime**, the bridge latches the first `authority_token` it accepts
   and refuses commands bearing a different one, so two concurrently running
   arbiters cannot both drive the robot. A token change is accepted only after
   the bridge has been stopped for `authority_handover_quiet_sec`, which lets a
   legitimate restart through while refusing an overlapping duplicate.

### Invariant E, simulation and hardware parity

Everything from perception to `/cmd_vel_safe` is byte-identical in dry-run and
on hardware. Only the adapter behind `HardwareBridgeInterface` differs.
`system_dry_run.launch.py` and `system.launch.py` differ in exactly two launch
arguments.

This is why the end-to-end test is worth something: it exercises the
production decision path, not a simulation of it.

---

## Defence in depth: why the bridge distrusts the arbiter

The bridge re-checks everything the arbiter already checked. That is not
redundancy for its own sake, the failure being defended against is *the
arbiter being wrong, absent, restarted, or duplicated*, and in every one of
those cases the arbiter's own checks are worth nothing.

| Bridge check | Failure it catches |
|---|---|
| Independent watchdog (0.30 s, tighter than the arbiter's) | Arbiter crashed, hung, was SIGKILLed, or lost the network |
| Steady-clock timing, never `/clock` | A stalled simulation clock, which previously froze the watchdog itself |
| Frame and reason-code validation | A malformed or internally inconsistent command |
| Adapter `tick()` hold timeout | The bridge's own loop stalling, on a latching transport |
| `valid_until` expiry | A command that was legitimate when issued and is not now |
| Authority-token latch | Two arbiters running at once |
| Sequence monotonicity | Replayed or reordered commands |
| Limit re-validation | An arbiter that has started emitting over-limit commands |
| State/velocity consistency | A command claiming `STOPPED` while carrying velocity |

The bridge's limit check **refuses** rather than clamping, deliberately. At
the arbiter, an over-limit candidate is an expected condition, planners
over-command, so it is clamped. At the bridge, an over-limit command means
the arbiter itself is malfunctioning, and silently clamping it would hide the
fault behind a second layer of correction.

---

## Nav2: an honest assessment

**Nav2 is available and wired, and it is not the default. Nothing in this
repository has run it successfully.**

### Why it is not the default

Nav2's planner and controller servers require a map, a localization source,
odometry, a TF tree and a laser scan. This repository provides **none** of
them: no AMCL, no SLAM, no map server with a real map, no odometry publisher,
no robot description, no TF broadcaster, no lidar driver.

The pre-existing `nav2_params.yaml` additionally referenced a behaviour-tree
plugin library that does not exist in Humble, so `bt_navigator` could not
complete `on_configure` and the lifecycle manager could not bring the stack
up, independently of everything above. See `docs/runtime_graph_audit.md` §3.

Shipping a Nav2 bring-up that fails at lifecycle configure, and describing it
as navigation, would have been worse than shipping nothing.

### The staged alternative, and why it is not a permanent hack

`planner:=staged` (the default) runs `go2_approach_controller`: a
straight-line approach with a turn-in-place threshold and a slowdown radius.
It performs **no planning and no obstacle avoidance**. Its obstacle response
is the arbiter's hazard policy, which is a stop, not a go-around.

What makes it *staged* rather than throwaway is that its runtime contract is
identical to Nav2's:

| | Consumes | Produces |
|---|---|---|
| `approach_controller_node` | `/goal_pose` `PoseStamped` | `/cmd_vel_candidate` `TwistStamped` |
| Nav2 `controller_server` | `/goal_pose` `PoseStamped` | `cmd_vel` `Twist` → stamper → `/cmd_vel_candidate` |

Switching is a launch argument, `planner:=nav2`. The safety and actuation half
of the stack is untouched by that switch, which is the property that makes
this a stage rather than a rewrite waiting to happen.

### How Nav2 attaches when the infrastructure exists

`system.launch.py planner:=nav2` starts `nav2_bringup` and remaps its
`cmd_vel` to `/cmd_vel_candidate_unstamped`, where `candidate_stamper_node`
converts it to the stamped contract.

Nav2 believes it is driving the robot. It is driving the arbiter's inlet.
Removing that remapping would not connect Nav2 to the hardware, it would
connect Nav2 to nothing, because the bridge consumes a different type.

**The honest limitation of the unstamped inlet:** the stamper applies the
*reception* time, so the age the arbiter computes excludes producer-side
latency. It is a lower bound on true age. This is recorded in the node's
docstring, in `config/safety.yaml`, and in a startup warning the arbiter
logs whenever `accept_unstamped_candidate` is true. It still catches the
dominant failure mode, the producer stopping, but it is weaker than a real
producer timestamp, and it should be turned off once Nav2 publishes
`TwistStamped` natively.

### What Stage 2 requires

1. A robot description publishing `base_link` and the sensor frames.
2. An odometry source (GO2 state → `nav_msgs/Odometry` + `odom -> base_link`).
3. A map and a localization source, or SLAM.
4. `/scan`, or the RealSense point cloud enabled and configured as a costmap
   source.
5. `nav2_params.yaml` repaired: drop the nonexistent plugin library, fix the
   `recoveries_server` → `behavior_server` rename, port the behaviour tree to
   BT.CPP v3 or upgrade the distribution.
6. `require_localization: true` in `safety.yaml`, which is false today only
   because nothing publishes `/go2/localization_valid` and enabling it would
   correctly but unhelpfully stop the robot forever.

---

## Safety states

| State | Meaning | Authorized velocity | Recovery |
|---|---|---|---|
| `SAFE_TO_MOVE` | Full authority | Candidate, clamped and rate-limited |, |
| `DEGRADED` | A slowdown-class hazard holds | Scaled by `degraded_scale` (0.35) | Automatic when the hazard clears |
| `STOPPED` | Motion forbidden | Zero | Automatic when the cause clears |
| `EMERGENCY_STOP` | Latched | Zero | **Explicit service call only** |

Emergency stop is asymmetric on purpose: a topic (`/go2/estop`) may **engage**
it, and only a service (`~/release_estop`) may **release** it. A stray
message, a replayed bag, or any process on the graph publishing `false` must
not be able to re-enable motion.

An e-stop never clears because the triggering condition went away. An e-stop
that self-releases is not an e-stop.

Recovery from any stop is acceleration-limited like any other command, so
releasing an e-stop produces a ramp, not a lurch.

---

## The hardware adapter contract

`HardwareBridgeInterface` requires `connect`, `send_velocity`, `send_zero`,
`emergency_stop`, `health` and `shutdown`. Adapters are **transport only**,
they perform no safety decisions, because by the time a velocity reaches one
it has been authorized by the arbiter and re-checked by the bridge. An adapter
that silently modified a command would break the audit chain.

Two rules that are easy to get wrong:

* `send_zero()` must work in **every** lifecycle state, including before
  `connect()` and after `shutdown()`. A stop must never fail because of
  lifecycle bookkeeping.
* `send_velocity` returns `False` on a failed transmission rather than
  raising, so the bridge can count failures and stop after
  `max_consecutive_transmit_failures`. A `True` return means *transmitted*,
  never *the robot moved*, no adapter may claim physical confirmation it does
  not have.
* `tick()` is called on every control cycle whether or not a command arrived.
  Adapters whose transport **latches**, where the robot keeps executing the
  last command until told otherwise, must use it to enforce their own
  command-hold timeout.

  That last rule exists because of a specific finding.
  `DryRunGo2Bridge` is a pure recorder with no latch, so **a dry-run test
  cannot detect a latching hazard by construction**. Sport API `Move` does
  latch: a SIGKILLed bridge left `Move(0.4, 0, 0)` as the last thing the robot
  heard, and it walks away. The hook turns bridge death into command starvation
  at the robot, which the robot can act on.

`build_adapter` has **no fallback** from `unitree_sport` to `dry_run`. An
operator who asked for hardware and silently got a simulator would believe the
robot was under command when it was not.

### `UnitreeSportBridge`

Implemented, **never executed against a GO2 by this repository**. It uses the
Sport API (`Move`, api_id 1008) rather than `/lowcmd`, because:

* `Move` takes exactly the body velocity `(vx, vy, wz)` this architecture
  produces;
* Unitree's onboard controller keeps the robot balanced, so a bug here
  produces a bad velocity rather than a fall;
* `/lowcmd` requires a CRC over the packed command struct and requires the
  onboard sport service to be released first, neither of which the removed
  `hw_bridge.py` did.

`send_zero` issues `StopMove` (1003) rather than `Move(0,0,0)`, so the
controller halts rather than actively tracking a commanded zero.
`emergency_stop` issues `StopMove` only and latches; while latched it re-asserts
`StopMove` at most once per second. Damp (1001) drops the robot and is refused by
`_publish` by construction (2026-10-03, motion-authority change).

Its `connect()` reports whether the sport service has a subscriber on the
request topic, and says in its own `health().detail` that subscriber presence
is not proof the robot will move. That is a weak liveness signal reported as
one.

---

## Configuration

Safety-critical constants live in versioned YAML, not in Python source, so
changing the robot's motion envelope is a reviewable diff.

| File | Contents |
|---|---|
| `go2_bringup/config/safety.yaml` | Arbiter and bridge limits, timing, requirement policy |
| `go2_bringup/config/fusion.yaml` | Bearing gate, weights, threshold derivation, interaction timeouts |
| `go2_bringup/config/navigation.yaml` | Controller gains, tolerances, goal lifetime |

Every parameter documents its unit, meaning, source, whether it has been
validated on hardware, and whether the default is safe.

**Every velocity and acceleration limit is marked
`hardware_validated: false`.** None has been measured on a GO2. They are
conservative desk values chosen to be slower than the platform's documented
capability, and `repo_doctor.py` fails CI if that disclaimer is removed.

There is deliberately **no `safety_enabled` parameter**. Individual
requirements can be relaxed, and doing so is reported in `SafetyStatus` so it
cannot happen silently, but no parameter combination turns the arbiter into a
pass-through.

**Every limit is additionally bounded above by a hard ceiling compiled into
`limits.py`.** This was not true originally, and the audit demonstrated the
consequence: `-p max_vx:=100.0` was accepted and the arbiter dutifully clamped
to 100 m/s, while `-p watchdog_timeout_sec:=100000` let motion continue 27
seconds after the producer died. A layer that enforces whatever it is told to
enforce is not a safety layer.

The ceilings are deliberately not configurable. The value of a bound that fails
the node at startup is precisely that it cannot be raised from a command line
or a params file; raising one is a code change and a review. Asserted in
`test_arbiter_core.py::test_hazard_requirement_can_be_relaxed_but_limits_still_apply`
and `test_arbiter_audit_regressions.py::TestH2ParametersAreBounded`.

---

## Launch architecture

| File | Purpose |
|---|---|
| `system.launch.py` | **Canonical entrypoint.** `perception:=real\|none`, `planner:=staged\|nav2`, `hardware_adapter:=dry_run\|unitree_sport` |
| `system_dry_run.launch.py` | No hardware at all; the same decision stack |
| `motion_authority.launch.py` | **The architecture.** Arbiter + bridge, included unchanged by every entrypoint |
| `go2_full.launch.py` | Deprecated; raises with a pointer to the replacement |

`motion_authority.launch.py` exports `get_motion_authority_nodes()` so launch
tests can inspect the exact node list, remappings and parameters without
starting a process. `go2_bringup/test/test_launch_wiring.py` uses it to assert
that the bridge names no candidate topic under any alias.

---

## What this architecture does not do

Stated here so it is not inferred from the diagrams:

* **It does not avoid obstacles.** The staged controller drives straight at
  the goal. The arbiter stops for hazards; it does not route around them.
* **It does not verify the caller's identity.** Any voice that produces a
  recognised phrase is a valid request. `evaluation/eval_speaker_id.py`
  remains an offline tool with no runtime consumer.
* **It does not localize.** There is no map and no odometry, so `/goal_pose`
  is only meaningful for as long as the TF chain that produced it holds.
* **It has never moved a physical robot.** Every actuation recorded by this
  repository went to `DryRunGo2Bridge`.

These are listed as gaps, not as future work that is nearly done. See
`docs/research_system_claims.md` for what may and may not be claimed.
