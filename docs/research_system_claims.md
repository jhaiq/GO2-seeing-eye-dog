# Research system claims

What this repository may honestly claim, and what it may not.

The categories are exclusive and the boundaries are strict. A claim moves up a
category only when executable evidence supports it, and no claim moves into
"implemented and tested" on the strength of a dry-run.

Three phrasings are banned outright, because each was either present in this
repository's history or is the obvious next overstatement:

* Publishing `/goal_pose` is **not** autonomous approach. It is publishing a
  message.
* A component that observes and annotates is **not** an arbiter. An arbiter
  owns the actuator output.
* A dry-run execution is **not** hardware validation, however complete the
  decision path it exercised.

---

## Implemented and tested

Executable, covered by tests that assert the behaviour, and reproducible via
`./scripts/reproduce.sh`. **298 tests pass.**

### Safety authority

* A deterministic, fail-closed safety arbiter holds final authority over every
  motion command in the system, **on a trusted DDS domain**. The property is
  enforced by the message type, the launch graph and the middleware, and it is
  demonstrated at runtime, not argued for in prose.
  *Evidence:* `go2_safety_arbiter/test/test_arbiter_core.py` (59 tests),
  `test_arbiter_node.py` (17 tests),
  `test_arbiter_audit_regressions.py` (22 tests).

  The trust qualifier is load-bearing and is not boilerplate. An independent
  adversarial audit demonstrated that any process on the domain can construct
  and publish a `SafeVelocityCommand`, and drove the dry-run actuator with a
  70-line script and no arbiter running. See
  `docs/safety_architecture_audit.md` findings C1 and H1. Closing that requires
  SROS2, which is a deployment change and is listed under "Planned".

* No motion command reaches the actuator without arbiter authorization. A
  controller publishing on the candidate topic at 5 m/s moves nothing; a
  `geometry_msgs/Twist` publisher on the safe topic cannot even be
  constructed, because the RMW refuses the type conflict.
  *Evidence:* `go2_hardware_bridge/test/test_bridge_node.py::TestNoBypass`.

* The system fails closed on: absent commands, stale commands, non-finite
  values, invalid timestamps, future timestamps, wrong frames, missing hazard
  context, stale hazard context, unrecognised hazard types, missing
  localization when required, and a safety rule raising an exception.
  *Evidence:* `test_arbiter_core.py::TestFailClosedUnderFaults`, which sweeps a
  grid of adverse inputs and asserts every one authorizes exactly zero.

* Killing the safety process while the robot is moving and the controller is
  still commanding stops the robot, via the bridge's independent watchdog.
  *Evidence:*
  `go2_bringup/test/test_end_to_end.py::test_killing_the_arbiter_stops_the_robot`.

* The last command is never carried forward. Absence of a command produces
  zero, not a repeat.
  *Evidence:*
  `test_arbiter_core.py::test_last_command_is_never_carried_forward`.

* Emergency stop latches and is releasable only by an explicit service call; a
  topic may engage it but not release it.
  *Evidence:* `test_arbiter_node.py::TestEmergencyStop`.

* Velocity limits, acceleration limits and derating under slowdown-class
  hazards are enforced. An over-limit candidate is clamped at the arbiter and
  refused at the bridge.
  *Evidence:* `test_arbiter_core.py::TestLimits`,
  `test_bridge_node.py::test_over_limit_command_is_refused_not_clamped`.

* Two arbiters cannot both drive the robot; a replayed sequence number is
  refused.
  *Evidence:* `test_bridge_node.py::TestAuthority`.

* No parameter combination disables the arbiter, and every limit is bounded
  above by a hard ceiling that cannot be raised from a parameter file or a
  command line.
  *Evidence:*
  `test_arbiter_core.py::test_hazard_requirement_can_be_relaxed_but_limits_still_apply`,
  `test_arbiter_audit_regressions.py::TestH2ParametersAreBounded`.

* Watchdogs run on a monotonic clock and therefore survive a stalled `/clock`.
  *Evidence:* `test_arbiter_audit_regressions.py::TestC3WatchdogsUseSteadyTime`.

* A hazard asserted on one channel cannot be cancelled by a lower-evidence
  message on the other, and stop-class hazards latch across dropped frames.
  *Evidence:*
  `test_arbiter_audit_regressions.py::TestC5HazardChannelsAreResolvedBySeverity`.

* An exception anywhere in the arbiter's control loop publishes a stop rather
  than killing the process.
  *Evidence:* `test_arbiter_audit_regressions.py::TestM2ATickExceptionStillStops`.

* A crashed arbiter can be restarted and reclaim authority; a malformed
  sequence number cannot permanently immobilise the robot.
  *Evidence:*
  `test_bridge_audit_regressions.py::TestH3AuthorityHandoverIsReachable`.

* A SIGTERM runs the teardown path, leaving the actuator explicitly stopped.
  *Evidence:*
  `test_bridge_node.py::TestProcessLevelShutdown` (a real subprocess, since the
  bug only exists at process level).

### End-to-end decision path

* The repository owns a complete motion decision path: a spoken request plus
  consistent perception produces an authorized velocity at the actuator,
  through the real fusion, the real confirmation state machine, the real
  intent grounding, the real controller and the real arbiter.
  *Evidence:* `go2_bringup/test/test_end_to_end.py` (9 tests).

* Candidate command is not actuator authority: the authorized value is
  observably different from, and bounded below, the candidate.
  *Evidence:*
  `test_end_to_end.py::test_the_authorized_command_differs_from_the_candidate`.

* A hazard stops the robot mid-approach while the goal, the controller and the
  candidate stream are all unchanged.
  *Evidence:* `test_end_to_end.py::test_a_hazard_stops_the_robot_mid_approach`.

### Multimodal caller confirmation

* A navigation goal requires a voice request. Perfect detections with no
  request produce nothing.
  *Evidence:* `test_intent_node.py::test_no_request_means_no_goal`,
  `test_grounding_state.py::test_detections_alone_never_confirm`.

* A confirmed caller produces exactly one goal, in the `map` frame, at the
  correct position.
  *Evidence:*
  `test_intent_node.py::test_confirmed_caller_produces_exactly_one_goal`.

* The advertised bearing tolerance is the effective one, and it does not move
  with detector confidence.
  *Evidence:*
  `test_fusion.py::test_the_boundary_does_not_move_with_detector_confidence`,
  which bisects the actual acceptance boundary for four confidence levels.

* The visual-only fallback is reachable by detections the perception node
  actually produces (confidence ≥ 0.518, versus 0.929 before).
  *Evidence:* `test_fusion.py::TestVisualOnlyFallback`,
  `test_intent_node.py::test_confirmation_succeeds_with_no_microphone_at_all`.

* Visual and acoustic bearings are compared in a single frame and sign
  convention. A caller on the robot's left is confirmed; their mirror image is
  rejected.
  *Evidence:* `test_fusion.py::TestBearingFrameConversion`,
  `test_intent_node.py::TestBearingConventionAtNodeLevel`.

* Every interaction terminates with a machine-readable outcome. A request that
  finds nobody times out and says so rather than searching silently forever.
  *Evidence:* `test_grounding_state.py::TestTimeout`,
  `test_intent_node.py::TestNoSilentDeadlock`.

### Observability

* Safety state, candidate age, last safe command, intervention count and
  reason, and e-stop state are published on `/go2/safety/status` and
  `/diagnostics`.
* Bridge state, adapter name, dry-run flag, connection state, command age,
  transmit status and rejection reasons are published on `/go2/bridge/status`.
* Fusion scores, bearing delta, confirmation progress and interaction state
  are published on `/go2/grounding_status`, on a timer, independent of whether
  any data is arriving.
  *Evidence:* the `TestObservability` classes in the arbiter, bridge and
  intent-node test files.

---

## Implemented but not physically validated

Written, reviewed, and never executed against a GO2. **These must not be
described as working.**

* **`UnitreeSportBridge`.** The Sport API adapter is complete and follows the
  documented request format. No line of it has run against a robot. Whether
  the GO2 accepts these requests, how it behaves at the commanded velocities,
  and what the real latency is, are all unknown.

* **Every velocity and acceleration limit.** `max_vx = 0.4 m/s`,
  `max_vy = 0.2 m/s`, `max_wz = 0.6 rad/s`, `max_accel_linear = 0.5 m/s²`,
  `max_accel_angular = 1.0 rad/s²`. Conservative desk values, chosen to be
  slower than the platform's documented capability. Marked
  `hardware_validated: false` in `config/safety.yaml`, and CI fails if that
  marking is removed.

* **Every timing constant.** Watchdog periods, freshness windows and command
  lifetimes were chosen as multiples of the control period. They have not been
  checked against real DDS latency on the GO2's onboard computer, over its
  actual network, under load.

* **Emergency-stop behaviour on hardware.** `StopMove` + `Damp` is the
  strongest stop reachable over the Sport API. Its actual stopping distance
  from any speed is unmeasured.

* **The camera-to-body extrinsic.** `camera_yaw_offset_deg` defaults to 0.0,
  assuming a boresighted camera. The real value must come from TF once a robot
  description exists. This is an approximation with a named parameter, which is
  a different thing from the sign inversion it replaced, but it is still
  unvalidated.

* **The acoustic bearing convention.** `audio_perception_node` asserts the
  REP-103 body convention. Whether `tau`'s sign actually maps to left-positive
  depends on the physical microphone ordering, which is undocumented in code.
  Confirming this requires a bench test with a known source direction. If it is
  inverted, the fusion association gate will systematically reject the correct
  caller, the same class of failure that was just fixed, one layer upstream.

* **The `0.8 m` goal tolerance.** Chosen as a social distance for an assistive
  robot approaching a person. Not evaluated with any user.

---

## Simulation and dry-run only

Executed, and only ever against `DryRunGo2Bridge`.

* Every actuation recorded by this repository. The JSONL logs in the
  integration tests are records of what *would* have been transmitted.
* The full perception-to-actuation chain, driven by synthetic detections and
  bearings rather than a camera and a microphone array.
* The staged controller's approach behaviour, exercised against a static TF
  chain rather than a moving robot with real odometry.

**No robot has been moved by this code.**

---

## Planned

Not implemented. Listed so the gap is visible, not to imply imminence.

* Nav2 integration (Stage 2), which requires a robot description, an odometry
  source, a map or SLAM, a laser scan or configured point cloud, and repairs to
  `nav2_params.yaml`. See `docs/target_runtime_architecture.md`.
* Runtime speaker verification, so a request is attributable to an authorized
  user rather than to any voice in the room.
* An audio feedback layer that speaks `GroundingStatus` to the user. The
  states and reasons exist and are published; nothing renders them.
* Obstacle avoidance, as distinct from hazard stopping.
* **SROS2 across the motion path**, to close the audit's open findings: an
  access-control policy permitting publication on `/cmd_vel_safe` and calls to
  `release_estop` only from the arbiter's enclave, plus a physical emergency
  stop that software cannot clear.
* An out-of-process supervisor (systemd `WatchdogSec`) issuing `StopMove` and
  `Damp` if the bridge process dies, as belt and braces over the adapter's own
  command-hold timeout.
* Hardware bring-up of `UnitreeSportBridge`, with the limits re-derived from a
  logged session.

---

## Unsupported

Claims that would be false. Recorded because several were previously made, or
are the obvious next overstatement.

* **"An end-to-end assistive navigation system running on a GO2."**
  Nothing has run on a GO2.
* **"Nav2-based autonomous navigation" (on the robot).** Nav2 now configures,
  plans and completes goals in the kinematic simulator (`go2_sim`,
  `closed_loop_trials`), with LiDAR localization and the real Unitree adapter
  in the loop. It has not run on a GO2. The default planner is still the
  straight-line controller.
* **"The robot navigates to the caller."** It drives at the caller. There is
  no map, no plan and no obstacle avoidance.
* **"Safety-validated velocity limits."** No limit has been measured on the
  platform.
* **"Speaker-verified caller identification."** There is no runtime speaker
  verification. Any voice producing a recognised phrase is a valid request.
* **"Validated audio-visual fusion."** The fusion model is coherent,
  documented and tested against its own stated semantics. It has not been
  evaluated against ground truth with real callers, and its equal error rate is
  unmeasured. The published EER in `evaluation/results/` is computed from
  `np.random` embeddings and characterises the metric implementation, not this
  system.
* **"Verified obstacle avoidance."** With `planner:=nav2` the planner routes
  around LiDAR-observed obstacles in simulation, and the LiDAR hazard source
  forbids motion toward close obstacles; neither is verified on hardware. The
  default staged planner still only stops for hazards.
* **"Tested on hardware."** The dry-run bridge and the kinematic simulator
  are not hardware; every test asserting on actuation asserts on one of them.
* **"A secure safety architecture."** It is a *deterministic, fail-closed*
  architecture on a trusted domain. It has no authentication, and an
  independent audit demonstrated takeover from an unprivileged process.

---

## What a paper or demo may say

Defensible, in these words:

> A safety-authoritative architecture for assistive quadruped navigation, in
> which a deterministic fail-closed arbiter holds exclusive authority over
> actuation on a trusted control network. Authority is enforced structurally,
> the actuator's input is a message type no planner can produce, rather than
> by convention, and is demonstrated by runtime integration tests that show
> planner output being bounded, hazard-stopped, and cut off entirely when the
> safety process dies. An independent adversarial audit of the design is
> reported alongside it, including the findings that remain open. The
> multimodal caller-confirmation front end is evaluated in dry-run against its
> documented semantics, including regression tests for three defects found in
> the prior implementation. The system has not been validated on physical
> hardware.

The last sentence is not optional, and neither is "on a trusted control
network". Reporting the open audit findings alongside the closed ones is also
not optional: an architecture paper that presents only the attacks its design
survives is advertising, not evidence.
