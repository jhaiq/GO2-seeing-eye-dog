# Safety architecture audit

An independent adversarial review of the motion-authority design, run against
a live ROS 2 graph with the instruction to break this claim:

> Every movement command sent to the GO2 is explicitly authorized by a
> deterministic, fail-closed safety layer, and no motion command can reach the
> physical GO2 without passing through that authority.

The auditor was given the source, the ability to write and execute attack
scripts, and no obligation to be fair. It found the claim, as originally
stated, **false**. Five paths delivered non-zero velocity to the hardware
adapter without arbiter authorization, four demonstrated end to end.

This document records every finding, what was done about it, and — for the
ones that remain open — exactly what the system does and does not guarantee.
Findings are not softened. An audit whose findings are edited to match what
was convenient to fix is worse than no audit.

---

## Summary

| # | Finding | Severity | Status |
|---|---|---|---|
| C1 | Any process on the DDS domain can publish `SafeVelocityCommand` | CRITICAL | **OPEN** — bounded and documented |
| C2 | First-come-first-served authority latch; an attacker can lock the real arbiter out | CRITICAL | **PARTIALLY MITIGATED** |
| C3 | A stalled `/clock` freezes every watchdog; the robot keeps walking | CRITICAL | **FIXED** |
| C4 | SIGKILL of the bridge leaves the GO2 walking on a latched `Move` | CRITICAL | **FIXED** |
| C5 | Hazard context spoofable; a `String` overrode a real `EMERGENCY_STOP` | CRITICAL | **FIXED** |
| H1 | Unauthenticated `release_estop` service | HIGH | **OPEN** — bounded and documented |
| H2 | Every safety bound unbounded above (`max_vx:=100` accepted) | HIGH | **FIXED** |
| H3 | Authority handover was dead code; one packet bricked motion permanently | HIGH | **FIXED** |
| H4 | Two `ros2 topic pub` commands drove the robot at the full envelope | HIGH | **PARTIALLY FIXED** |
| M1 | Bridge accepted structurally malformed commands | MEDIUM | **FIXED** |
| M2 | Unhandled exception in the arbiter timer killed the process | MEDIUM | **FIXED** |
| M3 | `UnitreeSportBridge` could never report a transmit failure | MEDIUM | **FIXED** |
| M4 | `require_initial_inputs` documented, declared, never implemented | MEDIUM | **FIXED** |
| L1 | `go2_gait_controller` auto-activation | LOW | Accepted; terminates at an unconsumed topic |

**Three of the five CRITICAL findings — C3, C4 and H2's stale-command
persistence — required no attacker at all.** They were reachable by a paused
simulator, a killed process, and a typo on a command line respectively. Those
are fixed.

The findings that remain open are all instances of one thing: **ROS 2 without
SROS2 has no authentication, so any process on the DDS domain is trusted.** No
in-band scheme fixes that, and the honest response is to bound the claim rather
than to invent a mitigation that only looks like one.

---

## Fixed

### C3 — a stalled clock froze every watchdog

**The finding.** With `use_sim_time` enabled, both `_now()` and every
`create_timer` drew from `/clock`. Freezing `/clock` froze the bridge's
"independent watchdog" completely: it could not fire, because firing is itself
clock-driven and every age computed as zero.

Measured: 14.9 s of wall-clock time with the adapter holding 0.4 m/s and every
watchdog in the system asleep. On hardware, indefinite.

This is the most instructive finding in the audit. The watchdog did not fail to
*notice* a fault; the watchdog's own notion of elapsed time was the thing that
broke. A safety mechanism that depends on a signal the rest of the graph can
stop publishing is not independent.

**The fix.** Both the arbiter and the bridge now hold a
`Clock(clock_type=ClockType.STEADY_TIME)` and use it for every watchdog and
freshness decision. Steady time is monotonic and independent of `/clock` and of
wall-clock adjustments. Message timestamps are still interpreted in the node
clock, because that is the base producers stamp in; the conversion happens once,
at intake.

The arbiter now also warns at startup when `use_sim_time` is true, stating that
safety timing is unaffected.

*Regression:* `test_arbiter_audit_regressions.py::TestC3WatchdogsUseSteadyTime`.

### C4 — a killed bridge left the robot walking

**The finding.** `destroy_node` was the only stop-on-exit path, and SIGKILL
does not run it. For `DryRunGo2Bridge` that is a missing log line. For
`UnitreeSportBridge` the last thing on `/api/sport/request` is
`Move(0.4, 0, 0)` — and Sport API `Move` **latches**: the onboard controller
executes it until it receives another command or a `StopMove`. Nothing in the
repository would ever send one.

The auditor also identified why no test could have caught this:
`DryRunGo2Bridge.send_velocity` is a pure recorder with no latching semantics.
**The hazard is invisible in dry-run by construction.**

**The fix.** Three parts:

1. `HardwareBridgeInterface` gained a `tick()` hook, called by the bridge on
   every control cycle whether or not a command arrived. The contract now
   states that adapters with latching transports must implement it.
2. `UnitreeSportBridge.tick()` issues `StopMove` if a non-zero `Move` has not
   been renewed within `command_hold_sec` (0.2 s). Bridge death therefore
   becomes *command starvation at the robot*, which the robot can act on,
   rather than a silent handover of control to the last message that arrived.
3. `send_velocity` transmits on every call rather than only on change, so the
   renewal the hold timer expects actually happens.

The module docstring now records the asymmetry between the two adapters
explicitly, since it is the reason a green dry-run suite proves nothing here.

*Regression:* `test_bridge_audit_regressions.py::TestC4LatchingAdapterHazard`,
including a fake `unitree_api` so the hold logic is exercised without hardware.

**Residual:** a SIGKILL still leaves up to `command_hold_sec` of motion, and
the fix depends on the GO2 honouring `StopMove`. An out-of-process supervisor
(systemd `WatchdogSec` issuing `StopMove` + `Damp` on bridge death) is the
belt-and-braces version and is not implemented.

### C5 — a `std_msgs/String` cancelled a real emergency stop

**The finding.** `_alert_cb` (from `/go2/safety_alert`) and `_safety_state_cb`
(from `/go2/safety_state`) both wrote the same `self._hazard_type`. Last writer
won. With the depth pipeline asserting `EMERGENCY_STOP` at 10 Hz and a `String`
reading `"CLEAR"` at 100 Hz, 77 non-zero commands reached the adapter.

A bare string on a stock message type cancelled a stair-and-drop detection,
purely by arriving more often.

**The fix.** The channels are tracked separately and resolved by
**most-restrictive-wins**, with an explicit severity ranking in which an
*unrecognised* alert type ranks above `CLEAR` — so a new hazard type the
arbiter has not been taught about can never be outvoted. Stop-class hazards
additionally latch for `hazard_clear_hold_sec` (0.5 s), so a single dropped or
flapping frame cannot re-permit motion.

*Regression:* `test_arbiter_audit_regressions.py::TestC5HazardChannelsAreResolvedBySeverity`,
which executes the original exploit.

### H2 — every safety bound was unbounded above

**The finding.** `limits.py` validated only "positive and finite". Demonstrated:

```
$ ros2 run go2_safety_arbiter safety_arbiter_node --ros-args -p max_vx:=100.0 ...
limits=(vx=100.0 vy=100.0 wz=100.0) require_hazard=False
max vx delivered to adapter: 99.0
```

and with `watchdog_timeout_sec:=100000`, motion continued **27 s after the
producer died** and was still going when observation stopped.

The auditor correctly flagged that
`docs/target_runtime_architecture.md` claimed "the most permissive reachable
configuration still clamps, rate-limits, and enforces both watchdogs." It
clamped at 100 m/s and enforced watchdogs at 10⁵ s. **That sentence was false
and has been corrected**, not merely made true — both were needed.

**The fix.** Hard ceilings compiled into `limits.py`
(`MAX_ALLOWED_VX = 1.5`, `MAX_ALLOWED_WATCHDOG_SEC = 2.0`, and so on) raising
`LimitConfigError`, which the node already treats as fatal. They are
deliberately not configurable: the value of a ceiling that fails the node at
startup is precisely that it cannot be raised from a command line. Raising one
is a code change and a review.

The `safe_command_lifetime_sec` ceiling also closes M2's overflow path.

*Regression:* `test_arbiter_audit_regressions.py::TestH2ParametersAreBounded`.

### H3 — authority handover was dead code

**The finding.** The handover branch required
`now - self._last_stop_time >= authority_handover_quiet_sec`, but every idle
tick called `_stop`, and `_stop` refreshed `_last_stop_time`. At 50 Hz the
measured quiet period never exceeded 0.02 s against a 1.0 s requirement.

**The branch could never be taken.** Consequences: a legitimate arbiter that
crashed and respawned could never reclaim authority — the robot was immobilised
until the bridge itself was restarted. Combined with the sequence check, a
single message carrying `sequence = 2**64 - 1` latched a value nothing could
exceed: **one packet, permanent motion lockout.**

**The fix.** A separate `_last_nonzero_time`, updated only when a non-zero
velocity is actually transmitted. "Quiet" now means "has not moved recently"
rather than "has called `_stop` recently". Sequence numbers above
`MAX_PLAUSIBLE_SEQUENCE` (2⁶⁰, unreachable by counting at any plausible rate)
are refused.

*Regression:* `test_bridge_audit_regressions.py::TestH3AuthorityHandoverIsReachable`.

### M1 — structurally malformed commands were accepted

**The finding.** 201 of 201 accepted, carrying an empty `frame_id`, a garbage
reason-code vocabulary including `"\x00GARBAGE"`, `EMERGENCY_STOP` among the
reasons alongside `arbiter_state: SAFE_TO_MOVE`, and a `valid_until` in 2038.

**The fix.** The bridge now requires `frame_id == expected_frame_id`, rejects a
validity horizon beyond `max_command_lifetime_sec`, rejects reason codes
outside the known vocabulary, and rejects a stopping-class reason code
accompanying a permissive state. An internally inconsistent command is not a
trustworthy one.

*Regression:* `test_bridge_audit_regressions.py::TestM1MalformedCommandsAreRefused`.

### M2 — an exception in the timer killed the arbiter

**The finding.** `core.evaluate` was exception-guarded, on the stated principle
that "a crashing rule must never leave the robot moving". But `_tick` and
`_publish_safe` were not, so a raise there killed the process — and the stop
was not published either. The guarantee was real one level down and absent one
level up.

**The fix.** `_tick` now wraps `_tick_inner` and publishes an explicit
`STOPPED` command with `RULE_EVALUATION_FAILED` on any exception. The bridge
gained the same treatment.

*Regression:* `test_arbiter_audit_regressions.py::TestM2ATickExceptionStillStops`.

### M3 — the physical adapter could never report a failure

**The finding.** DDS `publish()` on a fire-and-forget topic does not raise when
nobody is subscribed, so `send_velocity` returned `True` unconditionally and
`max_consecutive_transmit_failures` was unreachable on real hardware. The same
applied to `send_zero`: **the bridge would believe it had stopped a robot it
never reached.** Compounding it, the bridge discarded `connect()`'s return
value, so a bridge with zero subscribers started and reported healthy.

**The fix.** `_publish` checks `get_subscription_count()` and counts a
zero-subscriber publish as a failure. `connect()` raises when
`require_subscriber` is set. The bridge refuses to start when a non-dry-run
adapter's `connect()` fails.

*Regression:*
`test_bridge_audit_regressions.py::test_the_unitree_adapter_reports_failure_with_no_subscriber`.

### M4 — a documented safety feature was never implemented

**The finding.** `require_initial_inputs` was documented at length, declared,
and never read. `Reason.NOT_INITIALIZED` and `Reason.NO_COMMAND` were defined
and never emitted.

Documentation describing a safety property the code does not have is worse than
silence, because it is load-bearing for someone's trust.

**The fix.** Implemented. The arbiter now refuses to authorize motion until
every required input has been observed at least once since startup, reporting
`NOT_INITIALIZED` — which is distinct from `SAFETY_CONTEXT_STALE`. "Never
arrived" usually means a node was not started; "aged out" is a transient. The
reason codes now tell them apart.

*Regression:* `test_arbiter_audit_regressions.py::TestM4InitialInputsAreRequired`.

---

## Partially fixed

### H4 — two shell commands drove the robot at the full envelope

**The finding.** Against the canonical launch, unmodified:

```
$ ros2 topic pub -r 20 /go2/safety_state std_msgs/msg/String "{data: CLEAR}" &
$ ros2 topic pub -r 20 /cmd_vel_candidate_unstamped geometry_msgs/msg/Twist \
    "{linear: {x: 5.0, y: 5.0}, angular: {z: 5.0}}" &
adapter: 71 records  {"vx": 0.4, "vy": 0.2, "wz": 0.6}
```

The unstamped inlet also defeats Invariant C entirely: a receipt-stamped
command is "fresh" by construction, however old it really is.

**Credit where due, and it matters:** the arbiter *did* clamp 5.0 to the
configured envelope and slew-limit the ramp. The limit logic worked exactly as
designed. What failed was that the inlet was open by default and the hazard
context it required was trivially forged (C5).

**What was fixed.** `accept_unstamped_candidate` now defaults to **false**. It
is enabled only by `system.launch.py planner:=nav2`, where
`candidate_stamper_node` is the sanctioned producer. C5's fix closes the
hazard-forging half.

**What remains.** With `planner:=nav2` the inlet is open, and it accepts a
stock `geometry_msgs/Twist` from anything on the domain. This is C1 in a
different costume and needs the same answer.

### C2 — the authority latch trusts whoever speaks first

**The finding.** The bridge latched the first token it saw. With the bridge up
and the legitimate arbiter starting two seconds later, the attacker owned the
robot and **the real arbiter was rejected for the entire 15 s window**:

```
bridge log:  17 x "Rejected safe command : AUTHORITY_MISMATCH"   <- the REAL arbiter
adapter:     695 non-zero velocity records, all vx=0.3           <- from the attacker
```

The duplicate-arbiter defence worked. It was simply orientation-blind, and the
one that wins is whichever races fastest at boot.

**What was fixed.** H3's fix means a legitimate arbiter can now reclaim
authority after the robot has genuinely been stopped for the quiet period,
where previously it could never reclaim it at all.

**What remains, and a further residual found while fixing it.** The bridge
still cannot tell a legitimate arbiter from an impostor, because the token is
self-asserted. Worse, writing the regression test surfaced a chain the original
audit did not name: an impostor publishing at the same rate **displaces the
legitimate arbiter in the depth-1 queue**, which stops the robot, which
eventually satisfies the quiet condition, which permits the takeover. Denial of
service converts into takeover.

That chain is recorded here rather than papered over. The regression test is
deliberately scoped to the guarantee that does hold — a second token cannot take
over a robot that is *currently moving* — and its docstring says why. A test
asserting more than the code provides would be worse than no test.

The real fix is transport-level authentication. A tighter timer is not a fix.

---

## Open

These share one root cause: **ROS 2 without SROS2 has no authentication.** Any
process that can reach the DDS domain and import `go2_msgs` has exactly the
same privileges as the arbiter. No in-band token, sequence number or message
type changes that, because every one of them is forgeable by a process that can
already publish.

### C1 — any process can publish `SafeVelocityCommand`

Demonstrated with a 70-line script and no arbiter running at all: 138 non-zero
commands delivered to the adapter.

The custom message type is an **anti-footgun, not an access control**. It
reliably stops a misconfigured controller, a stray `ros2 topic pub` of a
`Twist`, and a copy-paste error — the auditor confirmed all three fail
correctly. It does not stop anything that decides to construct the right type.

### H1 — `release_estop` is unauthenticated

The service's docstring argued that release must be "an addressed,
acknowledged request, not a fire-and-forget message that any process can
broadcast". That reasoning is sound about topics and **wrong about services
without SROS2**: a ROS 2 service is exactly as open as a topic. Demonstrated —
a latched e-stop was released from a shell and motion resumed within 0.22 s.

The topic/service asymmetry is still worth keeping: it prevents an *accidental*
release from a replayed bag or a stray publisher. It is not a security control
and the docstring no longer implies it is.

### The fix these need

SROS2 (`ros2 security`) with an access-control policy permitting publication on
`/cmd_vel_safe` and calls to `release_estop` only from the arbiter's enclave,
plus a physical e-stop the software cannot clear. That is a deployment-level
change, not a code change, and it is listed under "Planned" in
`docs/research_system_claims.md`.

Until then, the honest claim — and the one the README and the architecture
document now make — is bounded:

> Every movement command produced by the in-tree controller passes through a
> deterministic, fail-closed arbiter, **on a trusted single-host DDS domain**.

---

## Attacks that correctly failed

Negative results are evidence, and several of these are load-bearing.

| Attack | Result |
|---|---|
| `ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist` (86 messages) | **0** adapter records. `Publisher count: 0` — the RMW refused to create the publisher at all. Invariant A's type argument holds. |
| BEST_EFFORT publisher against the bridge's RELIABLE subscriber | **0** adapter records. QoS incompatibility fails in the safe direction. |
| Sequence replay (200 messages, same sequence) | 1 accepted, then `SEQUENCE_REGRESSION` throughout. |
| Over-limit candidate (5.0 / 5.0 / 5.0) at default config | Clamped to 0.4 / 0.2 / 0.6 and slew-limited. The core limit logic is sound. |
| Second arbiter overlapping a live token holder | Rejected `AUTHORITY_MISMATCH`. |
| The old `go2_gait_controller` actuation path | **Genuinely gone.** `hw_bridge.py` and `gait_hw_launch.py` absent from the tree and the install space. No second actuation path exists. |

---

## What this audit changed about the claim

The original claim was unqualified. It is now bounded by a trust assumption
that is stated wherever the claim appears.

Three of the five critical findings needed no adversary — a paused simulator, a
killed process, a mistyped parameter. Those are the ones that would have bitten
an honest user on a first hardware session, and they are fixed. The ones that
remain need an adversary on the robot's own DDS domain, and closing them is a
deployment decision (SROS2) rather than a code change.

The single most useful thing this audit produced is not any individual finding.
It is the demonstration that **a safety mechanism which depends on a signal the
rest of the system can stop publishing is not independent** — C3 — and that
**a dry-run harness cannot detect a hazard that only exists in the transport it
replaces** — C4. Both are now written into the code at the point where someone
would otherwise reintroduce them.
