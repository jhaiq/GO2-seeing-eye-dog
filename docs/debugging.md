# Debugging

## First question: is the robot allowed to move?

Almost every "it does nothing" report resolves here. The arbiter says why, in
machine-readable form:

```bash
ros2 topic echo /go2/safety/status
```

| `reason_codes` | Meaning | Fix |
|---|---|---|
| `SAFETY_CONTEXT_STALE` | No fresh hazard information. **This is a stop by design.** | Start `safety_monitor_node`, or publish `/go2/safety_state`. Do not set `require_hazard_context: false` to make it go away without understanding what you are giving up. |
| `WATCHDOG_TIMEOUT` | No candidate command is arriving | Check the controller is running and publishing `/cmd_vel_candidate` |
| `STALE_COMMAND` | Candidates arrive but are too old | Clock skew, or a controller running far slower than `candidate_max_age_sec` |
| `INVALID_TIMESTAMP` | Zero, negative, or far-future stamp | A producer is not stamping its messages, or clocks disagree |
| `INVALID_COMMAND` | NaN, infinity, or the wrong frame | Check the candidate's `header.frame_id` matches `expected_frame_id` |
| `HAZARD_STOP` | A stop-class hazard is asserted | Look at `/go2/safety_alert`. Also fires for an **unrecognised** alert type, which is deliberate. |
| `NO_LOCALIZATION` | Localization required and absent | Only when `require_localization: true`, which is false by default because nothing publishes `/go2/localization_valid` yet |
| `EMERGENCY_STOP` | The latch is engaged | `ros2 service call /safety_arbiter_node/release_estop std_srvs/srv/Trigger`. Nothing else clears it. |
| `SPEED_LIMIT` / `ACCEL_LIMIT` | The command was clamped or rate-limited | Not a fault. Limits are in `config/safety.yaml`. |

## Second question: what actually reached the actuator?

```bash
ros2 topic echo /go2/bridge/status
```

`last_reject_reasons` explains anything the bridge refused independently of the
arbiter:

* `BRIDGE_WATCHDOG_TIMEOUT` — the arbiter went quiet. If the arbiter is
  running, suspect QoS or a domain mismatch.
* `COMMAND_EXPIRED` — the command's `valid_until` had passed on arrival.
  Usually clock skew between the two processes.
* `AUTHORITY_MISMATCH` — a second arbiter is publishing. Check
  `ros2 topic info /cmd_vel_safe --verbose`; there must be exactly one publisher.
* `SEQUENCE_REGRESSION` — replayed or reordered commands. A bag being played
  back into a live graph will do this.
* `SPEED_LIMIT` at the bridge — the arbiter emitted an over-limit command,
  which means the arbiter is malfunctioning or the two limit sets have drifted
  apart. `scripts/repo_doctor.py` checks for the latter.

## Third question: why was the caller not confirmed?

```bash
ros2 topic echo /go2/grounding_status
```

This publishes on a timer whether or not detections are arriving, so "nothing
is happening" is itself reported.

| `reason` | Meaning |
|---|---|
| `NO_REQUEST` | No voice command received. **A request is required**; detections alone never produce a goal. |
| `NO_DETECTIONS` | The request is armed but nobody is visible |
| `BEARING_MISMATCH` | Someone is visible, but not where the sound came from. Check the sign convention before assuming the person moved (see below). |
| `LOW_CONFIDENCE` | Detected, corroborated, but below threshold. `visual_score` and `fused_score` in the same message show by how much. |
| `ACCUMULATING` | Confirming. `consecutive_confirmations` of `required_confirmations`. |
| `CONFIRMATION_TIMEOUT` | The request expired. `request_timeout_sec` in `config/fusion.yaml`. |
| `TARGET_MOVED` | A locked target left the association gate; re-acquiring |

### Persistent `BEARING_MISMATCH` with the caller plainly in view

Check the sign convention before anything else. If the mismatch is roughly
**twice** the caller's true offset and flips when they cross the centre line,
the acoustic bearing's left/right sense is inverted relative to the body frame.

`audio_perception_node` asserts REP-103 (positive = left), but whether
GCC-PHAT's `tau` sign actually maps that way depends on the physical
microphone ordering, which is not documented in code and has not been bench
tested. A defect of exactly this shape existed on the *visual* side and is
fixed; the acoustic side is listed as unvalidated in
`docs/research_system_claims.md`.

To test: stand at a known angle, watch `bearing_delta_deg` in
`/go2/grounding_status`, and compare it against where you actually are.

## Environment problems

**Two `go2_msgs` installations.** If a previously-built workspace is on
`AMENT_PREFIX_PATH`, it can shadow the one you just built, and you will be
running code that is not in your tree. `scripts/reproduce.sh` unsets the
overlay for exactly this reason. Check with `ros2 pkg prefix go2_msgs`.

**Tests passing for the wrong reason.** If another ROS graph is live on your
domain, a test asserting "no command reached the actuator" can pass because
your nodes never connected at all. `reproduce.sh` refuses to start on a
populated domain.

**Perception nodes will not start.** They import hardware libraries at module
load and open devices in `__init__`: `pyaudio` needs a 4-channel input device,
`ultralytics` loads YOLO weights, `cv_bridge` needs a depth stream. Use
`perception:=none` and publish synthetic inputs.

**RealSense topic namespace.** `realsense2_camera` 4.58.x publishes under
`/camera/camera/...`, while the perception nodes subscribe to `/camera/...`.
On that driver version the subscriptions receive nothing. Remap, or check the
driver version.

## Historical faults worth recognising

These were real in this repository and are fixed; the symptoms are worth
knowing because they are easy to reintroduce.

* **A goal with no request.** If `/goal_pose` appears without anyone speaking,
  the state machine's `IDLE` guard has been broken.
* **Motion continuing after the safety process dies.** The bridge's watchdog
  must be strictly tighter than the arbiter's. `repo_doctor.py` checks this.
* **A confirmation that never fires despite good detections.** The lock edge is
  a latch consumed only by the status timer; anything that clears it on each
  incoming detection frame will drop the goal intermittently, because
  detections arrive faster than the timer ticks.
* **A launch file that starts and does nothing.** The previous
  `go2_full.launch.py` started a Nav2 stack that could not complete lifecycle
  bring-up, and connected nothing to actuation. It now refuses to run.
