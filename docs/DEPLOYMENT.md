# Deploying autonomous navigation on the GO2

This is the runbook for taking the navigation stack from simulation to the
robot. Every stage has a pass criterion and an abort condition. Do not skip a
stage because the previous session passed it: the payload has no RTC, no
internet, and its sensors are not guaranteed to be attached
(`docs/go2_field_notes.md`).

## What runs

```
/utlidar/robot_odom, /utlidar/cloud_deskewed   (stock GO2, robot clock)
        |
go2_state_relay_node      robot clock -> payload clock, /odom, TF odom->base_link,
        |                 /go2/lidar/points, /go2/localization_valid
pointcloud_to_laserscan   /scan (base_link, floor excluded)
slam_toolbox              /map, TF map->odom
        |
Nav2 (planner nav2)       or  approach controller + NavigateToPose adapter (staged_nav)
        |  /cmd_vel -> candidate_stamper -> /cmd_vel_candidate
lidar_hazard_node         CLEAR | SLOWDOWN | RESTRICT:<F|B|W> | EMERGENCY_STOP
        |
safety_arbiter_node       the only producer of /cmd_vel_safe
        |
hardware_bridge_node      dry_run | unitree_sport (Move 1008 / StopMove 1003)
```

One entrypoint: `ros2 launch go2_bringup system.launch.py` with
`localization:=slam_mapping|slam_localization`, `lidar_safety:=true`,
`planner:=nav2|staged_nav`, `hardware_adapter:=dry_run|unitree_sport`.
go2-semantic-nav wraps it in `deploy.launch.py`.

## Evidence so far

| Claim | Status | Evidence |
|---|---|---|
| Robot clock skew corrected | VERIFIED OFFLINE | real 2026-09-01 bag: 27,605,481 s skew; relay tests; sim learns 27605481.0001 s |
| Nav2 configures and activates on Humble | VERIFIED IN SIM | lifecycle "Managed nodes are active"; `test_nav2_params.py` checks every plugin |
| Goal -> Nav2 -> arbiter -> real Unitree adapter -> robot | VERIFIED IN KINEMATIC SIM | `ros2 run go2_sim closed_loop_trials` (see results below) |
| LiDAR hazard source replaces the camera | VERIFIED IN SIM | 42 tests incl. real arbiter; no RealSense needed |
| Anything on the physical robot | HW-UNVERIFIED | no session yet |
| `/utlidar/cloud_deskewed` is in `odom` | ASSUMPTION | preflight checks it; fallback is raw cloud + `calibrate_lidar` |
| Velocity, stop distance, gait at 0.35 m/s | HW-UNVERIFIED | field notes: 0.15 m/s commanded gave 0.16 m/s; clean trot needs vx >= 0.5 |

The simulator is kinematic (no gait dynamics, no slip, LiDAR noise only). Sim
results prove the software chain closes the loop; they are not hardware
evidence.

## Stage 0: before the lab (on the workstation)

1. `./scripts/reproduce.sh` passes on the exact revision you will deploy.
   Record the revision.
2. `ros2 run go2_sim closed_loop_trials --trials 3` passes.
3. Stage for the payload (no internet in the lab): Nav2, slam_toolbox,
   pointcloud_to_laserscan as aarch64 debs, this workspace built for aarch64,
   unitree_ros2 messages. The preflight lists anything missing.

## Stage 1: preflight (robot standing, operator on the remote)

```bash
./scripts/deploy_preflight.sh     # read-only; exit code = number of FAILs
```

GO only with 0 FAILs. It checks the DDS interface (the failure that shows
healthy topics with no data), the payload clock, package presence, measured
odom/LiDAR rates, the frame of `/utlidar/cloud_deskewed`, that the sport
service is subscribed, and that no other motion stack is publishing.

If `/utlidar/cloud_deskewed` is missing or not in `odom`: record a standing
bag (5 s still, then walk straight 1.5 m on the remote at low speed) and run
`ros2 run go2_localization calibrate_lidar <bag>`. It prints a `relay.yaml`
snippet. It refuses to report a yaw it could not measure. Then launch with
`cloud_in_topic:=/utlidar/cloud publish_lidar_extrinsic:=true`.

## Stage 2: sensing only, motors untouched

```bash
ros2 launch go2_bringup system.launch.py perception:=none planner:=nav2 \
  localization:=slam_mapping lidar_safety:=true hardware_adapter:=dry_run
```

Pass: `/go2/localization/status` shows `"valid": true` and a clock offset;
`/scan` ~15 Hz; RViz shows the map growing and obstacles where they are when
the operator walks the robot on the remote; `/go2/safety_state` goes
`RESTRICT:F` when a box is placed 0.3 m in front of the nose and back to
`CLEAR` when removed. Check a box at the side gives `RESTRICT:W` (not F):
this is the direction check on the LiDAR extrinsic.

Abort: localization flips invalid while the robot is still; obstacles appear
rotated or mirrored relative to reality; `/scan` shows the floor.

## Stage 3: full graph, dry-run actuator

Same launch. Send goals from RViz. The dry-run adapter logs what it would
have commanded (`dry_run_log_path:=...`). Pass: commands are within the
envelope, directionally sensible, and stop when a hazard is placed ahead.

## Stage 4: autonomous motion, low speed, tethered area

Only after stages 1-3 pass in the same session.

```bash
ros2 launch go2_bringup system.launch.py perception:=none planner:=nav2 \
  localization:=slam_mapping lidar_safety:=true hardware_adapter:=unitree_sport
```

* Clear area, operator holding the remote, a second person on the e-stop.
* First goal 1 m straight ahead. Then 2 m with a turn. Then around one
  obstacle. Then through a doorway.
* Measure and record: stopping distance from 0.3 m/s under StopMove, arrival
  error, time to goal, arbiter interventions (`/go2/safety/status`).

Abort immediately (remote stop, then `ros2 service call
/hardware_bridge_node/emergency_stop std_srvs/srv/Trigger`) on: motion not
matching the command, motion after a StopMove, a LiDAR hazard that does not
stop forward motion, any `TRANSMIT_FAILED` or `BRIDGE_WATCHDOG_TIMEOUT`.

**The bridge emergency stop sends Damp (1001): the robot drops where it
stands.** Use the remote first. Hazard stops from the arbiter use StopMove
(1003) and do not drop the robot.

## Middleware: CycloneDDS

Run the whole graph with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (the robot
speaks CycloneDDS; the preflight fails otherwise).

For single-host sim on loopback, use
`CYCLONEDDS_URI=file://$(ros2 pkg prefix go2_sim)/share/go2_sim/config/cyclonedds_localhost.xml`
(loopback cannot multicast and the default participant limit is too small
for this graph). Never use that file on the robot.

## Open issue: Nav2 controller loses map -> odom

In roughly 1 in 10 closed-loop sim trials, under FastDDS and CycloneDDS
alike, Nav2's controller_server stops seeing new `map -> odom` transforms
while slam_toolbox keeps publishing them and other readers keep receiving
them. From then on every path is rejected ("Transform data too old when
converting from map to odom") and navigation is dead. The robot fails safe:
no candidates reach the arbiter, which commands zero. **The root cause was
not isolated.** Ruled out: robot-clock correction (no resets or stamp jumps
in bags of affected runs), slam_toolbox stopping (its transform stayed fresh
on `/tf`), leftover processes from earlier trials, and FastDDS alone (it
also happened once under CycloneDDS).

Mitigation: `nav_tf_watchdog` (started with `planner:=nav2`) watches for the
planner publishing `/plan` while the controller publishes nothing on
`/received_global_plan` for 3 s, and resets and restarts the Nav2 lifecycle.
The goal in progress aborts and must be re-sent. Its recovery count is on
`/go2/nav_health`. Watch it during stage 4; a recovery on the robot is a
finding to record, not noise.

## Known limits

* Rotating in place: the stock `mcf` gait was measured to need about
  1.0 rad/s for clean counter-clockwise yaw; the arbiter caps yaw at 0.6 rad/s
  (0.21 rad/s when derated). If the robot will not turn at these rates,
  raising the cap is a safety decision, not a tuning change.
* Drop-off detection is off (`drop_check_enabled: false`) until the LiDAR
  geometry is calibrated on a standing capture.
* The DDS domain is trusted: there is no authentication
  (`docs/END_TO_END_UPGRADE_REPORT.md` s11).

## Rollback

Every launch argument above defaults to the safe value. Relaunch with
`hardware_adapter:=dry_run` to take the robot out of the loop; the previous
behaviour (`planner:=staged`, `localization:=none`) is unchanged.
