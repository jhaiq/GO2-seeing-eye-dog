"""
closed_loop_trials: repeated full-stack navigation trials against go2_sim.

    ros2 run go2_sim closed_loop_trials --trials 5 --planner nav2 --out /tmp/trials

Each trial launches, on its own ROS domain, the kinematic sim plus
go2_bringup/system.launch.py (localization, planner, LiDAR hazard source,
safety arbiter, hardware bridge with the REAL UnitreeSportBridge adapter),
waits for /go2/localization_valid, then sends a fixed goal sequence through
/navigate_to_pose and records per goal:

  nav2 status, time, ground-truth arrival error (sim frame), collisions,
  arbiter decisions/interventions, stale-TF errors in the launch log.

The sim is kinematic: no gait dynamics, slip, or perception noise beyond
LiDAR range noise. Results establish that the software chain closes the
loop; they are not hardware evidence.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import Counter
from pathlib import Path

DEFAULT_GOALS = [(3.0, 3.5), (9.5, 2.0), (10.0, 4.4), (6.0, 4.4), (1.5, 1.5)]


BAG_TOPICS = ["/plan", "/tf", "/tf_static", "/go2/safety/status", "/go2/localization/status",
              "/cmd_vel", "/cmd_vel_candidate", "/go2_sim/ground_truth", "/go2/safety_state",
              "/go2/safety_alert", "/navigate_to_pose/_action/status", "/local_plan"]


GDB_PREFIX = (
    "gdb -q -batch -ex 'set pagination off' -ex 'handle SIGUSR1 stop print nopass' "
    "-ex run -ex 'thread apply all bt 40' -ex continue --args"
)


def _launch(domain: int, planner: str, log_path: Path, bag: Path | None = None,
            tf_monitor: Path | None = None, gdb_controller: bool = False) -> subprocess.Popen:
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    world = subprocess.check_output(["ros2", "pkg", "prefix", "go2_sim"], text=True).strip()
    world += "/share/go2_sim/worlds/apartment.yaml"
    cmd = (
        f"ros2 launch go2_sim sim.launch.py world_file:={world} & "
        f"sleep 2; ros2 launch go2_bringup system.launch.py perception:=none planner:={planner} "
        "localization:=slam_mapping lidar_safety:=true hardware_adapter:=unitree_sport "
        "cloud_in_topic:=/utlidar/cloud_deskewed "
        + (f"controller_prefix:=\"{GDB_PREFIX}\" " if gdb_controller else "")
        + "& "
    )
    if bag is not None:
        cmd += f"sleep 3; ros2 bag record -o {bag} {' '.join(BAG_TOPICS)} & "
    if tf_monitor is not None:
        # An independent, long-lived C++ tf2 listener (not a Nav2 server):
        # if the controller's map->odom freezes, does this one freeze too?
        cmd += f"sleep 5; ros2 run tf2_ros tf2_echo odom map -r 2 > {tf_monitor} 2>&1 & "
    cmd += "wait"
    return subprocess.Popen(
        ["bash", "-c", cmd], env=env, stdout=open(log_path, "w"), stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _stop(proc: subprocess.Popen, grace_s: float = 15.0) -> None:
    """Stop EVERY process of the trial, not just the launch wrapper.

    Waiting on the wrapper alone left Nav2 servers alive after SIGINT; they
    kept answering lifecycle and TF traffic on the same domain and poisoned
    later trials that reused it (observed: planner_server survivors from
    every trial of a batch).
    """
    pgid = proc.pid
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline and _group_alive(pgid):
        time.sleep(0.2)
    if _group_alive(pgid):
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def _controller_pid(domain: int):
    """PID of this trial's controller_server (the gdb inferior when wrapped)."""
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            comm = Path(f"/proc/{pid}/comm").read_text().strip()
            env = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
        except OSError:
            continue
        if comm.startswith("controller_serv") and f"ROS_DOMAIN_ID={domain}".encode() in env:
            return int(pid)
    return None


def _stall_sampler(log_path: Path, domain: int, stop, dumped: list) -> None:
    """When the controller's map->odom stall persists, SIGUSR1 it once so gdb dumps stacks."""
    first = None
    while not stop.is_set():
        try:
            n = log_path.read_text(errors="ignore").count("Transform data too old")
        except OSError:
            n = 0
        if n and first is None:
            first = time.monotonic()
        if first is not None and time.monotonic() - first > 3.0 and not dumped:
            pid = _controller_pid(domain)
            if pid:
                os.kill(pid, signal.SIGUSR1)
                dumped.append(pid)
        time.sleep(1.0)


def _run_goals(goals, goal_timeout: float, ready_timeout: float) -> dict:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from std_msgs.msg import Bool, String, UInt32

    from go2_msgs.msg import SafetyStatus

    rclpy.init()
    node = rclpy.create_node("closed_loop_trials")
    state = {"valid": False, "gt": None, "collisions": 0, "safety": None,
             "reasons": Counter(), "hazard": Counter(), "invalid": 0}

    def on_safety(m):
        state["safety"] = m
        for code in m.reason_codes:
            state["reasons"][code] += 1

    def on_valid(m):
        state["valid"] = m.data
        if not m.data:
            state["invalid"] += 1
    node.create_subscription(Bool, "/go2/localization_valid", on_valid, 10)
    node.create_subscription(String, "/go2/safety_state",
                             lambda m: state["hazard"].update([m.data]), 10)
    node.create_subscription(PoseStamped, "/go2_sim/ground_truth", lambda m: state.update(gt=m), 10)
    node.create_subscription(UInt32, "/go2_sim/collisions", lambda m: state.update(collisions=m.data), 10)
    node.create_subscription(SafetyStatus, "/go2/safety/status", on_safety, 10)
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

    def spin_until(pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not pred():
            rclpy.spin_once(node, timeout_sec=0.05)
        return pred()

    out = {"ready": False, "goals": []}
    ready = spin_until(lambda: state["valid"] and client.server_is_ready() and state["gt"] is not None,
                       ready_timeout)
    out["ready"] = bool(ready)
    if not ready:
        node.destroy_node()
        rclpy.shutdown()
        return out
    spin_until(lambda: False, 3.0)  # let SLAM take a few scans before the first goal

    for gx, gy in goals:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.w = 1.0
        t0 = time.monotonic()
        dec0 = state["safety"].decision_count if state["safety"] else 0
        int0 = state["safety"].intervention_count if state["safety"] else 0
        state["reasons"].clear()
        state["hazard"].clear()
        state["invalid"] = 0
        send = client.send_goal_async(goal)
        spin_until(send.done, 5.0)
        handle = send.result() if send.done() else None
        status = "REJECTED"
        if handle is not None and handle.accepted:
            res = handle.get_result_async()
            if spin_until(res.done, goal_timeout):
                status = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(res.result().status, str(res.result().status))
            else:
                handle.cancel_goal_async()
                spin_until(lambda: False, 2.0)
                status = "TIMEOUT"
        p = state["gt"].pose.position
        s = state["safety"]
        out["goals"].append({
            "goal": [gx, gy],
            "status": status,
            "seconds": round(time.monotonic() - t0, 1),
            "final_gt": [round(p.x, 3), round(p.y, 3)],
            "arrival_error_m": round(math.hypot(p.x - gx, p.y - gy), 3),
            "collisions_total": int(state["collisions"]),
            "arbiter_decisions": (s.decision_count - dec0) if s else None,
            "arbiter_interventions": (s.intervention_count - int0) if s else None,
            "arbiter_reason_counts": dict(state["reasons"]),
            "hazard_state_counts": dict(state["hazard"]),
            "localization_invalid_msgs": state["invalid"],
        })
    node.destroy_node()
    rclpy.shutdown()
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--planner", default="nav2", choices=["nav2", "staged_nav"])
    ap.add_argument("--goal-timeout", type=float, default=120.0)
    ap.add_argument("--ready-timeout", type=float, default=90.0)
    ap.add_argument("--domain-base", type=int, default=150)
    ap.add_argument("--out", default="closed_loop_trials")
    ap.add_argument("--bag", action="store_true", help="record /plan, TF and safety topics per trial")
    ap.add_argument("--tf-monitor", action="store_true", help="run a long-lived tf2_echo odom map per trial")
    ap.add_argument("--gdb-controller", action="store_true",
                    help="run controller_server under gdb; dump all thread stacks on the map->odom stall")
    args = ap.parse_args(argv)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    trials = []
    for i in range(args.trials):
        domain = args.domain_base + i
        log = out_dir / f"trial_{i}.log"
        proc = _launch(domain, args.planner, log, out_dir / f"bag_{i}" if args.bag else None,
                       out_dir / f"tfmon_{i}.log" if args.tf_monitor else None,
                       gdb_controller=args.gdb_controller)
        sampler_stop, dumped = threading.Event(), []
        if args.gdb_controller:
            threading.Thread(target=_stall_sampler, args=(log, domain, sampler_stop, dumped),
                             daemon=True).start()
        try:
            env_domain = os.environ.get("ROS_DOMAIN_ID")
            os.environ["ROS_DOMAIN_ID"] = str(domain)
            os.environ["ROS_LOCALHOST_ONLY"] = "1"
            result = _run_goals(DEFAULT_GOALS, args.goal_timeout, args.ready_timeout)
        finally:
            sampler_stop.set()
            _stop(proc)
            if env_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = env_domain
        text = log.read_text(errors="ignore")
        result["trial"] = i
        result["stale_tf_errors"] = len(re.findall(r"Transform data too old", text))
        result["tracebacks"] = len(re.findall(r"Traceback", text))
        result["gdb_dumped_pid"] = dumped[0] if dumped else None
        trials.append(result)
        ok = sum(g["status"] == "SUCCEEDED" for g in result["goals"])
        print(f"trial {i}: ready={result['ready']} succeeded {ok}/{len(result['goals'])} "
              f"stale_tf_errors={result['stale_tf_errors']}", flush=True)

    goals = [g for t in trials for g in t["goals"]]
    summary = {
        "planner": args.planner,
        "trials": len(trials),
        "trials_ready": sum(t["ready"] for t in trials),
        "goals_sent": len(goals),
        "goals_succeeded": sum(g["status"] == "SUCCEEDED" for g in goals),
        "collisions_max": max((g["collisions_total"] for g in goals), default=0),
        "median_arrival_error_m": (sorted(g["arrival_error_m"] for g in goals)[len(goals) // 2]
                                   if goals else None),
        "trials_with_stale_tf": sum(t["stale_tf_errors"] > 0 for t in trials),
        "note": "kinematic sim; software-chain evidence only, not hardware evidence",
    }
    (out_dir / "results.json").write_text(json.dumps({"summary": summary, "trials": trials}, indent=2))
    print(json.dumps(summary, indent=2))
    return 0 if summary["goals_succeeded"] == summary["goals_sent"] and summary["goals_sent"] else 1


if __name__ == "__main__":
    sys.exit(main())
