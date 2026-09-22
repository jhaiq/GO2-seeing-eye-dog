"""
Stop and shutdown regressions for the full Nav2 stack, run against go2_sim.

    source /opt/ros/humble/setup.bash
    source <unitree_ros2>/cyclonedds_ws/install/local_setup.bash
    source install/setup.bash
    export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$(ros2 pkg prefix go2_sim)/share/go2_sim/config/cyclonedds_localhost.xml
    GO2_STOP_REG=1 STOP_REG_DOMAIN_BASE=90 python3 -m pytest -p no:cacheprovider \
        go2_bringup/test/test_stop_regressions.py -v

Without GO2_STOP_REG=1 only the fast PID-helper self-test runs; the full-stack
cases (marked slow, about a minute each) are skipped. STOP_REG_OUT=<dir> keeps
per-case logs and result.json.

Each case starts the kinematic sim and go2_bringup/system.launch.py (Nav2,
slam_toolbox, LiDAR hazard source, safety arbiter, hardware bridge with the
REAL unitree_sport adapter) in their own sessions, drives a real
NavigateToPose goal where motion is involved, applies one stop event, and
records /api/sport/request with an independent subscriber. Per case:

  * no Damp (1001) is ever sent (Damp drops the robot where it stands),
  * a StopMove (1003) arrives within the case's bound after the event,
  * no Move (1008) arrives later than that StopMove + 0.1 s,
  * the last request at the end is StopMove and the sim robot is not MOVING,
  * for shutdown cases, every process of the stack's process group is gone.

Process control: every process is started with start_new_session and is
addressed by its Popen handle and process group. A single node (for the crash
cases) is found by walking the process tree of our own launch and matching
/proc/<pid>/cmdline exactly to that node's executable path; anything other
than exactly one match is refused. Nothing is found by name matching over the
system process list.

The sim is kinematic (no gait dynamics). Passing here is evidence about the
software stop chain only, not about the robot.
"""
from __future__ import annotations

import json
import math
import os
import pty
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

import pytest

MOVE, STOP_MOVE, DAMP = 1008, 1003, 1001
#: A Move later than first StopMove + this is a failed stop.
MOVE_AFTER_STOP_TOLERANCE_S = 0.1
READY_TIMEOUT_S = 90.0
CASE_TIMEOUT_S = 240
GOAL = (6.0, 4.4)


# ── Process tree helpers (no name matching over the system process list) ──


def child_pids(pid: int) -> List[int]:
    """Direct children of ``pid`` from /proc/<pid>/task/*/children."""
    out: List[int] = []
    try:
        tids = os.listdir(f"/proc/{pid}/task")
    except OSError:
        return out
    for tid in tids:
        try:
            with open(f"/proc/{pid}/task/{tid}/children") as f:
                out.extend(int(x) for x in f.read().split())
        except OSError:
            pass
    return out


def process_tree(root_pid: int) -> List[int]:
    """``root_pid`` and all its descendants that are still alive."""
    seen: List[int] = []
    todo = [root_pid]
    while todo:
        pid = todo.pop()
        if pid in seen:
            continue
        seen.append(pid)
        todo.extend(child_pids(pid))
    return seen


def argv_of(pid: int) -> List[str]:
    with open(f"/proc/{pid}/cmdline", "rb") as f:
        return [a.decode(errors="replace") for a in f.read().split(b"\0") if a]


def executable_of(argv: List[str]) -> Optional[str]:
    """The program a process runs: argv[0], or the script for a python interpreter."""
    if not argv:
        return None
    if os.path.basename(argv[0]).startswith("python") and len(argv) > 1:
        return argv[1]
    return argv[0]


def find_node_pid(root_pid: int, exe_path: str) -> int:
    """
    PID of the one process under ``root_pid``'s tree, in ``root_pid``'s process
    group, whose executable (from /proc/<pid>/cmdline) is exactly ``exe_path``.

    Raises LookupError unless there is exactly one match.
    """
    pgid = os.getpgid(root_pid)
    matches = []
    for pid in process_tree(root_pid):
        if pid == root_pid:
            continue
        try:
            if os.getpgid(pid) != pgid:
                continue
            if executable_of(argv_of(pid)) == exe_path:
                matches.append(pid)
        except (OSError, ProcessLookupError):
            continue
    if len(matches) != 1:
        raise LookupError(
            f"expected exactly one process running {exe_path} in pgid {pgid}, found {matches}"
        )
    return matches[0]


def group_members(pgid: int) -> List[int]:
    """Live (non-zombie) PIDs whose process group is ``pgid``. Used to verify cleanup."""
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/stat") as f:
                stat = f.read()
        except OSError:
            continue
        fields = stat[stat.rfind(")") + 2:].split()
        if fields[0] != "Z" and int(fields[2]) == pgid:
            out.append(int(name))
    return out


def stop_group(proc: subprocess.Popen, first_sig=signal.SIGINT, grace_s: float = 20.0) -> List[int]:
    """Stop a session we started; escalate to SIGKILL. Returns survivors (should be [])."""
    pgid = proc.pid
    for sig, wait_s in ((first_sig, grace_s), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        if not group_members(pgid) and proc.poll() is not None:
            break
        try:
            if sig == first_sig and proc.poll() is None:
                os.kill(proc.pid, sig)
            else:
                os.killpg(pgid, sig)
        except ProcessLookupError:
            pass
        end = time.monotonic() + wait_s
        while time.monotonic() < end and group_members(pgid):
            time.sleep(0.1)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    return group_members(pgid)


# ── Self-test of the PID helper (fast, no ROS) ───────────────────────────


def test_find_node_pid_matches_exactly_one_in_own_tree(tmp_path):
    script = "#!/usr/bin/python3\nimport time\ntime.sleep(60)\n"
    paths = {}
    for name in ("node_a", "node_b", "node_c"):
        p = tmp_path / name
        p.write_text(script)
        p.chmod(0o755)
        paths[name] = str(p)
    a, b = paths["node_a"], paths["node_b"]
    # node_a once (as a grandchild via bash), node_b twice, node_c never.
    root = subprocess.Popen(["bash", "-c", f"{a} & {b} & {b} & wait"], start_new_session=True)
    # The same executable outside our tree and group must never match.
    outsider = subprocess.Popen([a], start_new_session=True)
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end and len(process_tree(root.pid)) < 4:
            time.sleep(0.05)
        pid_a = find_node_pid(root.pid, a)
        assert pid_a != outsider.pid
        assert argv_of(pid_a)[1] == a
        assert os.getpgid(pid_a) == root.pid
        with pytest.raises(LookupError):
            find_node_pid(root.pid, b)  # two matches: refused
        with pytest.raises(LookupError):
            find_node_pid(root.pid, paths["node_c"])  # none
        with pytest.raises(LookupError):
            find_node_pid(root.pid, a + "x")  # prefix is not a match
    finally:
        assert stop_group(root, signal.SIGTERM, grace_s=3) == []
        assert stop_group(outsider, signal.SIGTERM, grace_s=3) == []


# ── ROS side ─────────────────────────────────────────────────────────────


def _ros_imports():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("unitree_api.msg")
    pytest.importorskip("nav2_msgs.action")
    pytest.importorskip("go2_msgs.msg")
    return rclpy


_domain_counter = [0]


def _next_domain() -> int:
    base = int(os.environ.get("STOP_REG_DOMAIN_BASE", "90"))
    span = int(os.environ.get("STOP_REG_DOMAIN_SPAN", "10"))
    d = base + _domain_counter[0] % span
    _domain_counter[0] += 1
    return d


class Recorder:
    """Independent observer: Sport requests, safe commands, ground truth, readiness."""

    def __init__(self, rclpy, domain: int):
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        from nav2_msgs.srv import ManageLifecycleNodes
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, String
        from unitree_api.msg import Request

        from go2_msgs.msg import SafeVelocityCommand

        self.rclpy = rclpy
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=domain)
        self.node = rclpy.create_node("stop_regression_recorder", context=self.ctx)
        self.lock = threading.Lock()
        self.requests: list = []  # (t, api_id, speed, linear speed)
        self.safe: list = []  # (t, state, reasons, vx)
        self.gt: list = []  # (t, x, y)
        self.sim_state = None
        self.loc_valid = False
        n = self.node
        n.create_subscription(Request, "/api/sport/request", self._on_request, 500)
        control_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE,
                                 history=HistoryPolicy.KEEP_LAST)
        n.create_subscription(SafeVelocityCommand, "/cmd_vel_safe", self._on_safe, control_qos)
        n.create_subscription(PoseStamped, "/go2_sim/ground_truth", self._on_gt, 10)
        n.create_subscription(String, "/go2_sim/state",
                              lambda m: setattr(self, "sim_state", m.data), 10)
        n.create_subscription(Bool, "/go2/localization_valid",
                              lambda m: setattr(self, "loc_valid", m.data), 10)
        self.nav = ActionClient(n, NavigateToPose, "/navigate_to_pose")
        self.NavigateToPose = NavigateToPose
        self.lifecycle = n.create_client(ManageLifecycleNodes,
                                         "/lifecycle_manager_navigation/manage_nodes")
        self.ManageLifecycleNodes = ManageLifecycleNodes
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on_request(self, m):
        # speed = |v_xy| + |w_z|: one number that is zero only for a full stop.
        speed = linear = 0.0
        if m.header.identity.api_id == MOVE:
            try:
                p = json.loads(m.parameter)
                linear = math.hypot(float(p.get("x", 0.0)), float(p.get("y", 0.0)))
                speed = linear + abs(float(p.get("z", 0.0)))
            except (ValueError, TypeError):
                speed = linear = float("nan")
        with self.lock:
            self.requests.append((time.monotonic(), m.header.identity.api_id, speed, linear))

    def _on_safe(self, m):
        with self.lock:
            self.safe.append((time.monotonic(), m.arbiter_state, list(m.reason_codes),
                              m.twist.linear.x))

    def _on_gt(self, m):
        with self.lock:
            self.gt.append((time.monotonic(), m.pose.position.x, m.pose.position.y))

    def foreign_nodes(self) -> List[str]:
        return [n for n in self.node.get_node_names() if n != "stop_regression_recorder"]

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=2.0)
        except Exception:  # noqa: BLE001
            pass
        self.node.destroy_node()
        self.rclpy.try_shutdown(context=self.ctx)


def wait_for(pred, timeout: float, period: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return bool(pred())


def wait_future(fut, timeout: float):
    if not wait_for(fut.done, timeout):
        return None
    return fut.result()


@dataclass
class Run:
    sim: subprocess.Popen
    stack: subprocess.Popen
    rec: Recorder
    log_dir: Path
    pty_master: Optional[int] = None
    goal_handle: object = None
    notes: dict = field(default_factory=dict)


def _exe(package: str, executable: str) -> str:
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix(package), "lib", package, executable)


def start_run(rclpy, log_dir: Path, interactive: bool) -> Run:
    domain = _next_domain()
    rec = Recorder(rclpy, domain)
    time.sleep(2.0)
    foreign = rec.foreign_nodes()
    if foreign:
        rec.close()
        pytest.fail(f"ROS domain {domain} is busy ({foreign[:5]}); set STOP_REG_DOMAIN_BASE")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    log_dir.mkdir(parents=True, exist_ok=True)
    sim = subprocess.Popen(
        ["ros2", "launch", "go2_sim", "sim.launch.py"], env=env,
        stdin=subprocess.DEVNULL, stdout=open(log_dir / "sim.log", "w"),
        stderr=subprocess.STDOUT, start_new_session=True)
    master = None
    stdin = subprocess.DEVNULL
    if interactive:
        # A terminal on stdin makes `ros2 launch` interactive: on Ctrl-C it does
        # not re-send SIGINT, because the terminal already sent it to the group.
        master, slave = pty.openpty()
        stdin = slave
    stack = subprocess.Popen(
        ["ros2", "launch", "go2_bringup", "system.launch.py", "perception:=none",
         "planner:=nav2", "localization:=slam_mapping", "lidar_safety:=true",
         "hardware_adapter:=unitree_sport", "cloud_in_topic:=/utlidar/cloud_deskewed"],
        env=env, stdin=stdin, stdout=open(log_dir / "stack.log", "w"),
        stderr=subprocess.STDOUT, start_new_session=True)
    if interactive:
        os.close(slave)
    run = Run(sim=sim, stack=stack, rec=rec, log_dir=log_dir, pty_master=master)
    run.notes["domain"] = domain
    ready = wait_for(
        lambda: rec.loc_valid and rec.gt and rec.nav.server_is_ready()
        and any(a == STOP_MOVE for _, a, _, _ in rec.requests[-50:]),
        READY_TIMEOUT_S, 0.2)
    if not ready:
        teardown_run(run)
        pytest.fail(f"stack not ready in {READY_TIMEOUT_S}s (loc={rec.loc_valid} "
                    f"gt={bool(rec.gt)} nav={rec.nav.server_is_ready()}); logs {log_dir}")
    time.sleep(3.0)  # a few SLAM scans before the first goal
    return run


def teardown_run(run: Run) -> dict:
    survivors = {}
    if run.stack.poll() is None or group_members(run.stack.pid):
        survivors["stack"] = stop_group(run.stack)
    else:
        survivors["stack"] = []
    survivors["sim"] = stop_group(run.sim)
    run.rec.close()
    if run.pty_master is not None:
        os.close(run.pty_master)
    return survivors


def send_goal_and_cruise(run: Run, timeout: float = 40.0) -> None:
    rec = run.rec
    goal = rec.NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = GOAL
    goal.pose.pose.orientation.w = 1.0
    handle = wait_future(rec.nav.send_goal_async(goal), 10.0)
    assert handle is not None and handle.accepted, "goal not accepted"
    run.goal_handle = handle
    # Cruise: forward Moves at >= 0.2 for a sustained second.
    t0 = time.monotonic()
    first_fast = None
    while time.monotonic() - t0 < timeout:
        with rec.lock:
            fast = [t for t, a, s, _ in rec.requests if a == MOVE and s >= 0.2 and t >= t0]
        if fast and first_fast is None:
            first_fast = fast[0]
        if first_fast is not None and time.monotonic() - first_fast >= 1.5:
            with rec.lock:
                recent = [s for t, a, s, _ in rec.requests if t >= time.monotonic() - 0.3]
            if recent and all(s >= 0.1 for s in recent):
                return
        time.sleep(0.05)
    pytest.fail(f"robot never reached steady motion toward {GOAL}; logs {run.log_dir}")


# ── Events ───────────────────────────────────────────────────────────────


def ev_launch_signal(sig):
    def ev(run: Run):
        os.kill(run.stack.pid, sig)
    return ev


def ev_group_signal(sig):
    def ev(run: Run):
        os.killpg(run.stack.pid, sig)
    return ev


def ev_kill_node(package, executable, sig):
    def ev(run: Run):
        pid = find_node_pid(run.stack.pid, _exe(package, executable))
        run.notes["target_pid"] = pid
        os.kill(pid, sig)
    return ev


def ev_cancel(run: Run):
    run.notes["cancel_future"] = run.goal_handle.cancel_goal_async()


def ev_lifecycle(command_name):
    def ev(run: Run):
        rec = run.rec
        assert rec.lifecycle.wait_for_service(timeout_sec=5.0)
        req = rec.ManageLifecycleNodes.Request()
        req.command = getattr(rec.ManageLifecycleNodes.Request, command_name)
        run.notes["lifecycle_future"] = rec.lifecycle.call_async(req)
    return ev


# ── Analysis ─────────────────────────────────────────────────────────────


def analyze(requests, safe, gt, t_event: float) -> dict:
    r = {}
    before = [(t, a, s) for t, a, s, _ in requests if t < t_event]
    after = [(t, a, s) for t, a, s, _ in requests if t >= t_event]
    moves_before = [s for t, a, s in before if a == MOVE]
    r["speed_at_event"] = round(moves_before[-1], 3) if moves_before else 0.0
    r["damp_count"] = sum(1 for _, a, _, _ in requests if a == DAMP)
    stops = [t for t, a, _ in after if a == STOP_MOVE]
    moves = [(t, s) for t, a, s in after if a == MOVE]
    r["first_stop_s"] = round(stops[0] - t_event, 3) if stops else None
    r["last_move_s"] = round(moves[-1][0] - t_event, 3) if moves else None
    r["moves_after_event"] = len(moves)
    if stops:
        late = [t for t, _ in moves if t > stops[0] + MOVE_AFTER_STOP_TOLERANCE_S]
        r["moves_after_stop"] = len(late)
    else:
        r["moves_after_stop"] = None
    r["final_api"] = requests[-1][1] if requests else None
    # Blind hold: how long commanded speed stayed at (or above) the pre-event
    # speed before it started to fall or a StopMove arrived.
    hold_end = None
    for t, a, s in after:
        if a != MOVE or s < r["speed_at_event"] - 0.02:
            hold_end = t
            break
    r["blind_hold_s"] = round(hold_end - t_event, 3) if hold_end is not None else None
    # Translation stop: first request with no linear motion (a StopMove, or a
    # pure-rotation Move such as Nav2's Spin recovery).
    trans = [t for t, a, _, lin in requests if t >= t_event and (a != MOVE or lin < 0.01)]
    r["translation_stop_s"] = round(trans[0] - t_event, 3) if trans else None
    gt_before = [g for g in gt if g[0] <= t_event]
    if gt_before and gt:
        x0, y0 = gt_before[-1][1:]
        x1, y1 = gt[-1][1:]
        r["travel_after_event_m"] = round(math.hypot(x1 - x0, y1 - y0), 3)
    shutdown_msgs = [t for t, st, rs, _ in safe if t >= t_event and "SHUTDOWN" in rs]
    r["arbiter_shutdown_msg_s"] = (round(shutdown_msgs[0] - t_event, 3)
                                   if shutdown_msgs else None)
    return r


# ── Cases ────────────────────────────────────────────────────────────────


@dataclass
class Case:
    name: str
    event: Callable
    goal: bool = True
    stop_bound_s: float = 0.5
    observe_s: float = 6.0
    expect_exit: bool = False
    interactive: bool = False
    blind_hold_bound_s: Optional[float] = None
    translation_stop_bound_s: Optional[float] = None
    arbiter_exits: bool = False
    need_arbiter_shutdown_msg: bool = False
    # Nav2 recovery after a server crash (see RECOVERY_GAP): here all motion
    # must end within motion_end_bound_s; test_known_gap_recovery_motion
    # checks the stricter quiet_bound_s (motion ends once the stop chain has
    # run, no recovery motion afterwards).
    recovery_gap: bool = False
    motion_end_bound_s: Optional[float] = None
    quiet_bound_s: Optional[float] = None


# Bounds. The bridge sends StopMove on its own shutdown at once (0.5 s leaves
# room for launch forwarding the signal). A goal cancel or a lost controller
# is followed by the smoother's deceleration ramp, max speed 0.35 m/s at
# max_decel 0.45 m/s^2 = 0.78 s, plus the pipeline (20 Hz smoother, stamper,
# 20 Hz arbiter, 50 Hz bridge). The planner crash is detected by the lifecycle
# manager's bond timeout (4 s) before the nodes are deactivated.
# Without recovery (quiet_bound_s): 1.5 s for the controller crash (the ramp
# above, plus the 0.55 s yaw ramp overlap), 2.5 s for the planner crash (the
# BT notices the failed replan, measured 1.2 to 1.5 s, then the ramp).
# Motion after a server crash must end by that bond timeout plus the time to
# reset the servers (measured about 1 s): 6.5 s. For a controller crash the
# smoother must also give up on the dead controller quickly: it holds the last
# command for velocity_timeout (blind hold <= 0.5 s), then ramps down, so
# forward motion ends within velocity_timeout + 0.78 s + pipeline (<= 1.4 s).
# The first StopMove itself can come later there, because the Spin recovery
# (pure rotation) may start before the ramp reaches zero.
CASES = [
    Case("launch_sigint_idle", ev_launch_signal(signal.SIGINT), goal=False, expect_exit=True),
    Case("launch_sigint_goal", ev_launch_signal(signal.SIGINT), expect_exit=True),
    Case("ctrl_c_group_sigint_goal", ev_group_signal(signal.SIGINT), expect_exit=True,
         interactive=True),
    Case("group_sigterm_goal", ev_group_signal(signal.SIGTERM), expect_exit=True),
    Case("launch_sigterm_goal", ev_launch_signal(signal.SIGTERM), expect_exit=True),
    Case("nav2_goal_cancel", ev_cancel, stop_bound_s=1.5),
    Case("controller_server_sigkill",
         ev_kill_node("nav2_controller", "controller_server", signal.SIGKILL),
         stop_bound_s=6.5, blind_hold_bound_s=0.5, translation_stop_bound_s=1.4,
         recovery_gap=True, motion_end_bound_s=6.5, quiet_bound_s=1.5),
    Case("planner_server_sigkill",
         ev_kill_node("nav2_planner", "planner_server", signal.SIGKILL),
         stop_bound_s=7.0, observe_s=12.0, recovery_gap=True, motion_end_bound_s=6.5,
         quiet_bound_s=2.5),
    Case("lifecycle_shutdown", ev_lifecycle("SHUTDOWN"), stop_bound_s=1.5),
    Case("lifecycle_pause", ev_lifecycle("PAUSE"), stop_bound_s=1.5),
    Case("arbiter_sigint",
         ev_kill_node("go2_safety_arbiter", "safety_arbiter_node", signal.SIGINT),
         stop_bound_s=0.15, arbiter_exits=True, need_arbiter_shutdown_msg=True),
    Case("arbiter_sigterm",
         ev_kill_node("go2_safety_arbiter", "safety_arbiter_node", signal.SIGTERM),
         stop_bound_s=0.15, arbiter_exits=True, need_arbiter_shutdown_msg=True),
]

# Known, unfixed gaps: kept in the suite so they stay visible, and strict so a
# fix turns them into a failure that asks for the marker to be removed.
KNOWN_GAPS = {
    "launch_sigterm_goal": (
        "ros2 launch on SIGTERM cancels itself without stopping its children "
        "(launch_service.py _on_sigterm: 'can result in orphaned processes')"
    ),
}

RECOVERY_GAP = (
    "after a controller_server or planner_server crash the stock Humble BT runs "
    "its Spin recovery; behavior_server publishes /cmd_vel directly (not through "
    "the smoother), so the robot keeps moving after the stop chain has run, until "
    "the lifecycle manager's 4 s bond timeout resets the servers"
)

#: case name -> result dict, for the known-gap checks below.
RESULTS: dict = {}


def _params():
    out = []
    for c in CASES:
        marks = [pytest.mark.slow, pytest.mark.timeout(CASE_TIMEOUT_S)]
        if c.name in KNOWN_GAPS:
            marks.append(pytest.mark.xfail(strict=True, reason=KNOWN_GAPS[c.name]))
        out.append(pytest.param(c, id=c.name, marks=marks))
    return out


@pytest.mark.parametrize("case", _params())
def test_stop_case(case: Case, tmp_path):
    if os.environ.get("GO2_STOP_REG") != "1":
        pytest.skip("full-stack sim case; set GO2_STOP_REG=1 to run")
    rclpy = _ros_imports()
    out_dir = Path(os.environ.get("STOP_REG_OUT", tmp_path)) / case.name
    run = start_run(rclpy, out_dir, case.interactive)
    arbiter_pid = None
    result = {"case": case.name}
    survivors = {"stack": [], "sim": []}
    try:
        if case.goal:
            send_goal_and_cruise(run)
        else:
            time.sleep(1.0)
        if case.arbiter_exits:
            arbiter_pid = find_node_pid(
                run.stack.pid, _exe("go2_safety_arbiter", "safety_arbiter_node"))
        t_event = time.monotonic()
        case.event(run)
        exit_s = None
        if case.expect_exit:
            gone = wait_for(lambda: not group_members(run.stack.pid), 25.0, 0.05)
            exit_s = round(time.monotonic() - t_event, 2) if gone else None
            time.sleep(1.0)
        else:
            time.sleep(case.observe_s)
        if arbiter_pid is not None:
            result["arbiter_gone"] = wait_for(
                lambda: not os.path.exists(f"/proc/{arbiter_pid}")
                or open(f"/proc/{arbiter_pid}/stat").read().split(")")[-1].split()[0] == "Z",
                10.0)
        with run.rec.lock:
            requests = list(run.rec.requests)
            safe = list(run.rec.safe)
            gt = list(run.rec.gt)
        sim_state = run.rec.sim_state
    finally:
        survivors = teardown_run(run)
    result.update(analyze(requests, safe, gt, t_event))
    result["exit_s"] = exit_s
    result["sim_state_end"] = sim_state
    result["survivors"] = survivors
    result["target_pid"] = run.notes.get("target_pid")
    result["domain"] = run.notes.get("domain")
    (out_dir / "result.json").write_text(json.dumps(result, indent=2))
    print("RESULT " + json.dumps(result), flush=True)
    RESULTS[case.name] = result

    problems = []
    if case.goal and result["speed_at_event"] < 0.1:
        problems.append(f"precondition: robot not moving at event ({result['speed_at_event']})")
    if result["damp_count"]:
        problems.append(f"Damp sent {result['damp_count']}x")
    if result["first_stop_s"] is None:
        problems.append("no StopMove after the event")
    elif result["first_stop_s"] > case.stop_bound_s:
        problems.append(f"first StopMove {result['first_stop_s']}s > bound {case.stop_bound_s}s")
    if case.recovery_gap:
        if case.motion_end_bound_s is not None and (result["last_move_s"] or 0.0) > \
                case.motion_end_bound_s:
            problems.append(f"motion continued to +{result['last_move_s']}s > "
                            f"{case.motion_end_bound_s}s after the crash")
    elif result["moves_after_stop"]:
        problems.append(f"{result['moves_after_stop']} Move(s) later than StopMove+0.1s "
                        f"(last Move at +{result['last_move_s']}s)")
    if result["final_api"] != STOP_MOVE:
        problems.append(f"last Sport request was {result['final_api']}, not StopMove")
    if sim_state == "MOVING":
        problems.append("sim robot still MOVING at the end")
    if case.blind_hold_bound_s is not None and (
            result["blind_hold_s"] is None or result["blind_hold_s"] > case.blind_hold_bound_s):
        problems.append(f"blind hold {result['blind_hold_s']}s > {case.blind_hold_bound_s}s")
    if case.translation_stop_bound_s is not None and (
            result["translation_stop_s"] is None
            or result["translation_stop_s"] > case.translation_stop_bound_s):
        problems.append(f"forward motion lasted {result['translation_stop_s']}s > "
                        f"{case.translation_stop_bound_s}s")
    if case.need_arbiter_shutdown_msg and result["arbiter_shutdown_msg_s"] is None:
        problems.append("arbiter's SHUTDOWN stop never reached /cmd_vel_safe")
    if case.arbiter_exits and not result.get("arbiter_gone"):
        problems.append("arbiter did not exit")
    if case.expect_exit and exit_s is None:
        problems.append("stack process group still alive 25 s after the event")
    if survivors["stack"] or survivors["sim"]:
        problems.append(f"survivors after cleanup: {survivors}")
    assert not problems, f"{case.name}: " + "; ".join(problems) + f" (logs {out_dir})"


@pytest.mark.slow
@pytest.mark.xfail(strict=True, reason=RECOVERY_GAP)
@pytest.mark.parametrize("name", [c.name for c in CASES if c.recovery_gap])
def test_known_gap_recovery_motion(name):
    """After a server crash, all motion ends once the stop chain has run (no recovery motion)."""
    if name not in RESULTS:
        pytest.skip(f"{name} did not run in this session")
    r = RESULTS[name]
    bound = next(c.quiet_bound_s for c in CASES if c.name == name)
    assert (r["last_move_s"] or 0.0) <= bound, (
        f"last Move at +{r['last_move_s']}s > {bound}s (forward motion ended at "
        f"+{r['translation_stop_s']}s; later Moves are the recovery)")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-p", "no:cacheprovider"]))
