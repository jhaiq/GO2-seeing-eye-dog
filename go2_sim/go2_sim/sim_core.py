"""
Pure simulation core for the kinematic GO2 simulator. No ROS imports.

Everything here is deterministic given its inputs (the lidar noise takes an
explicit numpy Generator), so the node is a thin transport layer over it and
the physics can be unit tested without a ROS graph.

Frames
------
``odom``          world frame, z up, floor at z = 0.
``base_link``     REP-103 body frame, at height ``BASE_HEIGHT_M`` above the floor.
``utlidar_lidar`` sensor frame, placed by the extrinsic ``base_link -> utlidar_lidar``.

What this is not
----------------
A kinematic model: the commanded body velocity is tracked through a
first-order lag. There are no legs, no gait, no slip and no dynamics. It is
fit for testing the software contract (who commands what, when motion stops,
whether frames and timestamps line up), not for predicting how the real
robot moves.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

#: Standing body height of base_link above the floor (measured 0.314 to 0.323 m
#: on the real robot, docs/go2_field_notes.md).
BASE_HEIGHT_M = 0.32

#: Clock skew between the robot's message stamps and the payload's wall clock,
#: measured on 2026-09-01 from a real bag (robot stamps read 2025-10-17).
DEFAULT_CLOCK_SKEW_SEC = -27605481.0

API_ID_DAMP = 1001
API_ID_STOP_MOVE = 1003
API_ID_MOVE = 1008


# ── Rotations ────────────────────────────────────────────────────────────────


def rot_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), the URDF / tf2 convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ── World ────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Box:
    """Oriented box: yaw about z, spanning [z_min, z_max] vertically."""

    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    z_min: float
    z_max: float
    label: str = ""

    def corners_2d(self) -> np.ndarray:
        hx, hy = self.size_x / 2.0, self.size_y / 2.0
        local = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rot = np.array([[c, -s], [s, c]])
        return local @ rot.T + np.array([self.x, self.y])


@dataclass
class World:
    obstacles: list[Box] = field(default_factory=list)
    objects: list[Box] = field(default_factory=list)
    start_x: float = 0.0
    start_y: float = 0.0
    start_yaw: float = 0.0

    @property
    def boxes(self) -> list[Box]:
        return self.obstacles + self.objects


def load_world(path: str | Path) -> World:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return world_from_dict(data)


def world_from_dict(data: dict) -> World:
    obstacles = [
        Box(
            x=float(o["x"]),
            y=float(o["y"]),
            yaw=float(o.get("yaw", 0.0)),
            size_x=float(o["size_x"]),
            size_y=float(o["size_y"]),
            z_min=0.0,
            z_max=float(o.get("height", 1.0)),
            label=str(o.get("label", "")),
        )
        for o in data.get("obstacles", []) or []
    ]
    objects = []
    for o in data.get("objects", []) or []:
        z = float(o.get("z", float(o["size_z"]) / 2.0))
        half_z = float(o["size_z"]) / 2.0
        objects.append(
            Box(
                x=float(o["x"]),
                y=float(o["y"]),
                yaw=float(o.get("yaw", 0.0)),
                size_x=float(o["size_x"]),
                size_y=float(o["size_y"]),
                z_min=z - half_z,
                z_max=z + half_z,
                label=str(o["label"]),
            )
        )
    start = data.get("start_pose", {}) or {}
    for box in obstacles + objects:
        if box.size_x <= 0 or box.size_y <= 0 or box.z_max <= box.z_min:
            raise ValueError(f"degenerate box in world: {box}")
    return World(
        obstacles=obstacles,
        objects=objects,
        start_x=float(start.get("x", 0.0)),
        start_y=float(start.get("y", 0.0)),
        start_yaw=float(start.get("yaw", 0.0)),
    )


# ── Collision (2D separating axis test) ──────────────────────────────────────


def footprint_corners(x: float, y: float, yaw: float, length: float, width: float) -> np.ndarray:
    return Box(x, y, yaw, length, width, 0.0, 1.0).corners_2d()


def _polygons_overlap(a: np.ndarray, b: np.ndarray) -> bool:
    for poly in (a, b):
        for i in range(len(poly)):
            edge = poly[(i + 1) % len(poly)] - poly[i]
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def footprint_collides(
    x: float,
    y: float,
    yaw: float,
    length: float,
    width: float,
    boxes: list[Box],
    body_height: float = 0.45,
) -> Optional[Box]:
    """Return the first box whose footprint overlaps the robot, or None.

    Boxes entirely above the robot body (z_min > body_height) do not collide,
    so a table top the robot can walk under is not a wall.
    """
    robot = footprint_corners(x, y, yaw, length, width)
    for box in boxes:
        if box.z_min > body_height:
            continue
        if _polygons_overlap(robot, box.corners_2d()):
            return box
    return None


# ── Kinematics ───────────────────────────────────────────────────────────────


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def lag_velocity(current: float, command: float, dt: float, tau: float) -> float:
    """Exact discretisation of a first-order lag toward ``command``."""
    if tau <= 0.0:
        return command
    alpha = 1.0 - math.exp(-dt / tau)
    return current + alpha * (command - current)


def step_kinematics(
    state: RobotState, cmd: tuple[float, float, float], dt: float, tau: float
) -> RobotState:
    """Advance one step: lag the body velocity, then integrate the pose.

    Velocities are in the body frame (REP-103). The pose update uses the
    midpoint heading so that constant-twist arcs integrate accurately.
    """
    vx = lag_velocity(state.vx, cmd[0], dt, tau)
    vy = lag_velocity(state.vy, cmd[1], dt, tau)
    wz = lag_velocity(state.wz, cmd[2], dt, tau)
    mid_yaw = state.yaw + 0.5 * wz * dt
    c, s = math.cos(mid_yaw), math.sin(mid_yaw)
    return RobotState(
        x=state.x + (vx * c - vy * s) * dt,
        y=state.y + (vx * s + vy * c) * dt,
        yaw=normalize_angle(state.yaw + wz * dt),
        vx=vx,
        vy=vy,
        wz=wz,
    )


# ── Sport API request parsing ────────────────────────────────────────────────


@dataclass(frozen=True)
class SportCommand:
    api_id: int
    velocity: Optional[tuple[float, float, float]]
    error: str = ""


def parse_sport_request(api_id: int, parameter: str) -> SportCommand:
    """Interpret a Sport API request as the simulator executes it.

    Move (1008) carries JSON ``{"x": vx, "y": vy, "z": wz}``. StopMove (1003)
    and Damp (1001) carry no parameters. Anything else is reported unsupported.
    """
    if api_id == API_ID_MOVE:
        try:
            params = json.loads(parameter)
            vel = (float(params["x"]), float(params["y"]), float(params["z"]))
        except (ValueError, KeyError, TypeError) as exc:
            return SportCommand(api_id, None, f"malformed Move parameter {parameter!r}: {exc}")
        if not all(math.isfinite(v) for v in vel):
            return SportCommand(api_id, None, f"non-finite Move velocity {vel}")
        return SportCommand(api_id, vel)
    if api_id in (API_ID_STOP_MOVE, API_ID_DAMP):
        return SportCommand(api_id, (0.0, 0.0, 0.0))
    return SportCommand(api_id, None, f"unsupported api_id {api_id}")


# ── Clock skew ───────────────────────────────────────────────────────────────


def skewed_stamp_ns(now_ns: int, skew_sec: float) -> int:
    """The robot-clock stamp for a wall-clock instant."""
    return int(now_ns + round(skew_sec * 1e9))


# ── Lidar ────────────────────────────────────────────────────────────────────


def beam_directions(
    n_azimuth: int = 120,
    n_elevation: int = 16,
    elev_min_deg: float = -5.0,
    elev_max_deg: float = 85.0,
) -> np.ndarray:
    """Unit ray directions in the sensor frame, a hemisphere-ish grid about +z.

    Elevation is measured from the sensor xy plane toward +z.
    """
    az = np.linspace(0.0, 2.0 * math.pi, n_azimuth, endpoint=False)
    el = np.radians(np.linspace(elev_min_deg, elev_max_deg, n_elevation))
    az_g, el_g = np.meshgrid(az, el)
    dirs = np.stack(
        [np.cos(el_g) * np.cos(az_g), np.cos(el_g) * np.sin(az_g), np.sin(el_g)], axis=-1
    ).reshape(-1, 3)
    return dirs / np.linalg.norm(dirs, axis=1, keepdims=True)


def raycast(
    origin: np.ndarray, dirs: np.ndarray, boxes: list[Box], max_range: float
) -> np.ndarray:
    """Distance along each ray to the nearest surface (floor z=0 or a box).

    ``origin`` (3,) and ``dirs`` (N, 3) are in the world frame. Returns (N,)
    with ``inf`` for rays that hit nothing within ``max_range``. Vectorised as
    rays x boxes with a slab test in each box's own frame.
    """
    t_best = np.full(len(dirs), np.inf)

    # Floor.
    dz = dirs[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(dz < -1e-9, -origin[2] / dz, np.inf)
    t_floor[t_floor <= 0.0] = np.inf
    t_best = np.minimum(t_best, t_floor)

    if boxes:
        centers = np.array([[b.x, b.y] for b in boxes])  # (B, 2)
        yaws = np.array([b.yaw for b in boxes])
        half = np.array([[b.size_x / 2.0, b.size_y / 2.0] for b in boxes])
        zlo = np.array([b.z_min for b in boxes])
        zhi = np.array([b.z_max for b in boxes])
        c, s = np.cos(yaws), np.sin(yaws)  # (B,)

        # Origin and directions in each box frame (rotation by -yaw about z).
        ox = origin[0] - centers[:, 0]
        oy = origin[1] - centers[:, 1]
        o_bx = c * ox + s * oy  # (B,)
        o_by = -s * ox + c * oy
        d_bx = np.outer(dirs[:, 0], c) + np.outer(dirs[:, 1], s)  # (N, B)
        d_by = -np.outer(dirs[:, 0], s) + np.outer(dirs[:, 1], c)
        d_bz = np.repeat(dirs[:, 2:3], len(boxes), axis=1)

        def slab(o, d, lo, hi):
            with np.errstate(divide="ignore", invalid="ignore"):
                inv = 1.0 / d
                t1 = (lo - o) * inv
                t2 = (hi - o) * inv
            tmin = np.minimum(t1, t2)
            tmax = np.maximum(t1, t2)
            # Parallel rays: inside the slab means unconstrained, outside means miss.
            parallel = np.abs(d) < 1e-12
            inside = (o >= lo) & (o <= hi)
            tmin = np.where(parallel, np.where(inside, -np.inf, np.inf), tmin)
            tmax = np.where(parallel, np.where(inside, np.inf, -np.inf), tmax)
            return tmin, tmax

        tx0, tx1 = slab(o_bx, d_bx, -half[:, 0], half[:, 0])
        ty0, ty1 = slab(o_by, d_by, -half[:, 1], half[:, 1])
        tz0, tz1 = slab(origin[2], d_bz, zlo, zhi)
        t_near = np.maximum(np.maximum(tx0, ty0), tz0)
        t_far = np.minimum(np.minimum(tx1, ty1), tz1)
        # A sensor inside a box sees nothing of that box (t_near < 0).
        hit = (t_near <= t_far) & (t_near > 0.0)
        t_box = np.where(hit, t_near, np.inf).min(axis=1)
        t_best = np.minimum(t_best, t_box)

    t_best[t_best > max_range] = np.inf
    return t_best


@dataclass(frozen=True)
class LidarExtrinsic:
    xyz: tuple[float, float, float] = (0.29, 0.0, -0.05)
    rpy: tuple[float, float, float] = (0.0, 2.8782, 0.0)

    def rotation(self) -> np.ndarray:
        return rot_rpy(*self.rpy)


def lidar_pose_in_odom(
    state: RobotState, extrinsic: LidarExtrinsic, base_height: float = BASE_HEIGHT_M
) -> tuple[np.ndarray, np.ndarray]:
    """(R_odom_lidar, p_odom_lidar) for the current robot pose."""
    r_ob = rot_rpy(0.0, 0.0, state.yaw)
    p_ob = np.array([state.x, state.y, base_height])
    r_bl = extrinsic.rotation()
    p_bl = np.array(extrinsic.xyz, dtype=float)
    return r_ob @ r_bl, p_ob + r_ob @ p_bl


def simulate_scan(
    state: RobotState,
    world: World,
    extrinsic: LidarExtrinsic,
    dirs_sensor: np.ndarray,
    max_range: float = 10.0,
    noise_std: float = 0.01,
    rng: Optional[np.random.Generator] = None,
    base_height: float = BASE_HEIGHT_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (points in sensor frame, points in odom frame), both (M, 3)."""
    r_ol, p_ol = lidar_pose_in_odom(state, extrinsic, base_height)
    dirs_world = dirs_sensor @ r_ol.T
    t = raycast(p_ol, dirs_world, world.boxes, max_range)
    keep = np.isfinite(t)
    t = t[keep]
    if noise_std > 0.0 and len(t):
        rng = rng if rng is not None else np.random.default_rng()
        t = t + rng.normal(0.0, noise_std, size=t.shape)
    pts_sensor = dirs_sensor[keep] * t[:, None]
    pts_odom = pts_sensor @ r_ol.T + p_ol
    return pts_sensor.astype(np.float32), pts_odom.astype(np.float32)
