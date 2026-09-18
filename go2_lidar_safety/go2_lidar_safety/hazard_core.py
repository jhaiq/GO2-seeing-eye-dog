"""
Pure hazard evaluation for a point cloud already expressed in ``base_link``.

No ROS imports: this module is unit-tested without rclpy.

Semantics match ``go2_safety_monitor`` and the safety arbiter's hazard
vocabulary (``go2_safety_arbiter.core``):

* ``EMERGENCY_STOP`` and ``DROP_DETECTED`` are stop-class. The arbiter
  authorizes zero and latches them for ``hazard_clear_hold_sec``.
* ``SLOWDOWN`` is slowdown-class. The arbiter scales every limit by
  ``degraded_scale`` and does not latch it.
* ``RESTRICT:<flags>`` is directional. F forbids forward, B backward, W
  rotation; lateral motion is always forbidden under a restriction. The
  arbiter derates and zeroes only the forbidden components, immediately.
* ``CLEAR`` permits motion subject to limits.

Why directional. The first version asserted EMERGENCY_STOP whenever an
obstacle was within stop_distance ahead. The arbiter latches stops as total,
so the robot could not turn or back away either: in closed-loop sim the robot
deadlocked in front of furniture on its first turn and never recovered
(HAZARD_STOP on every tick for the rest of the run). A total stop is now
reserved for "no direction is safe" (F, B and W all restricted).

Distances "ahead" are measured from the FRONT FACE of the body box, not from
the ``base_link`` origin, so ``stop_distance_m`` means clearance in front of
the nose.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

CLEAR = "CLEAR"
SLOWDOWN = "SLOWDOWN"
EMERGENCY_STOP = "EMERGENCY_STOP"
DROP_DETECTED = "DROP_DETECTED"
RESTRICT_PREFIX = "RESTRICT:"


def restrict(flags: str) -> str:
    """Canonical restriction string, flags in F, B, W order."""
    ordered = "".join(f for f in "FBW" if f in flags)
    if ordered == "FBW":
        return EMERGENCY_STOP
    return RESTRICT_PREFIX + ordered


def severity(decision: str) -> int:
    """Higher wins when merging decisions."""
    if decision == EMERGENCY_STOP:
        return 10
    if decision == DROP_DETECTED:
        return 9
    if decision.startswith(RESTRICT_PREFIX):
        return 2 + len(decision) - len(RESTRICT_PREFIX)
    if decision == SLOWDOWN:
        return 1
    return 0


#: Kept for callers that index by name; see severity() for RESTRICT strings.
SEVERITY = {EMERGENCY_STOP: 10, DROP_DETECTED: 9, SLOWDOWN: 1, CLEAR: 0}


@dataclass(frozen=True)
class HazardParams:
    # Height band kept as potential obstacles, relative to base_link.
    # base_link is ~0.32 m above the floor when standing, so the floor sits at
    # about z = -0.32 and is excluded by z_min = -0.22 (10 cm curb margin).
    z_min: float = -0.22
    z_max: float = 0.60

    # Self-return box (GO2 body is ~0.70 x 0.31 m; legs splay slightly wider).
    body_x_min: float = -0.45
    body_x_max: float = 0.40
    body_half_width: float = 0.22

    # Corridor ahead: footprint half width plus a lateral margin on each side.
    footprint_half_width: float = 0.20
    corridor_margin: float = 0.10
    lookahead_m: float = 3.0

    stop_distance_m: float = 0.35
    slowdown_distance_m: float = 0.90

    # Behind the tail: backward motion is forbidden inside this distance.
    rear_stop_distance_m: float = 0.30

    # Rotation sweep: the body box's corners trace a circle of radius
    # hypot(max(|body_x_min|, body_x_max), body_half_width). An obstacle inside
    # that circle plus this margin forbids rotation (a turn would hit it).
    sweep_margin_m: float = 0.05

    # All-around guard: an obstacle this close to any side of the body box
    # derates motion (SLOWDOWN, not a stop, so the robot can still turn away).
    surround_radius_m: float = 0.15

    # Robustness: an obstacle is only real when at least this many returns
    # are at or inside its distance. A single speckle never stops the robot.
    min_obstacle_points: int = 3

    # Drop-off check: floor returns expected directly ahead.
    drop_check_enabled: bool = False
    floor_z: float = -0.32
    floor_tolerance_m: float = 0.08
    drop_band_near_m: float = 0.30
    drop_band_far_m: float = 0.80
    min_floor_points: int = 10

    def __post_init__(self) -> None:
        for name in ("stop_distance_m", "slowdown_distance_m", "lookahead_m",
                     "corridor_margin", "footprint_half_width", "body_half_width",
                     "surround_radius_m", "floor_tolerance_m",
                     "rear_stop_distance_m", "sweep_margin_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0, got {value}")
        if self.z_min >= self.z_max:
            raise ValueError("z_min must be below z_max")
        if self.body_x_min >= self.body_x_max:
            raise ValueError("body_x_min must be below body_x_max")
        if self.stop_distance_m > self.slowdown_distance_m:
            raise ValueError("stop_distance_m must not exceed slowdown_distance_m")
        if self.min_obstacle_points < 1:
            raise ValueError("min_obstacle_points must be >= 1")
        if self.drop_band_near_m >= self.drop_band_far_m:
            raise ValueError("drop_band_near_m must be below drop_band_far_m")

    @property
    def corridor_half_width(self) -> float:
        return self.footprint_half_width + self.corridor_margin

    @property
    def sweep_radius(self) -> float:
        reach = max(abs(self.body_x_min), abs(self.body_x_max))
        return math.hypot(reach, self.body_half_width) + self.sweep_margin_m


@dataclass(frozen=True)
class HazardDecision:
    decision: str
    nearest_ahead_m: float
    nearest_surround_m: float
    nearest_rear_m: float
    nearest_sweep_m: float
    floor_points_ahead: Optional[int]
    points_evaluated: int
    description: str

    @property
    def distance(self) -> float:
        """Distance reported in SafetyAlert.distance for this decision."""
        if self.decision == SLOWDOWN and self.nearest_ahead_m > self.nearest_surround_m:
            return self.nearest_surround_m
        if self.decision.startswith(RESTRICT_PREFIX) or self.decision == EMERGENCY_STOP:
            return min(self.nearest_ahead_m, self.nearest_rear_m, self.nearest_surround_m)
        return self.nearest_ahead_m


def _kth_smallest(values: np.ndarray, k: int) -> float:
    if values.size < k:
        return math.inf
    return float(np.partition(values, k - 1)[k - 1])


def evaluate(points_base: np.ndarray, params: HazardParams) -> HazardDecision:
    """Classify an (N, 3) cloud expressed in base_link."""
    pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    n_total = int(pts.shape[0])

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    in_body = (
        (x >= params.body_x_min)
        & (x <= params.body_x_max)
        & (np.abs(y) <= params.body_half_width)
    )
    in_band = (z >= params.z_min) & (z <= params.z_max)
    obstacles = pts[in_band & ~in_body]

    # Ahead: inside the corridor, beyond the nose, within lookahead.
    ox, oy = obstacles[:, 0], obstacles[:, 1]
    ahead_dist = ox - params.body_x_max
    in_corridor = (
        (ahead_dist >= 0.0)
        & (ahead_dist <= params.lookahead_m)
        & (np.abs(oy) <= params.corridor_half_width)
    )
    nearest_ahead = _kth_smallest(ahead_dist[in_corridor], params.min_obstacle_points)

    # Surround: distance from each obstacle to the body box, any direction.
    dx = np.maximum.reduce([params.body_x_min - ox, np.zeros_like(ox), ox - params.body_x_max])
    dy = np.maximum(np.abs(oy) - params.body_half_width, 0.0)
    box_dist = np.hypot(dx, dy)
    nearest_surround = _kth_smallest(box_dist, params.min_obstacle_points)

    # Behind: inside the corridor width, beyond the tail.
    rear_dist = params.body_x_min - ox
    in_rear = (rear_dist >= 0.0) & (rear_dist <= params.lookahead_m) & (
        np.abs(oy) <= params.corridor_half_width
    )
    nearest_rear = _kth_smallest(rear_dist[in_rear], params.min_obstacle_points)

    # Rotation sweep: radial distance of obstacles from base_link.
    nearest_sweep = _kth_smallest(np.hypot(ox, oy), params.min_obstacle_points)

    flags = ""
    if nearest_ahead < params.stop_distance_m:
        flags += "F"
    if nearest_rear < params.rear_stop_distance_m:
        flags += "B"
    if nearest_sweep < params.sweep_radius:
        flags += "W"

    decision = CLEAR
    description = "no obstacle in corridor"
    if flags:
        decision = restrict(flags)
        description = (
            f"restricted {flags}: ahead {nearest_ahead:.2f} m, rear {nearest_rear:.2f} m, "
            f"sweep {nearest_sweep:.2f} m (< {params.sweep_radius:.2f})"
        )
    elif nearest_ahead < params.slowdown_distance_m:
        decision = SLOWDOWN
        description = (
            f"obstacle {nearest_ahead:.2f} m ahead (slow < {params.slowdown_distance_m:.2f})"
        )
    elif nearest_surround < params.surround_radius_m:
        decision = SLOWDOWN
        description = f"obstacle {nearest_surround:.2f} m from body side"

    floor_count: Optional[int] = None
    if params.drop_check_enabled:
        fx = x - params.body_x_max
        floor = (
            (fx >= params.drop_band_near_m)
            & (fx <= params.drop_band_far_m)
            & (np.abs(y) <= params.corridor_half_width)
            & (np.abs(z - params.floor_z) <= params.floor_tolerance_m)
        )
        floor_count = int(np.count_nonzero(floor))
        if floor_count < params.min_floor_points and severity(DROP_DETECTED) > severity(decision):
            decision = DROP_DETECTED
            description = (
                f"only {floor_count} floor returns {params.drop_band_near_m:.1f}-"
                f"{params.drop_band_far_m:.1f} m ahead (need {params.min_floor_points})"
            )

    return HazardDecision(
        decision=decision,
        nearest_ahead_m=nearest_ahead,
        nearest_surround_m=nearest_surround,
        nearest_rear_m=nearest_rear,
        nearest_sweep_m=nearest_sweep,
        floor_points_ahead=floor_count,
        points_evaluated=n_total,
        description=description,
    )


def quaternion_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """3x3 rotation matrix from a unit quaternion (x, y, z, w)."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )


def transform_points(points: np.ndarray, rotation: np.ndarray, translation) -> np.ndarray:
    """Apply p' = R p + t to an (N, 3) array."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return pts @ np.asarray(rotation).T + np.asarray(translation, dtype=np.float64)
