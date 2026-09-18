"""
Estimate the base_link -> LiDAR extrinsic rotation from recorded data.

Why this exists. On the 2026-09-01 standing bag the floor plane in the
/utlidar/cloud frame is tilted ~14 deg, consistent with the ~165 deg mount
pitch in Unitree's URDF, but the tilt points ~132 deg away from where a
pure-pitch mount predicts. The frame therefore also carries a rotation about
the vertical that the URDF does not describe. Getting that wrong rotates every
obstacle around the robot, so it is measured, not assumed:

1. Floor (standing, still): RANSAC plane -> the rotation that levels the
   cloud and the LiDAR's height above the floor. Fixes roll and pitch.
2. Motion (short straight walk): align leveled clouds from two instants with
   2D ICP; the sensor's translation in its own leveled frame versus the
   odometry translation in base_link gives the remaining yaw.

x/y of the LiDAR in base_link are not observable from straight translation
and are taken from the URDF nominal. Everything here is pure numpy/scipy.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class FloorFit:
    normal: np.ndarray  # unit, in LiDAR frame, pointing from the LiDAR toward the floor
    height: float  # LiDAR distance to the floor plane (m)
    inlier_fraction: float


def fit_floor(points: np.ndarray, min_range: float = 0.6, max_range: float = 4.0,
              iters: int = 2000, tol: float = 0.03, seed: int = 0) -> FloorFit:
    """Dominant plane among points in a range band (the floor, for a standing robot)."""
    pts = np.asarray(points, dtype=float)
    r = np.linalg.norm(pts, axis=1)
    pts = pts[(r >= min_range) & (r <= max_range) & np.isfinite(pts).all(axis=1)]
    if len(pts) < 50:
        raise ValueError(f"too few points in range band for a floor fit ({len(pts)})")
    rng = np.random.default_rng(seed)
    best_count, best = 0, None
    for _ in range(iters):
        s = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        d = n @ s[0]
        count = int((np.abs(pts @ n - d) < tol).sum())
        if count > best_count:
            best_count, best = count, (n, d)
    n, d = best
    inliers = pts[np.abs(pts @ n - d) < tol]
    # Least-squares refinement on inliers.
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid)
    n = vt[-1]
    d = n @ centroid
    if d < 0:  # orient the normal from the sensor toward the plane
        n, d = -n, -d
    return FloorFit(normal=n, height=float(d), inlier_fraction=best_count / len(pts))


def rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation R with R @ a = b (unit vectors)."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        if c > 0:
            return np.eye(3)
        # 180 deg about any axis perpendicular to a
        perp = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
        axis = np.cross(a, perp)
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def leveling_rotation(floor: FloorFit) -> np.ndarray:
    """R0 mapping the LiDAR frame so the floor normal points to -z (floor below)."""
    return rotation_between(floor.normal, np.array([0.0, 0.0, -1.0]))


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def icp_2d(src: np.ndarray, dst: np.ndarray, iters: int = 40,
           max_dist: float = 0.5) -> tuple[np.ndarray, np.ndarray, float]:
    """Point-to-point 2D ICP: find R, t with R @ src + t ~ dst. Returns (R, t, rms)."""
    tree = cKDTree(dst)
    R, t = np.eye(2), np.zeros(2)
    rms = float("inf")
    for _ in range(iters):
        moved = src @ R.T + t
        dist, idx = tree.query(moved)
        keep = dist < max_dist
        if keep.sum() < 10:
            break
        a, b = moved[keep], dst[idx[keep]]
        ca, cb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        dR = (u @ vt).T
        if np.linalg.det(dR) < 0:
            vt[-1] *= -1
            dR = (u @ vt).T
        dt = cb - dR @ ca
        R, t = dR @ R, dR @ t + dt
        rms = float(np.sqrt(np.mean(dist[keep] ** 2)))
    return R, t, rms


def obstacle_slice(points_leveled: np.ndarray, height: float,
                   z_above_floor: tuple[float, float] = (0.15, 1.5),
                   max_range: float = 8.0) -> np.ndarray:
    """2D points from a leveled cloud within a height band above the floor."""
    z_floor = -height
    z = points_leveled[:, 2]
    keep = (z > z_floor + z_above_floor[0]) & (z < z_floor + z_above_floor[1])
    xy = points_leveled[keep, :2]
    return xy[np.linalg.norm(xy, axis=1) < max_range]


def yaw_from_motion(cloud_a: np.ndarray, cloud_b: np.ndarray, R0: np.ndarray, height: float,
                    base_translation: np.ndarray) -> Optional[tuple[float, float]]:
    """
    Yaw of the leveled LiDAR frame relative to base_link from one motion pair.

    cloud_a/cloud_b: raw LiDAR-frame points at t_a and t_b. base_translation:
    odometry displacement from t_a to t_b expressed in base_link at t_a
    (x forward). Returns (yaw, icp_rms) or None if the pair is unusable.
    """
    if np.linalg.norm(base_translation[:2]) < 0.2:
        return None
    a = obstacle_slice(cloud_a @ R0.T, height)
    b = obstacle_slice(cloud_b @ R0.T, height)
    if len(a) < 30 or len(b) < 30:
        return None
    # Scene points in frame b map into frame a by the sensor's motion:
    # p_a = R_ab p_b + t_ab, where t_ab is the sensor displacement in frame a.
    R_ab, t_ab, rms = icp_2d(b, a)
    if abs(math.atan2(R_ab[1, 0], R_ab[0, 0])) > math.radians(10):
        return None  # not a straight segment
    yaw = math.atan2(base_translation[1], base_translation[0]) - math.atan2(t_ab[1], t_ab[0])
    return (math.atan2(math.sin(yaw), math.cos(yaw)), rms)


def circular_median(angles: list[float]) -> float:
    """Median-like robust centre of angles: the sample minimising summed arc distance."""
    arr = np.asarray(angles)
    diffs = np.abs(np.angle(np.exp(1j * (arr[:, None] - arr[None, :]))))
    return float(arr[np.argmin(diffs.sum(axis=1))])


def compose_extrinsic(R0: np.ndarray, yaw: float) -> np.ndarray:
    """Rotation LiDAR -> base_link: level first, then undo the leveled frame's yaw."""
    return rot_z(yaw) @ R0


def rpy_from_matrix(R: np.ndarray) -> tuple[float, float, float]:
    """ZYX Euler (R = Rz(yaw) Ry(pitch) Rx(roll)), matching quaternion_from_rpy."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(yaw) @ Ry @ Rx
