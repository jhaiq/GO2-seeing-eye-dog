"""Extrinsic recovery on synthetic scenes with a known, off-URDF LiDAR rotation."""
import math

import numpy as np
import pytest
from go2_localization.lidar_calibration import (
    circular_median,
    compose_extrinsic,
    fit_floor,
    leveling_rotation,
    matrix_from_rpy,
    rot_z,
    rpy_from_matrix,
    yaw_from_motion,
)

BASE_H = 0.32
LIDAR_T = np.array([0.29, 0.0, -0.05])


def _world(rng):
    floor = np.c_[rng.uniform(-5, 5, 6000), rng.uniform(-5, 5, 6000), np.zeros(6000)]
    walls = []
    for x0, y0, x1, y1 in [(-4, -3, 4, -3), (-4, 3, 4, 3), (4, -3, 4, 3), (-4, -3, -4, 3),
                           (1.0, -1.0, 1.6, -0.4), (-2.0, 1.0, -1.2, 1.8)]:
        s = rng.uniform(0, 1, 1500)
        z = rng.uniform(0, 1.2, 1500)
        walls.append(np.c_[x0 + s * (x1 - x0), y0 + s * (y1 - y0), z])
    return np.vstack([floor] + walls)


def _to_lidar(world, base_xy, base_yaw, R_true, rng):
    Rb = rot_z(base_yaw)
    tb = np.array([base_xy[0], base_xy[1], BASE_H])
    p_base = (world - tb) @ Rb  # R_b^T (p - t_b)
    p_lidar = (p_base - LIDAR_T) @ R_true  # R_true^T (p_base - t_l)
    keep = np.linalg.norm(p_lidar, axis=1) < 8.0
    pts = p_lidar[keep]
    idx = rng.choice(len(pts), size=min(len(pts), 12000), replace=False)
    return pts[idx] + rng.normal(0, 0.005, (len(idx), 3))


@pytest.mark.parametrize("true_rpy", [(0.0, 2.8782, 0.0), (0.2, 2.9, 2.3), (-0.1, 2.85, -1.4)])
def test_recovers_rotation_including_off_model_yaw(true_rpy):
    rng = np.random.default_rng(1)
    R_true = matrix_from_rpy(*true_rpy)
    world = _world(rng)

    standing = _to_lidar(world, (0.0, 0.0), 0.3, R_true, rng)
    floor = fit_floor(standing)
    expected_height = BASE_H + (R_true.T @ np.zeros(3) + LIDAR_T)[2]
    assert floor.height == pytest.approx(expected_height, abs=0.02)
    R0 = leveling_rotation(floor)

    yaws = []
    for step in range(4):
        start = (0.2 * step, 0.0)
        a = _to_lidar(world, start, 0.3, R_true, rng)
        b = _to_lidar(world, (start[0] + 0.5 * math.cos(0.3), start[1] + 0.5 * math.sin(0.3)),
                      0.3, R_true, rng)
        res = yaw_from_motion(a, b, R0, floor.height, np.array([0.5, 0.0, 0.0]))
        assert res is not None
        yaws.append(res[0])
    R_est = compose_extrinsic(R0, circular_median(yaws))
    err = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_est.T @ R_true) - 1) / 2))))
    assert err < 1.5, f"rotation error {err:.2f} deg"


def test_rpy_roundtrip():
    for rpy in [(0.1, -0.4, 2.0), (0.0, 1.2, -3.0), (-0.3, 0.2, 0.7)]:
        assert np.allclose(rpy_from_matrix(matrix_from_rpy(*rpy)), rpy, atol=1e-9)


def test_short_motion_is_rejected():
    rng = np.random.default_rng(0)
    R_true = matrix_from_rpy(0, 2.8782, 0)
    world = _world(rng)
    a = _to_lidar(world, (0, 0), 0, R_true, rng)
    floor = fit_floor(a)
    assert yaw_from_motion(a, a, leveling_rotation(floor), floor.height, np.array([0.05, 0, 0])) is None


def test_floor_fit_needs_points():
    with pytest.raises(ValueError):
        fit_floor(np.zeros((10, 3)))
