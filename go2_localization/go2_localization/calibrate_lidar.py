"""
calibrate_lidar: estimate base_link -> utlidar_lidar from a rosbag.

    ros2 run go2_localization calibrate_lidar <bag> [--cloud /utlidar/cloud]
        [--odom /utlidar/robot_odom] [--base-height 0.32]

Record the bag STANDING (not sitting: the field notes measured a 23 deg roll
difference between poses), first still for ~5 s, then walking straight
forward ~1.5 m at low speed. Without the walk, only roll/pitch/height are
measured and yaw is reported as UNVERIFIED (URDF nominal).

Prints a relay.yaml snippet. It never edits configuration itself.
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from go2_localization.lidar_calibration import (
    circular_median,
    compose_extrinsic,
    fit_floor,
    leveling_rotation,
    matrix_from_rpy,
    rpy_from_matrix,
    yaw_from_motion,
)

URDF_XYZ = (0.28945, 0.0, -0.046825)
URDF_RPY = (0.0, 2.8782, 0.0)


def _quat_yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path: str, cloud_topic: str, odom_topic: str):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[cloud_topic, odom_topic]))
    clouds, odoms, frames = [], [], set()
    while reader.has_next():
        topic, data, t_rx = reader.read_next()
        if topic == cloud_topic:
            msg = deserialize_message(data, PointCloud2)
            frames.add(msg.header.frame_id)
            # The L1 cloud mixes field types (ring/time), so read structured.
            rec = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts = np.stack([rec["x"], rec["y"], rec["z"]], axis=1).astype(float)
            clouds.append((t_rx * 1e-9, pts))
        else:
            msg = deserialize_message(data, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            odoms.append((t_rx * 1e-9, p.x, p.y, p.z, _quat_yaw(q)))
    return clouds, np.array(odoms), frames


def _odom_at(odoms: np.ndarray, t: float) -> np.ndarray:
    i = int(np.clip(np.searchsorted(odoms[:, 0], t), 0, len(odoms) - 1))
    return odoms[i]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--cloud", default="/utlidar/cloud")
    ap.add_argument("--odom", default="/utlidar/robot_odom")
    ap.add_argument("--base-height", type=float, default=None,
                    help="base_link height above floor; default: median odom z while still")
    args = ap.parse_args(argv)

    clouds, odoms, frames = read_bag(args.bag, args.cloud, args.odom)
    if not clouds or len(odoms) == 0:
        print("ERROR: bag has no cloud or odom messages on the given topics", file=sys.stderr)
        return 2
    print(f"clouds={len(clouds)} odom={len(odoms)} cloud frame(s)={sorted(frames)}")

    speed = np.r_[0, np.linalg.norm(np.diff(odoms[:, 1:3], axis=0), axis=1) /
                  np.maximum(np.diff(odoms[:, 0]), 1e-3)]
    still = [c for c in clouds if speed[min(np.searchsorted(odoms[:, 0], c[0]), len(speed) - 1)] < 0.02]
    if len(still) < 5:
        print("ERROR: fewer than 5 clouds while standing still; record ~5 s still first", file=sys.stderr)
        return 2
    floor = fit_floor(np.vstack([p for _, p in still[:40]]))
    base_h = args.base_height
    if base_h is None:
        base_h = float(np.median([_odom_at(odoms, t)[3] for t, _ in still]))
    R0 = leveling_rotation(floor)
    print(f"floor: normal={np.round(floor.normal, 4)} lidar height={floor.height:.3f} m "
          f"inliers={floor.inlier_fraction:.2f} base height={base_h:.3f} m")

    yaws, rms = [], []
    cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(odoms[:, 1:3], axis=0), axis=1))]
    i = 0
    while i < len(clouds):
        ia = int(np.clip(np.searchsorted(odoms[:, 0], clouds[i][0]), 0, len(odoms) - 1))
        if speed[ia] < 0.05:
            i += 1
            continue
        paired = False
        for j in range(i + 1, len(clouds)):
            if clouds[j][0] - clouds[i][0] > 6.0:
                break
            ib = int(np.clip(np.searchsorted(odoms[:, 0], clouds[j][0]), 0, len(odoms) - 1))
            oa, ob = odoms[ia], odoms[ib]
            d_world = ob[1:3] - oa[1:3]
            dist = float(np.linalg.norm(d_world))
            if dist < 0.4:
                continue
            path = cum[ib] - cum[ia]
            turned = abs(math.remainder(ob[4] - oa[4], 2 * math.pi))
            if path > 1.1 * dist or turned > math.radians(5) or dist > 1.5:
                break  # not a straight segment starting at i
            c, s = math.cos(oa[4]), math.sin(oa[4])
            d_base = np.array([c * d_world[0] + s * d_world[1], -s * d_world[0] + c * d_world[1], 0.0])
            res = yaw_from_motion(clouds[i][1], clouds[j][1], R0, floor.height, d_base)
            if res is not None:
                yaws.append(res[0])
                rms.append(res[1])
                paired = True
            break
        i = i + 5 if paired else i + 1

    spread = None
    if yaws:
        centre = circular_median(yaws)
        spread = math.degrees(float(np.std(np.angle(np.exp(1j * (np.array(yaws) - centre))))))
    if yaws and spread is not None and spread > 5.0:
        print(f"WARNING: {len(yaws)} motion pairs disagree (spread {spread:.1f} deg); "
              "yaw NOT accepted", file=sys.stderr)
        yaws = []

    if yaws:
        yaw = circular_median(yaws)
        R = compose_extrinsic(R0, yaw)
        yaw_note = f"MEASURED from {len(yaws)} motion pairs, spread {spread:.1f} deg, icp rms {np.median(rms):.3f} m"
    else:
        # Keep URDF yaw: choose the leveled-frame yaw closest to the URDF rotation.
        R_urdf = matrix_from_rpy(*URDF_RPY)
        best = min(np.linspace(-math.pi, math.pi, 721),
                   key=lambda y: np.linalg.norm(compose_extrinsic(R0, y) - R_urdf))
        R = compose_extrinsic(R0, float(best))
        yaw_note = "UNVERIFIED: no straight-walk segment found; yaw taken from the URDF"

    roll, pitch, yaw_e = rpy_from_matrix(R)
    # Floor is base_h below base_link and floor.height below the LiDAR.
    lidar_z = floor.height - base_h
    print(f"yaw: {yaw_note}")
    print("\n# relay.yaml (go2_state_relay_node) for cloud_in_topic: " + args.cloud)
    print("    publish_lidar_extrinsic: true")
    print(f"    lidar_xyz: [{URDF_XYZ[0]}, {URDF_XYZ[1]}, {lidar_z:.4f}]  # x,y URDF nominal; z measured")
    print(f"    lidar_rpy: [{roll:.4f}, {pitch:.4f}, {yaw_e:.4f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
