#!/usr/bin/env python3
"""
Shared odometry for a robot bag whose own odometry is unusable (Marina di Pisa 14/10: /odometry/local is 2D, GLIM
diverged), computed ONCE and given to every method, as make_odom.sh does with GLIM+EKF for the lab bags.

KISS-ICP on /velodyne_points (no deskew), then:
  odom = base_link at the first scan, but gravity-aligned: roll/pitch of base_link from the Xsens /imu/data orientation
  (through the bag's base_link->imu_link TF) at the first scan, yaw 0. On a slope the robot does not start level, and
  "up" must be the real vertical for any ground model.
Writes <out>/ with /odometry/filtered + /tf odom->base_link at every scan stamp. Run with eval/datasets/.venv/bin/python.
usage: lidar_odom_bag.py <sensor_bag> <out_bag> [--max-range 40]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer

sys.path.insert(0, str(Path(__file__).resolve().parent))
from coda_to_bag import M, TS, header, quat_from_rot, rot_from_quat, transform_msg   # noqa: E402


def T_of(tr):
    T = np.eye(4)
    q, p = tr.rotation, tr.translation
    T[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def xyz(m):
    off = {f.name: f.offset for f in m.fields}
    b = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    P = np.stack([b[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1).astype(np.float64)
    return P[np.isfinite(P).all(1) & (np.abs(P).sum(1) > 0)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--max-range', type=float, default=40.0)
    a = ap.parse_args()
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP
    cfg = KISSConfig()
    cfg.data.max_range, cfg.data.min_range, cfg.data.deskew = a.max_range, 1.0, False
    cfg.mapping.voxel_size = a.max_range / 100.0
    icp = KissICP(cfg)

    T_base_velo = T_base_imu = None
    imu_q, stamps, poses = None, [], []
    with AnyReader([Path(a.bag)]) as r:
        cons = [c for c in r.connections if c.topic in ('/velodyne_points', '/tf', '/tf_static', '/imu/data')]
        for c, t, raw in r.messages(connections=cons):
            m = r.deserialize(raw, c.msgtype)
            if c.topic in ('/tf', '/tf_static'):
                for x in m.transforms:
                    if x.header.frame_id == 'base_link' and x.child_frame_id == 'velodyne':
                        T_base_velo = T_of(x.transform)
                    if x.header.frame_id == 'base_link' and x.child_frame_id == 'imu_link':
                        T_base_imu = T_of(x.transform)
            elif c.topic == '/imu/data':
                imu_q = m.orientation
            elif T_base_velo is not None:
                icp.register_frame(xyz(m), np.zeros(1))
                stamps.append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
                poses.append(icp.last_pose.copy())
                if len(poses) == 1:
                    R_wi = rot_from_quat(imu_q.x, imu_q.y, imu_q.z, imu_q.w)
                    R_wb = R_wi @ T_base_imu[:3, :3].T
                    yaw = np.arctan2(R_wb[1, 0], R_wb[0, 0])
                    cz, sz = np.cos(-yaw), np.sin(-yaw)
                    T_odom_base0 = np.eye(4)
                    T_odom_base0[:3, :3] = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ R_wb   # roll/pitch only
                if len(poses) % 500 == 0:
                    print(f'{len(poses)} scans', flush=True)
    W0i = np.linalg.inv(poses[0])
    Tbv, Tvb = T_base_velo, np.linalg.inv(T_base_velo)
    with Writer(Path(a.out), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        prev = None
        V3 = M['geometry_msgs/msg/Vector3']
        for t, W in zip(stamps, poses):
            T = T_odom_base0 @ Tbv @ W0i @ W @ Tvb
            q = quat_from_rot(T[:3, :3])
            v = np.zeros(3) if prev is None else T[:3, :3].T @ (T[:3, 3] - prev[1][:3, 3]) / ((t - prev[0]) * 1e-9)
            prev = (t, T)
            odom = M['nav_msgs/msg/Odometry'](
                header=header(t, 'odom'), child_frame_id='base_link',
                pose=M['geometry_msgs/msg/PoseWithCovariance'](pose=M['geometry_msgs/msg/Pose'](
                    position=M['geometry_msgs/msg/Point'](x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
                    orientation=M['geometry_msgs/msg/Quaternion'](x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))),
                    covariance=np.zeros(36)),
                twist=M['geometry_msgs/msg/TwistWithCovariance'](twist=M['geometry_msgs/msg/Twist'](
                    linear=V3(x=float(v[0]), y=float(v[1]), z=float(v[2])), angular=V3(x=0.0, y=0.0, z=0.0)),
                    covariance=np.zeros(36)))
            w.write(c_odom, t, TS.serialize_cdr(odom, 'nav_msgs/msg/Odometry'))
            w.write(c_tf, t, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](
                transforms=[transform_msg(T, 'odom', 'base_link', t)]), 'tf2_msgs/msg/TFMessage'))
    P = np.array([(T_odom_base0 @ Tbv @ W0i @ W @ Tvb)[:3, 3] for W in poses])
    R0 = T_odom_base0[:3, :3]
    print(f'done: {len(poses)} scans, path {np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1).sum():.1f} m, '
          f'z {P[:, 2].min():.2f}..{P[:, 2].max():.2f} m, start roll/pitch '
          f'{np.degrees(np.arctan2(R0[2, 1], R0[2, 2])):.1f}/{np.degrees(-np.arcsin(R0[2, 0])):.1f} deg')


if __name__ == '__main__':
    main()
