#!/usr/bin/env python3
"""
Input bag for M-detector (ROS1): the raw /velodyne_points and the shared odometry expressed for the LiDAR frame.

M-detector takes the point cloud in the sensor ("body") frame plus the odometry of that same frame (upstream it is
FAST-LIO's /aft_mapped_to_init). The benchmark's shared odometry (bags/replay/odom_<seq>, /odometry/filtered) is the
pose of base_link in odom; here it is composed with the URDF base_link -> velodyne transform (pure translation
0.03, 0.029, 0.6 m) and published as /odom_velodyne (frame odom, child velodyne), ONE message per cloud at the cloud's
stamp (position interpolated): upstream pairs the n-th cloud with the n-th odometry message (FAST-LIO publishes one
per scan), so a 40 Hz odometry would be paired with the wrong clouds. Input adapter only.

usage: make_input.py <seq e.g. 20260519_1950> <out.bag>
"""

import sys
from pathlib import Path

import numpy as np
import subprocess

from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

R = Path(__file__).resolve().parents[3]
BASE_TO_VELODYNE = np.array([0.03, 0.029, 0.6])


def quat_rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def main():
    seq, out = sys.argv[1], Path(sys.argv[2])
    ts1 = get_typestore(Stores.ROS1_NOETIC)
    tmp = out.with_suffix('.raw.bag')
    for f in (out, tmp):
        f.unlink(missing_ok=True)
    # 1) ROS2 -> ROS1 with rosbags-convert (message definitions converted), 2) rewrite the odometry in ROS1
    subprocess.run(['rosbags-convert', '--src', str(R / 'bags' / 'bags_benchmark' / f'duccio_test_{seq}_0'),
                    str(R / 'bags' / 'replay' / f'odom_{seq}'), '--dst', str(tmp), '--dst-typestore', 'ros1_noetic',
                    '--include-topic', '/velodyne_points', '/odometry/filtered'], check=True)
    with Reader(tmp) as r:
        odom, clouds = [], []
        for c, t, raw in r.messages():
            if c.topic == '/velodyne_points':
                clouds.append((t, raw))
            else:
                m = ts1.deserialize_ros1(raw, 'nav_msgs/msg/Odometry')
                odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m))
    ot = np.array([o[0] for o in odom])
    with Writer(out) as w:
        c_pc = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=ts1)
        c_od = w.add_connection('/odom_velodyne', 'nav_msgs/msg/Odometry', typestore=ts1)
        for t, raw in clouds:
            pc = ts1.deserialize_ros1(raw, 'sensor_msgs/msg/PointCloud2')
            st = pc.header.stamp.sec + pc.header.stamp.nanosec * 1e-9
            i = int(np.clip(np.searchsorted(ot, st), 1, len(ot) - 1))
            (t0, m0), (t1, m1) = odom[i - 1], odom[i]
            a = float(np.clip((st - t0) / max(t1 - t0, 1e-9), 0.0, 1.0))
            m = m0 if a < 0.5 else m1          # orientation of the nearest sample (40 Hz, planar motion)
            p0, p1 = m0.pose.pose.position, m1.pose.pose.position
            pos = np.array([p0.x, p0.y, p0.z]) * (1 - a) + np.array([p1.x, p1.y, p1.z]) * a
            q = m.pose.pose.orientation
            pos = pos + quat_rotate((q.x, q.y, q.z, q.w), BASE_TO_VELODYNE)
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = pos
            m.header.stamp = pc.header.stamp
            m.child_frame_id = 'velodyne'
            # one odometry per cloud, written just before it: M-detector pairs the n-th cloud with the n-th odometry
            w.write(c_od, t - 1, ts1.serialize_ros1(m, 'nav_msgs/msg/Odometry'))
            w.write(c_pc, t, raw)
    tmp.unlink()
    print('wrote', out)


if __name__ == '__main__':
    main()
