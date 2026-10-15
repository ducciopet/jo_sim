#!/usr/bin/env python3
"""
Input bag for M-detector (ROS1): the raw /velodyne_points and the shared odometry expressed for the LiDAR frame.

M-detector takes the point cloud in the sensor ("body") frame plus the odometry of that same frame (upstream it is
FAST-LIO's /aft_mapped_to_init). The benchmark's shared odometry (bags/replay/odom_<seq>, /odometry/filtered) is the
pose of base_link in odom; here it is composed with the URDF base_link -> velodyne transform (pure translation
0.03, 0.029, 0.6 m) and published as /odom_velodyne (frame odom, child velodyne). Input adapter only.

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
    with Reader(tmp) as r, Writer(out) as w:
        c_pc = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=ts1)
        c_od = w.add_connection('/odom_velodyne', 'nav_msgs/msg/Odometry', typestore=ts1)
        for c, t, raw in r.messages():
            if c.topic == '/velodyne_points':
                w.write(c_pc, t, raw)
                continue
            m = ts1.deserialize_ros1(raw, 'nav_msgs/msg/Odometry')
            p, q = m.pose.pose.position, m.pose.pose.orientation
            off = quat_rotate((q.x, q.y, q.z, q.w), BASE_TO_VELODYNE)
            p.x, p.y, p.z = p.x + off[0], p.y + off[1], p.z + off[2]
            m.child_frame_id = 'velodyne'
            w.write(c_od, t, ts1.serialize_ros1(m, 'nav_msgs/msg/Odometry'))
    tmp.unlink()
    print('wrote', out)


if __name__ == '__main__':
    main()
