#!/usr/bin/env python3
"""
M-detector input bag (ROS1) for one CODa window: the window's /velodyne_points and its odometry expressed for the
LiDAR frame (odom <- base_link from <window>_odom, base_link <- velodyne from <window>_gt.json), one /odom_velodyne
message per cloud at the cloud's stamp (M-detector pairs the n-th cloud with the n-th odometry). Input adapter only.
usage: make_input_coda.py <window dir e.g. bags/coda/coda0_w1> <out.bag>
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation


def main():
    win, out = Path(sys.argv[1]), Path(sys.argv[2])
    ts1 = get_typestore(Stores.ROS1_NOETIC)
    T_bv = np.array(json.load(open(f'{win}_gt.json'))['T_base_velodyne'])
    tmp = out.with_suffix('.raw.bag')
    for f in (out, tmp):
        f.unlink(missing_ok=True)
    subprocess.run(['rosbags-convert', '--src', str(win), f'{win}_odom', '--dst', str(tmp), '--dst-typestore',
                    'ros1_noetic', '--include-topic', '/velodyne_points', '/odometry/filtered'], check=True)
    with Reader(tmp) as r:
        odom, clouds = {}, []
        for c, t, raw in r.messages():
            if c.topic == '/velodyne_points':
                clouds.append((t, raw))
            else:
                m = ts1.deserialize_ros1(raw, 'nav_msgs/msg/Odometry')
                odom[m.header.stamp.sec * 10**9 + m.header.stamp.nanosec] = m      # one per scan, same stamps
    with Writer(out) as w:
        c_pc = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=ts1)
        c_od = w.add_connection('/odom_velodyne', 'nav_msgs/msg/Odometry', typestore=ts1)
        for t, raw in clouds:
            pc = ts1.deserialize_ros1(raw, 'sensor_msgs/msg/PointCloud2')
            m = odom[pc.header.stamp.sec * 10**9 + pc.header.stamp.nanosec]
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [p.x, p.y, p.z]
            T = T @ T_bv
            p.x, p.y, p.z = T[:3, 3]
            q.x, q.y, q.z, q.w = Rotation.from_matrix(T[:3, :3]).as_quat()
            m.child_frame_id = 'velodyne'
            w.write(c_od, t - 1, ts1.serialize_ros1(m, 'nav_msgs/msg/Odometry'))
            w.write(c_pc, t, raw)
    tmp.unlink()
    print('wrote', out)


if __name__ == '__main__':
    main()
