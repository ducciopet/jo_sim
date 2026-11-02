#!/usr/bin/env python3
"""
/glim_ros/odom (odom -> imu_link) of a GLIM replay -> /odometry/filtered + /tf odom->base_link, as lidar_odom_bag.py.
GLIM publishes more than one estimate on the same topic (seen on Marina di Pisa 14/10, also in the robot's own
recording: one at the origin, others drifting hundreds of metres): the main stream is the one starting at the origin,
followed by continuity (< --jump m between consecutive messages); the rest is dropped.
usage: glim_odom_bag.py <glim_bag> <sensor_bag (for base_link->imu_link)> <out_bag> [--jump 2.0]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer

sys.path.insert(0, str(Path(__file__).resolve().parent))
from coda_to_bag import M, TS, header, quat_from_rot, rot_from_quat, transform_msg   # noqa: E402
from lidar_odom_bag import T_of                                                      # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('glim')
    ap.add_argument('sensor')
    ap.add_argument('out')
    ap.add_argument('--jump', type=float, default=2.0)
    a = ap.parse_args()
    with AnyReader([Path(a.sensor)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == '/tf_static']):
            for x in r.deserialize(raw, c.msgtype).transforms:
                if (x.header.frame_id, x.child_frame_id) == ('base_link', 'imu_link'):
                    T_base_imu = T_of(x.transform)
    msgs = []
    with AnyReader([Path(a.glim)]) as r:
        c = [c for c in r.connections if c.topic == '/glim_ros/odom']
        for con, t, raw in r.messages(connections=c):
            m = r.deserialize(raw, con.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T = np.eye(4)
            T[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
            T[:3, 3] = [p.x, p.y, p.z]
            msgs.append((m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, T))
    main_, last = [], None
    for st, T in msgs:
        if last is None:
            if np.linalg.norm(T[:3, 3]) < a.jump:
                main_.append((st, T)); last = T
        elif np.linalg.norm(T[:3, 3] - last[:3, 3]) < a.jump and st > main_[-1][0]:
            main_.append((st, T)); last = T
    Tib = np.linalg.inv(T_base_imu)
    T0 = main_[0][1] @ Tib
    # odom = base_link at the first message, gravity-aligned (GLIM's frame is): keep roll/pitch, drop yaw and position
    yaw = np.arctan2(T0[1, 0], T0[0, 0])
    Rz = np.eye(4); Rz[:2, :2] = [[np.cos(-yaw), -np.sin(-yaw)], [np.sin(-yaw), np.cos(-yaw)]]
    Rz[:3, 3] = -(Rz[:3, :3] @ T0[:3, 3])
    with Writer(Path(a.out), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        V3 = M['geometry_msgs/msg/Vector3']
        for st, Tg in main_:
            T = Rz @ Tg @ Tib
            q = quat_from_rot(T[:3, :3])
            odom = M['nav_msgs/msg/Odometry'](
                header=header(st, 'odom'), child_frame_id='base_link',
                pose=M['geometry_msgs/msg/PoseWithCovariance'](pose=M['geometry_msgs/msg/Pose'](
                    position=M['geometry_msgs/msg/Point'](x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
                    orientation=M['geometry_msgs/msg/Quaternion'](x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))),
                    covariance=np.zeros(36)),
                twist=M['geometry_msgs/msg/TwistWithCovariance'](twist=M['geometry_msgs/msg/Twist'](
                    linear=V3(x=0.0, y=0.0, z=0.0), angular=V3(x=0.0, y=0.0, z=0.0)), covariance=np.zeros(36)))
            w.write(c_odom, st, TS.serialize_cdr(odom, 'nav_msgs/msg/Odometry'))
            w.write(c_tf, st, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](
                transforms=[transform_msg(T, 'odom', 'base_link', st)]), 'tf2_msgs/msg/TFMessage'))
    P = np.array([(Rz @ Tg @ Tib)[:3, 3] for _, Tg in main_])
    print(f'done: {len(main_)}/{len(msgs)} messages in the main stream, path '
          f'{np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1).sum():.1f} m, z {P[:, 2].min():.2f}..{P[:, 2].max():.2f}')


if __name__ == '__main__':
    main()
