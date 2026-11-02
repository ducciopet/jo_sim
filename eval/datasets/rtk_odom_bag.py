#!/usr/bin/env python3
"""
Reference odometry for a robot bag with RTK GNSS (Marina di Pisa 14/10: Xsens GNSS, RTK fixed on 1166/1169 fixes),
computed once and shared by every method (same role as make_odom.sh's GLIM+EKF for the lab bags):
  position     : RTK (/gnss, local ENU from the first fix) interpolated at each scan, lever arm imu_link->base_link
                 from the bag's TF (the fix is reported in imu_link)
  roll, pitch  : Xsens /imu/data orientation (gravity-referenced)
  yaw          : the LiDAR odometry (lidar_odom_bag.py output), rotated onto the RTK track (its heading is right
                 locally; the magnetometer yaw is not trusted)
On 14/10 slopes: GLIM drifted 26 m (median) in xy, KISS-ICP 0.4 m in xy but up to 4 m in z; RTK is cm-level.
odom = ENU at the first fix shifted so that base_link starts at the origin, z up.
usage: rtk_odom_bag.py <sensor_bag (imu, tf)> <gnss_bag> <lidar_odom_bag> <out_bag>
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


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sensor')
    ap.add_argument('gnss')
    ap.add_argument('lodom')
    ap.add_argument('out')
    a = ap.parse_args()
    imu_t, imu_q = [], []
    with AnyReader([Path(a.sensor)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in ('/tf_static', '/imu/data')]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/tf_static':
                for x in m.transforms:
                    if (x.header.frame_id, x.child_frame_id) == ('base_link', 'imu_link'):
                        T_bi = T_of(x.transform)
            else:
                imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                q = m.orientation
                imu_q.append(rot_from_quat(q.x, q.y, q.z, q.w))
    imu_t = np.array(imu_t)
    G, TG = [], []
    with AnyReader([Path(a.gnss)]) as r:
        c = [c for c in r.connections if c.topic == '/gnss']
        for _, t, raw in r.messages(connections=c):
            m = r.deserialize(raw, c[0].msgtype)
            TG.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            G.append([m.latitude, m.longitude, m.altitude])
    G, TG = np.array(G), np.array(TG)
    R_e, lat0 = 6378137.0, np.radians(G[0, 0])
    E = np.stack([np.radians(G[:, 1] - G[0, 1]) * R_e * np.cos(lat0), np.radians(G[:, 0] - G[0, 0]) * R_e,
                  G[:, 2] - G[0, 2]], 1)
    LT, LP, LR = [], [], []
    with AnyReader([Path(a.lodom)]) as r:
        c = [c for c in r.connections if c.topic == '/odometry/filtered']
        for _, t, raw in r.messages(connections=c):
            m = r.deserialize(raw, c[0].msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            LT.append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
            LP.append([p.x, p.y, p.z])
            LR.append(rot_from_quat(q.x, q.y, q.z, q.w))
    LT, LP = np.array(LT), np.array(LP)
    ts = LT * 1e-9
    ok = (ts >= TG[0]) & (ts <= TG[-1])
    # yaw offset LiDAR odometry -> ENU from the tracks (2D Procrustes on positions)
    Pg = np.stack([np.interp(ts[ok], TG, E[:, i]) for i in range(3)], 1)
    A, B = LP[ok, :2] - LP[ok, :2].mean(0), Pg[:, :2] - Pg[:, :2].mean(0)
    U, _, Vt = np.linalg.svd(A.T @ B)
    R2 = (U @ Vt).T
    dyaw = np.arctan2(R2[1, 0], R2[0, 0])
    out, origin = [], None
    for st, t, R_l in zip(LT[ok], ts[ok], [LR[i] for i in np.where(ok)[0]]):
        k = min(np.searchsorted(imu_t, t), len(imu_t) - 1)
        R_wb_imu = imu_q[k] @ T_bi[:3, :3].T
        yaw_l = np.arctan2((Rz(dyaw) @ R_l)[1, 0], (Rz(dyaw) @ R_l)[0, 0])
        yaw_i = np.arctan2(R_wb_imu[1, 0], R_wb_imu[0, 0])
        R_wb = Rz(yaw_l - yaw_i) @ R_wb_imu                      # IMU roll/pitch, LiDAR-odometry heading
        p_imu = np.array([np.interp(t, TG, E[:, i]) for i in range(3)])
        p_base = p_imu - R_wb @ T_bi[:3, 3]
        if origin is None:
            origin = p_base.copy()
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R_wb, p_base - origin
        out.append((int(st), T))
    with Writer(Path(a.out), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        V3 = M['geometry_msgs/msg/Vector3']
        prev = None
        for st, T in out:
            q = quat_from_rot(T[:3, :3])
            v = np.zeros(3) if prev is None else T[:3, :3].T @ (T[:3, 3] - prev[1][:3, 3]) / ((st - prev[0]) * 1e-9)
            prev = (st, T)
            odom = M['nav_msgs/msg/Odometry'](
                header=header(st, 'odom'), child_frame_id='base_link',
                pose=M['geometry_msgs/msg/PoseWithCovariance'](pose=M['geometry_msgs/msg/Pose'](
                    position=M['geometry_msgs/msg/Point'](x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
                    orientation=M['geometry_msgs/msg/Quaternion'](x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))),
                    covariance=np.zeros(36)),
                twist=M['geometry_msgs/msg/TwistWithCovariance'](twist=M['geometry_msgs/msg/Twist'](
                    linear=V3(x=float(v[0]), y=float(v[1]), z=float(v[2])), angular=V3(x=0.0, y=0.0, z=0.0)),
                    covariance=np.zeros(36)))
            w.write(c_odom, st, TS.serialize_cdr(odom, 'nav_msgs/msg/Odometry'))
            w.write(c_tf, st, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](
                transforms=[transform_msg(T, 'odom', 'base_link', st)]), 'tf2_msgs/msg/TFMessage'))
    P = np.array([T[:3, 3] for _, T in out])
    print(f'done: {len(out)} scans (of {len(LT)} with RTK coverage), path {np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1).sum():.1f} m, '
          f'z {P[:, 2].min():.2f}..{P[:, 2].max():.2f} m, heading offset LiDAR->ENU {np.degrees(dyaw):.1f} deg')


if __name__ == '__main__':
    main()
