#!/usr/bin/env python3
"""
Re-anchor a converted JRDB sequence (jrdb_to_bag.py output) on the robot's fixed LiDAR mount instead of a per-sequence
ground-plane fit, without reading the dataset again.

base_link is a frame of the ROBOT: JackRabbot's upper VLP-16 sits 1.24 m above a flat floor, level (fits on the clear
floors of huang-basement, packard-poster-session_2 and tressider-2019-04-26_2 at their first frame: 1.24 m, 0.1 deg).
A per-sequence fit locked onto stairs (forbes-cafe, 0.71 m) or table tops (bytes-cafe, 0.43 m).

With A = old T_velodyne_base and B = new one, the odom frame (base_link at the first frame) changes by the fixed rigid
C = B^-1 A: GT rows p' = C p, odometry O' = C O C^-1, /tf_static base_link->velodyne = B^-1.
Writes <out>.reframed/ then swaps it in place of <out>/ (same for _odom), rewrites _gt.csv/_gt.json.
usage: jrdb_reframe.py bags/jrdb/<name> [--height 1.24]
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from coda_to_bag import LATCHED, M, TS, header, quat_from_rot, rot_from_quat, transform_msg   # noqa: E402
from rosbags.rosbag2 import Reader, StoragePlugin, Writer                                     # noqa: E402


def rewrite(src, dst, fix):
    """Copy a bag message by message; fix(topic, msg) -> msg for the topics it handles (others copied raw)."""
    with Reader(src) as r, Writer(dst, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        conns = {}
        for c in r.connections:
            qos = LATCHED if c.topic == '/tf_static' else None
            conns[c.id] = w.add_connection(c.topic, c.msgtype, typestore=TS,
                                           **({'offered_qos_profiles': qos} if qos else {}))
        for c, t, raw in r.messages():
            out = fix(c.topic, c.msgtype, raw)
            w.write(conns[c.id], t, raw if out is None else TS.serialize_cdr(out, c.msgtype))


def T_of(tr):
    T = np.eye(4)
    q, p = tr.rotation, tr.translation
    T[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('--height', type=float, default=1.24)
    a = ap.parse_args()
    out = Path(a.out)
    meta = json.load(open(f'{out}_gt.json'))
    if abs(meta['lidar_height_m'] - a.height) < 1e-9 and meta.get('mount') == 'fixed':
        print(f'{out}: already on the fixed mount')
        return
    T_bv_old = np.array(meta['T_base_velodyne'])
    T_bv_new = np.eye(4)
    T_bv_new[2, 3] = a.height
    C = T_bv_new @ np.linalg.inv(T_bv_old)
    Ci = np.linalg.inv(C)

    def fix_sensor(topic, msgtype, raw):
        if topic != '/tf_static':
            return None
        m = TS.deserialize_cdr(raw, msgtype)
        tfs = [transform_msg(T_bv_new, 'base_link', 'velodyne', 0) if (t.header.frame_id, t.child_frame_id) ==
               ('base_link', 'velodyne') else t for t in m.transforms]
        for t, o in zip(tfs, m.transforms):
            t.header = o.header
        return M['tf2_msgs/msg/TFMessage'](transforms=tfs)

    prev = {}

    def fix_odom(topic, msgtype, raw):
        m = TS.deserialize_cdr(raw, msgtype)
        if topic == '/tf':
            for t in m.transforms:
                T = C @ T_of(t.transform) @ Ci
                t.transform = transform_msg(T, 'odom', 'base_link', 0).transform
            return m
        p, q = m.pose.pose.position, m.pose.pose.orientation
        T0 = np.eye(4)
        T0[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
        T0[:3, 3] = [p.x, p.y, p.z]
        T = C @ T0 @ Ci
        qq = quat_from_rot(T[:3, :3])
        p.x, p.y, p.z = map(float, T[:3, 3])
        q.x, q.y, q.z, q.w = map(float, qq)
        t = m.header.stamp.sec * 1e9 + m.header.stamp.nanosec
        if 'T' in prev:
            v = T[:3, :3].T @ (T[:3, 3] - prev['T'][:3, 3]) / ((t - prev['t']) * 1e-9)
            lin = m.twist.twist.linear
            lin.x, lin.y, lin.z = map(float, v)
        prev.update(T=T, t=t)
        return m

    for src, fix in ((out, fix_sensor), (Path(f'{out}_odom'), fix_odom)):
        tmp = Path(f'{src}.reframed')
        shutil.rmtree(tmp, ignore_errors=True)
        rewrite(src, tmp, fix)
        shutil.rmtree(src)
        tmp.rename(src)

    gt = pd.read_csv(f'{out}_gt.csv')
    obj = gt['track_id'] >= 0
    P = gt.loc[obj, ['x', 'y', 'z']].to_numpy(float) @ C[:3, :3].T + C[:3, 3]
    gt.loc[obj, ['x', 'y', 'z']] = P
    yaw = gt.loc[obj, 'yaw'].to_numpy(float)
    d = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], 1) @ C[:3, :3].T
    gt.loc[obj, 'yaw'] = np.arctan2(d[:, 1], d[:, 0])
    gt.to_csv(f'{out}_gt.csv', index=False, float_format='%.4f')
    meta.update(fit_lidar_height_m=meta['lidar_height_m'], fit_lidar_tilt_deg=meta['lidar_tilt_deg'],
                lidar_height_m=a.height, lidar_tilt_deg=0.0, T_base_velodyne=T_bv_new.tolist(), mount='fixed')
    json.dump(meta, open(f'{out}_gt.json', 'w'), indent=1)
    print(f'{out}: fit {meta["fit_lidar_height_m"]:.2f} m / {meta["fit_lidar_tilt_deg"]:.1f} deg -> fixed {a.height:.2f} m, level')


if __name__ == '__main__':
    main()
