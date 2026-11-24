#!/usr/bin/env python3
"""
DOALS sequence (ROS1 bag.bag + indices.csv) -> ROS2 bags in the shape SINS reads, plus the GT for doals_eval.py.
Run with eval/datasets/.venv/bin/python (rosbags, numpy). Facts from the dataset (lab PC check 24/11, readme.md):
  - Ouster OS1-64 Gen1, handheld, 10 Hz; /os1_cloud_node/points PointCloud2, organised 64x2048, frame os1_lidar,
    fields x y z intensity t(uint32 ns offset) ...; NO camera.
  - poses: T_map_os1_lidar (geometry_msgs/TransformStamped, ~200 Hz, frame map -> os1_lidar; no leading slash),
    globally bundle-adjusted VI-SLAM. The readme asks to undistort the clouds with them (handheld motion).
  - indices.csv: <scan header.stamp ns>,<flat indices of the dynamic points in the organised cloud> (10 scans/sequence;
    pedestrians and what they carry, labelled by APPEARANCE, not motion).

What this writes (as the other converters):
  - every scan DESKEWED with the poses (each column moved to the pose at stamp + t, then expressed at the pose at the
    header stamp) and LEVELLED (roll/pitch of that pose removed: the cloud frame 'velodyne' = the sensor position with
    the map's z axis and the sensor's yaw), point ORDER KEPT (invalid returns = NaN), so the GT indices stay valid;
  - base_link = 'velodyne' moved down by the sensor height above the ground (median RANSAC fit over the sequence),
    published static; odom = map shifted to the first base_link pose; /odometry/filtered + /tf odom->base_link at the scan
    rate (yaw + position of the sensor);
  - <out>_doals_gt.json: per annotated scan its stamp and indices (doals_eval.py).
  Deskew/levelling is input preparation shared by every method run on these bags (stated in the manifest); Dynablox and
  M-detector, run on the original bags with their own undistortion, are scored on the same indices.

usage: doals_to_bag.py <DOALS root> <scene/sequence_n> <out>
NOTE (24/11): written on the laptop, NOT run there (no tests on the laptop); first run = the lab smoke test.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from coda_to_bag import LATCHED, M, TS, ground_frame, header, quat_from_rot, transform_msg, cloud_msg   # noqa: E402
from rosbags.highlevel import AnyReader                                                         # noqa: E402
from rosbags.rosbag2 import StoragePlugin, Writer                                               # noqa: E402

CLOUD, POSE = '/os1_cloud_node/points', 'T_map_os1_lidar'


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Poses:
    def __init__(self, t, p, q):
        o = np.argsort(t)
        self.t, self.p, self.q = t[o], p[o], q[o]
        for k in range(1, len(self.q)):                       # same hemisphere for interpolation
            if np.dot(self.q[k], self.q[k - 1]) < 0:
                self.q[k] = -self.q[k]

    def at(self, ts):
        """T_map_sensor at times ts [ns] (array): linear position, normalised-lerp rotation (5 ms spacing)."""
        ts = np.atleast_1d(ts).astype(np.int64)
        k = np.clip(np.searchsorted(self.t, ts), 1, len(self.t) - 1)
        a = ((ts - self.t[k - 1]) / np.maximum(self.t[k] - self.t[k - 1], 1)).clip(0, 1)[:, None]
        p = (1 - a) * self.p[k - 1] + a * self.p[k]
        q = (1 - a) * self.q[k - 1] + a * self.q[k]
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        T = np.repeat(np.eye(4)[None], len(ts), axis=0)
        for j in range(len(ts)):
            T[j, :3, :3] = quat_to_R(q[j])
            T[j, :3, 3] = p[j]
        return T


def yaw_only(R):
    yaw = np.arctan2(R[1, 0], R[0, 0])
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def read_cloud(m):
    n = m.width * m.height
    raw = np.frombuffer(bytes(m.data), np.uint8).reshape(n, m.point_step)
    off = {f.name: f.offset for f in m.fields}
    xyz = np.stack([raw[:, off[c]:off[c] + 4].copy().view(np.float32)[:, 0] for c in 'xyz'], axis=1)
    inten = raw[:, off['intensity']:off['intensity'] + 4].copy().view(np.float32)[:, 0]
    t = raw[:, off['t']:off['t'] + 4].copy().view(np.uint32)[:, 0].astype(np.int64)
    return xyz.astype(np.float64), inten, t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('root')
    ap.add_argument('seq', help='scene/sequence_n')
    ap.add_argument('out')
    a = ap.parse_args()
    src, out = Path(a.root) / a.seq, Path(a.out)
    for p in (out, Path(f'{out}_odom')):
        if p.exists():
            raise SystemExit(f'{p} exists')
    gt = {}
    for line in open(src / 'indices.csv'):
        f = line.strip().split(',')
        if len(f) > 1:
            gt[int(f[0])] = [int(x) for x in f[1:] if x]

    with AnyReader([src / 'bag.bag']) as r:
        pc = [c for c in r.connections if c.topic.lstrip('/') == POSE]
        tp, pp, qq = [], [], []
        for c, _, raw in r.messages(pc):
            m = r.deserialize(raw, c.msgtype)
            tp.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
            tr = m.transform
            pp.append([tr.translation.x, tr.translation.y, tr.translation.z])
            qq.append([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
        poses = Poses(np.array(tp, np.int64), np.array(pp), np.array(qq))
        print(f'{a.seq}: {len(tp)} poses', flush=True)

        def scans():
            cc = [c for c in r.connections if c.topic == CLOUD]
            for c, _, raw in r.messages(cc):
                m = r.deserialize(raw, c.msgtype)
                yield m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec, read_cloud(m)

        # pass 1: sensor height above the ground (levelled, deskewed scans, every 20th)
        def level(stamp, xyz, t):
            valid = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > 0.1)
            T0 = poses.at([stamp])[0]
            ut, inv = np.unique(t, return_inverse=True)
            Tc = poses.at(stamp + ut)
            Rl = yaw_only(T0[:3, :3])
            # map point of each column, then into the levelled frame at the header stamp
            pm = np.einsum('nij,nj->ni', Tc[inv, :3, :3], xyz) + Tc[inv, :3, 3]
            pl = (pm - T0[:3, 3]) @ Rl                           # = Rl^T (pm - p0)
            pl[~valid] = np.nan
            return pl, T0, Rl

        hs = []
        for j, (stamp, (xyz, inten, t)) in enumerate(scans()):
            if j % 20:
                continue
            pl, _, _ = level(stamp, xyz, t)
            ok = np.isfinite(pl).all(1)
            try:
                _, h, _, n = ground_frame(pl[ok])
                if n > 500:
                    hs.append(h)
            except Exception:
                pass
        H = float(np.median(hs)) if hs else 1.5
        print(f'sensor height {H:.2f} m ({len(hs)} fits)', flush=True)

        T_base_v = np.eye(4)
        T_base_v[2, 3] = H
        first = None
        records = []
        with Writer(out, version=9, storage_plugin=StoragePlugin.MCAP) as w:
            c_cloud = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=TS)
            c_tfs = w.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=TS, offered_qos_profiles=LATCHED)
            for j, (stamp, (xyz, inten, t)) in enumerate(scans()):
                pl, T0, Rl = level(stamp, xyz, t)
                if first is None:
                    first = stamp
                    w.write(c_tfs, stamp - 1, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](transforms=[
                        transform_msg(T_base_v, 'base_link', 'velodyne', stamp),
                        transform_msg(np.eye(4), 'base_link', 'base_footprint', stamp)]), 'tf2_msgs/msg/TFMessage'))
                pts = np.concatenate([pl, inten[:, None]], axis=1).astype(np.float32)
                msg = cloud_msg(pts, stamp)
                msg.is_dense = False
                w.write(c_cloud, stamp, TS.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2'))
                Tb = np.eye(4)                                   # map <- base_link (levelled, yaw only)
                Tb[:3, :3] = Rl
                Tb[:3, 3] = T0[:3, 3] - [0, 0, H]
                records.append((stamp, Tb))
                if j % 200 == 0:
                    print(f'bag {j}', flush=True)

    T_odom_map = np.linalg.inv(records[0][1])
    with Writer(Path(f'{out}_odom'), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        prev = None
        V3 = M['geometry_msgs/msg/Vector3']
        for t, Tb in records:
            T = T_odom_map @ Tb
            q = quat_from_rot(T[:3, :3])
            v = np.zeros(3) if prev is None else T[:3, :3].T @ (T[:3, 3] - prev[1][:3, 3]) / max((t - prev[0]) * 1e-9, 1e-3)
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

    stamps = {t for t, _ in records}
    missing = [s for s in gt if s not in stamps]
    json.dump(dict(seq=a.seq, sensor_height_m=H, T_base_velodyne=T_base_v.tolist(), scans=len(records),
                   annotated=[dict(t_ns=s, indices=gt[s]) for s in sorted(gt)], missing_stamps=missing,
                   note='indices = flat indices of the organised 64x2048 cloud; the converted cloud keeps the order'),
              open(f'{out}_doals_gt.json', 'w'))
    print(f'done: {len(records)} scans, {len(gt)} annotated ({len(missing)} stamps not found), height {H:.2f} m')


if __name__ == '__main__':
    main()
