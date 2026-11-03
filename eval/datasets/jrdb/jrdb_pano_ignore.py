#!/usr/bin/env python3
"""
JRDB-PanoTrack 3D labels -> movable non-person objects of one converted JRDB sequence, in its odom frame.

JRDB's 3D boxes cover people only, so a still bicycle that SINS rightly flags as POTENTIALLY_DYNAMIC counts as a false
positive. JRDB-PanoTrack (labels/labels_3d_pseudo_panoptic_CW/<seq>/<frame>.bin, every 15th frame = 1 Hz) labels every
LiDAR point: one uint32 per point, upper VLP-16 points first then lower, label = instance << 16 | class (categories in
labels_2d_panoptic_CW). This writes the upper-LiDAR points of the movable non-person instances (MOVABLE below), voxel-
subsampled, as <out>: t_ns, x, y, z, cls, inst. coda_eval.py --extra-ignore drops predictions centred near them (they are
neither TP nor FP), the same rule as for annotated objects without LiDAR returns. Points within 0.8 m of an annotated
person of the same frame are left out (a rider's bicycle under a moving pedestrian).

usage: jrdb_pano_ignore.py <jrdb root> <seq> <converted short name, e.g. hewlett>   (writes bags/jrdb/<short>_pano.csv)
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jrdb_io import read_pcd                                          # noqa: E402

REPO = Path(__file__).resolve().parents[3]

MOVABLE = {16: 'animal', 43: 'golfcart', 58: 'car', 59: 'bicycle/scooter', 61: 'big_vehicle', 65: 'trolley',
           69: 'skateboard/segway/hoverboard'}
PERSON = 63
VOXEL = 0.15


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    root, seq, short = sys.argv[1:4]
    import json
    side = json.load(open(REPO / f'bags/jrdb/{short}_gt.json'))
    first, rate = side['first'], side['rate_hz']
    T_bv = np.array(side['T_base_velodyne'])
    gt = pd.read_csv(REPO / f'bags/jrdb/{short}_gt.csv')
    gt = gt[gt.track_id >= 0]
    od = odom_table(str(REPO / f'bags/jrdb/{short}_odom'))
    rows, check = [], []
    for f in sorted(Path(f'{root}/labels/labels_3d_pseudo_panoptic_CW/{seq}').glob('*.bin')):
        i = int(f.stem)
        if i < first or i > side['last']:
            continue
        lab = np.fromfile(f, np.uint32)
        up = read_pcd(f'{root}/pointclouds/upper_velodyne/{seq}/{i:06d}.pcd')[:, :3]
        lab = lab[:len(up)]
        cls, inst = lab & 0xFFFF, lab >> 16
        t = int(1_000_000_000 + (i - first) * 1e9 / rate)
        k = np.argmin(np.abs(od[:, 0] - t))
        R, p = quat_R(od[k, 4:8]), od[k, 1:4]
        pts = (R @ (T_bv[:3, :3] @ up.T + T_bv[:3, 3:4])).T + p
        g = gt[gt.t_ns == gt.t_ns.iloc[np.argmin(np.abs(gt.t_ns - t))]][['x', 'y']].to_numpy() if len(gt) else np.zeros((0, 2))
        # check of the point order (upper first): person-labelled points should lie on annotated people
        pp = pts[cls == PERSON]
        if len(pp) and len(g):
            check.append(np.mean(np.min(np.hypot(pp[:, None, 0] - g[None, :, 0], pp[:, None, 1] - g[None, :, 1]), 1) < 0.6))
        for c in MOVABLE:
            for n in np.unique(inst[cls == c]):
                q = pts[(cls == c) & (inst == n)]
                if len(g):
                    q = q[np.min(np.hypot(q[:, None, 0] - g[None, :, 0], q[:, None, 1] - g[None, :, 1]), 1) > 0.8]
                if len(q) < 3:
                    continue
                _, keep = np.unique(np.floor(q / VOXEL).astype(int), axis=0, return_index=True)
                for x, y, z in q[keep]:
                    rows.append((t, x, y, z, MOVABLE[c], int(n)))
    df = pd.DataFrame(rows, columns=['t_ns', 'x', 'y', 'z', 'cls', 'inst'])
    df.to_csv(REPO / f'bags/jrdb/{short}_pano.csv', index=False, float_format='%.3f')
    print(f'{short}: {df.t_ns.nunique() if len(df) else 0} labelled frames with movable objects, '
          f'{df.groupby(["t_ns", "cls", "inst"]).ngroups if len(df) else 0} instance-frames '
          f'{df.groupby("cls").inst.nunique().to_dict() if len(df) else {}}; '
          f'person points on annotated people: {np.mean(check) if check else float("nan"):.0%}')


def odom_table(bag):
    """[t_ns, x, y, z, qx, qy, qz, qw] of base_link in odom."""
    from rosbags.highlevel import AnyReader
    out = []
    with AnyReader([Path(bag)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            P, Q = m.pose.pose.position, m.pose.pose.orientation
            out.append((m.header.stamp.sec * 1e9 + m.header.stamp.nanosec, P.x, P.y, P.z, Q.x, Q.y, Q.z, Q.w))
    return np.array(sorted(out))


if __name__ == '__main__':
    main()
