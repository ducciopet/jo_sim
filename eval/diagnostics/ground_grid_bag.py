#!/usr/bin/env python3
"""
The ground-grid prototype (ground_grid_proto.py's GroundGrid) on a robot bag without labels, for a visual check:
VLP-16 /velodyne_points + the two RealSense aligned depth images (compressedDepth), TF from the bag, odometry from an
odometry bag (/odometry/filtered: lidar_odom_bag.py or glim_odom_bag.py).

Outputs (in --out-dir):
  map.png      top view: the ground height the grid holds at the end (accepted ground cells), robot path on top
  frame_<k>.png for a few scans: top view and side view of that scan's points, ground (grey) vs kept (red)
  stats.json   per scan: share of LiDAR returns within 1.2-15 m left above the ground (= obstacle candidates)
usage: ground_grid_bag.py <sensor_bag> <odom_bag> --out-dir D [--every 1 --frames 500,1500,2500 --no-cams]
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'datasets'))
from ground_grid_proto import GroundGrid, cloud, stamp     # noqa: E402
from coda_to_bag import rot_from_quat                      # noqa: E402


def T_of(tr):
    T = np.eye(4)
    q, p = tr.rotation, tr.translation
    T[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def chain(edges, parent, child):
    """Transform parent<-child through a tree of static edges {(p, c): T}."""
    up = {c: (p, T) for (p, c), T in edges.items()}
    T, f = np.eye(4), child
    while f != parent:
        p, Tpc = up[f]
        T = Tpc @ T
        f = p
    return T


def depth_points(m, K, stride=4, zmax=4.0):
    """compressedDepth (12-byte header + 16-bit PNG, mm) -> points in the image's optical frame."""
    img = cv2.imdecode(np.frombuffer(bytes(m.data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        return np.zeros((0, 3))
    d = img[::stride, ::stride].astype(np.float64) / 1000.0
    v, u = np.nonzero((d > 0.3) & (d < zmax))
    z = d[v, u]
    u, v = u * stride, v * stride
    return np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('odom')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--cell', type=float, default=0.25)
    ap.add_argument('--slope', type=float, default=20.0)
    ap.add_argument('--step', type=float, default=0.10)
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--radius', type=float, default=15.0)
    ap.add_argument('--frames', default='300,1000,1700,2400')
    ap.add_argument('--no-cams', action='store_true')
    ap.add_argument('--depth-max', type=float, default=4.0)
    a = ap.parse_args()
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    want = {int(x) for x in a.frames.split(',')}

    odom = {}
    with AnyReader([Path(a.odom)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T = np.eye(4)
            T[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
            T[:3, 3] = [p.x, p.y, p.z]
            odom[stamp(m)] = T
    ost = np.array(sorted(odom))

    def pose_at(st):
        k = np.searchsorted(ost, st)
        k = min(max(k, 0), len(ost) - 1)
        if k > 0 and abs(ost[k - 1] - st) < abs(ost[k] - st):
            k -= 1
        return odom[ost[k]] if abs(ost[k] - st) < 1.5e8 else None

    grid = GroundGrid(a.cell, a.slope, a.step, a.radius, 30.0)
    edges, K, depth_last = {}, {}, {}
    cams = [] if a.no_cams else ['front_camera', 'back_camera']
    topics = {'/velodyne_points', '/tf_static'} | {f'/{c}/camera/aligned_depth_to_color/image_raw/compressedDepth' for c in cams} \
        | {f'/{c}/camera/aligned_depth_to_color/camera_info' for c in cams}
    stats, path, k = [], [], -1
    with AnyReader([Path(a.bag)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in topics]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/tf_static':
                for x in m.transforms:
                    edges[(x.header.frame_id, x.child_frame_id)] = T_of(x.transform)
                continue
            if c.topic.endswith('camera_info'):
                K[c.topic.split('/')[1]] = np.array(m.k).reshape(3, 3)
                continue
            if c.topic.endswith('compressedDepth'):
                cam = c.topic.split('/')[1]
                if cam in K:
                    depth_last[cam] = (stamp(m), depth_points(m, K[cam], zmax=a.depth_max), m.header.frame_id)
                continue
            st = stamp(m)
            T_ob = pose_at(st)
            if T_ob is None or ('base_link', 'velodyne') not in edges:
                continue
            k += 1
            T_bv = chain(edges, 'base_link', 'velodyne')
            L = cloud(m)
            L = L[np.isfinite(L).all(1) & (np.abs(L).sum(1) > 0)]
            Lg = L @ (T_ob @ T_bv)[:3, :3].T + (T_ob @ T_bv)[:3, 3]
            parts = [Lg]
            for cam, (dst, Pc, frame) in depth_last.items():
                if abs(dst - st) < 2e8:
                    T = T_ob @ chain(edges, 'base_link', frame)
                    parts.append(Pc @ T[:3, :3].T + T[:3, 3])
            grid.update(np.vstack(parts), T_ob[:3, 3], st * 1e-9)
            path.append(T_ob[:3, 3].copy())
            rng = np.linalg.norm(L[:, :2], axis=1)
            sel = (rng > 1.2) & (rng < 15)
            ha = grid.height_above(Lg[sel])
            kept = ~(np.nan_to_num(ha, nan=np.inf) < a.margin)
            stats.append(dict(scan=k, t=st * 1e-9, n=int(sel.sum()), kept=int(kept.sum()),
                              robot_z=float(T_ob[2, 3])))
            if k in want:
                P = Lg[sel]
                img = np.full((600, 1200, 3), 255, np.uint8)
                c0 = T_ob[:3, 3]
                for (x, y, z), kp in zip(P, kept):
                    col = (0, 0, 220) if kp else (170, 170, 170)
                    u, v = int(300 + (x - c0[0]) * 20), int(300 - (y - c0[1]) * 20)
                    if 0 <= u < 600 and 0 <= v < 600:
                        img[v, u] = col
                    # side view along the robot's heading: distance along x_base vs height
                    d = (np.array([x, y, z]) - c0) @ T_ob[:3, 0]
                    u2, v2 = int(900 + d * 20), int(450 - (z - c0[2]) * 60)
                    if 600 <= u2 < 1200 and 0 <= v2 < 600 and abs((np.array([x, y, z]) - c0) @ T_ob[:3, 1]) < 1.5:
                        img[v2, u2] = col
                cv2.putText(img, f'scan {k}: top (1 px = 5 cm) | side, 3 m wide band along heading (x 5 cm, z 1.7 cm)',
                            (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
                cv2.putText(img, 'grey = ground, red = kept above ground', (5, 590), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
                cv2.imwrite(str(out / f'frame_{k}.png'), img)
            if k % 500 == 0:
                print(f'{k} scans', flush=True)
    path = np.array(path)
    cells = np.array([(i, j, z) for (i, j), (z, _) in grid.mem.items()])
    if len(cells):
        lo, hi = np.percentile(cells[:, 2], [2, 98])
        x0, y0 = cells[:, 0].min(), cells[:, 1].min()
        W, H = int(cells[:, 0].max() - x0 + 1), int(cells[:, 1].max() - y0 + 1)
        hm = np.full((H, W), np.nan)
        for i, j, z in cells:
            hm[int(H - 1 - (j - y0)), int(i - x0)] = z
        norm = np.nan_to_num((hm - lo) / max(1e-6, hi - lo), nan=-1)
        col = cv2.applyColorMap((np.clip(norm, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        col[norm < 0] = 255
        for x, y, _ in path[::5]:
            u, v = int(x / a.cell - x0), int(H - 1 - (y / a.cell - y0))
            if 0 <= u < W and 0 <= v < H:
                col[v, u] = (0, 0, 255)
        s = max(1, 900 // max(W, H))
        col = cv2.resize(col, (W * s, H * s), interpolation=cv2.INTER_NEAREST)
        cv2.putText(col, f'ground height {lo:.1f} (purple) .. {hi:.1f} m (yellow), cell {a.cell} m, red = robot path',
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.imwrite(str(out / 'map.png'), col)
    kept = np.array([s['kept'] / max(1, s['n']) for s in stats])
    json.dump(dict(args=vars(a), scans=len(stats), kept_mean=float(kept.mean()),
                   kept_p50=float(np.median(kept)), kept_p90=float(np.percentile(kept, 90)), per_scan=stats),
              open(out / 'stats.json', 'w'))
    print(f'{len(stats)} scans; LiDAR returns 1.2-15 m left above ground: mean {kept.mean():.1%}, '
          f'median {np.median(kept):.1%}, p90 {np.percentile(kept, 90):.1%}')


if __name__ == '__main__':
    main()
