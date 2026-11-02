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
    """compressedDepth (12-byte header + 16-bit PNG) or raw 16UC1 Image, mm -> points in the image's optical frame."""
    if hasattr(m, 'encoding'):
        img = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
    else:
        img = cv2.imdecode(np.frombuffer(bytes(m.data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        return np.zeros((0, 3))
    d = img[::stride, ::stride].astype(np.float64) / 1000.0
    v, u = np.nonzero((d > 0.3) & (d < zmax))
    z = d[v, u]
    u, v = u * stride, v * stride
    return np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)


def render(P, kept, T_ob, color_msg, k, grid, a):
    """One video frame: top view (10 m around the robot) with the ground grid underneath, side view along the heading
    (3 m wide band), front camera."""
    W = 480
    top = np.full((W, W, 3), 250, np.uint8)
    c0, s = T_ob[:3, 3], W / 20.0                                       # 20 m wide, 2.4 cm per pixel
    # ground grid: accepted ground cells shaded by height relative to the robot
    for (i, j), z in grid.h.items():
        u, v = int(W / 2 + ((i + 0.5) * grid.c - c0[0]) * s), int(W / 2 - ((j + 0.5) * grid.c - c0[1]) * s)
        if 0 <= u < W and 0 <= v < W:
            t = np.clip((z - c0[2] + 1.0) / 2.0, 0, 1)
            col = (int(235 - 60 * t), int(225 - 20 * t), int(200 + 40 * t))
            r = max(1, int(grid.c * s / 2))
            cv2.rectangle(top, (u - r, v - r), (u + r, v + r), col, -1)
    uv = np.stack([W / 2 + (P[:, 0] - c0[0]) * s, W / 2 - (P[:, 1] - c0[1]) * s], 1).astype(int)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < W)
    for msk, col in ((~kept, (120, 120, 120)), (kept, (40, 40, 210))):
        q = uv[ok & msk]
        top[q[:, 1], q[:, 0]] = col
    cv2.circle(top, (W // 2, W // 2), 5, (0, 140, 0), -1)
    hx = T_ob[:3, 0]
    cv2.line(top, (W // 2, W // 2), (int(W / 2 + hx[0] * 25), int(W / 2 - hx[1] * 25)), (0, 140, 0), 2)
    # side view: points within 1.5 m of the heading line, distance along heading vs height
    side = np.full((W, W, 3), 250, np.uint8)
    d = (P - c0) @ T_ob[:3, 0]
    lat = (P - c0) @ T_ob[:3, 1]
    band = np.abs(lat) < 1.5
    sx, sz = W / 16.0, W / 5.0                                          # 16 m along, 5 m height
    uv2 = np.stack([W * 0.25 + d * sx, W * 0.6 - (P[:, 2] - c0[2]) * sz], 1).astype(int)
    ok2 = band & (uv2[:, 0] >= 0) & (uv2[:, 0] < W) & (uv2[:, 1] >= 0) & (uv2[:, 1] < W)
    # estimated ground along the heading line
    prev = None
    for dd in np.arange(-4, 12, grid.c):
        p = c0 + dd * T_ob[:3, 0]
        z = grid.height_above(np.array([[p[0], p[1], 0.0]]))[0]
        if np.isfinite(z):
            gz = -z                                                      # ground height at p (height_above of z=0)
            pt = (int(W * 0.25 + dd * sx), int(W * 0.6 - (gz - c0[2]) * sz))
            if prev is not None:
                cv2.line(side, prev, pt, (60, 160, 60), 2)
            prev = pt
        else:
            prev = None
    for msk, col in ((~kept, (120, 120, 120)), (kept, (40, 40, 210))):
        q = uv2[ok2 & msk]
        for x, y in q:
            cv2.circle(side, (x, y), 1, col, -1)
    cv2.circle(side, (int(W * 0.25), int(W * 0.6)), 5, (0, 140, 0), -1)
    cam = np.full((W, W, 3), 250, np.uint8)
    if color_msg is not None:
        if hasattr(color_msg, 'encoding'):             # raw Image (CODa: rgb8)
            img = np.frombuffer(bytes(color_msg.data), np.uint8).reshape(color_msg.height, color_msg.width, -1)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if color_msg.encoding == 'rgb8' else img.copy()
        else:
            img = cv2.imdecode(np.frombuffer(bytes(color_msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            h = int(img.shape[0] * W / img.shape[1])
            cam[(W - h) // 2:(W - h) // 2 + h] = cv2.resize(img, (W, h))
    out = np.hstack([top, side, cam])
    bar = np.full((44, out.shape[1], 3), 255, np.uint8)
    cv2.putText(bar, f'scan {k}   grey = ground   red = kept as possible obstacle   green = estimated ground', (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1)
    cv2.putText(bar, 'top view 20 m x 20 m (shade = ground height)  |  side view along heading, 16 m x 5 m  |  front camera',
                (8, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1)
    return np.vstack([bar, out])


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
    ap.add_argument('--mode', default='step', choices=['step', 'grad'])
    ap.add_argument('--max-grad', type=float, default=40.0)
    ap.add_argument('--tol', type=float, default=None, help="mode grad: accepted |surface - predicted| (default --step)")
    ap.add_argument('--depth-max', type=float, default=4.0)
    ap.add_argument('--color-topic', default='/front_camera/camera/color/image_raw/compressed')
    ap.add_argument('--depth', nargs='*', default=None,
                    help='depth image topics as IMAGE:INFO (compressedDepth or raw 16UC1); default: the two lab RealSense')
    ap.add_argument('--video', help='write an mp4: top view, side view, front camera, ground grey / kept red')
    ap.add_argument('--video-range', default='0:100000', help='scans first:last for the video')
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

    grid = GroundGrid(a.cell, a.slope, a.step, a.radius, 30.0, mode=a.mode, max_grad_deg=a.max_grad)
    if a.tol is not None:
        grid.step_tol = a.tol
    edges, K, depth_last = {}, {}, {}
    if a.depth is None:
        a.depth = [f'/{c}/camera/aligned_depth_to_color/image_raw/compressedDepth:/{c}/camera/aligned_depth_to_color/camera_info'
                   for c in ('front_camera', 'back_camera')]
    dpairs = [] if a.no_cams else [tuple(x.split(':')) for x in a.depth]
    info_of = {i: d for d, i in dpairs}
    vr = [int(x) for x in a.video_range.split(':')]
    color_last, writer = None, None
    topics = {'/velodyne_points', '/tf_static', a.color_topic} | {d for d, _ in dpairs} | {i for _, i in dpairs}
    stats, path, k = [], [], -1
    with AnyReader([Path(a.bag)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in topics]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/tf_static':
                for x in m.transforms:
                    edges[(x.header.frame_id, x.child_frame_id)] = T_of(x.transform)
                continue
            if c.topic == a.color_topic:
                if a.video:
                    color_last = m
                continue
            if c.topic in info_of:
                K[info_of[c.topic]] = np.array(m.k).reshape(3, 3)
                continue
            if c.topic in K:
                depth_last[c.topic] = (stamp(m), depth_points(m, K[c.topic], zmax=a.depth_max), m.header.frame_id)
                continue
            if c.topic != '/velodyne_points':           # a depth image before its camera_info
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
            grid.update(np.vstack(parts), T_ob[:3, 3], st * 1e-9, T_ob[:3, :3])
            path.append(T_ob[:3, 3].copy())
            rng = np.linalg.norm(L[:, :2], axis=1)
            sel = (rng > 1.2) & (rng < 15)
            ha = grid.height_above(Lg[sel])
            kept = ~(np.nan_to_num(ha, nan=np.inf) < a.margin)
            stats.append(dict(scan=k, t=st * 1e-9, n=int(sel.sum()), kept=int(kept.sum()),
                              robot_z=float(T_ob[2, 3])))
            if a.video and k > vr[1]:
                break
            if a.video and vr[0] <= k <= vr[1]:
                frame = render(Lg[sel], kept, T_ob, color_last, k, grid, a)
                if writer is None:
                    writer = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*'mp4v'), 10, (frame.shape[1], frame.shape[0]))
                writer.write(frame)
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
    if writer is not None:
        writer.release()
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
