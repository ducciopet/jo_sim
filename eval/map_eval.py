#!/usr/bin/env python3
"""
Map experiment: how much of the person ends up in the SLAM map, and how much static structure survives, for each
dynamic-rejection variant (eval/scripts/run_glim_map.sh: GLIM alone, no rejection / DYNAMIC only / DYNAMIC +
POTENTIALLY_DYNAMIC, driven by the obstacles a detector actually published).

Map content = union of every scan as it entered GLIM's odometry (/glim_ros/aligned_points, after the rejection,
world frame), voxelised at --voxel. Brought into the mocap frame through the shared odometry: T_odom_world from
GLIM's own pose (/glim_ros/odom, IMU) vs the shared /odometry/filtered (base_link) at the same stamps and the static
base_link -> imu_link TF, then the benchmark's T_mocap_odom (SE(2)); z from the floor (the densest height below
base_link in the reference map, shared by every variant).

  ghost voxels     occupied voxels inside the person's region: discs of --radius around every mocap position of the
                   person (head marker), from 0.2 to 1.8 m above the floor (the body, not the floor under it).
                   0.8 m: T_mocap_odom leaves the base_link -> robot-marker offset unobservable (alignment_v2.json,
                   t_base_JO_observable false), so with the robot standing still the person's points sit up to ~0.6 m
                   from its mocap position (1942: the body's cluster 0.4-0.8 m away); inside the benchmark's own
                   1.0 m matching tolerance.
  static kept      share of the voxels farther than --static-dist from anywhere the person has been that the
                   no-rejection map has and the variant still has: real structure removed by mistake. The ring in
                   between (arms, legs, the person's own trail, the rejection's inflated boxes) is left out of both
                   numbers. GLIM run-to-run noise: repeat the no-rejection run and compare it with the reference.

usage: python3 eval/map_eval.py --runs <name>=<bag> ... --seq 20260519_1950 [--ref <name>]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.gt_mocap import compose  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402

REPO = Path(__file__).resolve().parents[1]


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_of(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def xyz_of(msg):
    f = {fl.name: fl.offset for fl in msg.fields}
    buf = np.frombuffer(bytes(msg.data), np.uint8).reshape(-1, msg.point_step)
    cols = [buf[:, f[k]:f[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz']
    p = np.stack(cols, 1).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def load_run(bag):
    pts, odom = [], []
    with AnyReader([Path(bag)]) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic in ('/glim_ros/aligned_points', '/glim_ros/odom')]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/glim_ros/odom':
                odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, T_of(m.pose.pose.position, m.pose.pose.orientation)))
            else:
                pts.append(xyz_of(m))
    return np.concatenate(pts), odom


def shared_odom(seq):
    out = []
    with AnyReader([REPO / 'bags' / 'replay' / f'odom_{seq}']) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, T_of(m.pose.pose.position, m.pose.pose.orientation)))
    return out


def base_to_imu(seq):
    with AnyReader([REPO / 'bags' / 'bags_benchmark' / f'duccio_test_{seq}_0']) as r:
        tfs = {}
        for c, t, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
            for tr in r.deserialize(raw, c.msgtype).transforms:
                tfs[tr.child_frame_id] = (tr.header.frame_id, T_of(tr.transform.translation, tr.transform.rotation))
    T, f = np.eye(4), 'imu_link'
    while f != 'base_link':  # chain imu_link -> ... -> base_link
        parent, Tp = tfs[f]
        T, f = Tp @ T, parent
    return T


def T_odom_world(odom_glim, odom_shared, T_base_imu):
    ts = np.array([o[0] for o in odom_shared])
    est = []
    for t, T_world_imu in odom_glim:
        i = int(np.argmin(np.abs(ts - t)))
        if abs(ts[i] - t) < 0.03:
            est.append(odom_shared[i][1] @ T_base_imu @ np.linalg.inv(T_world_imu))
    est = np.array(est)
    xyyaw = np.stack([est[:, 0, 3], est[:, 1, 3], np.arctan2(est[:, 1, 0], est[:, 0, 0])], 1)
    med = np.median(xyyaw, 0)
    spread = np.percentile(np.linalg.norm(xyyaw[:, :2] - med[:2], axis=1), 95)
    return med, float(np.median(est[:, 2, 3])), float(spread)


def voxels(p, size):
    return set(map(tuple, np.floor(p / size).astype(np.int64)))


def aligned_maps(seq, runs, ref='none', gt_campaign='own_r9a', floor=None):
    """runs: ['name=bag', ...] -> ({name: Nx3 points, mocap x/y, z above the floor}, {name: info}, person xy, floor)."""
    res = REPO / 'eval' / 'results' / gt_campaign
    al = json.loads((res / 'alignment_v2.json').read_text())
    T_mo = np.array(al['T_mocap_odom'][seq])
    gt = read_tracks(res / seq / 'gt' / 'tracks.csv')
    person = gt[gt['track_id'] >= 0][['x', 'y']].to_numpy()
    sh, Tbi = shared_odom(seq), base_to_imu(seq)

    maps, info = {}, {}
    for spec in runs:
        name, bag = spec.split('=', 1)
        pts, og = load_run(bag)
        (x, y, yaw), z_off, spread = T_odom_world(og, sh, Tbi)
        c, s = np.cos(yaw), np.sin(yaw)
        po = np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1], pts[:, 2] + z_off], 1)
        pm2 = compose(np.broadcast_to(T_mo, (len(po), 3)), np.concatenate([po[:, :2], np.zeros((len(po), 1))], 1))
        maps[name] = np.stack([pm2[:, 0], pm2[:, 1], po[:, 2]], 1)
        info[name] = dict(points=len(po), align_spread_m=spread)

    # floor: the densest height of the reference map BELOW base_link (z = 0 in the shared odometry, which is planar;
    # the floor is under the robot's base), one value for every variant (they share the alignment to the mm). The
    # unconstrained densest height jumps between near-equal levels, floor vs a dense shelf ~1 m up, between maps.
    zr = maps[ref][:, 2] if ref in maps else next(iter(maps.values()))[:, 2]
    zr = zr[zr < 0.0]
    hist, edges = np.histogram(zr, bins=np.arange(zr.min(), zr.max() + 0.05, 0.05))
    if floor is None:
        floor = edges[np.argmax(hist)] + 0.025
    for pm in maps.values():
        pm[:, 2] -= floor
    return maps, info, person, floor


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seq', required=True)
    ap.add_argument('--runs', nargs='+', required=True, help='name=bag')
    ap.add_argument('--ref', default='none', help='run name of the no-rejection map (static reference)')
    ap.add_argument('--voxel', type=float, default=0.1)
    ap.add_argument('--radius', type=float, default=0.8)
    ap.add_argument('--static-dist', type=float, default=1.5)
    ap.add_argument('--floor', type=float, help='floor height in the map frame (default: estimated on the reference)')
    ap.add_argument('--gt', default='own_r9a')
    ap.add_argument('--out')
    args = ap.parse_args()
    maps, info, person, floor = aligned_maps(args.seq, args.runs, args.ref, args.gt, args.floor)
    print(f"{args.seq} floor {floor:.2f} m (map frame)")

    # person region: voxel columns within --radius of the person's track, body heights only
    vs = args.voxel
    body = (0.2, 1.8)
    from scipy.spatial import cKDTree
    tree = cKDTree(person)

    def split(pm):
        m = (pm[:, 2] >= body[0]) & (pm[:, 2] <= body[1])
        d, _ = tree.query(pm[:, :2])
        inside = m & (d <= args.radius)
        far = (d > args.static_dist) & (pm[:, 2] > 0.1)
        return voxels(pm[inside], vs), voxels(pm[far], vs), int(inside.sum())

    parts = {n: split(pm) for n, pm in maps.items()}
    ref_static = parts[args.ref][1] if args.ref in parts else None
    out = {}
    for n, (ghost, static, ghost_pts) in parts.items():
        out[n] = dict(info[n], ghost_voxels=len(ghost), ghost_points=ghost_pts, static_voxels=len(static),
                      static_kept=(len(static & ref_static) / len(ref_static)) if ref_static else None)
        print(f"{args.seq} {n:10s} ghost voxels {len(ghost):5d}  ghost points {ghost_pts:7d}  static voxels "
              f"{len(static):6d}  static kept {100 * out[n]['static_kept']:.1f}%  (align spread {info[n]['align_spread_m']:.3f} m)")
    if args.out:
        Path(args.out).write_text(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
