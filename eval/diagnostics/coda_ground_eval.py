#!/usr/bin/env python3
"""
How well SINS's floor/roof/wall crop separates ground from objects on CODa, against CODa's per-point TERRAIN labels
(3d_semantic: 0 = not terrain, 1-24 = a terrain class: concrete, grass, rocks, stairs, ...). Seq 0 frames 4750-4849
(window w2) and 5600-5699 (w3) are labelled.

Needs a run that recorded /sins/lidar/processed (MORE_TOPICS=/sins/lidar/processed with coda_ab.sh). For every labelled
scan, the processed cloud with the scan's own stamp (odom frame) is matched point by point to the raw scan brought into
odom (nearest raw return within --match m gives the label), then:
  - terrain_kept_per_scan : processed points that are terrain = ground left in the obstacle cloud (lower is better)
  - terrain_kept_frac     : of the raw terrain returns' 0.1 m voxels (1.2-25 m), those still in the processed cloud
  - agent_kept_frac       : of the 0.1 m voxels of raw non-terrain returns inside annotated agents (the window's GT
                            boxes, >= 5 raw returns), those still in the processed cloud (higher is better)
  - agent_low_kept_frac   : same, only the lowest 0.5 m of each agent (feet, legs, wheels: what a ground crop eats)
usage: coda_ground_eval.py <run_bag> <window>   (e.g. bags/coda/runs/G0_v2_coda0_w2 w2)  [--out json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'datasets'))
from coda_to_bag import load_scan, rot_from_quat     # noqa: E402

CODA = '/mnt/kloster-reichenau/datasets_SINS/CODa'
REPO = HERE.parents[1]


def vox(p, s=0.1):
    return set(map(tuple, np.floor(p / s).astype(np.int64)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('window')
    ap.add_argument('--seq', default='0')
    ap.add_argument('--match', type=float, default=0.15)
    ap.add_argument('--out')
    a = ap.parse_args()
    base = REPO / f'bags/coda/coda{a.seq}_{a.window}'
    meta = json.load(open(f'{base}_gt.json'))
    T_base_os1 = np.array(meta['T_base_velodyne'])
    t_scan = (np.loadtxt(f'{CODA}/timestamps/{a.seq}.txt') * 1e9).astype(np.int64)
    frames = [i for i in range(meta['first'], meta['last'] + 1)
              if Path(f'{CODA}/3d_semantic/os1/{a.seq}/3d_semantic_os1_{a.seq}_{i}.bin').exists()]
    want = {int(t_scan[i]): i for i in frames}

    odom = {}
    with AnyReader([Path(f'{base}_odom')]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T = np.eye(4)
            T[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
            T[:3, 3] = [p.x, p.y, p.z]
            odom[m.header.stamp.sec * 10**9 + m.header.stamp.nanosec] = T
    proc = {}
    with AnyReader([Path(a.run)]) as r:
        cons = [c for c in r.connections if c.topic == '/sins/lidar/processed']
        if not cons:
            raise SystemExit('run has no /sins/lidar/processed (record it with MORE_TOPICS)')
        for c, t, raw in r.messages(connections=cons):
            m = r.deserialize(raw, c.msgtype)
            st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if st in want:
                off = {f.name: f.offset for f in m.fields}
                b = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
                proc[st] = np.stack([b[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1)
    gt = pd.read_csv(f'{base}_gt.csv')
    gt = gt[gt['track_id'] >= 0]

    rows = []
    for st, i in sorted(want.items()):
        if st not in proc or st not in odom:
            continue
        scan = load_scan(CODA, a.seq, i)
        lab = np.fromfile(f'{CODA}/3d_semantic/os1/{a.seq}/3d_semantic_os1_{a.seq}_{i}.bin', np.uint8)
        full = np.fromfile(f'{CODA}/3d_comp/os1/{a.seq}/3d_comp_os1_{a.seq}_{i}.bin', np.float32).reshape(-1, 4)
        lab = lab[np.abs(full[:, :3]).sum(1) > 0]          # load_scan drops the empty returns: same mask
        T = odom[st] @ T_base_os1
        P = scan[:, :3] @ T[:3, :3].T + T[:3, 3]
        rng = np.linalg.norm(scan[:, :2], axis=1)
        inr = (rng > 1.2) & (rng < 25)
        Q = proc[st]
        d, k = cKDTree(P).query(Q, distance_upper_bound=a.match)
        ok = np.isfinite(d)
        q_terrain = ok & (lab[np.minimum(k, len(lab) - 1)] > 0)
        kept_vox = vox(Q)
        terr_vox = vox(P[inr & (lab > 0)])
        boxes = gt[gt['t_ns'] == st]
        agent_v, agent_k, low_v, low_k = 0, 0, 0, 0
        for _, b in boxes.iterrows():
            c, s = np.cos(b['yaw']), np.sin(b['yaw'])
            dd = P - [b['x'], b['y'], b['z']]
            u, v = dd[:, 0] * c + dd[:, 1] * s, -dd[:, 0] * s + dd[:, 1] * c
            inside = (np.abs(u) <= b['l'] / 2) & (np.abs(v) <= b['w'] / 2) & (np.abs(dd[:, 2]) <= b['h'] / 2) & (lab == 0)
            if inside.sum() < 5:
                continue
            bv = vox(P[inside])
            low = vox(P[inside & (P[:, 2] < b['z'] - b['h'] / 2 + 0.5)])
            agent_v += len(bv)
            agent_k += len(bv & kept_vox)
            low_v += len(low)
            low_k += len(low & kept_vox)
        rows.append(dict(frame=i, terrain_kept=int(q_terrain.sum()), terrain_vox=len(terr_vox),
                         terrain_vox_kept=len(terr_vox & kept_vox), agent_vox=agent_v, agent_vox_kept=agent_k,
                         low_vox=low_v, low_vox_kept=low_k))
    df = pd.DataFrame(rows)
    res = dict(run=a.run, window=a.window, scans=len(df),
               terrain_kept_per_scan=float(df.terrain_kept.mean()),
               terrain_kept_frac=float(df.terrain_vox_kept.sum() / max(1, df.terrain_vox.sum())),
               agent_kept_frac=float(df.agent_vox_kept.sum() / max(1, df.agent_vox.sum())),
               agent_low_kept_frac=float(df.low_vox_kept.sum() / max(1, df.low_vox.sum())))
    print(f"{Path(a.run).name}: {res['scans']} scans | terrain kept {res['terrain_kept_per_scan']:.0f} pts/scan "
          f"({res['terrain_kept_frac']:.1%} of terrain voxels) | agents kept {res['agent_kept_frac']:.1%}, "
          f"lowest 0.5 m {res['agent_low_kept_frac']:.1%}")
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
