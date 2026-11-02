#!/usr/bin/env python3
"""
Offline prototype of a terrain-following ground model for SINS, scored on CODa's per-point terrain labels.

Inputs are what SINS itself has at run time (a run recorded with MORE_TOPICS listing /sins/lidar/voxelized_sensor,
/sins/<cam>/depth/voxelized_global, /sins/static_structures/wall_markers): the LiDAR scan, each camera's depth cloud,
the odometry. Causal: scan k uses scans <= k only.

Ground grid (odom frame, cell --cell m). Per cell: the lowest return this scan from any source, plus memory of earlier
accepted ground. Growth from the robot: the cells under the robot are ground at the robot's own height (it stands on
them: this fills the LiDAR's blind cone without trusting the LiDAR there); cells are then visited nearest-first, each
reached from an already-set neighbour, and a cell's lowest return is accepted as ground only if it rises no more than
--step + cell*tan(--slope) above that neighbour (otherwise it is the bottom of an object: the cell inherits the
neighbour's height). A point is ground when it lies less than --margin above its cell's ground height.

Scored per labelled raw scan (CODa 3d_comp + 3d_semantic): terrain points left as obstacles (fraction), agent points
removed (all / lowest 0.5 m), both for this grid and for SINS's own crop (min over the cameras' floor planes from
wall_markers, same margin) as the reference.
usage: ground_grid_proto.py <run_bag> <window> [--cell 0.25 --slope 20 --step 0.1 --margin 0.15]
The LiDAR input is the FULL raw scan (CODa files, 0.1 m voxels): SINS's voxelized_sensor topic has the floor band cut.
"""
import argparse
import heapq
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'datasets'))
from coda_to_bag import load_scan, rot_from_quat     # noqa: E402

CODA = '/mnt/kloster-reichenau/datasets_SINS/CODa'
REPO = HERE.parents[1]


def cloud(m):
    off = {f.name: f.offset for f in m.fields}
    b = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    return np.stack([b[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1).astype(np.float64)


def stamp(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


class GroundGrid:
    def __init__(self, cell, slope_deg, step, radius, mem_s, robot_r=0.6, ref_min=False):
        self.ref_min = ref_min
        self.c, self.rise = cell, step + cell * np.tan(np.radians(slope_deg))
        self.radius, self.mem_s, self.robot_r = radius, mem_s, robot_r
        self.mem = {}                 # cell -> (ground z, time) of accepted ground observations

    def update(self, pts, robot_xyz, t):
        c = self.c
        ij = np.floor(pts[:, :2] / c).astype(np.int64)
        cells = {}
        order = np.argsort(pts[:, 2])
        for k, z in zip(map(tuple, ij[order]), pts[order, 2]):
            cells.setdefault(k, []).append(z)          # ascending heights per cell
        for k, (z, tm) in list(self.mem.items()):
            if t - tm > self.mem_s:
                del self.mem[k]
        r0 = np.floor(np.array(robot_xyz[:2]) / c).astype(int)
        R = int(self.radius / c)
        h = {}
        heap = []
        nr = int(np.ceil(self.robot_r / c))
        for di in range(-nr, nr + 1):
            for dj in range(-nr, nr + 1):
                k = (r0[0] + di, r0[1] + dj)
                # under the robot: what the sensors see there if they see it (odometry z drifts), else the robot's
                # own height (the blind cone of a LiDAR mounted high)
                z = self.surface(cells.get(k, ()), robot_xyz[2] - 0.3)
                h[k] = z if z is not None and z <= robot_xyz[2] + 0.3 else robot_xyz[2]
                heapq.heappush(heap, (np.hypot(di, dj), k))
        while heap:
            _, k = heapq.heappop(heap)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (k[0] + di, k[1] + dj)
                    if n in h or abs(n[0] - r0[0]) > R or abs(n[1] - r0[1]) > R:
                        continue
                    lim = self.rise * (np.sqrt(2) if di and dj else 1)
                    ref = h[k]
                    if self.ref_min:   # the LOWEST already-set neighbour: no climbing up an object one step at a time
                        ref = min([h[(n[0] + a_, n[1] + b_)] for a_ in (-1, 0, 1) for b_ in (-1, 0, 1)
                                   if (n[0] + a_, n[1] + b_) in h] or [ref])
                    lo, hi = ref - lim, ref + lim
                    # lowest return of the cell not more than one slope step BELOW the neighbour (deeper ones: floor
                    # reflections, multipath) and accepted if not more than one step ABOVE it (higher: an object)
                    z = self.surface(cells.get(n, ()), lo)
                    if z is None or z > hi:
                        mz = self.mem.get(n)
                        z = mz[0] if mz is not None and lo <= mz[0] <= hi else None
                    if z is not None and z <= hi:
                        h[n] = z
                        if n in cells:
                            self.mem[n] = (z, t)
                    else:
                        h[n] = ref
                    heapq.heappush(heap, (np.hypot(n[0] - r0[0], n[1] - r0[1]), n))
        self.h = h
        return h

    @staticmethod
    def surface(zs, lo, band=0.10):
        """Height of a cell's surface: median of its returns within `band` above the lowest one not below `lo`
        (the lowest return alone sits under the surface by the sensor noise)."""
        v = [z for z in zs if z >= lo]
        if not v:
            return None
        return float(np.median([z for z in v if z <= v[0] + band]))

    def height_above(self, P):
        ij = np.floor(P[:, :2] / self.c).astype(np.int64)
        g = np.array([self.h.get(tuple(k), np.nan) for k in ij])
        return P[:, 2] - g


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('window')
    ap.add_argument('--seq', default='0')
    ap.add_argument('--cell', type=float, default=0.25)
    ap.add_argument('--slope', type=float, default=20.0)
    ap.add_argument('--step', type=float, default=0.10)
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--radius', type=float, default=25.0)
    ap.add_argument('--mem', type=float, default=30.0)
    ap.add_argument('--ref-min', action='store_true')
    ap.add_argument('--no-lidar', action='store_true')
    ap.add_argument('--no-cams', action='store_true')
    ap.add_argument('--out')
    a = ap.parse_args()
    base = REPO / f'bags/coda/coda{a.seq}_{a.window}'
    meta = json.load(open(f'{base}_gt.json'))
    T_base_os1 = np.array(meta['T_base_velodyne'])
    t_scan = (np.loadtxt(f'{CODA}/timestamps/{a.seq}.txt') * 1e9).astype(np.int64)
    labelled = {int(t_scan[i]): i for i in range(meta['first'], meta['last'] + 1)
                if Path(f'{CODA}/3d_semantic/os1/{a.seq}/3d_semantic_os1_{a.seq}_{i}.bin').exists()}
    gt = pd.read_csv(f'{base}_gt.csv')
    gt = gt[gt['track_id'] >= 0]
    frame_of = {int(t_scan[i]): i for i in range(meta['first'], meta['last'] + 1)}

    # all messages of the run in time order
    ev = []
    with AnyReader([Path(a.run)]) as r:
        want = {'/odometry/filtered', '/sins/lidar/voxelized_sensor', '/sins/static_structures/wall_markers',
                '/sins/front_camera/depth/voxelized_global', '/sins/stereo_camera/depth/voxelized_global'}
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in want]):
            ev.append((t, c.topic, r.deserialize(raw, c.msgtype)))
    ev.sort(key=lambda e: e[0])

    grid = GroundGrid(a.cell, a.slope, a.step, a.radius, a.mem, ref_min=a.ref_min)
    T_ob = None
    cams = {}
    planes = {}
    rows = []
    band = {k: [0, 0, 0, 0] for k in ((0, 3), (3, 8), (8, 15), (15, 25))}
    for t, topic, m in ev:
        if topic == '/odometry/filtered':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T_ob = np.eye(4)
            T_ob[:3, :3] = rot_from_quat(q.x, q.y, q.z, q.w)
            T_ob[:3, 3] = [p.x, p.y, p.z]
        elif topic.endswith('voxelized_global'):
            cams[topic] = cloud(m)
        elif topic.endswith('wall_markers'):
            for mk in m.markers:
                if mk.ns == 'floor':
                    o = mk.pose.orientation
                    n = rot_from_quat(o.x, o.y, o.z, o.w)[:, 2]
                    pos = np.array([mk.pose.position.x, mk.pose.position.y, mk.pose.position.z])
                    planes[mk.id] = (n, -n @ pos)
        elif T_ob is not None:
            st = stamp(m)
            T = T_ob @ T_base_os1
            if a.no_lidar or st not in frame_of:
                parts = []
            else:   # the full scan (SINS's voxelized_sensor topic already has the floor band cut for the wall search)
                L = load_scan(CODA, a.seq, frame_of[st])[:, :3].astype(np.float64)
                L = L[np.unique(np.floor(L / 0.1).astype(np.int64), axis=0, return_index=True)[1]]
                parts = [L @ T[:3, :3].T + T[:3, 3]]
            if not a.no_cams:
                parts += list(cams.values())
            pts = np.vstack(parts) if parts else np.zeros((0, 3))
            grid.update(pts, T_ob[:3, 3], st * 1e-9)
            if st not in labelled:
                continue
            i = labelled[st]
            scan = load_scan(CODA, a.seq, i)
            full = np.fromfile(f'{CODA}/3d_comp/os1/{a.seq}/3d_comp_os1_{a.seq}_{i}.bin', np.float32).reshape(-1, 4)
            lab = np.fromfile(f'{CODA}/3d_semantic/os1/{a.seq}/3d_semantic_os1_{a.seq}_{i}.bin', np.uint8)
            lab = lab[np.abs(full[:, :3]).sum(1) > 0]
            P = scan[:, :3] @ T[:3, :3].T + T[:3, 3]
            rng = np.linalg.norm(scan[:, :2], axis=1)
            sel = (rng > 1.2) & (rng < 25)
            P, lab = P[sel], lab[sel]
            hg = grid.height_above(P)
            g_grid = np.nan_to_num(hg, nan=np.inf) < a.margin
            if planes:
                hp = np.min(np.stack([P @ n + d for n, d in planes.values()], 1), 1)
                g_pl = hp < a.margin
            else:
                g_pl = np.zeros(len(P), bool)
            agent, low = np.zeros(len(P), bool), np.zeros(len(P), bool)
            for _, b in gt[gt['t_ns'] == st].iterrows():
                c, s = np.cos(b['yaw']), np.sin(b['yaw'])
                dd = P - [b['x'], b['y'], b['z']]
                u, v = dd[:, 0] * c + dd[:, 1] * s, -dd[:, 0] * s + dd[:, 1] * c
                ins = (np.abs(u) <= b['l'] / 2) & (np.abs(v) <= b['w'] / 2) & (np.abs(dd[:, 2]) <= b['h'] / 2) & (lab == 0)
                if ins.sum() >= 5:
                    agent |= ins
                    zr = P[:, 2] - (b['z'] - b['h'] / 2)
                    low |= ins & (zr > 0.25) & (zr < 0.75)   # legs/wheels well clear of the ground: no correct crop cuts them
            terr = lab > 0
            dr = np.hypot(*(P[:, :2] - T_ob[:2, 3]).T)
            for r0, r1 in ((0, 3), (3, 8), (8, 15), (15, 25)):
                s_ = terr & (dr >= r0) & (dr < r1)
                band[(r0, r1)][0] += s_.sum()
                band[(r0, r1)][1] += (s_ & ~g_grid).sum()
                band[(r0, r1)][2] += (s_ & ~g_pl).sum()
                hh = hg[s_ & ~g_grid]
                band[(r0, r1)][3] += np.isnan(hh).sum()
            rows.append(dict(terr=terr.sum(), agent=agent.sum(), low=low.sum(),
                             grid_terr_left=(terr & ~g_grid).sum(), grid_agent_cut=(agent & g_grid).sum(),
                             grid_low_cut=(low & g_grid).sum(),
                             pl_terr_left=(terr & ~g_pl).sum(), pl_agent_cut=(agent & g_pl).sum(),
                             pl_low_cut=(low & g_pl).sum()))
    d = pd.DataFrame(rows).sum()
    res = {k: float(d[f'{k}_terr_left'] / d.terr) for k in ('grid', 'pl')}
    out = dict(window=a.window, scans=len(rows), args=vars(a),
               grid=dict(terrain_left=float(d.grid_terr_left / d.terr), agent_cut=float(d.grid_agent_cut / d.agent),
                         low_cut=float(d.grid_low_cut / d.low)),
               planes=dict(terrain_left=float(d.pl_terr_left / d.terr), agent_cut=float(d.pl_agent_cut / d.agent),
                           low_cut=float(d.pl_low_cut / d.low)))
    for k in ('planes', 'grid'):
        o = out[k]
        print(f"{a.window} {k:6s}: terrain left as obstacle {o['terrain_left']:.1%} | agent points cut {o['agent_cut']:.1%}"
              f" | legs 0.25-0.75 m cut {o['low_cut']:.1%}")
    for k, (n, gl, pl, nan) in band.items():
        print(f'  terrain {k[0]}-{k[1]} m: {n} pts, left by grid {gl / max(1, n):.1%} (outside grid {nan / max(1, n):.1%}), by planes {pl / max(1, n):.1%}')
    if a.out:
        json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
