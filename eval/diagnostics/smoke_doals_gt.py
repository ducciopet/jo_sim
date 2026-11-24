#!/usr/bin/env python3
"""DOALS smoke check (FINAL_RUN_SPEC 7): do the GT flat indices land on people? For annotated scans of the converted bag,
the GT points (indices into the stored cloud order) in BEV over the whole scan, plus their spatial spread (people-sized
blobs expected) and the fraction that are NaN (invalid returns). usage: smoke_doals_gt.py <bag> <gt.json> <out.png> [k scans]"""
import json, sys
from pathlib import Path
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from scipy.cluster.hierarchy import fcluster, linkage
bag, gtf, out = sys.argv[1:4]; K = int(sys.argv[4]) if len(sys.argv) > 4 else 3
G = json.load(open(gtf)); ann = {a['t_ns']: np.array(a['indices']) for a in G['annotated']}
want = sorted(ann)[:: max(1, len(ann) // K)][:K]; got = {}
with AnyReader([Path(bag)]) as r:
    for c, _, raw in r.messages([c for c in r.connections if c.topic == '/velodyne_points']):
        m = r.deserialize(raw, c.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if t not in want: continue
        off = {f.name: f.offset for f in m.fields}; a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
        got[t] = (np.stack([a[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1), m.height, m.width)
fig, ax = plt.subplots(1, len(got), figsize=(6 * len(got), 6))
for a_, t in zip(np.atleast_1d(ax), sorted(got)):
    xyz, h, w = got[t]; I = ann[t]; I = I[I < len(xyz)]; g = xyz[I]; ok = np.isfinite(g).all(1); g = g[ok]
    allp = xyz[np.isfinite(xyz).all(1)]
    lab = fcluster(linkage(g[:, :2], 'single'), 0.5, 'distance') if len(g) > 1 else np.ones(len(g), int)
    sz = [np.ptp(g[lab == l, :2], 0).max() for l in np.unique(lab) if (lab == l).sum() >= 5]
    a_.scatter(allp[:, 0], allp[:, 1], s=0.1, c='0.6'); a_.scatter(g[:, 0], g[:, 1], s=2, c='r')
    a_.set_aspect('equal'); a_.set_xlim(-20, 20); a_.set_ylim(-20, 20)
    msg = (f'cloud {h}x{w}, GT idx {len(ann[t])} (NaN {100 * (1 - ok.mean()):.0f}%), blobs>=5pts {len(sz)}, '
           f'blob extent med {np.median(sz) if sz else 0:.2f} m max {max(sz) if sz else 0:.2f} m')
    a_.set_title(f't {t}\n{msg}', fontsize=8); print(t, msg)
plt.tight_layout(); plt.savefig(out, dpi=80); print('saved', out)
