"""Why do moving vehicles keep getting new tracks? Follows every annotated vehicle of the Marina di Pisa street, scan by
scan, through the fused detections (pieces inside the GT box) and the tracker output (track ids on it), and joins the
'[lifecycle] spawn' lines of the same run (track_debug_lifecycle) to tell apart:
  - fragment spawn: the vehicle came in >= 2 pieces and one of them, not taken by the vehicle's track, became a track;
  - lost spawn: the vehicle came in ONE piece and its existing track did not take it (gate), so a new one started;
  - first spawn: no track on the vehicle in the previous scan.
usage: vehicle_passages.py <run dir> <det.log> [gt csv]"""
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

run, log = sys.argv[1], sys.argv[2]
gtf = sys.argv[3] if len(sys.argv) > 3 else 'bags/mdp/mdp_street_vehicle_gt.csv'
G = pd.read_csv(gtf)
G = G[(G.track_id >= 0) & (G['range'] <= 15) & (G['range'] >= 1.2)]

DET, TRK = {}, {}
with AnyReader([Path(run)]) as r:
    cons = [c for c in r.connections if c.topic in ('/sins/dbscan_detector/fused/detections3d', '/sins/tracker/tracks')]
    for c, _, raw in r.messages(cons):
        m = r.deserialize(raw, c.msgtype)
        t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        rows = []
        for d in m.detections:
            b = d.bbox
            q = b.center.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            st = d.results[0].hypothesis.class_id if d.results else ''
            v = d.results[0].pose.pose.position if d.results else None
            rows.append((b.center.position.x, b.center.position.y, b.size.x, b.size.y, yaw, d.id, st,
                         0.0 if v is None else np.hypot(v.x, v.y)))
        (DET if c.topic.endswith('detections3d') else TRK)[t] = rows

spawn_pat = re.compile(r"spawn #(\d+) t=([\d.]+) det_class='([^']*)' size=([\d.]+)x([\d.]+) at \(([-\d.]+),([-\d.]+)\)(.*)")
why_pat = re.compile(r"#(\d+) hits=(\d+) conf=(\d) spd=([\d.]+) d=([\d.]+) cls='([^']*)'.*?why=(\w+)")
SP = defaultdict(list)
for line in open(log, errors='replace'):
    m = spawn_pat.search(line)
    if m:
        near = why_pat.findall(m.group(8))
        best = min(near, key=lambda n: float(n[4])) if near else None
        SP[round(float(m.group(2)) * 10) ].append(dict(id=int(m.group(1)), x=float(m.group(6)), y=float(m.group(7)),
                                                       size=max(float(m.group(4)), float(m.group(5))),
                                                       why=best[6] if best else 'none<4m', near_id=int(best[0]) if best else -1,
                                                       near_d=float(best[4]) if best else np.nan))


def inside(rows, g, margin):
    c, s = np.cos(g.yaw), np.sin(g.yaw)
    out = []
    for r_ in rows:
        lx = c * (r_[0] - g.x) + s * (r_[1] - g.y)
        ly = -s * (r_[0] - g.x) + c * (r_[1] - g.y)
        if abs(lx) <= g.l / 2 + margin and abs(ly) <= g.w / 2 + margin:
            out.append(r_)
    return out


dkeys = np.array(sorted(DET))
frames = []
for gid, gg in G.groupby('track_id'):
    prev_ids = set()
    for _, g in gg.sort_values('t_ns').iterrows():
        k = dkeys[np.argmin(np.abs(dkeys - g.t_ns))]
        if abs(k - g.t_ns) > 0.06e9:
            continue
        pieces = inside(DET[k], g, 0.3)
        trks = inside(TRK.get(k, []), g, 0.3)
        ids = {t_[5] for t_ in trks}
        sp = [s for s in SP.get(round(k / 1e8), []) if any(abs(s['x'] - p[0]) < 0.05 and abs(s['y'] - p[1]) < 0.05 for p in pieces)]
        for s in sp:
            kind = 'first' if not prev_ids else ('fragment' if len(pieces) >= 2 else 'lost')
            frames.append(dict(gid=gid, t=k, kind='spawn', spawn_kind=kind, why=s['why'], near_in_veh=f"track_{s['near_id']}" in prev_ids,
                               near_d=s['near_d'], n_pieces=len(pieces), spd=g.get('score', np.nan), rng=g['range'], gl=g.l))
        frames.append(dict(gid=gid, t=k, kind='frame', n_pieces=len(pieces), n_tracks=len(ids),
                           max_piece=max([max(p[2], p[3]) for p in pieces], default=0.0), gl=g.l, rng=g['range'],
                           states=','.join(sorted({t_[6] for t_ in trks})), ids=','.join(sorted(ids)),
                           new_id=bool(ids - prev_ids) and bool(prev_ids)))
        if ids:
            prev_ids = ids
F = pd.DataFrame(frames)
fr, spw = F[F.kind == 'frame'], F[F.kind == 'spawn']
print(f'vehicles {fr.gid.nunique()}, annotated scans {len(fr)}')
print('pieces of the vehicle per scan (share of scans):', (fr.n_pieces.value_counts(normalize=True).sort_index() * 100).round(1).to_dict())
print('tracks on the vehicle per scan:', (fr.n_tracks.value_counts(normalize=True).sort_index() * 100).round(1).to_dict())
print('largest piece / GT length (median):', round((fr.max_piece / fr.gl)[fr.n_pieces > 0].median(), 2))
print('scans with an id change vs previous scan: %.1f%%' % (100 * fr.new_id.mean()))
ids_per = fr.groupby('gid').ids.apply(lambda s: len({i for x in s for i in x.split(',') if i}))
print('distinct track ids per vehicle: median %.0f, mean %.1f' % (ids_per.median(), ids_per.mean()))
print(f'\nspawns on vehicles: {len(spw)}')
print('kind:', (spw.spawn_kind.value_counts(normalize=True) * 100).round(1).to_dict())
for k in ('fragment', 'lost'):
    s = spw[spw.spawn_kind == k]
    if len(s):
        print(f'  {k}: why the nearest track did not take it:', (s.why.value_counts(normalize=True) * 100).round(1).to_dict())
        print(f'  {k}: nearest track was the vehicle\'s own track: {100 * s.near_in_veh.mean():.0f}%, median distance {s.near_d.median():.2f} m')
print('state of the tracks on moving vehicles (share of scans with a track):',
      (fr[fr.n_tracks > 0].states.str.split(',').explode().value_counts(normalize=True) * 100).round(1).to_dict())
print('pieces vs range band:', fr.groupby(pd.cut(fr.rng, [0, 5, 10, 15])).n_pieces.mean().round(2).to_dict())
F.to_csv(Path(__file__).with_name('vehicle_passages.csv'), index=False)
