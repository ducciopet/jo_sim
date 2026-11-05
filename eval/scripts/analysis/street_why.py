"""Per reference object-frame on the street bag: where SINS loses it and why, from the diagnostic run
(track_debug_csv + [lifecycle] spawn lines + clusters in the run bag).
usage: street_why.py <run_dir> <tracks.csv> <det.log>"""
import sys, re
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np, pandas as pd
from rosbags.highlevel import AnyReader

run, csvp, logp = sys.argv[1:4]
g = pd.read_csv('bags/mdp/mdp_street_gt.csv'); g = g[(g.track_id >= 0) & (g.range <= 15) & (g.range >= 1.2)].sort_values(['track_id', 't_ns'])
sp = []
for _, d in g.groupby('track_id', sort=False):
    t = d.t_ns.values / 1e9; x = d.x.values; y = d.y.values
    for i in range(len(d)):
        j0 = np.searchsorted(t, t[i] - 1); j1 = np.searchsorted(t, t[i] + 1, 'right') - 1
        sp.append(np.hypot(x[j1] - x[j0], y[j1] - y[j0]) / max(t[j1] - t[j0], .1) if t[j1] - t[j0] > .5 else np.nan)
g['speed'] = sp
g['grp'] = np.where(g['class'] == 'vehicle', 'car', np.where(g.speed > 3, 'fast_small', np.where(g.speed < .5, 'still_person', 'walker')))

# clusters
D = {}
with AnyReader([Path(run)]) as r:
    c = [c for c in r.connections if c.topic == '/sins/dbscan_detector/fused/detections3d']
    for cc, _, raw in r.messages(c):
        m = r.deserialize(raw, cc.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        D[t] = np.array([[d.bbox.center.position.x, d.bbox.center.position.y, max(d.bbox.size.x, d.bbox.size.y)] for d in m.detections]).reshape(-1, 3)

T = pd.read_csv(csvp)
T['t_ns'] = (T.t * 1000).round().astype('int64') * 1000000
gt_ts = np.unique(g.t_ns.values)
# align csv stamps (ms precision) to the exact GT stamps
idx = np.searchsorted(gt_ts, T.t_ns.values).clip(1, len(gt_ts) - 1)
best = np.where(np.abs(gt_ts[idx] - T.t_ns.values) < np.abs(gt_ts[idx - 1] - T.t_ns.values), gt_ts[idx], gt_ts[idx - 1])
T['t_ns'] = best
TT = {t: d for t, d in T[T.matched == 1].groupby('t_ns')}

# spawn lines: new track id -> reasons the nearby existing tracks did not take that detection
spawn = {}
pat = re.compile(r'\[lifecycle\] spawn #(\d+) .*?at \(([-\d.]+),([-\d.]+)\)(.*)')
for l in open(logp, errors='ignore'):
    m = pat.search(l)
    if m:
        whys = re.findall(r'#(\d+) d=([\d.]+) [^|]*why=(\w+)', m.group(4))
        spawn[int(m.group(1))] = whys

rows = []
for _, o in g.iterrows():
    t = o.t_ns; R = 2.5 if o['class'] == 'vehicle' else 1.0
    cl = D.get(t)
    has_cl = cl is not None and len(cl) and np.min(np.hypot(cl[:, 0] - o.x, cl[:, 1] - o.y)) < R
    tr = TT.get(t)
    st = None
    if tr is not None and len(tr):
        dd = np.hypot(tr.ox.values - o.x, tr.oy.values - o.y); k = np.argmin(dd)
        if dd[k] < R: st = tr.iloc[k]
    if not has_cl: why = '1 no cluster'
    elif st is None: why = '2 cluster, no matched track'
    elif st.confirmed == 0: why = '3 track not confirmed'
    elif st.status == 'dynamic': why = '9 DYNAMIC'
    elif st.status == 'suppressed': why = '4 suppressed (speed < 0.07)'
    elif not st.speed_ok: why = '5 KF speed < 0.25'
    elif not (st.kf_conf or st.vote or st.cls != '-'): why = '6 no corroboration (kf_conf/vote)'
    elif st.disocc: why = '7 disocclusion block'
    elif not st.size_ok: why = '7b size gate'
    else: why = '8 debounce (dyn_ticks < confirm)'
    rows.append(dict(grp=o.grp, gt=o.track_id, fov=bool(o.in_fov), why=why, tid=-1 if st is None else int(st.id),
                     hits=np.nan if st is None else st.hits, dyn=np.nan if st is None else st.dyn_ticks,
                     ct=np.nan if st is None else st.confirm_ticks))
R = pd.DataFrame(rows)
pd.set_option('display.width', 200)
print(R.groupby('grp').why.value_counts(normalize=True).unstack(0).round(2).fillna(0).to_string())
print('\nfov split (car, walker):')
print(R[R.grp.isin(['car', 'walker'])].groupby(['grp', 'fov']).why.value_counts(normalize=True).unstack([0, 1]).round(2).fillna(0).to_string())
# fragmentation: distinct track ids per reference object (matched frames only)
fr = R[R.tid >= 0].groupby(['grp', 'gt']).tid.nunique()
life = R[R.tid >= 0].groupby(['grp', 'gt', 'tid']).size()
print('\ntrack ids per object (median, mean):', fr.groupby('grp').agg(['median', 'mean']).round(1).to_dict())
print('frames per (object, track id) median:', life.groupby('grp').median().to_dict())
# why new tracks were spawned on reference objects: reasons the closest existing track gave
sp_reason = defaultdict(Counter)
for (grp, gt), d in R[R.tid >= 0].groupby(['grp', 'gt']):
    for tid in d.tid.unique()[1:]:
        w = spawn.get(tid, [])
        if w:
            w = sorted(w, key=lambda z: float(z[1]))[0][2]
        else:
            w = 'no track within 1.5 m'
        sp_reason[grp][w] += 1
print('\nre-spawn on the same object, reason of the nearest old track:')
for k, c in sp_reason.items(): print(' ', k, dict(c.most_common(8)))
