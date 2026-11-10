"""Why does a moving vehicle keep getting new tracks? Every '[lifecycle] spawn' line of a run with
track_debug_lifecycle (tracker_node) near an annotated vehicle (street GT) -> the tracks within 4 m of the new
detection and the gate that rejected each pairing. usage: spawn_reasons.py <det.log> <gt.csv>"""
import re, sys, numpy as np, pandas as pd
log, gtf = sys.argv[1], sys.argv[2]
G = pd.read_csv(gtf); G = G[G.track_id >= 0]; Gt = np.unique(G.t_ns)
GG = {t: g for t, g in G.groupby('t_ns')}
pat = re.compile(r"spawn #(\d+) t=([\d.]+) det_class='([^']*)' size=([\d.]+)x([\d.]+) at \(([-\d.]+),([-\d.]+)\)(.*)")
near_pat = re.compile(r"#(\d+) hits=(\d+) conf=(\d) spd=([\d.]+) d=([\d.]+) cls='([^']*)'.*?why=(\w+)")
rows = []
for line in open(log, errors='replace'):
    m = pat.search(line)
    if not m:
        continue
    t = float(m.group(2)); x, y = float(m.group(6)), float(m.group(7))
    k = Gt[np.argmin(np.abs(Gt - t * 1e9))]
    if abs(k - t * 1e9) > 0.06e9:
        continue
    g = GG[k]; dd = np.hypot(g.x - x, g.y - y); i = int(np.argmin(dd))
    if dd.iloc[i] > max(1.5, 0.5 * g.l.iloc[i] + 0.5):
        continue
    near = [dict(id=int(a), hits=int(b), conf=int(c), spd=float(s), d=float(d_), cls=cl, why=w)
            for a, b, c, s, d_, cl, w in near_pat.findall(m.group(8))]
    best = min(near, key=lambda n: n['d']) if near else None
    rows.append(dict(t=t, gid=int(g.track_id.iloc[i]), size=max(float(m.group(4)), float(m.group(5))),
                     gl=float(g.l.iloc[i]), n_near=len(near),
                     why=best['why'] if best else 'no track within 4 m', near_d=best['d'] if best else np.nan,
                     near_hits=best['hits'] if best else -1, near_conf=best['conf'] if best else -1,
                     whys=','.join(sorted({n['why'] for n in near}))))
R = pd.DataFrame(rows)
print(f'spawns on annotated vehicles: {len(R)} ({R.gid.nunique()} vehicles, {len(R)/max(1,R.gid.nunique()):.1f} per vehicle)')
print('\nnearest existing track -> why it was not associated:')
print((R.why.value_counts(normalize=True) * 100).round(1).to_string())
print('\nspawn box size vs GT length (median):', round(R['size'].median(), 2), '/', round(R.gl.median(), 2))
print('nearest track distance by reason (median):', R.groupby('why').near_d.median().round(2).to_dict())
print('nearest track confirmed share by reason:', R.groupby('why').near_conf.mean().round(2).to_dict())
print('\nany reason among ALL tracks within 4 m:', R.whys.str.split(',').explode().value_counts().to_dict())
