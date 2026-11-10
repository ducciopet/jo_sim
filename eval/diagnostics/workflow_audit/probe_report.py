"""Report of a topic_probe run: per topic messages, rate, empty share, stamp vs receive delay, frame ids, publishers
and subscribers; inputs nobody publishes; outputs nobody reads. usage: probe_report.py <prefix>"""
import sys, json, numpy as np, pandas as pd
pre = sys.argv[1]
M = pd.read_csv(pre + '_msgs.csv', keep_default_na=False)
G = [json.loads(l) for l in open(pre + '_graph.jsonl')]
graph = {}
for g in G:
    for t, v in g['topics'].items():
        e = graph.setdefault(t, {'type': v['type'], 'pub': set(), 'sub': set()})
        e['pub'] |= set(v['pub']) - {'topic_probe', '_ros2cli'}; e['sub'] |= set(v['sub']) - {'topic_probe', '_ros2cli'}
rows = []
V = M[M.topic == '/velodyne_points']
OFF = np.median(V.recv.astype(float) - pd.to_numeric(V.stamp)) if len(V) else 0.0   # sim->wall offset (bag replays at stamp)
for t, e in sorted(graph.items()):
    m = M[M.topic == t]
    n = len(m)
    st = pd.to_numeric(m.stamp, errors='coerce'); rv = m.recv.astype(float)
    nn = pd.to_numeric(m.n, errors='coerce')
    rate = (n - 1) / (rv.max() - rv.min()) if n > 2 else 0
    lag = ''
    if st.notna().sum() > 5:
        d = (rv - st).dropna()
        d = d - OFF; lag = f'{np.median(d)*1e3:.0f}/{np.percentile(d, 90)*1e3:.0f}'
    empty = f'{(nn == 0).mean():.0%}' if nn.notna().sum() else ''
    frames = ','.join(sorted(set(m.frame) - {''}))[:40]
    rows.append(dict(topic=t, type=e['type'].split('/')[-1], n=n, hz=round(rate, 1), empty=empty, med_n=('' if nn.notna().sum() == 0 else int(np.median(nn.dropna()))),
                     age_ms_med_p90=lag, frames=frames, pub=','.join(sorted(e['pub']))[:45], sub=','.join(sorted(e['sub']))[:60]))
R = pd.DataFrame(rows)
pd.set_option('display.width', 250); pd.set_option('display.max_colwidth', 60); pd.set_option('display.max_rows', 500)
print(R.to_string(index=False))
print('\nSUBSCRIBED, NO PUBLISHER:', [r.topic for r in R.itertuples() if r.sub and not r.pub])
print('PUBLISHED, NO MESSAGE RECEIVED:', [r.topic for r in R.itertuples() if r.pub and r.n == 0])
print('ALWAYS EMPTY (n=0 in every message):', [r.topic for r in R.itertuples() if r.empty == '100%'])
