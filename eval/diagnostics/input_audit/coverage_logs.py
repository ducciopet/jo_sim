#!/usr/bin/env python3
"""Input audit (17/11), per SINS run: output coverage and log anomalies.
 - coverage: annotated GT frames before the first / after the last /sins/tracker/tracks message (and > 0.15 s from any),
   gaps > 0.3 s between tracker messages, tracker messages vs LiDAR scans played;
 - logs: WARN / ERROR lines of det.log grouped by message type (numbers, ids and stamps stripped), top kinds.
usage: coverage_logs.py <run dir> <gt csv with t_ns column | 'mocap:<qualisys bag>'> <n scans in the sensor bag> [delta_s]
  (delta_s: lab only, mocap clock minus robot clock, from run_own_eval's alignment)"""
import re, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from rosbags.highlevel import AnyReader

run, gt_src, n_scans = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
delta = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
tt = []
with AnyReader([run]) as r:
    from rosbags.typesys import Stores, get_typestore
    for c, _, raw in r.messages([c for c in r.connections if c.topic == '/sins/tracker/tracks']):
        tt.append(st(r.deserialize(raw, c.msgtype).header))
tt = np.array(sorted(tt))
if gt_src.startswith('mocap:'):
    sys.path.insert(0, 'eval'); from harness.ros_io import load_mocap
    b = load_mocap(gt_src[6:]); p = b.get('person')
    fr = np.round(p[:, 0] * 10) / 10 * 1e9 - delta * 1e9            # 10 Hz grid of mocap times, robot clock
    fr = np.unique(fr.astype(np.int64))
else:
    g = pd.read_csv(gt_src, keep_default_na=False); fr = np.unique(g.t_ns.astype(np.int64).to_numpy())
k = np.clip(np.searchsorted(tt, fr), 1, len(tt) - 1)
near = np.minimum(np.abs(fr - tt[k - 1]), np.abs(fr - tt[k]))
before, after = int((fr < tt[0] - 0.15e9).sum()), int((fr > tt[-1] + 0.15e9).sum())
uncovered = int((near > 0.15e9).sum())
gaps = np.diff(tt) / 1e9
print(f'COVERAGE {run.name}: tracker msgs {len(tt)} / scans {n_scans} ({100 * len(tt) / max(1, n_scans):.0f}%), '
      f'annotated frames {len(fr)}, before first output {before}, after last {after}, >0.15 s from any output {uncovered}, '
      f'gaps > 0.3 s {int((gaps > 0.3).sum())} (max {gaps.max():.2f} s)')
log = run.parent / (run.name + '_logs') / 'det.log'
kinds = collections.Counter()
for l in open(log, errors='replace'):
    m = re.search(r'\[(WARN|ERROR|FATAL)\].*?\]: (.*)', l)
    if not m:
        continue
    node = re.match(r'\[([^\]]+)\]', l); node = node.group(1).split('-')[0] if node else '?'
    msg = re.sub(r'#\d+|\b\d+(\.\d+)?\b|\[[-\d., ]+\]|0x[0-9a-f]+|track_\d+', 'N', m.group(2))[:90]
    kinds[(m.group(1), node, msg)] += 1
print(f'LOG {run.name}: {sum(kinds.values())} warn/error lines, {len(kinds)} kinds')
for (lv, node, msg), n in kinds.most_common(12):
    print(f'  {n:6d} {lv:5s} {node:28s} {msg}')
