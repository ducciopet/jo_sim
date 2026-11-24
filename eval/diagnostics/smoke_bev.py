#!/usr/bin/env python3
"""Smoke-test BEV check (FINAL_RUN_SPEC 7): a few input scans moved to odom (static base_link->sensor TF + odometry pose)
with the published obstacles (/perception/tracked_dynamic_obstacles, odom) of the nearest stamp drawn on top.
usage: smoke_bev.py <input bag> <odom bag> <run bag> <out.png> [cloud topic] [n panels]"""
import sys
from pathlib import Path
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402
inb, odb, run, out = sys.argv[1:5]; TOP = sys.argv[5] if len(sys.argv) > 5 else None; N = int(sys.argv[6]) if len(sys.argv) > 6 else 4
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
def qm(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def T(tr, rot): M = np.eye(4); M[:3, :3] = qm(rot); M[:3, 3] = [tr.x, tr.y, tr.z]; return M
stat, clouds = {}, []
with AnyReader([Path(inb)]) as r:
    cs = [c for c in r.connections if c.msgtype == 'sensor_msgs/msg/PointCloud2' and (TOP is None or c.topic == TOP)]
    n = cs[0].msgcount; pick = set(np.linspace(n * 0.1, n * 0.9, N).astype(int))
    for c, _, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
        for t in r.deserialize(raw, c.msgtype).transforms: stat[t.child_frame_id] = (t.header.frame_id, T(t.transform.translation, t.transform.rotation))
    for i, (c, _, raw) in enumerate(r.messages(cs)):
        if i not in pick: continue
        m = r.deserialize(raw, c.msgtype); off = {f.name: f.offset for f in m.fields}
        a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
        xyz = np.stack([a[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1)
        clouds.append((st(m.header), m.header.frame_id, xyz[np.isfinite(xyz).all(1)]))
def to_base(fr):
    M = np.eye(4)
    while fr in stat and fr not in ('base_link',): p, A = stat[fr]; M = A @ M; fr = p
    return M, fr
odo = []
with AnyReader([Path(odb)]) as r:
    for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
        m = r.deserialize(raw, c.msgtype); odo.append((st(m.header), T(m.pose.pose.position, m.pose.pose.orientation)))
ot = np.array([o[0] for o in odo])
pred, _, _ = ce.load_obstacles(run, None); pt = np.unique(pred.t_ns.to_numpy())
fig, ax = plt.subplots(1, len(clouds), figsize=(6 * len(clouds), 6))
for a_, (t, fr, xyz) in zip(np.atleast_1d(ax), clouds):
    Mb, root = to_base(fr); Mo = odo[int(np.argmin(np.abs(ot - t)))][1]
    P = (Mo @ Mb @ np.c_[xyz, np.ones(len(xyz))].T).T[:, :3]
    c = Mo[:2, 3]; k = np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < 30; P = P[k]
    a_.scatter(P[:, 0], P[:, 1], s=0.2, c=P[:, 2], cmap='viridis', vmin=c[0] * 0 + Mo[2, 3] - 2, vmax=Mo[2, 3] + 2)
    kk = pt[np.argmin(np.abs(pt - t))]; O = pred[pred.t_ns == kk]
    a_.scatter(O.x, O.y, s=60, facecolors='none', edgecolors='r', lw=1.5, label=f'obstacles ({len(O)}, dt {(kk - t) / 1e6:.0f} ms)')
    a_.plot(*c, 'k^', ms=10); a_.set_aspect('equal'); a_.set_xlim(c[0] - 30, c[0] + 30); a_.set_ylim(c[1] - 30, c[1] + 30)
    a_.set_title(f't {t / 1e9:.1f}  cloud {fr} -> {root} -> odom', fontsize=9); a_.legend(fontsize=8, loc='upper right')
plt.tight_layout(); plt.savefig(out, dpi=80); print('saved', out, 'panels', len(clouds), 'obstacles total', len(pred), 'tracks', pred.track_id.nunique())
