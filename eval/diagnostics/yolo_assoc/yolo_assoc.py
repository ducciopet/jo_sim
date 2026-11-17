#!/usr/bin/env python3
"""YOLO 2D <-> 3D track association diagnosis (17/11), CODa, measure only.
For a SINS run (recorded /sins/tracker/tracks and /yolo_seg_track/<cam>/detections for front_camera and stereo_camera):
1. class-loss events: a track published with an object class at tick i-1 and without one at tick i (still alive).
   The 3 ticks i-2, i-1, i (the decay streak) are checked per camera with a fresh YOLO frame (<= 0.1 s): the track box
   projected (odometry + the run's static TF + camera_info) ->
     a  a YOLO box of the SAME class covers it (projected centre inside the box, or IoU >= 0.3): YOLO saw it, the
        association did not happen;
     b  truncated by the image border (> 30% of the projected box outside) or covered by a closer track (>= 50% of its
        projected box inside a nearer track's box);
     c  visible, unoccluded, no YOLO box of its class.
   event = a if any tick is a, else b if any is b, else c; 'nofov' if no camera sees it on any of the 3 ticks.
2. ByteTrack ids (det.id per camera): the (camera, id) that covered the track on the 6 ticks before the streak; whether
   that same (camera, id) is still published in that camera during the streak ("loss with the 2D id alive").
3. multi-camera: per classified track tick, how many cameras have a same-class box covering it (both = seen twice),
   class conflicts between cameras, and along each track the covering (camera, id): id changes within a camera vs
   camera switches (an id that differs only because another camera now sees it).
4. what-if: over the unclassified period after each loss (until the class returns or the track ends, max 50 ticks),
   ticks that (3) keep-while-2D-id-alive would have kept (that (camera, id) still covers it), that (4) evidence-only
   decay would have kept (tick b or not visible), and either.
usage: yolo_assoc.py <window e.g. 10_w4> <run e.g. LXA_v2_coda10_w4>  -> prints, appends a row to yolo_assoc_rows.csv"""
import json, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from rosbags.highlevel import AnyReader

R = Path(__file__).resolve().parents[3]
w, runname = sys.argv[1], sys.argv[2]
run = R / 'bags/coda/runs' / runname
NAS = Path('/mnt/nas-imperatore/datasets_SINS/derived/coda_windows')
STATES = {'static', 'dynamic', 'potentially_dynamic', 'tracked'}
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec

def T_of(t, q):
    x, y, z, ww = q.x, q.y, q.z, q.w
    Rm = np.array([[1-2*(y*y+z*z), 2*(x*y-z*ww), 2*(x*z+y*ww)], [2*(x*y+z*ww), 1-2*(x*x+z*z), 2*(y*z-x*ww)], [2*(x*z-y*ww), 2*(y*z+x*ww), 1-2*(x*x+y*y)]])
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = [t.x, t.y, t.z]; return T

# run: tracks, YOLO per camera, static TF
tracks, yolo, statics = [], collections.defaultdict(list), {}
with AnyReader([run]) as r:
    for c, _, raw in r.messages([c for c in r.connections if c.topic in ('/sins/tracker/tracks', '/tf_static')
                                 or (c.topic.startswith('/yolo_seg_track/') and c.topic.endswith('/detections'))]):
        m = r.deserialize(raw, c.msgtype)
        if c.topic == '/tf_static':
            for tr in m.transforms:
                statics[(tr.header.frame_id, tr.child_frame_id)] = T_of(tr.transform.translation, tr.transform.rotation)
        elif c.topic == '/sins/tracker/tracks':
            rows = []
            for d in m.detections:
                cls = [h.hypothesis.class_id for h in d.results if h.hypothesis.class_id not in STATES]
                b = d.bbox
                rows.append((int(d.id.split('_')[-1]), cls[0] if cls else '', np.array([b.center.position.x, b.center.position.y, b.center.position.z]),
                             np.array([b.size.x, b.size.y, b.size.z]), b.center.orientation))
            tracks.append((st(m.header), rows))
        else:
            cam = c.topic.split('/')[2]
            yolo[cam].append((st(m.header), [(d.id, d.results[0].hypothesis.class_id if d.results else '', d.bbox.center.position.x,
                                             d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y) for d in m.detections]))
tracks.sort(key=lambda x: x[0])
# cameras
cams = {}
for cam, suf in (('front_camera', ''), ('stereo_camera', '_stereo')):
    if cam not in yolo or not yolo[cam]:
        continue
    with AnyReader([NAS / f'coda{w}{suf}']) as r:
        con = [c for c in r.connections if c.topic == f'/{cam}/camera/color/camera_info']
        if not con:
            continue
        info = next(r.deserialize(raw, c.msgtype) for c, _, raw in r.messages(con))
    T_vc = statics[('velodyne', f'{cam}_depth_optical_frame')] @ statics[(f'{cam}_depth_optical_frame', f'{cam}_color_optical_frame')]
    yolo[cam].sort(key=lambda x: x[0])
    cams[cam] = dict(K=np.array(info.k).reshape(3, 3), W=info.width, H=info.height, T_vc=T_vc,
                     yt=np.array([y[0] for y in yolo[cam]]), y=yolo[cam])
T_bv = np.array(json.load(open(R / f'bags/coda/coda{w}_gt.json'))['T_base_velodyne'])
odo = []
with AnyReader([R / f'bags/coda/coda{w}_odom']) as r:
    for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
        m = r.deserialize(raw, c.msgtype); odo.append((st(m.header), T_of(m.pose.pose.position, m.pose.pose.orientation)))
ot = np.array([o[0] for o in odo])

def corners(cn, sz, q):
    Rm = T_of(type('p', (), dict(x=0, y=0, z=0))(), q)[:3, :3]
    s = np.array([[sx, sy, sz_] for sx in (-1, 1) for sy in (-1, 1) for sz_ in (-1, 1)]) * sz / 2
    return s @ Rm.T + cn

def project(cam, t, P):
    C = cams[cam]
    Tc = np.linalg.inv(odo[int(np.argmin(np.abs(ot - t)))][1] @ T_bv @ C['T_vc'])
    q = P @ Tc[:3, :3].T + Tc[:3, 3]
    if (q[:, 2] < 0.5).any():
        return None
    uv = (q @ C['K'].T)[:, :2] / q[:, 2:3]
    return np.array([uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max()]), float(np.median(q[:, 2]))

def area(b): return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
def inter(a, b): return area([max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])])

def tick_view(k, tid, cls):
    """per camera: (state, covering (cam,id) or None) for track tid at tick k; state a/b/c or None (not in view)."""
    t, rows = tracks[k]
    me = next((r for r in rows if r[0] == tid), None)
    out = {}
    if me is None:
        return out
    for cam, C in cams.items():
        j = int(np.argmin(np.abs(C['yt'] - t)))
        if abs(C['yt'][j] - t) > 0.1e9:
            continue
        pr = project(cam, t, corners(me[2], np.maximum(me[3], 0.3), me[4]))
        if pr is None:
            continue
        b, z = pr
        clip = [max(b[0], 0), max(b[1], 0), min(b[2], C['W']), min(b[3], C['H'])]
        if area(clip) <= 0:
            continue
        cx, cy = (clip[0] + clip[2]) / 2, (clip[1] + clip[3]) / 2
        cov = None
        for did, dcls, x, y, sx, sy in C['y'][j][1]:
            yb = [x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2]
            iou = inter(clip, yb) / max(1e-6, area(clip) + area(yb) - inter(clip, yb))
            if dcls == cls and ((yb[0] <= cx <= yb[2] and yb[1] <= cy <= yb[3]) or iou >= 0.3):
                cov = (cam, did); break
        if cov:
            out[cam] = ('a', cov); continue
        trunc = 1 - area(clip) / max(1e-6, area(b)) > 0.3
        occ = False
        for o in rows:
            if o[0] == tid:
                continue
            po = project(cam, t, corners(o[2], np.maximum(o[3], 0.3), o[4]))
            if po and po[1] < z - 0.5 and inter(clip, po[0]) >= 0.5 * area(clip):
                occ = True; break
        out[cam] = ('b' if trunc or occ else 'c', None)
    return out

# class sequence per track
seq = collections.defaultdict(list)          # tid -> [(k, cls)]
for k, (t, rows) in enumerate(tracks):
    for r in rows:
        seq[r[0]].append((k, r[1]))
ev, rows_out = collections.Counter(), []
alive_loss = alive_known = 0
wi3 = wi4 = wi34 = wtot = 0
for tid, s in seq.items():
    for n in range(1, len(s)):
        (k0, c0), (k1, c1) = s[n - 1], s[n]
        if not c0 or c1 or k1 != k0 + 1:
            continue
        streak = [kk for kk in (k1 - 2, k1 - 1, k1) if kk >= 0]
        views = [tick_view(kk, tid, c0) for kk in streak]
        sts = [v[0] for vw in views for v in vw.values()]
        e = 'nofov' if not sts else 'a' if 'a' in sts else 'b' if 'b' in sts else 'c'
        ev[(c0, e)] += 1
        # the (camera, id) that covered it before the streak
        prev = collections.Counter(v[1] for kk in range(max(0, k1 - 8), k1 - 2) for v in tick_view(kk, tid, c0).values() if v[1])
        pid = prev.most_common(1)[0][0] if prev else None
        alive = False
        if pid:
            alive_known += 1
            C = cams[pid[0]]
            for kk in streak:
                j = int(np.argmin(np.abs(C['yt'] - tracks[kk][0])))
                if any(d[0] == pid[1] for d in C['y'][j][1]): alive = True
            alive_loss += alive
        # what-if over the unclassified period
        for kk, cc in s[n:n + 50]:
            if cc: break
            vw = tick_view(kk, tid, c0); wtot += 1
            k3 = pid is not None and any(v[1] == pid for v in vw.values())
            k4 = not vw or all(v[0] == 'b' for v in vw.values())
            wi3 += k3; wi4 += k4; wi34 += k3 or k4
        rows_out.append(dict(w=w, run=runname, tid=tid, cls=c0, event=e, prev_id=str(pid), id_alive=alive))
# multi-camera
both = single = conflict = 0; idchg_same_cam = cam_switch = 0
last = {}
for k, (t, rows) in enumerate(tracks):
    for r in rows:
        if not r[1]:
            continue
        vw = tick_view(k, r[0], r[1])
        cov = [v[1] for v in vw.values() if v[1]]
        if len(cov) >= 2: both += 1
        elif len(cov) == 1: single += 1
        if cov:
            cur = cov[0]
            if r[0] in last and last[r[0]] != cur:
                if last[r[0]][0] == cur[0]: idchg_same_cam += 1
                else: cam_switch += 1
            last[r[0]] = cur
# conflicts: a same-position box of ANOTHER movable class in the other camera
print(f'== {w} {runname}: cameras {list(cams)}; class-loss events {sum(ev.values())}')
for cl in sorted({c for c, _ in ev}):
    print(f'  {cl:10s} a {ev[(cl, "a")]:3d}  b {ev[(cl, "b")]:3d}  c {ev[(cl, "c")]:3d}  nofov {ev[(cl, "nofov")]:3d}')
print(f'  losses with a known previous 2D (camera,id) {alive_known}, of which that id still published during the streak {alive_loss}')
print(f'  what-if over {wtot} unclassified ticks after the losses: kept by (3) {wi3}, by (4) {wi4}, by either {wi34}')
print(f'  classified-track ticks covered by 2 cameras {both}, by 1 {single}; covering (cam,id) changes: same camera new id '
      f'{idchg_same_cam}, camera switch {cam_switch}')
out = Path(__file__).parent / 'yolo_assoc_rows.csv'
pd.DataFrame(rows_out).to_csv(out, mode='a', header=not out.exists(), index=False)
