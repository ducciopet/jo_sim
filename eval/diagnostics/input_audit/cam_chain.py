#!/usr/bin/env python3
"""Input audit (17/11): is the camera chain SINS receives sensible? For one CODa tuning window and a SINS run on it:
 - rates: LiDAR scans in the window bag vs tracker outputs vs YOLO detection messages per camera;
 - time: YOLO detection stamp minus nearest LiDAR scan stamp;
 - geometry: CODa-annotated pedestrians (>= 20 LiDAR points, in the image) projected with the bag's own static TF +
   camera_info + odometry at the detection stamp: fraction inside a YOLO 'person' box (10% margin), and the median
   pixel offset from the box centre. A wrong extrinsic or a time shift shows up as a low hit rate / a biased offset.
usage: cam_chain.py <window e.g. 10_w1> <run e.g. LVE_v2_coda10_w1> [camera, default front_camera]"""
import json, sys
from pathlib import Path
import numpy as np, pandas as pd
from rosbags.highlevel import AnyReader

def T_of(t, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [t.x, t.y, t.z]; return T
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec

w, run = sys.argv[1], sys.argv[2]; cam = sys.argv[3] if len(sys.argv) > 3 else 'front_camera'
B = Path('bags/coda'); win = B / f'coda{w}'
bags = [win] + ([B / f'coda{w}_stereo'] if cam == 'stereo_camera' else [])
statics, K, size, scans = {}, None, None, []
with AnyReader(bags) as r:
    for c, t, raw in r.messages([c for c in r.connections if c.topic in ('/tf_static', f'/{cam}/camera/color/camera_info', '/velodyne_points')]):
        if c.topic == '/velodyne_points':
            scans.append(t); continue
        m = r.deserialize(raw, c.msgtype)
        if c.topic == '/tf_static':
            for tr in m.transforms: statics[(tr.header.frame_id, tr.child_frame_id)] = T_of(tr.transform.translation, tr.transform.rotation)
        elif K is None: K, size = np.array(m.k).reshape(3, 3), (m.height, m.width)
if ('velodyne', f'{cam}_depth_optical_frame') not in statics:   # NAS window bags carry an empty /tf_static: the run's
    with AnyReader([B / 'runs' / run]) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
            for tr in r.deserialize(raw, c.msgtype).transforms:
                statics[(tr.header.frame_id, tr.child_frame_id)] = T_of(tr.transform.translation, tr.transform.rotation)
T_bv = np.array(json.load(open(f'{win}_gt.json'))['T_base_velodyne'])
T_v_cam = statics[('velodyne', f'{cam}_depth_optical_frame')] @ statics[(f'{cam}_depth_optical_frame', f'{cam}_color_optical_frame')]
odo = []
with AnyReader([Path(f'{win}_odom')]) as r:
    for c, t, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
        m = r.deserialize(raw, c.msgtype); odo.append((st(m.header), T_of(m.pose.pose.position, m.pose.pose.orientation)))
ot = np.array([o[0] for o in odo])
dets, ntr = [], 0
with AnyReader([B / 'runs' / run]) as r:
    for c, t, raw in r.messages([c for c in r.connections if c.topic in (f'/yolo_seg_track/{cam}/detections', '/sins/tracker/tracks')]):
        if c.topic == '/sins/tracker/tracks': ntr += 1; continue
        m = r.deserialize(raw, c.msgtype)
        bx = [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y) for d in m.detections
              if d.results and d.results[0].hypothesis.class_id == 'person']
        dets.append((st(m.header), bx))
sc = np.array(sorted(scans))
lag = np.array([t - sc[np.argmin(np.abs(sc - t))] for t, _ in dets]) / 1e6
g = pd.read_csv(f'{win}_gt.csv', keep_default_na=False); g = g[(g.track_id >= 0) & (g['class'] == 'Pedestrian')]
for col in ('x', 'y', 'z', 'npts'): g[col] = g[col].astype(float)
g = g[g.npts >= 20]; gt_by_t = {t: d for t, d in g.groupby('t_ns')}; gts = np.array(sorted(gt_by_t))
hit = tot = 0; offs = []
for t, bx in dets:
    k = np.argmin(np.abs(gts - t))
    if abs(gts[k] - t) > 60_000_000: continue
    j = np.argmin(np.abs(ot - t)); T_cam_odom = np.linalg.inv(odo[j][1] @ T_bv @ T_v_cam)
    for o in gt_by_t[gts[k]].itertuples():
        p = T_cam_odom[:3, :3] @ [o.x, o.y, o.z] + T_cam_odom[:3, 3]
        if p[2] < 0.5: continue
        u, v = (K @ p)[:2] / p[2]
        if not (0 <= u < size[1] and 0 <= v < size[0]): continue
        tot += 1
        best = min(bx, key=lambda b: (b[0] - u) ** 2 + (b[1] - v) ** 2) if bx else None
        if best and abs(u - best[0]) <= 0.6 * best[2] and abs(v - best[1]) <= 0.6 * best[3]:
            hit += 1; offs.append((u - best[0], v - best[1]))
offs = np.array(offs) if offs else np.zeros((0, 2))
print(f'{w} {cam}: scans {len(sc)}, tracker msgs {ntr}, YOLO msgs {len(dets)} ({100*len(dets)/max(1,len(sc)):.0f}% of scans); '
      f'YOLO-scan lag ms median {np.median(lag):.0f} p90 {np.percentile(np.abs(lag),90):.0f}; '
      f'GT pedestrians in image {tot}, inside a YOLO person box {100*hit/max(1,tot):.0f}%, '
      f'median offset from box centre du {np.median(offs[:,0]) if len(offs) else float("nan"):.0f} dv {np.median(offs[:,1]) if len(offs) else float("nan"):.0f} px')
