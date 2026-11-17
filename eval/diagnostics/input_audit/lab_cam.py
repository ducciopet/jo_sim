#!/usr/bin/env python3
"""Input audit (17/11), lab: is the camera chain SINS receives sensible on the Qualisys sequences?
For one SINS run (recorded /yolo_seg_track/front_camera/detections, /tf odom->base_link and the online calibration
velodyne->front_camera_refined, /tf_static) and its sensor bag (camera_info, scan stamps):
 - lag: YOLO detection stamp minus the nearest LiDAR scan stamp;
 - the mocap person (run_own_eval alignment: mocap -> odom, clock delta) projected at mid-body height (1.0 m) with the
   calibrated extrinsic: on the ticks it falls inside the image (> 0.5 m ahead), the fraction inside a YOLO 'person' box
   (10% margin) and the median pixel offset from the box centre.
usage: lab_cam.py <run dir> <sensor bag dir> <mocap bag dir> <alignment json> <seq>"""
import json, sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
sys.path.insert(0, 'eval')
from harness.ros_io import load_mocap
from harness.gt_mocap import compose, inverse

run, sb, mb, alf, seq = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3], sys.argv[4], sys.argv[5]
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
def T_of(t, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [t.x, t.y, t.z]; return T
odo, cal, stat, dets = [], [], {}, []
with AnyReader([run]) as r:
    for c, _, raw in r.messages([c for c in r.connections if c.topic in ('/tf', '/tf_static', '/yolo_seg_track/front_camera/detections')]):
        m = r.deserialize(raw, c.msgtype)
        if c.topic == '/yolo_seg_track/front_camera/detections':
            dets.append((st(m.header), [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y)
                                        for d in m.detections if d.results and d.results[0].hypothesis.class_id == 'person']))
            continue
        for tr in m.transforms:
            T = T_of(tr.transform.translation, tr.transform.rotation); k = (tr.header.frame_id, tr.child_frame_id)
            if k == ('odom', 'base_link'): odo.append((st(tr.header), T))
            elif k == ('velodyne', 'front_camera_refined'): cal.append((st(tr.header), T))
            elif c.topic == '/tf_static': stat[k] = T
K, size, scans = None, None, []
with AnyReader([sb]) as r:
    for c, t, raw in r.messages([c for c in r.connections if c.topic in ('/front_camera/camera/color/camera_info', '/velodyne_points')]):
        m = r.deserialize(raw, c.msgtype)
        if c.topic == '/velodyne_points': scans.append(st(m.header))
        elif K is None: K, size = np.array(m.k).reshape(3, 3), (m.height, m.width)
odo.sort(key=lambda x: x[0]); cal.sort(key=lambda x: x[0]); ot = np.array([o[0] for o in odo]); ct = np.array([c[0] for c in cal])
sc = np.array(sorted(scans)); T_bv = stat[('base_link', 'velodyne')]
al = json.load(open(alf)); Ti = inverse(np.array(al['T_mocap_odom'][seq])); dl = al['delta'][seq]
p = load_mocap(mb)['person']
lag = np.array([t - sc[np.argmin(np.abs(sc - t))] for t, _ in dets]) / 1e6
hit = tot = 0; offs = []; dus = []; dvs = []
for t, bx in dets:
    if not len(cal) or t < ct[0]: continue
    tm = t / 1e9 + dl; k = np.argmin(np.abs(p[:, 0] - tm))
    if abs(p[k, 0] - tm) > 0.05: continue
    ox, oy, _ = compose(Ti, np.array([p[k, 1], p[k, 2], 0.0]))
    T_oc = odo[np.argmin(np.abs(ot - t))][1] @ T_bv @ cal[max(0, np.searchsorted(ct, t) - 1)][1]
    Tc = np.linalg.inv(T_oc); q = Tc[:3, :3] @ [ox, oy, 1.0] + Tc[:3, 3]
    if q[2] < 0.5: continue
    u, v = (K @ q)[:2] / q[2]
    if not (0 <= u < size[1] and 0 <= v < size[0]): continue
    tot += 1
    ins = [b for b in bx if abs(u - b[0]) <= 0.6 * b[2] and abs(v - b[1]) <= 0.6 * b[3]]
    if ins:
        hit += 1; b = min(ins, key=lambda b: np.hypot(u - b[0], v - b[1]))
        offs.append(np.hypot(u - b[0], v - b[1])); dus.append((u - b[0]) / b[2]); dvs.append((v - b[1]) / b[3])
print(f'LABCAM {run.name}: YOLO msgs {len(dets)}, scans {len(sc)}, lag YOLO-scan median {np.median(lag):.0f} ms p90 '
      f'{np.percentile(np.abs(lag), 90):.0f} ms | person in image {tot} ticks, inside a YOLO person box {hit} '
      f'({100 * hit / max(1, tot):.0f}%), median offset {np.median(offs) if offs else float("nan"):.0f} px, '
      f'du/box width {np.median(dus) if dus else float("nan"):+.2f}, dv/box height {np.median(dvs) if dvs else float("nan"):+.2f} (1.0 m point)')
