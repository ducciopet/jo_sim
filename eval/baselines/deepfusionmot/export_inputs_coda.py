#!/usr/bin/env python3
"""
Multi-class camera-LiDAR inputs for one CODa window, for the hybrid trackers that take ready-made 3D and 2D boxes
(DeepFusionMOT, and the next ones). Same sources as EagerMOT's CODa inputs (eval/baselines/eagermot/export_inputs_coda.py),
but every class instead of pedestrians only, because SINS targets dynamic scenes in general:

  3D detections  CenterPoint with the UniMT authors' CODa weights (bags/unimt_out/unimt_coda<w>_dets.csv, odom frame,
                 every scan): car, pedestrian, cyclist; score > 0.3 as for EagerMOT
  2D detections  the front-camera (ZED left) YOLO boxes of a SINS run on the window (the detector SINS uses), nearest
                 in time (<= 0.1 s): person -> pedestrian, car/truck/bus -> car, bicycle/motorcycle -> cyclist
  camera         K, image size and the per-frame T_cam_odom of EagerMOT's input json (bags/eagermot_out/in_coda<w>.json),
                 which were computed from the window's camera_info, static TF and odometry

usage: export_inputs_coda.py <window e.g. w1 or 10_w2> <SINS run e.g. LVE_v2_coda0_w1>
       -> bags/deepfusionmot_out/in_coda<w>.json
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'eval'))
from harness.ros_io import typestore  # noqa: E402

CLASS_2D = {'person': 'pedestrian', 'car': 'car', 'truck': 'car', 'bus': 'car', 'bicycle': 'cyclist',
            'motorcycle': 'cyclist'}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    w, run = sys.argv[1], sys.argv[2]
    w = w if '_' in w else f'0_{w}'
    B = REPO / 'bags'
    em = json.loads((B / 'eagermot_out' / f'in_coda{w}.json').read_text())
    d3 = pd.read_csv(B / 'unimt_out' / f'unimt_coda{w}_dets.csv')
    d3 = d3[(d3.id == -1) & d3.label.isin(['car', 'pedestrian', 'cyclist']) & (d3.score > 0.3)]
    by_t = {t * 1e-9: g for t, g in d3.groupby('t_ns')}
    dets2d = []
    with AnyReader([B / 'coda' / 'runs' / run], default_typestore=typestore()) as r:
        conns = [c for c in r.connections if c.topic == '/yolo_seg_track/front_camera/detections'] or \
            [c for c in r.connections if c.topic == '/yolo_semantic_seg/front_camera/detections']
        assert conns, 'no front-camera YOLO detections in ' + run
        for c, t, raw in r.messages(conns):
            m = r.deserialize(raw, c.msgtype)
            boxes = []
            for d in m.detections:
                cls = CLASS_2D.get(d.results[0].hypothesis.class_id) if d.results else None
                if cls is None:
                    continue
                cx, cy, sx, sy = d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y
                boxes.append(dict(x1=cx - sx / 2, y1=cy - sy / 2, x2=cx + sx / 2, y2=cy + sy / 2,
                                  score=float(d.results[0].hypothesis.score), cls=cls))
            dets2d.append((stamp(m.header), boxes))
    t2d = np.array([d[0] for d in dets2d])
    keys = np.array(sorted(by_t))
    frames = []
    for f in em['frames']:
        ts = f['t']
        g = by_t.get(keys[np.argmin(np.abs(keys - ts))]) if len(keys) and np.min(np.abs(keys - ts)) < 1e-6 else None
        dets3d = [] if g is None else [dict(x=r.x, y=r.y, z=r.z, l=r.l, w=r.w, h=r.h, yaw=r.yaw, score=r.score, cls=r.label)
                                       for r in g.itertuples()]
        boxes2d = []
        if len(t2d):
            i = int(np.argmin(np.abs(t2d - ts)))
            if abs(t2d[i] - ts) <= 0.1:
                boxes2d = dets2d[i][1]
        frames.append(dict(t=ts, dets3d=dets3d, dets2d=boxes2d, T_cam_odom=f['T_cam_odom']))
    out = B / 'deepfusionmot_out' / f'in_coda{w}.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(dict(seq=f'coda{w}', rep='a', K=em['K'], img_shape=em['img_shape'], frames=frames)))
    n3 = pd.Series([d['cls'] for f in frames for d in f['dets3d']]).value_counts().to_dict()
    n2 = pd.Series([d['cls'] for f in frames for d in f['dets2d']]).value_counts().to_dict()
    print(f'wrote {out}: {len(frames)} frames; 3D {n3}; 2D {n2}')


if __name__ == '__main__':
    main()
