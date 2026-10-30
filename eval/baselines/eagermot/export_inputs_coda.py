#!/usr/bin/env python3
"""
EagerMOT inputs for one CODa window (host python), same JSON as export_inputs.py for the lab:

  3D detections  CenterPoint with the UniMT authors' CODa weights, before tracking (eval/baselines/unimt/infer_coda.py):
                 bags/unimt_out/unimt_coda0_<w>_dets.csv (odom frame, every scan), score > 0.3 as the lab input (the
                 detections UniMT's node publishes, infer_jo.py)
  2D detections  the YOLO person boxes of a SINS run on the window (the same detector SINS uses):
                 bags/coda/runs/<run>/yolo_semantic_seg/front_camera/detections, nearest in time (<= 0.1 s)
  camera         the window's color camera (ZED left, cam3): intrinsics from camera_info, pose
                 T_cam_odom = (T_odom_base(t) T_base_velodyne T_velodyne_cam)^-1 from the odometry bag and the static TF

usage: export_inputs_coda.py <window e.g. w1> <SINS run e.g. C5_v2_coda0_w1>   -> bags/eagermot_out/in_coda0_<w>.json
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
from map_eval import T_of  # noqa: E402


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    w, run = sys.argv[1], sys.argv[2]
    B = REPO / 'bags'
    d3 = pd.read_csv(B / 'unimt_out' / f'unimt_coda0_{w}_dets.csv')
    dets3d = [(t * 1e-9, [dict(x=r.x, y=r.y, z=r.z, l=r.l, w=r.w, h=r.h, yaw=r.yaw, score=r.score, cls=r.label)
                          for r in g.itertuples() if r.id == -1 and isinstance(r.label, str) and r.score > 0.3])
              for t, g in d3.groupby('t_ns')]
    statics, K, size = {}, None, None
    with AnyReader([B / 'coda' / f'coda0_{w}'], default_typestore=typestore()) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic in ('/tf_static', '/front_camera/camera/color/camera_info')]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/tf_static':
                for tr in m.transforms:
                    statics[(tr.header.frame_id, tr.child_frame_id)] = T_of(tr.transform.translation, tr.transform.rotation)
            elif K is None:
                K, size = np.array(m.k).reshape(3, 3).tolist(), (int(m.height), int(m.width))
    T_base_cam = statics[('base_link', 'velodyne')] @ statics[('velodyne', 'front_camera_depth_optical_frame')] @ \
        statics[('front_camera_depth_optical_frame', 'front_camera_color_optical_frame')]
    dets2d = []
    with AnyReader([B / 'coda' / 'runs' / run], default_typestore=typestore()) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic == '/yolo_semantic_seg/front_camera/detections']):
            m = r.deserialize(raw, c.msgtype)
            boxes = []
            for d in m.detections:
                if not d.results or d.results[0].hypothesis.class_id != 'person':
                    continue
                cx, cy, sx, sy = d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y
                boxes.append(dict(x1=cx - sx / 2, y1=cy - sy / 2, x2=cx + sx / 2, y2=cy + sy / 2,
                                  score=float(d.results[0].hypothesis.score)))
            dets2d.append((stamp(m.header), boxes))
    t2d = np.array([d[0] for d in dets2d]) if dets2d else np.zeros(0)
    odo = []
    with AnyReader([B / 'coda' / f'coda0_{w}_odom']) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            odo.append((stamp(m.header), T_of(m.pose.pose.position, m.pose.pose.orientation)))
    to = np.array([o[0] for o in odo])
    frames = []
    for ts, frame in dets3d:
        j = int(np.argmin(np.abs(to - ts)))
        T_cam_odom = np.linalg.inv(odo[j][1] @ T_base_cam)
        boxes2d = []
        if len(t2d):
            i = int(np.argmin(np.abs(t2d - ts)))
            if abs(t2d[i] - ts) <= 0.1:
                boxes2d = dets2d[i][1]
        frames.append(dict(t=ts, dets3d=frame, dets2d=boxes2d, T_cam_odom=T_cam_odom.tolist()))
    out = B / 'eagermot_out' / f'in_coda0_{w}.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(dict(seq=f'coda0_{w}', rep='a', K=K, img_shape=size, frames=frames)))
    print(f'wrote {out}: {len(frames)} frames, {sum(len(f["dets3d"]) for f in frames)} 3D and '
          f'{sum(len(f["dets2d"]) for f in frames)} 2D detections')


if __name__ == '__main__':
    main()
