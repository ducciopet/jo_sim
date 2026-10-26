#!/usr/bin/env python3
"""
EagerMOT inputs for one benchmark sequence, one JSON per run (host python: reads the recorded bags).

  3D detections  CMDT (the UniMT detector, wheelchair weights, LiDAR + camera, 16 beams) as published before its own
                 tracker: bags/unimt_out/unimtdet<rep>_<seq>.bag, /unimt/detections (odom frame, score in color.g)
  2D detections  the YOLO person boxes of the same replay: bags/replay/r9<rep>_v2_<seq>,
                 /yolo_semantic_seg/front_camera/detections (the same detector SINS uses), nearest in time (<= 0.1 s)
  camera         intrinsics from camera_info, pose T_cam_odom = (T_odom_base(t) T_base_cam)^-1 from the shared
                 odometry (bags/replay/odom_<seq>) and the static TF of the benchmark bag

One EagerMOT frame per CMDT output (5 Hz: the detector runs on every other scan, as in its upstream node).
usage: export_inputs.py <seq e.g. 20260519_1950> <rep a|b|c>   -> bags/eagermot_out/in_<rep>_<seq>.json
"""

import json
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'eval'))
from harness.ros_io import typestore  # noqa: E402
from map_eval import T_of  # noqa: E402

CAM_FRAME = 'front_camera_color_optical_frame'


def static_chain(seq, child, root='base_link'):
    with AnyReader([REPO / 'bags' / 'bags_benchmark' / f'duccio_test_{seq}_0']) as r:
        tfs = {}
        for c, t, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
            for tr in r.deserialize(raw, c.msgtype).transforms:
                tfs[tr.child_frame_id] = (tr.header.frame_id, T_of(tr.transform.translation, tr.transform.rotation))
    T, f = np.eye(4), child
    while f != root:
        parent, Tp = tfs[f]
        T, f = Tp @ T, parent
    return T


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    seq, rep = sys.argv[1], sys.argv[2]
    out_dir = REPO / 'bags' / 'eagermot_out'
    out_dir.mkdir(exist_ok=True)
    # 3D: box_marker() layout in eval/baselines/unimt/infer_jo.py: points = corners of EDGES, corners 0-3 bottom
    dets3d = []
    with AnyReader([REPO / 'bags' / 'unimt_out' / f'unimtdet{rep}_{seq}.bag']) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic == '/unimt/detections']):
            m = r.deserialize(raw, c.msgtype)
            boxes = [k for k in m.markers if k.action == 0 and len(k.points) == 24]
            ts = stamp(m.markers[0].header) if m.markers else t / 1e9
            frame = []
            for k in boxes:
                p = np.array([[q.x, q.y, q.z] for q in k.points])
                c0, c1, c2 = p[0], p[1], p[3]  # (l/2, w/2), (-l/2, w/2), (-l/2, -w/2) at the bottom
                z0, z1 = p[:, 2].min(), p[:, 2].max()
                ctr = p[[0, 1, 3, 5]].mean(0)
                frame.append(dict(x=float(ctr[0]), y=float(ctr[1]), z=float((z0 + z1) / 2),
                                  l=float(np.linalg.norm(c0 - c1)), w=float(np.linalg.norm(c1 - c2)), h=float(z1 - z0),
                                  yaw=float(np.arctan2(c0[1] - c1[1], c0[0] - c1[0])), score=float(k.color.g),
                                  cls=k.ns.replace('unimt_det_', '')))
            dets3d.append((ts, frame))
    # 2D YOLO and camera_info from the SINS replay of the same repetition
    dets2d, K, size = [], None, None
    with AnyReader([REPO / 'bags' / 'replay' / f'r9{rep}_v2_{seq}'], default_typestore=typestore()) as r:
        topics = ('/yolo_semantic_seg/front_camera/detections', '/front_camera/camera/color/camera_info')
        for c, t, raw in r.messages([c for c in r.connections if c.topic in topics]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic.endswith('camera_info'):
                if K is None:
                    K, size = np.array(m.k).reshape(3, 3).tolist(), (int(m.height), int(m.width))
                continue
            boxes = []
            for d in m.detections:
                if not d.results or d.results[0].hypothesis.class_id != 'person':
                    continue
                cx, cy, sx, sy = d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y
                boxes.append(dict(x1=cx - sx / 2, y1=cy - sy / 2, x2=cx + sx / 2, y2=cy + sy / 2,
                                  score=float(d.results[0].hypothesis.score)))
            dets2d.append((stamp(m.header), boxes))
    t2d = np.array([d[0] for d in dets2d]) if dets2d else np.zeros(0)
    # camera pose
    T_base_cam = static_chain(seq, CAM_FRAME)
    odo = []
    with AnyReader([REPO / 'bags' / 'replay' / f'odom_{seq}']) as r:
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
    out = out_dir / f'in_{rep}_{seq}.json'
    out.write_text(json.dumps(dict(seq=seq, rep=rep, K=K, img_shape=size, frames=frames)))
    n3, n2 = sum(len(f['dets3d']) for f in frames), sum(len(f['dets2d']) for f in frames)
    print(f'wrote {out}: {len(frames)} frames, {n3} 3D and {n2} 2D detections')


if __name__ == '__main__':
    main()
