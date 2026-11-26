#!/usr/bin/env python3
"""SINS output (odom frame 3D boxes) -> KITTI tracking result file (image_02 2D boxes), for the official TrackEval HOTA
(Kitti2DBox) next to the baselines evaluated the same way in Phase A (FINAL_RUN_SPEC 3.1, paper Sec. 5).

Per output frame (KITTI frame index = round((t - t0) * 10), t0 = the converter's first stamp): every published object
(D or PD) with a class mapped to KITTI's (person -> Pedestrian, car/truck/bus -> Car; other or no class -> dropped) is
projected into camera 2 with the per-frame T_cam_odom and K of <seq>_cam.json (kitti_to_bag.py): the 8 box corners in
front of the camera, 2D box = their bounds clipped to the image; boxes with no corner in front or a clipped area < 25
px^2 are dropped. Track ids are SINS's own. Score = 1. Line format (KITTI tracking):
frame id type truncated occluded alpha left top right bottom h w l x y z ry score (x y z ry in camera-2 coordinates,
filled for completeness; the 2D evaluation uses only the box).

usage: sins_to_kitti.py --run <SINS run bag> --cam <seq>_cam.json --gt-json <seq>_gt.json --out <seq>.txt
Pure conversion of existing outputs.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

E = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(E))
from harness.ros_io import load_obstacles, load_track_classes   # noqa: E402
from harness.tracks import objects                              # noqa: E402

CLS = {'person': 'Pedestrian', 'car': 'Car', 'truck': 'Car', 'bus': 'Car'}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', required=True)
    ap.add_argument('--cam', required=True)
    ap.add_argument('--gt-json', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    cam = json.load(open(a.cam))
    K = np.array(cam['K'])
    H, W = cam['img_shape']
    ct = np.array([f['t'] for f in cam['frames']])
    T = np.array([f['T_cam_odom'] for f in cam['frames']])
    t0 = json.load(open(a.gt_json))['t0_ns'] / 1e9
    pred, _, _ = load_obstacles(a.run)
    pred = objects(pred)
    pred = pred[pred['state'].isin(['dynamic', 'potentially_dynamic'])]
    classes = load_track_classes(a.run)
    lines = []
    for r in pred.itertuples():
        c = classes.get((int(r.t_ns), int(r.track_id)), '')
        kt = CLS.get(str(c).lower())
        if kt is None:
            continue
        t = r.t_ns / 1e9
        k = int(np.argmin(np.abs(ct - t)))
        if abs(ct[k] - t) > 0.06:
            continue
        frame = int(round((t - t0) * 10))
        cs, sn = np.cos(r.yaw), np.sin(r.yaw)
        dx = np.array([1, 1, -1, -1, 1, 1, -1, -1]) * r.l / 2
        dy = np.array([1, -1, -1, 1, 1, -1, -1, 1]) * r.w / 2
        dz = np.array([-1, -1, -1, -1, 1, 1, 1, 1]) * r.h / 2
        corners = np.stack([r.x + dx * cs - dy * sn, r.y + dx * sn + dy * cs, r.z + dz], axis=1)
        pc = corners @ T[k][:3, :3].T + T[k][:3, 3]
        front = pc[:, 2] > 0.1
        if not front.any():
            continue
        uv = (pc[front] @ K.T)[:, :2] / pc[front][:, 2:3]
        l, tp = np.clip(uv.min(0), 0, [W - 1, H - 1])
        rr, b = np.clip(uv.max(0), 0, [W - 1, H - 1])
        if (rr - l) * (b - tp) < 25:
            continue
        ctr = np.array([r.x, r.y, r.z - r.h / 2]) @ T[k][:3, :3].T + T[k][:3, 3]     # bottom centre, camera 2
        hd = np.array([cs, sn, 0.0]) @ T[k][:3, :3].T
        ry = float(np.arctan2(-hd[2], hd[0]))
        alpha = float(ry - np.arctan2(ctr[0], ctr[2]))
        lines.append((frame, int(r.track_id), f'{frame} {int(r.track_id)} {kt} 0 0 {alpha:.4f} {l:.2f} {tp:.2f} {rr:.2f} {b:.2f} '
                                              f'{r.h:.3f} {r.w:.3f} {r.l:.3f} {ctr[0]:.3f} {ctr[1]:.3f} {ctr[2]:.3f} {ry:.4f} 1.0'))
    lines.sort()
    Path(a.out).write_text(''.join(x[2] + '\n' for x in lines))
    print(f'{len(lines)} boxes written to {a.out}')


if __name__ == '__main__':
    main()
