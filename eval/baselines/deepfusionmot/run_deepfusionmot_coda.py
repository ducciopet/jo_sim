#!/usr/bin/env python3
"""
DeepFusionMOT (Wang et al., RA-L / IROS 2022; github.com/wangxiyang2022/DeepFusionMOT, commit 8f4e2d7, in
eval/baselines/deepfusionmot_src) on one CODa window, from export_inputs_coda.py's json.

Upstream, unchanged: the 3D-2D detection fusion (datasets/data_fusion.py), the four-level association, the 3D and 2D
Kalman filters and the track management (tracking/), and the per-class hyperparameters of config/kitti.yaml.
This file plays the role of upstream's main.py (which reads KITTI files):
  - boxes are given in a pseudo camera frame built from the world (odom) frame, KITTI convention:
    x_c = X, y_c = -Z_bottom (y down, at the box bottom), z_c = Y, rot_y = -yaw (a proper rotation of the odom frame).
    Tracking therefore happens in the world frame and upstream's ego-motion compensation (KITTI oxts) is skipped,
    exactly as EagerMOT runs with compensate_ego=False on these data;
  - the "3D box projected in the image" that the fusion needs (KITTI's detection files carry it) is computed by
    projecting the 8 corners with the frame's T_cam_odom and K, clipped to the image; a box behind or outside the
    camera (never the case on KITTI, where every detection is in the image) gets a 1-pixel box off the image;
  - classes: Car and Pedestrian with upstream's parameters; Cyclist, which upstream does not configure, with the
    Pedestrian parameters (declared). One tracker per class, as upstream (cat_list).
  - output: every track upstream reports (confirmed, or any in the first min_frames frames), as the unified tracks
    CSV in the odom frame. DeepFusionMOT has no motion state: every track is labelled 'dynamic', as for EagerMOT.

usage (EagerMOT's venv + pyyaml, easydict): run_deepfusionmot_coda.py <in json> <out csv>
"""
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[1] / 'deepfusionmot_src'
sys.path.insert(0, str(SRC))
from datasets.coordinate_transformation import convert_x1y1x2y2_to_tlwh  # noqa: E402
from datasets.data_fusion import data_fusion  # noqa: E402
from tracking.DeepFusionMOT import DeepFusionMOT  # noqa: E402
from tracking.detection import Detection_3D_Fusion, Detection_3D_only, Detection_2D  # noqa: E402
from utils.config import Config  # noqa: E402

CATS = {'Car': ('car', 'Car'), 'Pedestrian': ('pedestrian', 'Pedestrian'), 'Cyclist': ('cyclist', 'Pedestrian')}
OUT_CLASS = {'Car': 'car', 'Pedestrian': 'person', 'Cyclist': 'bicycle'}
ID_BASE = {'Car': 0, 'Pedestrian': 1000000, 'Cyclist': 2000000}   # one id space for the three per-class trackers


class DeepFusionMOTWorld(DeepFusionMOT):
    """Upstream's update() without the KITTI oxts/calib ego-motion compensation (tracking in the world frame)."""

    def update(self, dets_3d_fusion, dets_2d_only, dets_3d_only):
        f_cam = np.array(dets_3d_fusion['dets_3d_fusion'])
        f_info = np.array(dets_3d_fusion['dets_3d_fusion_info'])
        o_cam = np.array(dets_3d_only['dets_3d_only'])
        o_info = np.array(dets_3d_only['dets_3d_only_info'])
        if len(f_cam):
            f_cam = f_cam[:, self.reorder]
        if len(o_cam):
            o_cam = o_cam[:, self.reorder]
        f = [Detection_3D_Fusion(d, f_info[i]) for i, d in enumerate(f_cam)]
        o = [Detection_3D_only(d, o_info[i]) for i, d in enumerate(o_cam)]
        d2 = [Detection_2D(d) for d in dets_2d_only]
        self.tracker.predict_2d()
        self.tracker.predict_3d()
        self.tracker.update(f, o, d2)
        self.frame_count += 1
        out = []
        for track in self.tracker.tracks_3d:
            if track.is_confirmed() or self.frame_count <= self.min_frames:
                out.append((track.track_id_3d, np.array(track.pose[self.reorder_back]), track.additional_info))
        return out


def corners_world(d):
    c, s = np.cos(d['yaw']), np.sin(d['yaw'])
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                lx, ly = sx * d['l'] / 2, sy * d['w'] / 2
                pts.append([d['x'] + c * lx - s * ly, d['y'] + s * lx + c * ly, d['z'] + sz * d['h'] / 2])
    return np.array(pts)


def box2d_of(d, T, K, shape):
    pc = corners_world(d) @ T[:3, :3].T + T[:3, 3]
    pc = pc[pc[:, 2] > 0.1]
    off = [-2.0, -2.0, -1.0, -1.0]   # behind or outside the camera: a 1-pixel box off the image (never overlaps, no 0 area)
    if len(pc) == 0:
        return off
    uv = pc @ K.T
    uv = uv[:, :2] / uv[:, 2:3]
    h, w = shape
    x1, y1 = np.clip(uv.min(0), 0, [w - 1, h - 1])
    x2, y2 = np.clip(uv.max(0), 0, [w - 1, h - 1])
    if x2 - x1 < 1 or y2 - y1 < 1:
        return off
    return [float(x1), float(y1), float(x2), float(y2)]


def main():
    inp = json.loads(Path(sys.argv[1]).read_text())
    cfg, _ = Config(str(SRC / 'config' / 'kitti.yaml'))
    K = np.array(inp['K'])
    shape = tuple(inp['img_shape'])
    rows, n_tracks, t_total = [], {}, 0.0
    for cat, (cls3d, params) in CATS.items():
        cfg[cat] = cfg[params]
        trk = DeepFusionMOTWorld(cfg, cat)
        ids = set()
        for f in inp['frames']:
            T = np.array(f['T_cam_odom'])
            d3 = [d for d in f['dets3d'] if d['cls'] == cls3d]
            cam = np.array([[d['h'], d['w'], d['l'], d['x'], -(d['z'] - d['h'] / 2), d['y'], -d['yaw']] for d in d3]).reshape(-1, 7)
            # additional_info as upstream's KITTI files: [alpha, type, x1, y1, x2, y2, score]
            info = np.array([[-10.0, 0.0, *box2d_of(d, T, K, shape), d['score']] for d in d3]).reshape(-1, 7)
            d2 = np.array([[b['x1'], b['y1'], b['x2'], b['y2']] for b in f['dets2d'] if b['cls'] == cls3d]).reshape(-1, 4)
            t0 = time.time()
            fus, only3d, only2d = data_fusion(cam, d2, info[:, 2:6], info)
            only2d_tlwh = np.array([convert_x1y1x2y2_to_tlwh(b) for b in only2d])
            out = trk.update(fus, only2d_tlwh, only3d)
            t_total += time.time() - t0
            for tid, b, _info in out:
                h, w, l, x, y, z, ry = b
                rows.append([int(round(f['t'] * 1e9)), ID_BASE[cat] + int(tid), x, z, -y + h / 2, l, w, h, -ry,
                             '', '', '', 'dynamic', OUT_CLASS[cat], float(_info[6])])
                ids.add(int(tid))
        n_tracks[cat] = len(ids)
    have = {r[0] for r in rows}
    for f in inp['frames']:
        t_ns = int(round(f['t'] * 1e9))
        if t_ns not in have:
            rows.append([t_ns, -1] + [''] * 13)   # empty-frame marker: the method ran
    rows.sort(key=lambda r: r[0])
    with open(sys.argv[2], 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_ns', 'track_id', 'x', 'y', 'z', 'l', 'w', 'h', 'yaw', 'vx', 'vy', 'vz', 'state', 'class', 'score'])
        w.writerows(rows)
    nf = len(inp['frames'])
    print(f"{inp['seq']}: {nf} frames, {sum(1 for r in rows if r[1] != -1)} track rows, ids per class {n_tracks}, "
          f"{1000 * t_total / max(nf, 1):.2f} ms/frame (fusion + 3 trackers)")


if __name__ == '__main__':
    main()
