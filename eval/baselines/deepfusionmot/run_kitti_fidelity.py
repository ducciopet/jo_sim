#!/usr/bin/env python3
"""
Fidelity check of DeepFusionMOT on its own dataset: upstream main.py's tracking() on the KITTI tracking training split
(Car, the authors' PointRCNN 3D + RRC 2D detections shipped in the repository, config/kitti.yaml unchanged), then
upstream's bundled TrackEval (HOTA, CLEAR, Identity) on its own seqmap.

The only difference from main.py: the number of frames of a sequence is read from its oxts file instead of listing
image_02 (the images are not used by the tracker or by save_results; they are only opened and drawn on, and the
drawing is commented out upstream). So KITTI's 15 GB of images are not needed.

usage (EagerMOT's venv + pyyaml, easydict), from eval/baselines/deepfusionmot_src:
  ../eagermot/.venv/bin/python ../deepfusionmot/run_kitti_fidelity.py <KITTI_tracking dir>
"""
import os
import shutil
import sys
import time

import numpy as np

sys.path.insert(0, os.getcwd())
import types  # noqa: E402
for _m, _f in (('utils.visualization.visualization_2d', 'show_image_with_boxes_2d'),
               ('utils.visualization.visualization_3d', 'show_image_with_boxes_3d')):
    # upstream's drawing helpers (they import torch); save_results imports them but their calls are commented out
    sys.modules[_m] = types.ModuleType(_m)
    setattr(sys.modules[_m], _f, None)
from datasets.coordinate_transformation import convert_x1y1x2y2_to_tlwh  # noqa: E402
from datasets.data_fusion import data_fusion  # noqa: E402
from tracking.DeepFusionMOT import DeepFusionMOT  # noqa: E402
from utils.combine_trk_cat import combine_category_result  # noqa: E402
from utils.config import Config  # noqa: E402
from utils.save_results import save_results  # noqa: E402


def main():
    shutil.rmtree('results', ignore_errors=True)
    cfg, _ = Config('./config/kitti.yaml')
    cfg.dataset_path = sys.argv[1]
    total_time, total_frames = 0.0, 0
    for category in cfg.cat_list:
        for seq_id in cfg.tracking_seqs:
            tracker = DeepFusionMOT(cfg, category)
            seq_name = str(seq_id).zfill(4)
            dets_3d = np.loadtxt(os.path.join(cfg.dets_path_3d, cfg.detector_3d, cfg.spilt, category, seq_name + '.txt'), delimiter=',')
            dets_2d = np.loadtxt(os.path.join(cfg.dets_path_2d, cfg.detector_2d, cfg.spilt, category, seq_name + '.txt'), delimiter=',')
            with open(os.path.join(cfg.dataset_path, cfg.spilt, 'oxts', seq_name + '.txt')) as f:
                max_frame = sum(1 for _ in f)
            for frame in range(max_frame):
                sel = dets_3d[:, 0] == frame
                dets_3d_camera = dets_3d[sel, 7:14]
                additional_info = np.concatenate((dets_3d[sel, -1].reshape((-1, 1)), dets_3d[sel, 1:7]), axis=1)
                dets_3dto2d_image = dets_3d[sel, 2:6]
                dets_2d_frame = dets_2d[dets_2d[:, 0] == frame, 1:5] if len(dets_2d) else []
                fus, only3d, only2d = data_fusion(dets_3d_camera, dets_2d_frame, dets_3dto2d_image, additional_info)
                only2d_tlwh = np.array([convert_x1y1x2y2_to_tlwh(i) for i in only2d])
                t0 = time.time()
                trackers = tracker.update(fus, only2d_tlwh, only3d, cfg, frame, seq_id)
                total_time += time.time() - t0
                total_frames += 1
                save_results(trackers, cfg, seq_name, frame, category, None)
    print(f'frames {total_frames}, FPS {total_frames / total_time:.0f}')
    combine_category_result(cfg)


if __name__ == '__main__':
    main()
