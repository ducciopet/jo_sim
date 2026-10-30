#! /usr/bin/env python3
# UniMT's tracking stack (SimpleTrack) on a CODa window, offline, with the authors' CODa CenterPoint detector. Copy of
# upstream perception_test/scripts/infer.py (github.com/TossherO/ros_packages, the authors' own CODa script): score
# threshold 0.1, SimpleTrack with coda_configs/diou.yaml, classes car/pedestrian/cyclist — unchanged.
# JO: detector. The CMDT CODa checkpoint that infer.py loads is not published; of the authors' CODa checkpoints
# (ckpts/*_coda.pth) unimt_coda and cmt_coda need code they did not release (my_projects.UniMT / .CMT), centerpoint_coda
# is a standard mmdet3d CenterPoint: its embedded config (centerpoint_coda_cfg.py, custom imports removed) with the
# stock CenterHead loads every weight (no missing/unexpected key). LiDAR only, as CenterPoint.
# Only the I/O is adapted, marked # JO:
#  - frames from a manifest (make_coda_manifest.py) instead of the authors' info pkl: same fields (OS1 file, cam0/cam1
#    rectified images with cam2img/lidar2cam, ego2global, timestamp), ego2global in the window's odom frame
#  - trajectory prediction skipped (not evaluated)
#  - output: CSV of alive tracks (as infer_jo.py: only 'alive' tracks) and of the detections before tracking
#    (EagerMOT input), boxes in odom: t_ns, id, label, x, y, z (centre), l, w, h, yaw, score
# Run from the workspace root (/ws) as upstream.
import sys
import os.path as osp
sys.path.append(osp.abspath('./'))
sys.path.append(osp.abspath('./src/tracking/scripts'))
import csv
import yaml
import numpy as np
import torch
from mmengine.config import Config
from mmdet3d.registry import MODELS, TRANSFORMS
from mmdet3d.utils import register_all_modules
from mmengine.runner import load_checkpoint
from mmdet3d.structures import LiDARInstance3DBoxes
from mmdet3d.structures import Box3DMode
from mot_3d.mot import MOTModel
from mot_3d.data_protos import BBox
from mot_3d.frame_data import FrameData


def main():
    manifest, out_tracks, out_dets = sys.argv[1:4]  # JO
    m = np.load(manifest)

    cfg = Config.fromfile('/jo/centerpoint_coda_cfg.py')  # JO
    cfg.model.pts_bbox_head.type = 'CenterHead'  # JO: stock head, see above
    checkpoint = '/jo/ckpts/centerpoint_coda.pth'  # JO
    register_all_modules()
    detect_model = MODELS.build(cfg.model)
    pipeline = [TRANSFORMS.build(t) for t in cfg.test_dataloader.dataset.pipeline]
    load_checkpoint(detect_model, checkpoint, map_location='cpu')
    detect_model.cuda().eval()

    track_config = yaml.load(open('./src/tracking/scripts/configs/coda_configs/diou.yaml', 'r'), Loader=yaml.Loader)
    class_labels = [0, 1, 2]
    class_names = ['car', 'pedestrian', 'cyclist']
    trackers = [MOTModel(track_config, class_names[label]) for label in class_labels]
    print('finish init', flush=True)

    ft = open(out_tracks, 'w', newline='')
    fd = open(out_dets, 'w', newline='')
    wt, wd = csv.writer(ft), csv.writer(fd)
    head = ['t_ns', 'id', 'label', 'x', 'y', 'z', 'l', 'w', 'h', 'yaw', 'score']
    wt.writerow(head)
    wd.writerow(head)
    for k in range(len(m['t_ns'])):
        t_ns = int(m['t_ns'][k])
        timestamp = t_ns * 1e-9
        ego2global = m['ego2global'][k]
        # JO: the info-pkl entry upstream builds its input from
        input = {'lidar_points': {'lidar_path': str(m['lidar'][k]), 'num_pts_feats': 4},
                 'timestamp': timestamp, 'ego2global': ego2global,
                 'box_type_3d': LiDARInstance3DBoxes, 'box_mode_3d': Box3DMode.LIDAR}
        with torch.no_grad():
            for transform in pipeline:
                input = transform(input)
            input['data_samples'] = [input['data_samples']]
            input['inputs']['points'] = [input['inputs']['points']]
            output = detect_model.data_preprocessor(input, training=False)
            output = detect_model(**output, mode='predict')
            bboxes_3d = output[0].get('pred_instances_3d')['bboxes_3d']
            labels_3d = output[0].get('pred_instances_3d')['labels_3d']
            scores_3d = output[0].get('pred_instances_3d')['scores_3d']
            bboxes_3d = bboxes_3d[scores_3d > 0.1].tensor.cpu().numpy()
            labels_3d = labels_3d[scores_3d > 0.1].cpu().numpy()
            scores_3d = scores_3d[scores_3d > 0.1].cpu().numpy()
        n = 0
        for i, label in enumerate(class_labels):
            mask = labels_3d == label
            dets = np.concatenate([bboxes_3d[mask], scores_3d[mask][:, None]], axis=1).tolist()
            frame_data = FrameData(dets=dets, ego=ego2global, pc=None, det_types=labels_3d[mask], time_stamp=float(timestamp))
            frame_data.dets = [BBox.bbox2world(ego2global, det) for det in frame_data.dets]
            for d in frame_data.dets:  # JO: world-frame detections (BBox x,y,z,l,w,h,o,s; z = bottom, as box_marker in infer_jo.py)
                a = BBox.bbox2array(d)
                wd.writerow([t_ns, -1, class_names[label], a[0], a[1], a[2] + a[5] / 2, a[3], a[4], a[5], a[6], a[7]])
            results = trackers[i].frame_mot(frame_data)
            for trk in results:
                if trk[2].split('_')[0] == 'alive':  # JO: as infer_jo.py, only alive tracks
                    a = BBox.bbox2array(trk[0])
                    wt.writerow([t_ns, trk[1] + 100000 * i, class_names[label], a[0], a[1], a[2] + a[5] / 2,
                                 a[3], a[4], a[5], a[6], a[7] if len(a) > 7 else 1.0])
                    n += 1
        if n == 0:  # JO: an empty frame still marks that the method ran
            wt.writerow([t_ns, -1, '', '', '', '', '', '', '', '', ''])
        if k % 50 == 0:
            print(k, len(m['t_ns']), n, flush=True)
    ft.close()
    fd.close()
    print('done', flush=True)


if __name__ == '__main__':
    main()
