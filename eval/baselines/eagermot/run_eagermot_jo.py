#!/usr/bin/env python3
"""
EagerMOT (Kim, Osep, Leal-Taixe, ICRA 2021; github.com/aleksandrkim61/EagerMOT, commit 2792a76, in
eval/baselines/eagermot_src) on one JO run, from the inputs of export_inputs.py.

The fusion (3D-2D instance fusion by 2D IoU) and the tracking (two-stage association, KF, track management) are the
upstream classes, unchanged. This file only plays the role of a dataset class, as upstream's nuScenes one does:
  - boxes in the world (odom) frame, converted to EagerMOT's internal KITTI-rect convention exactly as for nuScenes
    (Bbox3d.from_nu_box_convert; compensate_ego=False, tracking in the world frame);
  - one camera, 'front_camera', projection world -> camera optical -> pixels with the frame's T_cam_odom and K;
  - one class, pedestrian (CMDT's cyclist boxes are dropped, as the person is never on a bike here);
  - parameters: upstream's NUSCENES_BEST_PARAMS for pedestrian, unchanged.
Output: every track reported for MOT in a frame (upstream's `report_mot`: confirmed and updated this frame), as the
unified tracks CSV (odom frame). EagerMOT has no motion state: every track is labelled 'dynamic', as for UniMT.

usage (its venv): .venv/bin/python run_eagermot_jo.py <in json> <out csv>
"""

import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from nuscenes.utils.data_classes import Box
from pyquaternion import Quaternion

SRC = Path(__file__).resolve().parents[1] / 'eagermot_src'
sys.path.insert(0, str(SRC))
import types  # noqa: E402
sys.modules.setdefault('open3d', types.ModuleType('open3d'))  # only upstream's visualisation (utils_viz) imports it
from configs.params import build_params_dict, NUSCENES_BEST_PARAMS  # noqa: E402
from dataset_classes.mot_frame import MOTFrame  # noqa: E402
from dataset_classes.mot_sequence import MOTSequence  # noqa: E402
from dataset_classes.nuscenes.classes import id_from_name  # noqa: E402
from inputs.bbox import Bbox2d, Bbox3d  # noqa: E402
from inputs.detection_2d import Detection2D  # noqa: E402
from transform.nuscenes import kitti_to_nuscenes, convert_kitti_bbox_coordinates_to_nu  # noqa: E402
from transform.transformation import Transformation, cam_points_to_image_coordinates  # noqa: E402

CAM = 'front_camera'
PED = id_from_name('pedestrian')
k = PED - 1  # per-class parameter index
P = NUSCENES_BEST_PARAMS
PARAMS = build_params_dict(det_scores=(P['det_scores'][k],), seg_scores=(P['seg_scores'][k],),
                           max_ages=(P['max_ages'][k],), min_hits=(P['min_hits'][k],),
                           max_age_2d=(P['max_age_2d'][k],), fusion_iou_threshold=(P['fusion_iou_threshold'][k],),
                           first_matching_method=P['first_matching_method'],
                           thresholds_per_class={1: P['thresholds_per_class'][PED]},
                           leftover_matching_thres=P['leftover_matching_thres'], compensate_ego=False)
TRACK_CLASS = 1  # the single tracked class, index 1 in the reduced per-class parameter tuples


class TransformationJO(Transformation):
    def __init__(self, K):
        super().__init__()
        self.K = np.array(K)
        self.current = None  # the frame being tracked: upstream's single-camera (KITTI) path projects with frame_data=None

    def img_from_tracking(self, track_points, cam, frame_data):
        world = kitti_to_nuscenes(track_points)  # KITTI-rect convention -> world (odom), as upstream for nuScenes
        T = np.array((frame_data or self.current)['T_cam_odom'])
        pc = world @ T[:3, :3].T + T[:3, 3]
        pc = pc[pc[:, 2] > 0]
        return cam_points_to_image_coordinates(pc @ self.K.T).astype(int, copy=False)

    def rect_from_lidar(self, lidar_points, frame_data, only_forward=False):
        raise NotImplementedError  # no point clouds used


class FrameJO(MOTFrame):
    def __init__(self, sequence, name, data):
        super().__init__(sequence, name)
        self.data = data
        sequence.transformation.current = data

    def get_image_original(self, cam): raise NotImplementedError
    def get_image_original_uint8(self, cam): raise NotImplementedError
    def load_raw_pcd(self): raise NotImplementedError
    def transform_instances_to_world_frame(self): return None, None
    @property
    def bboxes_3d_world(self): return self.bboxes_3d
    @property
    def bboxes_3d_ego(self): raise NotImplementedError
    @property
    def points_world(self): raise NotImplementedError
    @property
    def center_world_point(self): raise NotImplementedError


class SequenceJO(MOTSequence):
    def __init__(self, inp):
        self.inp = inp
        self._transformation = TransformationJO(inp['K'])
        names = [str(i) for i in range(len(inp['frames']))]
        super().__init__('cmdt', 'yolo', '/nonexistent', inp['seq'], names)
        self.instance_fusion_bbox_dir = '/nonexistent'  # fused instances are never saved/loaded
        self.img_shape_per_cam = {CAM: tuple(inp['img_shape'])}
        self.mot.transformation = self._transformation
        self.mot.img_shape_per_cam = self.img_shape_per_cam

    @property
    def transformation(self): return self._transformation
    @property
    def cameras(self): return [CAM]
    @property
    def camera_default(self): return CAM
    @property
    def classes_to_track(self): return [TRACK_CLASS]

    def get_frame(self, name):
        return FrameJO(self, name, self.inp['frames'][int(name)])

    def load_detections_3d(self):
        out = {}
        for i, f in enumerate(self.inp['frames']):
            boxes = []
            for j, d in enumerate(f['dets3d']):
                if d['cls'] != 'pedestrian':
                    continue
                nu = Box([d['x'], d['y'], d['z']], [d['w'], d['l'], d['h']],
                         Quaternion(axis=[0, 0, 1], radians=d['yaw']), score=d['score'], name='pedestrian')
                b = Bbox3d.from_nu_box_convert(nu, instance_id=j)
                b.seg_class_id = TRACK_CLASS
                boxes.append(b)
            out[str(i)] = boxes
        return out

    def load_detections_2d(self):
        out = {}
        for i, f in enumerate(self.inp['frames']):
            dd = defaultdict(list)
            for d in f['dets2d']:
                dd[CAM].append(Detection2D(Bbox2d(d['x1'], d['y1'], d['x2'], d['y2']), CAM, d['score'], TRACK_CLASS))
            out[str(i)] = dd
        return out

    def load_ego_motion_transforms(self): pass
    def save_ego_motion_transforms_if_new(self): pass
    def report_mot_results(self, *a, **kw): pass
    def save_mot_results(self, *a, **kw): pass


def main():
    inp = json.loads(Path(sys.argv[1]).read_text())
    seq = SequenceJO(inp)
    seq.mot.set_track_manager_params(PARAMS)
    run_info = defaultdict(int)
    rows = []
    for name in seq.frame_names:
        frame = seq.get_frame(name)
        t_ns = int(round(frame.data['t'] * 1e9))
        n = 0
        for inst in frame.perform_tracking(PARAMS, run_info):
            if not inst.report_mot:
                continue
            center, wlh, rot = convert_kitti_bbox_coordinates_to_nu(inst.coordinates_3d)
            rows.append([t_ns, inst.track_id, *center, wlh[1], wlh[0], wlh[2], rot.yaw_pitch_roll[0],
                         '', '', '', 'dynamic', 'person', inst.bbox3d.confidence])
            n += 1
        if n == 0:
            rows.append([t_ns, -1] + [''] * 10 + ['', '', ''])  # empty-frame marker: the method ran
    with open(sys.argv[2], 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_ns', 'track_id', 'x', 'y', 'z', 'l', 'w', 'h', 'yaw', 'vx', 'vy', 'vz', 'state', 'class', 'score'])
        w.writerows(rows)
    print(f"{inp['seq']} {inp['rep']}: {len(seq.frame_names)} frames, {sum(1 for r in rows if r[1] != -1)} track rows, "
          f"{len({r[1] for r in rows if r[1] != -1})} ids; fused 3D+2D {run_info['instances_both']}, "
          f"3D only {run_info['instances_3d']}, 2D only {run_info['instances_2d']}")


if __name__ == '__main__':
    main()
