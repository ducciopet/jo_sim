#!/usr/bin/env python3
"""
EagerMOT multi-class on one CODa window: car, pedestrian, bicycle, from the same input json as DeepFusionMOT
(eval/baselines/deepfusionmot/export_inputs_coda.py: CenterPoint CODa 3D boxes score > 0.3 + front-camera YOLO boxes
of a SINS run, each with its class), so the two hybrid trackers see identical detections.

Everything as run_eagermot_jo.py (upstream fusion and tracking unchanged, world frame, compensate_ego=False), except:
  - classes: nuScenes ids car=1, pedestrian=2, bicycle=3 (CenterPoint's 'cyclist' = nuScenes bicycle, the
    rider-on-bike box); YOLO person -> pedestrian, car/truck/bus -> car, bicycle/motorcycle -> bicycle;
  - parameters: upstream's NUSCENES_BEST_PARAMS for all classes, unchanged (indexed by class id, as upstream);
  - output class names as DeepFusionMOT's adapter: car, person, bicycle.

usage (its venv): .venv/bin/python run_eagermot_coda_mc.py <in json> <out csv>
"""
import csv
import json
import sys
import time
from collections import defaultdict
from pathlib import Path

from nuscenes.utils.data_classes import Box
from pyquaternion import Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_eagermot_jo as jo  # noqa: E402  (upstream imports, camera/frame/sequence adapters)
from configs.params import NUSCENES_BEST_PARAMS as PARAMS  # noqa: E402
from inputs.bbox import Bbox2d, Bbox3d  # noqa: E402
from inputs.detection_2d import Detection2D  # noqa: E402
from transform.nuscenes import convert_kitti_bbox_coordinates_to_nu  # noqa: E402

CLS_ID = {'car': 1, 'pedestrian': 2, 'cyclist': 3}          # input class -> nuScenes tracking class id
OUT_CLASS = {1: 'car', 2: 'person', 3: 'bicycle'}
OUT_NU = {1: 'car', 2: 'pedestrian', 3: 'bicycle'}             # nuScenes names (Bbox3d.from_nu_box_convert)


class SequenceMC(jo.SequenceJO):
    @property
    def classes_to_track(self): return sorted(OUT_CLASS)

    def load_detections_3d(self):
        out = {}
        for i, f in enumerate(self.inp['frames']):
            boxes = []
            for j, d in enumerate(f['dets3d']):
                cid = CLS_ID.get(d['cls'])
                if cid is None:
                    continue
                nu = Box([d['x'], d['y'], d['z']], [d['w'], d['l'], d['h']],
                         Quaternion(axis=[0, 0, 1], radians=d['yaw']), score=d['score'], name=OUT_NU[cid])
                b = Bbox3d.from_nu_box_convert(nu, instance_id=j)
                b.seg_class_id = cid
                boxes.append(b)
            out[str(i)] = boxes
        return out

    def load_detections_2d(self):
        out = {}
        for i, f in enumerate(self.inp['frames']):
            dd = defaultdict(list)
            for d in f['dets2d']:
                cid = CLS_ID.get(d['cls'])
                if cid is not None:
                    dd[jo.CAM].append(Detection2D(Bbox2d(d['x1'], d['y1'], d['x2'], d['y2']), jo.CAM, d['score'], cid))
            out[str(i)] = dd
        return out


def main():
    inp = json.loads(Path(sys.argv[1]).read_text())
    seq = SequenceMC(inp)
    seq.mot.set_track_manager_params(PARAMS)
    run_info = defaultdict(int)
    rows, ids, t_total = [], defaultdict(set), 0.0
    for name in seq.frame_names:
        frame = seq.get_frame(name)
        t_ns = int(round(frame.data['t'] * 1e9))
        t0 = time.time()
        insts = frame.perform_tracking(PARAMS, run_info)
        t_total += time.time() - t0
        n = 0
        for inst in insts:
            if not inst.report_mot:
                continue
            center, wlh, rot = convert_kitti_bbox_coordinates_to_nu(inst.coordinates_3d)
            rows.append([t_ns, inst.track_id, *center, wlh[1], wlh[0], wlh[2], rot.yaw_pitch_roll[0],
                         '', '', '', 'dynamic', OUT_CLASS[inst.class_id], inst.bbox3d.confidence])
            ids[OUT_CLASS[inst.class_id]].add(inst.track_id)
            n += 1
        if n == 0:
            rows.append([t_ns, -1] + [''] * 13)  # empty-frame marker: the method ran
    with open(sys.argv[2], 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_ns', 'track_id', 'x', 'y', 'z', 'l', 'w', 'h', 'yaw', 'vx', 'vy', 'vz', 'state', 'class', 'score'])
        w.writerows(rows)
    nf = len(seq.frame_names)
    print(f"{inp['seq']}: {nf} frames, {sum(1 for r in rows if r[1] != -1)} track rows, ids per class "
          f"{ {k: len(v) for k, v in ids.items()} }; fused 3D+2D {run_info['instances_both']}, 3D only "
          f"{run_info['instances_3d']}, 2D only {run_info['instances_2d']}; {1000 * t_total / max(nf, 1):.2f} ms/frame")


if __name__ == '__main__':
    main()
