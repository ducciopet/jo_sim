#!/usr/bin/env python3
"""
Evaluate method runs on CODa windows (eval/datasets/coda_to_bag.py) against CODa's annotated 3D boxes.

GT = every annotated movable agent (Pedestrian, Bike, Scooter, Service Vehicle, ...: the --classes of the conversion),
state from speed as for the lab (DYNAMIC above --dyn-speed over a 1 s centred difference, else POTENTIALLY_DYNAMIC).
  - visibility: a box with fewer than --min-pts LiDAR returns inside it is ignored (neither TP nor FP: a prediction
    within --ignore-dist of it at the nearest annotated time is dropped, the box itself is never a miss), the
    KITTI/nuScenes criterion. CODa's isOccluded flag is NOT used: it describes the camera view (the operators walking
    1-2 m from the robot are "Full" with thousands of LiDAR returns);
  - still objects (POTENTIALLY_DYNAMIC) outside the camera image are ignored the same way (only the camera's
    semantics can tell a parked bike from a bollard; --pd-everywhere scores them too); moving ones count everywhere;
  - predictions centred more than --max-height above the ground under the robot are dropped (canopies, roofs);
  - range: GT and predictions farther than --range from the robot (odometry) are dropped, the same radius as the
    dataset's SINS profile (cfg/coda lidar_range_x/y); so are GT and predictions closer than --min-range: the
    operator holding the robot walks ~0.9 m behind the LiDAR (fixed offset, 6 cm std), annotated as a Pedestrian in
    some windows and not in others, and seen there only in a 0.7 m band of the OS1's vertical field of view.
Both GT and predictions are in the odom frame of the window's own odometry bag.

usage: coda_eval.py --run <out bag> --gt <window>_gt.csv --odom <window>_odom [--out metrics.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.gt_mocap import gt_tracks                               # noqa: E402
from harness.metrics import EvalConfig, evaluate                     # noqa: E402
from harness.ros_io import load_obstacles, load_odometry             # noqa: E402
from harness.tracks import EMPTY_FRAME_ID, make_tracks, objects      # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent / 'adapters'))
from marker_adapter import load_marker_boxes                         # noqa: E402
from points_adapter import load_dynamic_points                       # noqa: E402

# baseline outputs (topic, persistent ids; None = dynamic points clustered by points_adapter), as in baseline_eval.py
BASELINES = {'dynablox': ('/motion_detector/visualization/clusters', True),
             'lvdot': ('/onboard_detector/dynamic_bboxes', False),
             'mdetector': ('/m_detector/point_out', None)}
# offline methods writing CSV tracks (t_ns, id, label, x, y, z, l, w, h, yaw, score; odom frame, id -1 = empty frame):
# no motion state, every track counts as 'dynamic' (as UniMT/EagerMOT in the lab)
CSV_METHODS = ('simpletrack', 'eagermot')


def load_csv_tracks(path):
    df = pd.read_csv(path)
    if 'track_id' in df:                    # already the unified tracks CSV (run_eagermot_jo.py)
        return make_tracks(df.to_dict('records'))
    rows = [dict(t_ns=int(t), track_id=EMPTY_FRAME_ID) for t in df.loc[df['id'] < 0, 't_ns'].unique()]
    d = df[df['id'] >= 0]
    rows += [dict(t_ns=int(r.t_ns), track_id=int(r.id), x=r.x, y=r.y, z=r.z, l=r.l, w=r.w, h=r.h, yaw=r.yaw,
                  state='dynamic', score=r.score, **{'class': r.label}) for r in d.itertuples()]
    return make_tracks(rows)



def odom_ground_z(bag):
    """[t_s, z] of base_link (on the ground under the robot) in odom."""
    from rosbags.highlevel import AnyReader
    out = []
    with AnyReader([Path(bag)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.z))
    return np.array(sorted(out))


def robot_dist(df, odom):
    t = df['t_ns'].to_numpy() / 1e9
    rx = np.interp(t, odom[:, 0], odom[:, 1])
    ry = np.interp(t, odom[:, 0], odom[:, 2])
    return np.hypot(df['x'].to_numpy() - rx, df['y'].to_numpy() - ry)


def build_gt(gt_csv, odom, a):
    raw = pd.read_csv(gt_csv, keep_default_na=False)
    frames = np.unique(raw['t_ns'].to_numpy())
    obj = raw[raw['track_id'] != EMPTY_FRAME_ID].copy()
    for c in ('x', 'y', 'z', 'yaw'):
        obj[c] = obj[c].astype(float)
    obj['npts'] = obj['npts'].astype(int)
    ped = obj
    bodies = {int(i): g.sort_values('t_ns')[['t_ns', 'x', 'y', 'z', 'yaw']].to_numpy(float) * [1e-9, 1, 1, 1, 1]
              for i, g in ped.groupby('track_id')}
    gt, _ = gt_tracks(bodies, dyn_speed=a.dyn_speed)        # sorted names = ids, so map back
    names = sorted(bodies)
    cls = ped.groupby('track_id')['class'].first().to_dict()
    gt['track_id'] = [names[i] for i in gt['track_id']]
    # gt_tracks goes through float seconds: snap its stamps back onto the annotated frames' own
    k = np.clip(np.searchsorted(frames, gt['t_ns'].to_numpy()), 1, len(frames) - 1)
    gt['t_ns'] = np.where(np.abs(frames[k - 1] - gt['t_ns']) < np.abs(frames[k] - gt['t_ns']), frames[k - 1], frames[k])
    # gt_tracks resamples onto the annotated stamps; visibility of each row from the nearest annotation of that id
    ann = ped.assign(fov=ped['in_fov'].astype(str) == 'True').set_index(['t_ns', 'track_id'])
    hidden = np.array([ann.at[(t, i), 'npts'] < a.min_pts for t, i in zip(gt['t_ns'], gt['track_id'])], bool)
    # a still object can only be told from static structure through the camera's semantics: outside its view
    # it is ignored (moving objects count everywhere)
    if not a.pd_everywhere:
        hidden |= np.array([not ann.at[(t, i), 'fov'] for t, i in zip(gt['t_ns'], gt['track_id'])], bool) & \
            (gt['state'] != 'dynamic').to_numpy()
    rd = robot_dist(gt, odom)
    far = (rd > a.range) | (rd < a.min_range)
    ign = gt[hidden | far][['t_ns', 'x', 'y']]
    keep = gt[~hidden & ~far]
    # every annotated frame stays on the grid (an annotated frame with nobody to find is an empty GT frame)
    empty = [dict(t_ns=int(t), track_id=EMPTY_FRAME_ID) for t in frames if t not in set(keep['t_ns'])]
    stats = dict(annotated_frames=len(frames), pedestrians=len(bodies), gt_rows=len(keep),
                 ignored_hidden=int((hidden & ~far).sum()), out_of_range=int(far.sum()),
                 dynamic_rows=int((keep['state'] == 'dynamic').sum()),
                 rows_by_class=keep.assign(c=[cls[i] for i in keep['track_id']]).groupby('c').size().to_dict())
    return make_tracks(keep.to_dict('records') + empty), ign, frames, stats


def filter_pred(pred, ign, frames, odom, odom_z, a):
    obj = objects(pred).copy()
    if obj.empty:
        return pred, 0, 0
    rd = robot_dist(obj, odom)
    far = (rd > a.range) | (rd < a.min_range)
    # above any annotated agent (tree canopies, roofs): not an obstacle for a ground robot
    far |= obj['z'].to_numpy() - np.interp(obj['t_ns'].to_numpy() / 1e9, odom_z[:, 0], odom_z[:, 1]) > a.max_height
    # nearest annotated time for each prediction row; ignore objects are only known there
    ft = frames / 1e9
    t = obj['t_ns'].to_numpy() / 1e9
    k = np.clip(np.searchsorted(ft, t), 1, len(ft) - 1)
    near_t = np.where(np.abs(ft[k - 1] - t) < np.abs(ft[k] - t), frames[k - 1], frames[k])
    by_t = {tt: g[['x', 'y']].to_numpy() for tt, g in ign.groupby('t_ns')}
    hit = np.zeros(len(obj), bool)
    for j, (tt, x, y) in enumerate(zip(near_t, obj['x'], obj['y'])):
        p = by_t.get(tt)
        if p is not None and np.min(np.hypot(p[:, 0] - x, p[:, 1] - y)) < a.ignore_dist:
            hit[j] = True
    if getattr(a, 'extra_ignore', None):
        ex = pd.read_csv(a.extra_ignore)
        et = np.unique(ex['t_ns'].to_numpy())
        ex_by_t = {tt: g[['x', 'y']].to_numpy() for tt, g in ex.groupby('t_ns')}
        if len(et):
            k = np.clip(np.searchsorted(et, obj['t_ns'].to_numpy()), 1, len(et) - 1) if len(et) > 1 else np.zeros(len(obj), int)
            for j, (tp, x, y) in enumerate(zip(obj['t_ns'].to_numpy(), obj['x'], obj['y'])):
                if hit[j]:
                    continue
                for kk in {max(0, k[j] - 1), k[j]}:
                    if abs(et[kk] - tp) / 1e9 <= a.extra_ignore_dt:
                        p = ex_by_t[et[kk]]
                        if np.min(np.hypot(p[:, 0] - x, p[:, 1] - y)) < a.extra_ignore_dist:
                            hit[j] = True
                            break
    kept = obj[~far & ~hit]
    marks = [dict(t_ns=int(t), track_id=EMPTY_FRAME_ID) for t in np.unique(pred['t_ns']) if t not in set(kept['t_ns'])]
    return make_tracks(kept.to_dict('records') + marks), int(far.sum()), int(hit.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', required=True)
    ap.add_argument('--gt', required=True)
    ap.add_argument('--odom', required=True)
    ap.add_argument('--topic', default=None)
    ap.add_argument('--method', default='sins', choices=['sins', *BASELINES, *CSV_METHODS])
    ap.add_argument('--range', type=float, default=25.0)
    ap.add_argument('--ignore-dist', type=float, default=1.0)
    ap.add_argument('--min-range', type=float, default=1.2)
    ap.add_argument('--max-height', type=float, default=2.5, help='predictions centred higher above the robot ground')
    ap.add_argument('--min-pts', type=int, default=1)
    ap.add_argument('--pd-everywhere', action='store_true', help='score still objects outside the camera view too')
    ap.add_argument('--dyn-speed', type=float, default=0.3)
    ap.add_argument('--match-dist', type=float, default=1.0)
    ap.add_argument('--extra-ignore', help='CSV t_ns,x,y[,..] of points of objects the GT does not annotate (JRDB: '
                    'movable non-person objects from JRDB-PanoTrack, eval/datasets/jrdb/jrdb_pano_ignore.py), labelled at a '
                    'lower rate: a prediction centred within --extra-ignore-dist of one of them, within --extra-ignore-dt of '
                    'its labelled frame, is dropped (neither TP nor FP)')
    ap.add_argument('--extra-ignore-dist', type=float, default=0.3)
    ap.add_argument('--extra-ignore-dt', type=float, default=0.5)
    ap.add_argument('--out')
    a = ap.parse_args()

    odom = load_odometry(a.odom)
    gt, ign, frames, gstats = build_gt(a.gt, odom, a)
    if a.method == 'sins':
        pred, frame_id, topic = load_obstacles(a.run, a.topic)
        if frame_id not in ('odom', ''):
            raise SystemExit(f'predictions in {frame_id}, expected odom')
    elif a.method in CSV_METHODS:
        pred, topic = load_csv_tracks(a.run), a.run
    else:                                   # baselines publish in the odom frame of the same odometry (global frame)
        topic, ids = BASELINES[a.method]
        # OS1-128 clouds are ~8x denser than the lab VLP-16: dynamic points are clustered with Dynablox's own default
        # minimum cluster size (20) instead of the lab's 5
        pred = load_dynamic_points(a.run, topic, min_points=20) if ids is None else load_marker_boxes(a.run, topic, persistent_ids=ids)
    pred, n_far, n_ign = filter_pred(pred, ign, frames, odom, odom_ground_z(a.odom), a)
    res = {}
    for mode in ('full', 'merged', 'dynamic_only'):
        r = evaluate(gt, pred, EvalConfig(match_dist=a.match_dist, label_mode=mode, skip_missing_gt=False))
        res[mode] = r.to_dict()
    out = dict(run=a.run, topic=topic, gt=gstats, pred_dropped_out_of_range=n_far, pred_dropped_ignore=n_ign,
               config=vars(a), metrics=res)
    m, s, d = res['merged'], res['full'], res['dynamic_only']
    dur_min = (frames[-1] - frames[0]) / 6e10
    print(f"{Path(a.run).name}: merged recall {m['tp'] / max(1, m['tp'] + m['fn']):.1%}  "
          f"FP/min {m['fp'] / dur_min:.1f}  IDF1 {m['idf1']:.1%}  IDSW {m['idsw']}  MOTA {m['mota']:.1%}  "
          f"MOTP {m['motp']:.2f} m | moving only: recall {d['tp'] / max(1, d['tp'] + d['fn']):.1%} FP/min {d['fp'] / dur_min:.1f} "
          f"IDF1 {d['idf1']:.1%} MOTA {d['mota']:.1%} | F1 D {s['state_f1'].get('dynamic', float('nan')):.1%}  "
          f"F1 PD {s['state_f1'].get('potentially_dynamic', float('nan')):.1%}  | GT {gstats}")
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        json.dump(out, open(a.out, 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
