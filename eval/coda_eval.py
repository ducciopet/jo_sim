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
  - range: GT and predictions farther than --range (20 m) from the robot (odometry) are dropped: in several CODa
    windows the annotations stop at ~20 m, beyond it real movers are unannotated. So are GT and predictions closer
    than --min-range: the operator holding the robot walks ~0.9 m behind the LiDAR (fixed offset, 6 cm std),
    annotated as a Pedestrian in some windows and not in others, and seen there only in a 0.7 m band of the OS1's
    vertical field of view. A prediction is ignored on such a near box only if its centre lies inside the box (a
    pedestrian walking beside the robot is scored); on hidden / out-of-range objects within --ignore-dist.
  - --ignore-gt: a class view (e.g. the street vehicles) scored against the whole GT: predictions on objects of the
    other classes are ignored the same way, so they are neither true nor false positives of this class.
Both GT and predictions are in the odom frame of the window's own odometry bag.

usage: coda_eval.py --run <out bag> --gt <window>_gt.csv --odom <window>_odom [--out metrics.json]
"""
import argparse
import json
import re
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
CSV_METHODS = ('simpletrack', 'eagermot', 'deepfusionmot')


def convert_states(pred, mode, v_d=0.25, v_s=0.07, n_d=3):
    """(25/11) motion state for methods that output no state ('dynamic' everywhere): from the track's speed (vx, vy
    when given, else the centre displacement between consecutive outputs). raw: D if speed >= v_d on that output, else
    PD. debounced: the speed over the last 0.5 s (smoothed like SINS's Kalman velocity) and the debounce SINS applies to
    classified tracks: D after n_d consecutive outputs >= v_d, back to PD after n_d consecutive outputs < v_d."""
    obj = objects(pred).copy()
    if obj.empty or mode == 'none':
        return pred
    obj = obj.sort_values(['track_id', 't_ns'])
    sp = np.full(len(obj), np.nan)
    if {'vx', 'vy'} <= set(obj.columns):
        sp = np.hypot(pd.to_numeric(obj['vx'], errors='coerce'), pd.to_numeric(obj['vy'], errors='coerce')).to_numpy()
    dx = obj.groupby('track_id')['x'].diff().to_numpy()
    dy = obj.groupby('track_id')['y'].diff().to_numpy()
    dt = obj.groupby('track_id')['t_ns'].diff().to_numpy() / 1e9
    fd = np.where(dt > 0, np.hypot(dx, dy) / np.where(dt > 0, dt, 1), np.nan)
    has_v = np.isfinite(sp)
    sp = np.where(has_v, sp, fd)
    if mode == 'debounced':
        # a smoothed speed, as SINS's Kalman velocity: displacement over the last 0.5 s (causal), when no vx/vy is given
        sm = np.full(len(obj), np.nan)
        for _, idx in obj.groupby('track_id').indices.items():
            t = obj['t_ns'].to_numpy()[idx] / 1e9
            xy = obj[['x', 'y']].to_numpy(float)[idx]
            j = np.searchsorted(t, t - 0.5)
            ok = (t - t[j]) >= 0.3
            sm[idx] = np.where(ok, np.hypot(*(xy - xy[j]).T) / np.maximum(t - t[j], 1e-3), np.nan)
        sp = np.where(has_v, sp, sm)            # the method's own velocity where it gives one, else the 0.5 s speed
    state = []
    for _, idx in obj.groupby('track_id').indices.items():
        cur, up, down = 'potentially_dynamic', 0, 0
        for v in sp[idx]:
            v = 0.0 if not np.isfinite(v) else v
            if mode == 'raw':
                state.append('dynamic' if v >= v_d else 'potentially_dynamic')
                continue
            up = up + 1 if v >= v_d else 0
            down = down + 1 if v < v_d else 0          # SINS drops D when the speed falls below the D bar (no v_s hysteresis)
            if cur != 'dynamic' and up >= n_d:
                cur = 'dynamic'
            elif cur == 'dynamic' and down >= n_d:
                cur = 'potentially_dynamic'
            state.append(cur)
    order = np.concatenate(list(obj.groupby('track_id').indices.values()))
    st = np.empty(len(obj), object)
    st[order] = state
    obj['state'] = st
    empty = [dict(t_ns=int(t), track_id=EMPTY_FRAME_ID) for t in set(pred['t_ns']) - set(obj['t_ns'])]
    return make_tracks(obj.to_dict('records') + empty)


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
    for c in ('x', 'y', 'z', 'yaw', 'l', 'w'):
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
    if a.camera_only:   # camera-view evaluation: every object outside the camera image is ignored
        hidden |= np.array([not ann.at[(t, i), 'fov'] for t, i in zip(gt['t_ns'], gt['track_id'])], bool)
    elif not a.pd_everywhere:
        out_fov_still = np.array([not ann.at[(t, i), 'fov'] for t, i in zip(gt['t_ns'], gt['track_id'])], bool) & \
            (gt['state'] != 'dynamic').to_numpy()
        if a.pd_prev_observable:
            # (view B, 25/11) a still agent outside the FOV is scored if its track was observable before: an earlier
            # annotated row inside a camera FOV, or an earlier dynamic row (it moved); never-observable ones stay ignored
            fov_rows = ped[ped['in_fov'].astype(str) == 'True'].groupby('track_id')['t_ns'].min()
            dyn_rows = gt[gt['state'] == 'dynamic'].groupby('track_id')['t_ns'].min()
            first = {i: min(fov_rows.get(i, np.inf), dyn_rows.get(i, np.inf)) for i in gt['track_id'].unique()}
            prev = np.array([t > first[i] for t, i in zip(gt['t_ns'], gt['track_id'])], bool)
            out_fov_still &= ~prev
        hidden |= out_fov_still
    if a.state_buffer > 0:
        # (25/11) rows within --state-buffer s of a GT D<->PD transition of the same track become ignored regions
        # (for the STATE metrics: run this view separately; the primary MOTA-all is computed without it)
        near_tr = pd.Series(False, index=gt.index)
        for i, g in gt.groupby('track_id'):
            g = g.sort_values('t_ns')
            st = g['state'].to_numpy()
            tt = g['t_ns'].to_numpy() / 1e9
            ch = tt[1:][st[1:] != st[:-1]]
            if len(ch):
                close = np.min(np.abs(tt[:, None] - ch[None, :]), axis=1) <= a.state_buffer
                near_tr.loc[g.index] = close
        hidden |= near_tr.to_numpy()
    rd = robot_dist(gt, odom)
    near = rd < a.min_range
    far = (rd > a.range) | near
    # ignore regions: hidden or beyond range -> a disc of --ignore-dist; closer than --min-range (the operator behind
    # the robot) -> only the box itself, so a pedestrian walking beside the robot is still scored
    ign = gt[(hidden | far)][['t_ns', 'x', 'y', 'yaw']].assign(near=near[hidden | far])
    ign['l'] = [ann.at[(t, i), 'l'] for t, i in zip(gt['t_ns'][hidden | far], gt['track_id'][hidden | far])]
    ign['w'] = [ann.at[(t, i), 'w'] for t, i in zip(gt['t_ns'][hidden | far], gt['track_id'][hidden | far])]
    keep = gt[~hidden & ~far]
    if a.ignore_undefined:
        # annotated rows gt_tracks could not label (first/last ~0.5 s of a track, gaps): motion state undefined, so
        # they are ignored regions (disc of --ignore-dist), never silently removed (a prediction on them is no FP)
        lab = set(zip(gt['t_ns'], gt['track_id']))
        und = ped[[(t, i) not in lab for t, i in zip(ped['t_ns'], ped['track_id'])]]
        ign = pd.concat([ign, und[['t_ns', 'x', 'y', 'yaw', 'l', 'w']].assign(near=False)], ignore_index=True)
        n_undef = len(und)
    else:
        n_undef = 0
    # every annotated frame stays on the grid (an annotated frame with nobody to find is an empty GT frame)
    empty = [dict(t_ns=int(t), track_id=EMPTY_FRAME_ID) for t in frames if t not in set(keep['t_ns'])]
    stats = dict(annotated_frames=len(frames), pedestrians=len(bodies), gt_rows=len(keep),
                 ignored_hidden=int((hidden & ~far).sum()), ignored_undefined=n_undef, out_of_range=int(far.sum()),
                 dynamic_rows=int((keep['state'] == 'dynamic').sum()),
                 rows_by_class=keep.assign(c=[cls[i] for i in keep['track_id']]).groupby('c').size().to_dict())
    return make_tracks(keep.to_dict('records') + empty), ign, frames, stats


def in_camera_view(obj, cam_json):
    """Prediction centres projecting inside the camera image (in front of it, > 0.5 m), with the camera pose of the
    nearest frame of cam_json (EagerMOT's CODa input: per-frame T_cam_odom of the ZED left camera, K, image size)."""
    cam = json.load(open(cam_json))
    K, (h, w) = np.array(cam['K']), cam['img_shape']
    ct = np.array([f['t'] for f in cam['frames']])
    T = np.array([f['T_cam_odom'] for f in cam['frames']])
    t = obj['t_ns'].to_numpy() / 1e9
    k = np.clip(np.searchsorted(ct, t), 1, len(ct) - 1)
    k = np.where(np.abs(ct[k - 1] - t) < np.abs(ct[k] - t), k - 1, k)
    p = np.einsum('nij,nj->ni', T[k, :3, :3], obj[['x', 'y', 'z']].to_numpy(float)) + T[k, :3, 3]
    z = np.maximum(p[:, 2], 1e-6)
    uv = (p @ K.T)[:, :2] / z[:, None]
    return (p[:, 2] > 0.5) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)


def filter_pred(pred, ign, frames, odom, odom_z, a):
    obj = objects(pred).copy()
    if obj.empty:
        return pred, 0, 0
    rd = robot_dist(obj, odom)
    far = (rd > a.range) | (rd < a.min_range)
    # above any annotated agent (tree canopies, roofs): not an obstacle for a ground robot
    far |= obj['z'].to_numpy() - np.interp(obj['t_ns'].to_numpy() / 1e9, odom_z[:, 0], odom_z[:, 1]) > a.max_height
    if a.camera_only:   # same criterion as the GT's in_fov (coda_to_bag.in_camera_fov): centre in the image, > 0.5 m ahead
        far |= ~in_camera_view(obj, a.cam_json)
    # nearest annotated time for each prediction row; ignore objects are only known there
    ft = frames / 1e9
    t = obj['t_ns'].to_numpy() / 1e9
    k = np.clip(np.searchsorted(ft, t), 1, len(ft) - 1)
    near_t = np.where(np.abs(ft[k - 1] - t) < np.abs(ft[k] - t), frames[k - 1], frames[k])
    if 'near' not in ign:
        ign = ign.assign(near=False, l=0.0, w=0.0, yaw=0.0)
    by_t = {tt: g[['x', 'y', 'near', 'l', 'w', 'yaw']].to_numpy(float) for tt, g in ign.groupby('t_ns')}
    hit = np.zeros(len(obj), bool)
    for j, (tt, x, y) in enumerate(zip(near_t, obj['x'], obj['y'])):
        p = by_t.get(tt)
        if p is None:
            continue
        dx, dy = x - p[:, 0], y - p[:, 1]
        c, s = np.cos(p[:, 5]), np.sin(p[:, 5])
        in_box = (np.abs(c * dx + s * dy) <= p[:, 3] / 2) & (np.abs(-s * dx + c * dy) <= p[:, 4] / 2)
        disc = np.hypot(dx, dy) < a.ignore_dist
        if np.any(np.where(p[:, 2] > 0, in_box, (in_box | disc) if getattr(a, 'ignore_box', False) else disc)):
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
    # 20 m: in several CODa windows the annotations stop at ~20 m (p99 of the GT range 19.5-22 m in 0_w1, 10_w3, 10_w4),
    # so beyond it real movers are unannotated and would count as false positives
    ap.add_argument('--range', type=float, default=20.0)
    ap.add_argument('--ignore-dist', type=float, default=1.0)
    ap.add_argument('--ignore-box', action='store_true',
                    help='ignored objects cover their own box (+ the disc) instead of the disc only: a piece of an ignored 6 m vehicle away from its centre is no FP (20/11)')
    ap.add_argument('--min-range', type=float, default=1.2)
    ap.add_argument('--max-height', type=float, default=2.5, help='predictions centred higher above the robot ground')
    ap.add_argument('--min-pts', type=int, default=1)
    ap.add_argument('--pd-everywhere', action='store_true', help='score still objects outside the camera view too')
    ap.add_argument('--camera-only', action='store_true', help='score only the camera view (GT in_fov, predictions '
                    'projected with --cam-json): the condition of camera-confirmed trackers (DeepFusionMOT, KITTI)')
    ap.add_argument('--cam-json', help='per-frame camera pose json (bags/eagermot_out/in_coda<w>.json)')
    ap.add_argument('--ignore-gt', help='a GT CSV of the OTHER classes (e.g. the street persons when scoring the vehicle '
                    'GT): predictions on those objects are ignored, so a class view is scored against the whole GT')
    ap.add_argument('--ignore-undefined', action='store_true', help='17/11 evaluation (e3): annotated rows whose motion '
                    'state is undefined (track ends, gaps) become ignored regions instead of being dropped')
    ap.add_argument('--dyn-speed', type=float, default=0.3)
    ap.add_argument('--pd-prev-observable', action='store_true', help='(25/11, view B) still agents outside the camera '
                    'FOV are scored if their track was inside a FOV, or moving, earlier; never-observable ones stay ignored')
    ap.add_argument('--state-buffer', type=float, default=0.0, help='(25/11) [s] GT rows this close to a D<->PD '
                    'transition of the same track are ignored; for the state metrics only (run separately)')
    ap.add_argument('--baseline-state', default='none', choices=['none', 'raw', 'debounced'],
                    help='(25/11) motion state of CSV baselines without one: none = all dynamic (default, MOTA-all), '
                    'raw = single-output speed >= 0.25 m/s, debounced = 0.5 s speed + SINS classified-track debounce (3 outputs in and out)')
    ap.add_argument('--match-dist', type=float, default=1.0)
    ap.add_argument('--points-min-cluster', type=int, default=20, help='point-output baselines (M-detector): minimum '
                    'cluster size; 20 for the OS1-128 (CODa), 5 for the VLP-16 (lab, street), as in the lab evaluation')
    ap.add_argument('--extra-ignore', help='CSV t_ns,x,y[,..] of points of objects the GT does not annotate (JRDB: '
                    'movable non-person objects from JRDB-PanoTrack, eval/datasets/jrdb/jrdb_pano_ignore.py), labelled at a '
                    'lower rate: a prediction centred within --extra-ignore-dist of one of them, within --extra-ignore-dt of '
                    'its labelled frame, is dropped (neither TP nor FP)')
    ap.add_argument('--extra-ignore-dist', type=float, default=0.3)
    ap.add_argument('--extra-ignore-dt', type=float, default=0.5)
    ap.add_argument('--frames', default='all', choices=['all', 'training', 'validation', 'testing'],
                    help="score only the annotated frames of this split of CODa's official per-frame split "
                         "(metadata/<seq>.json ObjectTracking); 'testing' = frames never used to train the learned baselines")
    ap.add_argument('--coda-root', default='/mnt/nas-imperatore/datasets_SINS/CODa')
    ap.add_argument('--out')
    a = ap.parse_args()
    if a.camera_only and not a.cam_json:
        ap.error('--camera-only needs --cam-json')

    odom = load_odometry(a.odom)
    gt, ign, frames, gstats = build_gt(a.gt, odom, a)
    if a.ignore_gt:
        other = objects(build_gt(a.ignore_gt, odom, argparse.Namespace(**{**vars(a), 'min_pts': 0, 'pd_everywhere': True,
                                                                         'camera_only': False, 'range': 1e9}))[0])
        ign = pd.concat([ign, other[['t_ns', 'x', 'y', 'yaw']].assign(near=False, l=0.0, w=0.0)], ignore_index=True)
    if a.method == 'sins':
        pred, frame_id, topic = load_obstacles(a.run, a.topic)
        if frame_id not in ('odom', ''):
            raise SystemExit(f'predictions in {frame_id}, expected odom')
    elif a.method in CSV_METHODS:
        pred, topic = load_csv_tracks(a.run), a.run
        pred = convert_states(pred, a.baseline_state)
    else:                                   # baselines publish in the odom frame of the same odometry (global frame)
        topic, ids = BASELINES[a.method]
        # OS1-128 clouds are ~8x denser than the lab VLP-16: dynamic points are clustered with Dynablox's own default
        # minimum cluster size (20) instead of the lab's 5
        pred = load_dynamic_points(a.run, topic, min_points=a.points_min_cluster) if ids is None else load_marker_boxes(a.run, topic, persistent_ids=ids)
    pred, n_far, n_ign = filter_pred(pred, ign, frames, odom, odom_ground_z(a.odom), a)
    res = {}
    ftimes = None
    if a.frames != 'all':
        info = json.load(open(Path(a.gt).with_suffix('.json')))
        meta = json.load(open(Path(a.coda_root) / 'metadata' / f"{info['seq']}.json"))['ObjectTracking'][a.frames]
        split = {int(re.findall(r'_(\d+)\.json$', f)[0]) for f in meta}
        ftimes = [t / 1e9 for f, t in zip(info['annotated_frames'], info['annotated_t_ns']) if int(f) in split]
        gstats = dict(gstats, split=a.frames, split_frames=len(ftimes))
    for mode in ('full', 'merged', 'dynamic_only'):
        r = evaluate(gt, pred, EvalConfig(match_dist=a.match_dist, label_mode=mode, skip_missing_gt=False,
                                          frame_times=ftimes))
        res[mode] = r.to_dict()
    out = dict(run=a.run, topic=topic, gt=gstats, pred_dropped_out_of_range=n_far, pred_dropped_ignore=n_ign,
               config=vars(a), metrics=res)
    m, s, d = res['merged'], res['full'], res['dynamic_only']
    dur_min = (frames[-1] - frames[0]) / 6e10 * (1.0 if ftimes is None else len(ftimes) / len(frames))   # split: its share of the time
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
