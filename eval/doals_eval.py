#!/usr/bin/env python3
"""
DOALS point IoU for box-output methods (SINS and CSV baselines), the Dynablox authors' metric (evaluator.cpp,
evaluate_data.py): per annotated scan, points with min_range <= distance to the sensor <= max_range (default 0.5-20 m,
Dynablox default.yaml); TP/FP/FN of "dynamic" vs the GT indices; IoU = TP/(TP+FP+FN), precision, recall; mean (+- std)
over the annotated scans. A point is "dynamic" if it lies inside an output box of an accepted state (default: dynamic;
--states dynamic potentially_dynamic for the second column) from the output message nearest in time (<= --max-dt) to
the scan. The GT labels pedestrians and what they carry by APPEARANCE, not motion (state it next to the numbers).

Inputs come from eval/datasets/doals/doals_to_bag.py: the converted cloud bag (deskewed, levelled, original point order),
its _odom bag and _doals_gt.json. Predictions in the odom frame (as for coda_eval.py).

usage: doals_eval.py --run <run bag | csv> [--method sins|<csv method>] --bag <converted> --odom <converted>_odom
                     --gt <converted>_doals_gt.json [--states dynamic] [--out scores.json]
Pure evaluation of existing outputs.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.ros_io import load_obstacles                       # noqa: E402
from rosbags.highlevel import AnyReader                         # noqa: E402

CSV_METHODS = ('eagermot', 'simpletrack', 'deepfusionmot', 'centerpoint', 'csv')


def q2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def stamp(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', required=True)
    ap.add_argument('--method', default='sins')
    ap.add_argument('--bag', required=True)
    ap.add_argument('--odom', required=True)
    ap.add_argument('--gt', required=True)
    ap.add_argument('--states', nargs='+', default=['dynamic'])
    ap.add_argument('--min-range', type=float, default=0.5)
    ap.add_argument('--max-range', type=float, default=20.0)
    ap.add_argument('--max-dt', type=float, default=0.05)
    ap.add_argument('--out')
    a = ap.parse_args()
    G = json.load(open(a.gt))
    T_base_v = np.array(G['T_base_velodyne'])
    want = {e['t_ns']: e['indices'] for e in G['annotated']}

    # odom <- base_link per scan stamp
    odo = {}
    with AnyReader([Path(a.odom)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            T = np.eye(4)
            T[:3, :3] = q2R(m.pose.pose.orientation)
            p = m.pose.pose.position
            T[:3, 3] = [p.x, p.y, p.z]
            odo[stamp(m.header)] = T
    clouds = {}
    with AnyReader([Path(a.bag)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/velodyne_points']):
            m = r.deserialize(raw, c.msgtype)
            s = stamp(m.header)
            if s not in want:
                continue
            n = m.width * m.height
            d = np.frombuffer(bytes(m.data), np.float32).reshape(n, m.point_step // 4)
            clouds[s] = d[:, :3].astype(np.float64)

    if a.method == 'sins':
        pred, frame_id, _ = load_obstacles(a.run)
        if frame_id not in ('odom', ''):
            raise SystemExit(f'predictions in {frame_id}, expected odom')
    else:
        from coda_eval import load_csv_tracks
        pred = load_csv_tracks(a.run)
    pred = pred[pred['track_id'] >= 0] if 'track_id' in pred else pred
    pt = np.sort(pred['t_ns'].unique()) if len(pred) else np.array([], np.int64)

    rows = []
    for s, idx in sorted(want.items()):
        if s not in clouds or s not in odo:
            rows.append(dict(t_ns=s, missing=True))
            continue
        pv = clouds[s]
        dist = np.linalg.norm(pv, axis=1)
        ev = np.isfinite(pv).all(1) & (dist >= a.min_range) & (dist <= a.max_range)
        gtm = np.zeros(len(pv), bool)
        gtm[np.array(idx, int)] = True
        T = odo[s] @ T_base_v
        po = pv @ T[:3, :3].T + T[:3, 3]
        dyn = np.zeros(len(pv), bool)
        if len(pt):
            k = int(np.argmin(np.abs(pt - s)))
            if abs(int(pt[k]) - s) <= a.max_dt * 1e9:
                boxes = pred[(pred['t_ns'] == pt[k]) & pred['state'].isin(a.states)]
                for b in boxes.itertuples():
                    c, sn = np.cos(b.yaw), np.sin(b.yaw)
                    dx, dy, dz = po[:, 0] - b.x, po[:, 1] - b.y, po[:, 2] - b.z
                    dyn |= (np.abs(dx * c + dy * sn) <= b.l / 2) & (np.abs(-dx * sn + dy * c) <= b.w / 2) & (np.abs(dz) <= b.h / 2)
        tp = int(np.sum(dyn & gtm & ev)); fp = int(np.sum(dyn & ~gtm & ev)); fn = int(np.sum(~dyn & gtm & ev))
        rows.append(dict(t_ns=s, tp=tp, fp=fp, fn=fn,
                         iou=tp / (tp + fp + fn) if tp + fp + fn else np.nan,
                         precision=tp / (tp + fp) if tp + fp else np.nan, recall=tp / (tp + fn) if tp + fn else np.nan))
    ok = [x for x in rows if not x.get('missing')]
    res = dict(seq=G['seq'], states=a.states, range=[a.min_range, a.max_range], scans=rows,
               iou_mean=float(np.nanmean([x['iou'] for x in ok])) if ok else None,
               iou_std=float(np.nanstd([x['iou'] for x in ok])) if ok else None,
               precision_mean=float(np.nanmean([x['precision'] for x in ok])) if ok else None,
               recall_mean=float(np.nanmean([x['recall'] for x in ok])) if ok else None,
               missing=sum(1 for x in rows if x.get('missing')),
               note='GT labels persons and what they carry by appearance, not motion')
    print(json.dumps({k: v for k, v in res.items() if k != 'scans'}, indent=1))
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
