#!/usr/bin/env python3
"""
Why does a v2 run miss the person (FN) or report things that aren't there (FP)?

Needs a run recorded with the tracker's full output and its input detections
(eval/scripts/run_method.sh with EXTRA_TOPICS="/sins/tracker/tracks
/sins/dbscan_detector/fused/detections3d ..."), plus the GT and the
alignment written by run_own_eval.py.

FN causes, per grid frame where the GT person is not matched:
  no_detection          no fused detection within the gate: detection/preprocessing problem
  detection_no_track    a detection is there but no published track: track unconfirmed or coasting
  track_static          a published track is there, labelled static: classification problem
  track_not_matched     a D/PD track is near but the matcher took another / gate edge
FP causes, per unmatched D/PD prediction:
  near_person (< 1.5 m from the GT person: fragment / revealed background) vs far, with class.

Usage
  python3 eval/analyze_errors.py --runs bags/replay --prefix base --gt-dir eval/results/own
"""

import argparse
import collections
import json
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.gt_mocap import compose  # noqa: E402
from harness.metrics import EvalConfig, _match, make_grid, sample_gt, sample_pred  # noqa: E402
from harness.ros_io import load_obstacles, stamp_ns, typestore  # noqa: E402
from harness.gt_mocap import tracks_to_mocap  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402


def frames_by_time(bag, topic, parse):
    out = {}
    with AnyReader([Path(bag)], default_typestore=typestore()) as r:
        # runs recorded before the package rename (2026-10-26) carry the old /jo_detector_v2 prefix
        cs = [c for c in r.connections if c.topic in (topic, topic.replace('/sins/', '/jo_detector_v2/', 1))]
        for c, _, raw in r.messages(cs):
            m = r.deserialize(raw, c.msgtype)
            out[stamp_ns(m.header)] = parse(m)
    t = np.array(sorted(out))
    return t, out


def latest(t_arr, frames, t_ns, max_age_ns):
    i = np.searchsorted(t_arr, t_ns, side='right') - 1
    if i < 0 or t_ns - t_arr[i] > max_age_ns:
        return []
    return frames[t_arr[i]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', required=True)
    ap.add_argument('--prefix', required=True, help='run dirs are <runs>/<prefix>_v2_<seq>')
    ap.add_argument('--inputs-prefix', default=None,
                    help='read the fused detections from <runs>/<inputs-prefix>_v2_<seq> instead (tracker-only replays)')
    ap.add_argument('--gt-dir', default='eval/results/own')
    ap.add_argument('--alignment', default=None, help='default: <gt-dir>/alignment_v2.json')
    args = ap.parse_args()
    gt_dir = Path(args.gt_dir)
    S = json.loads((gt_dir / 'summary.json').read_text())
    A = json.loads(Path(args.alignment or gt_dir / 'alignment_v2.json').read_text())
    cfg = EvalConfig(roi=tuple(S['roi']))

    tot_fn, tot_fp = collections.Counter(), collections.Counter()
    for s in S['sequences']:
        run = Path(args.runs) / f'{args.prefix}_v2_{s}'
        if not run.is_dir():
            continue
        T, d = np.array(A['T_mocap_odom'][s]), A['delta'][s]
        dn = int(round(d * 1e9))

        def to_mocap(x, y):
            p = compose(T, np.array([x, y, 0.0]))
            return p[0], p[1]

        det_run = Path(args.runs) / f'{args.inputs_prefix}_v2_{s}' if args.inputs_prefix else run
        det_t, dets = frames_by_time(det_run, '/sins/dbscan_detector/fused/detections3d',
                                     lambda m: [to_mocap(x.bbox.center.position.x, x.bbox.center.position.y)
                                                for x in m.detections])
        trk_t, trks = frames_by_time(run, '/sins/tracker/tracks',
                                     lambda m: [(int(x.id.split('_')[1]),
                                                 *to_mocap(x.bbox.center.position.x, x.bbox.center.position.y),
                                                 x.results[0].hypothesis.class_id,
                                                 x.results[1].hypothesis.class_id if len(x.results) > 1 else '')
                                                for x in m.detections])
        p, _, _ = load_obstacles(run)
        pred = tracks_to_mocap(p, T, d)
        gt = read_tracks(gt_dir / s / 'gt' / 'tracks.csv')
        grid = make_grid(gt, cfg)
        G, P = sample_gt(gt, grid, cfg), sample_pred(pred, grid, cfg)
        fn, fp = collections.Counter(), collections.Counter()
        fp_tracks = collections.defaultdict(list)
        for t, g, pp in zip(grid, G, P):
            if not g:
                continue
            tr_ns = int(round(t * 1e9)) - dn        # robot clock
            m, _ = _match(g, pp, cfg.match_dist)
            mg, mp = {i for i, _, _ in m}, {j for _, j, _ in m}
            for i, (gid, gx, gy, gst) in enumerate(g):
                if i in mg:
                    continue
                near_det = any(np.hypot(x - gx, y - gy) <= cfg.match_dist
                               for x, y in latest(det_t, dets, tr_ns, 250_000_000))
                near_trk = [k for k in latest(trk_t, trks, tr_ns, 250_000_000)
                            if np.hypot(k[1] - gx, k[2] - gy) <= cfg.match_dist]
                if not near_det and not near_trk:
                    cause = 'no_detection'
                elif not near_trk:
                    cause = 'detection_no_track'
                elif all(k[3] == 'static' for k in near_trk):
                    cause = 'track_static'
                else:
                    cause = 'track_not_matched'
                fn[(gst, cause)] += 1
            for j, (pid, x, y, st) in enumerate(pp):
                if j in mp:
                    continue
                dg = min(np.hypot(x - gx, y - gy) for _, gx, gy, _ in g)
                cls = next((k[4] for k in latest(trk_t, trks, tr_ns, 250_000_000) if k[0] == pid), '?')
                key = (st, cls or 'unclassified', 'near_person' if dg < 1.5 else 'far')
                fp[key] += 1
                fp_tracks[pid].append(t)
        tot_fn.update(fn)
        tot_fp.update(fp)
        spans = [v[-1] - v[0] + 0.1 for v in fp_tracks.values()]
        print(f'== {s}  FN {sum(fn.values())}: {dict(fn)}')
        print(f'   FP {sum(fp.values())} on {len(fp_tracks)} tracks (median life {np.median(spans) if spans else 0:.1f}s): {dict(fp)}')
    print(f'\nALL FN {sum(tot_fn.values())}: {dict(tot_fn)}')
    print(f'ALL FP {sum(tot_fp.values())}: {dict(tot_fp)}')


if __name__ == '__main__':
    main()
