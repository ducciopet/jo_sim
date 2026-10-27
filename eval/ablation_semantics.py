#!/usr/bin/env python3
"""
Ablation "semantics off": tracker_node replayed alone (deterministic, eval/scripts/run_tracker_only.sh) on the SAME
recorded inputs of every sequence (<inputs>_v2_<seq>: fused detections, clusters, YOLO 2D, TF, odometry), scored with
the same per-label definitions as eval/label_eval.py, plus the stop-and-go identity metric (eval/stopgo_eval.py).

  v2          the deployed configuration (tracker-only replay, the reference for the two below)
  nosem       semantic class evidence ignored: the dynamic-class whitelist (track_semantic_dynamic_classes) holds
              no real class, so no track ever HAS a class (voteClass only lets a whitelisted class win): no
              class-driven confirmation, PD, longer coasting, size-gate exemption or class-based ID guards. PD can
              still come from motion history (track_motion_pd_min_dynamic_ticks: published dynamic >= 20 ticks).
  nosem_nopd  as nosem, and the motion-history PD off too: a plain moving/not-moving tracker (D or nothing).

The detector upstream is unchanged (its fused boxes still carry YOLO refinement): this isolates what the TRACKER
does with the semantic state. Usage (host; the container must be running):
  python3 eval/ablation_semantics.py --inputs r9a
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import static_eval as se  # noqa: E402
from harness.gt_mocap import tracks_to_mocap  # noqa: E402
from harness.metrics import EvalConfig, evaluate  # noqa: E402
from harness.ros_io import load_obstacles  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402
from label_eval import person_scores  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / 'bags' / 'replay'
CONT = '/home/ros/bags/replay'
CONTAINER = 'jo_sim_sins'
SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']
NOSEM = ['-p', '"track_semantic_dynamic_classes:=[\'__none__\']"']
CONFIGS = {
    'v2': [],
    'nosem': NOSEM,
    'nosem_nopd': NOSEM + ['-p', 'track_motion_pd_min_dynamic_ticks:=0'],
}


def run(cfg, inputs, seq):
    out = f'semab_{inputs}_{cfg}_v2_{seq}'
    if (REPLAY / out / 'metadata.yaml').exists():
        return REPLAY / out
    cmd = f'cd {CONT} && bash run_tracker_only.sh {inputs}_v2_{seq} {out} 1.0 -- ' + ' '.join(CONFIGS[cfg])
    subprocess.run(['docker', 'exec', CONTAINER, 'bash', '-c', cmd], check=True, capture_output=True)
    return REPLAY / out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--inputs', default='r9a')
    ap.add_argument('--configs', nargs='+', default=list(CONFIGS))
    args = ap.parse_args()
    res = REPO / 'eval' / 'results' / f'own_{args.inputs}'
    S = json.loads((res / 'summary.json').read_text())
    A = json.loads((res / 'alignment_v2.json').read_text())
    base = dict(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'])
    out = {}
    for cfg in args.configs:
        conf, n, tot = {}, None, {lvl: dict(tp=0, fp=0, fn=0, idtp=0, num_gt=0, num_pred=0, frames=0)
                                  for lvl in ('dynamic_only', 'merged')}
        for s in SEQS:
            bag = run(cfg, args.inputs, s)
            p, _, _ = load_obstacles(bag)
            pred = tracks_to_mocap(p, np.array(A['T_mocap_odom'][s]), A['delta'][s])
            gt = read_tracks(res / s / 'gt' / 'tracks.csv')
            e = evaluate(gt, pred, EvalConfig(label_mode='full', **base)).to_dict()
            for g, row in e['confusion'].items():
                for k, v in row.items():
                    conf.setdefault(g, {}).setdefault(k, 0)
                    conf[g][k] += v
            for lvl in tot:
                e2 = evaluate(gt, pred, EvalConfig(label_mode=lvl, **base)).to_dict()
                for k in tot[lvl]:
                    tot[lvl][k] += e2['num_frames'] if k == 'frames' else e2[k]
            fused = tracks_to_mocap(se.load_fused(REPLAY / f'{args.inputs}_v2_{s}'), A['T_mocap_odom'][s], A['delta'][s])
            ns = se.count_seq(gt, pred, fused, EvalConfig(**base), 1.0)
            n = ns if n is None else {k: n[k] + v for k, v in ns.items()}
            print(f'{cfg} {s} done', flush=True)
        st = se.scores(n)
        r = dict(person_scores(conf), prec_S=st['prec_S'], recall_S=st['recall_S'], f1_S=st['f1_S'],
                 false_dyn_S_per_min=st['false_dyn_per_min'])
        for lvl, c in tot.items():
            minutes = c['frames'] / 10.0 / 60.0
            r[f'{lvl}_recall'] = c['tp'] / c['num_gt']
            r[f'{lvl}_fa_per_min'] = c['fp'] / minutes
            r[f'{lvl}_IDF1'] = 2 * c['idtp'] / (c['num_gt'] + c['num_pred'])
        out[cfg] = r
    (REPO / 'eval' / 'results' / f'ablation_semantics_{args.inputs}.json').write_text(json.dumps(out, indent=1))
    keys = [('f1_D', 'F1 D'), ('prec_D', 'P D'), ('recall_D', 'R D'), ('f1_PD', 'F1 PD'), ('recall_PD', 'R PD'),
            ('false_dyn_S_per_min', 'S false/min'), ('dynamic_only_fa_per_min', 'FA/min dyn'),
            ('merged_recall', 'R merged'), ('merged_IDF1', 'IDF1')]
    print('config        ' + '  '.join(f'{h:>11s}' for _, h in keys))
    for cfg, r in out.items():
        print(f'{cfg:12s}  ' + '  '.join(f'{r[k]:11.2f}' if 'min' in k else f'{100 * r[k]:10.1f}%' for k, _ in keys))


if __name__ == '__main__':
    main()
