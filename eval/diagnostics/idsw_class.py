#!/usr/bin/env python3
"""ID switches per class group (21/11, chain D request): persons / vehicles / bikes, all annotated agents and moving only.
Same GT, ignore regions and prediction filtering as coda_eval.py e3 (--ignore-undefined), same evaluator (harness.metrics
evaluate, 1 m gate). For each class group the GT is restricted to that group's tracks (empty-frame rows kept) and evaluated
against ALL predictions: IDSW then counts only switches on that group's GT tracks (the assignment can differ marginally from
the full evaluation since other GT objects no longer compete). Also prints GT rows and TP per group, summed over windows.
usage: idsw_class.py [--frames testing] --tags TAG [TAG ...] [--windows ...]    (runs bags/coda/runs/<TAG>_v2_coda<c>)"""
import argparse, json, re, sys, collections
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402
from harness.metrics import EvalConfig, evaluate   # noqa: E402

GROUP = {'Pedestrian': 'persons'}
for c in ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck', 'Bus', 'Truck', 'Van']: GROUP[c] = 'vehicles'
for c in ['Bike', 'Scooter', 'Motorcycle']: GROUP[c] = 'bikes'
ap = argparse.ArgumentParser()
ap.add_argument('--tags', nargs='+', required=True)
ap.add_argument('--windows', nargs='+', default='w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1'.split())
ap.add_argument('--frames', default='all')
ap.add_argument('--coda-root', default='/mnt/nas-imperatore/datasets_SINS/CODa')
a = ap.parse_args()
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root=a.coda_root, ignore_gt=None, topic=None,
                        method='sins', ignore_box=False)
cache = {}
def window(c):
    if c not in cache:
        odom = ce.load_odometry(B / f'coda{c}_odom')
        gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
        import pandas as pd   # the evaluator's GT class column is normalised ('person'): the CODa class comes from the CSV
        raw = pd.read_csv(B / f'coda{c}_gt.csv')
        grp = gt.track_id.map(raw[['track_id', 'class']].drop_duplicates('track_id').set_index('track_id')['class']).map(GROUP)
        ft = None
        if a.frames != 'all':
            info = json.load(open(B / f'coda{c}_gt.json'))
            meta = json.load(open(Path(a.coda_root) / 'metadata' / f"{info['seq']}.json"))['ObjectTracking'][a.frames]
            split = {int(re.findall(r'_(\d+)\.json$', f)[0]) for f in meta}
            ft = [t / 1e9 for f, t in zip(info['annotated_frames'], info['annotated_t_ns']) if int(f) in split]
        cache[c] = (odom, gt, grp, ign, frames, ft)
    return cache[c]
print(f'frames {a.frames}: ID switches per class group (sum over {len(a.windows)} windows), all / moving; GT rows; TP')
for tag in a.tags:
    S = collections.Counter()
    for w in a.windows:
        c = w if '_' in w else f'0_{w}'
        run = B / 'runs' / f'{tag}_v2_coda{c}'
        if not run.exists(): print('missing', run, file=sys.stderr); continue
        odom, gt, grp, ign, frames, ft = window(c)
        pred, _, _ = ce.load_obstacles(str(run), None)
        pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
        for g in ('persons', 'vehicles', 'bikes'):
            G = gt[(gt.track_id < 0) | (grp == g)]
            for mode in ('full', 'dynamic_only'):
                r = evaluate(G, pred, EvalConfig(match_dist=1.0, label_mode=mode, skip_missing_gt=False, frame_times=ft)).to_dict()
                S[(g, mode, 'idsw')] += r['idsw']; S[(g, mode, 'gt')] += r['num_gt']; S[(g, mode, 'tp')] += r['tp']
    print(f'{tag:7s} ' + ' | '.join(f"{g} IDSW {S[(g,'full','idsw')]}/{S[(g,'dynamic_only','idsw')]} GT {S[(g,'full','gt')]}/{S[(g,'dynamic_only','gt')]} "
                                    f"TP {S[(g,'full','tp')]}/{S[(g,'dynamic_only','tp')]}" for g in ('persons', 'vehicles', 'bikes')), flush=True)
