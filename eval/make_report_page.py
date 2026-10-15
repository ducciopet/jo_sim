#!/usr/bin/env python3
"""
Self-contained HTML report of a repeated campaign (campaign_report.py output) + top-down videos (make_video.py).

Reads eval/results/own_<C>_reps.json and the videos in eval/videos/<C>/, writes eval/report_<C>/index.html
(data embedded as JSON, charts drawn in the page as SVG) and copies the videos next to it, ready to publish.

Usage
  python3 eval/make_report_page.py --campaign r8
  python3 eval/make_report_page.py --campaign r9 --compare r8 --jobs-total 18
    (--compare adds that campaign's v2 as 'v2r8' and its greedy ablation as 'v2g', next to this campaign's v1/v2)
"""

import argparse
import datetime
import json
import shutil
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SEQ_INFO = {
    '20260519_1942': 'robot fermo, persona ferma',
    '20260519_1945': 'robot fermo, persona cammina',
    '20260519_1947': 'robot fermo, cammina e si ferma',
    '20260519_1949': 'robot avanti/indietro, persona ferma',
    '20260519_1950': 'robot avanti/indietro, mista',
    '20260519_1952': 'robot avanti/indietro, persona cammina',
}
# full-pipeline MOTA of v2 along the development (eval/STATUS.md); v1 reference = corrected benchmark
HISTORY = [
    ('c2ca9fd', 'duplicati corretti', 79.6), ('7a91090', 'messa a punto', 87.0),
    ('889bae0', 'passaggio di classe', 91.5), ('5677ac5', 'muri limitati', 93.1),
    ('52a5573', 'static confirm 5', 93.3), ('2289910', 'r7: PD dal moto, YOLO warm-up', 97.4),
]


def git(path, *a):
    return subprocess.run(['git', '-C', str(path), *a], capture_output=True, text=True).stdout.strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r8')
    ap.add_argument('--compare', default=None, help='earlier campaign whose v2 (and v2g) are shown alongside')
    ap.add_argument('--jobs-total', type=int, default=54)
    args = ap.parse_args()
    c = args.campaign
    reps = json.loads((REPO / 'eval' / 'results' / f'own_{c}_reps.json').read_text())
    if args.compare:
        old = json.loads((REPO / 'eval' / 'results' / f'own_{args.compare}_reps.json').read_text())
        for lvl, ms in old.items():
            if 'v2' in ms:
                reps[lvl]['v2r8'] = ms['v2']
            if 'v2g' in ms and 'v2g' not in reps[lvl]:
                reps[lvl]['v2g'] = ms['v2g']
    # label S (eval/static_eval.py): everything that is not the person is static
    static = {}
    for camp, src, dst in ((c, 'v1', 'v1'), (c, 'v2', 'v2'), (args.compare, 'v2', 'v2r8' if args.compare else None)):
        f = REPO / 'eval' / 'results' / f'own_{camp}_static.json'
        if camp and dst and f.exists():
            js = json.loads(f.read_text())
            if src in js:
                static[dst] = js[src]
    out = REPO / 'eval' / f'report_{c}'
    (out / 'videos').mkdir(parents=True, exist_ok=True)
    vids = []
    for v in sorted((REPO / 'eval' / 'videos' / c).glob('*.mp4')):
        shutil.copy(v, out / 'videos' / v.name)
        seq = '20260519_' + v.stem.split('_')[-1]
        vids.append({'file': f'videos/{v.name}', 'seq': seq, 'info': SEQ_INFO.get(seq, '')})
    state_log = REPO / 'bags' / 'replay' / f'{c}_state.log'
    done_jobs = len({l.split(' done ', 1)[1].strip() for l in state_log.read_text().splitlines() if ' done ' in l}) \
        if state_log.exists() else 0   # a redone job counts once
    data = {
        'campaign': c, 'levels': reps, 'seq_info': SEQ_INFO, 'videos': vids,
        'history': [{'commit': h, 'label': l, 'mota': m} for h, l, m in HISTORY],
        'commits': {'v2': git(REPO / 'jo_detector_v2', 'rev-parse', '--short', 'HEAD'),
                    'v1': git(REPO / 'jo_detector', 'rev-parse', '--short', 'HEAD'),
                    'repo': git(REPO, 'rev-parse', '--short', 'HEAD')},
        'jobs_done': done_jobs, 'jobs_total': args.jobs_total, 'compare': args.compare, 'static': static,
        # hand-written findings (HTML), kept next to the results so a regenerated page keeps them
        'notes': (REPO / 'eval' / f'report_{c}_notes.html').read_text()
        if (REPO / 'eval' / f'report_{c}_notes.html').exists() else '',
        'generated': datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
    }
    html = (Path(__file__).resolve().parent / 'report_template.html').read_text()
    html = html.replace('__CAMPAIGN__', c)
    html = html.replace('/*__DATA__*/null', json.dumps(data, default=lambda x: None).replace('</', '<\\/'))
    (out / 'index.html').write_text(html)
    print('wrote', out / 'index.html', 'videos:', len(vids))


if __name__ == '__main__':
    main()
