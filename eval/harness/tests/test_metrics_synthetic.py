"""Phase-1 acceptance test (BENCHMARK_PLAN.md): known GT + predictions perturbed
in known ways; every metric must recover the injected value.

Scenario (30 s, grid 10 Hz):
  GT 1  walks along x at 1 m/s, stops for 10 s, walks again: D [0,10) PD [10,20) D [20,30]
  GT 2  stands still at (5, 5): PD throughout
Injected into the predictions:
  - +0.10 m position offset on every object            -> MOTP = 0.10
  - GT 1's pred id changes 101 -> 103 at t = 15 s       -> IDSW = 1
  - state of GT 1 lags every GT transition by 0.8 s     -> latency = 0.8 s
  - GT 2 not reported for t in [5, 7)                   -> FN = 20
  - a fake dynamic object far away for t in [2, 7)      -> FP = 50
  - a 'static' pred object (must be ignored)            -> no effect
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from harness.metrics import EvalConfig, evaluate  # noqa: E402
from harness.tracks import (DYNAMIC, POTENTIALLY_DYNAMIC, STATIC, make_tracks,  # noqa: E402
                            read_tracks, write_tracks)

T0 = 1_000_000_000_000  # arbitrary epoch offset [ns]


def gt_state1(t):
    return DYNAMIC if t < 10 or t >= 20 else POTENTIALLY_DYNAMIC


def gt_pos1(t):
    if t < 10:
        return t, 0.0
    if t < 20:
        return 10.0, 0.0
    return 10.0 + (t - 20), 0.0


def build():
    gt = []
    for k in range(3001):                                  # 100 Hz GT
        t = k / 100.0
        x, y = gt_pos1(t)
        gt.append(dict(t_ns=T0 + round(t * 1e9), track_id=1, x=x, y=y, state=gt_state1(t), **{'class': 'person'}))
        gt.append(dict(t_ns=T0 + round(t * 1e9), track_id=2, x=5.0, y=5.0, state=POTENTIALLY_DYNAMIC, **{'class': 'person'}))
    pred = []
    for k in range(301):                                   # 10 Hz predictions, 1 ms before each grid time
        t = k / 10.0
        tf = t - 0.001
        tn = T0 + round(tf * 1e9)
        x, y = gt_pos1(tf)
        lagged = gt_state1(tf - 0.8) if tf >= 0.8 else gt_state1(tf)
        pred.append(dict(t_ns=tn, track_id=101 if t < 15 else 103, x=x + 0.1, y=y, state=lagged))
        if not (5 <= t < 7):
            pred.append(dict(t_ns=tn, track_id=102, x=5.1, y=5.0, state=POTENTIALLY_DYNAMIC))
        if 2 <= t < 7:
            pred.append(dict(t_ns=tn, track_id=900, x=20.0, y=20.0, state=DYNAMIC))
        pred.append(dict(t_ns=tn, track_id=950, x=-3.0, y=0.0, state=STATIC))
    return make_tracks(gt), make_tracks(pred)


@pytest.fixture(scope='module')
def result():
    gt, pred = build()
    return evaluate(gt, pred, EvalConfig(rate_hz=10.0, match_dist=1.0))


def test_counts(result):
    assert result.num_frames == 301
    assert result.num_gt == 602
    assert result.fn == 20
    assert result.fp == 50
    assert result.idsw == 1
    assert result.mota == pytest.approx(1 - (20 + 50 + 1) / 602)


def test_motp(result):
    assert result.motp == pytest.approx(0.10, abs=0.002)


def test_idf1(result):
    # best assignment: GT1 -> 103 (151 frames), GT2 -> 102 (281 frames)
    assert result.idtp == 151 + 281
    num_pred = 301 + 281 + 50
    assert result.num_pred == num_pred
    assert result.idf1 == pytest.approx(2 * 432 / (602 + num_pred))


def test_latency(result):
    lat = {l['to']: l['latency'] for l in result.latencies}
    assert set(lat) == {DYNAMIC, POTENTIALLY_DYNAMIC}
    # 0.801 s injected lag, measured on a 0.1 s grid -> first agreeing frame at +0.9 s
    assert lat[POTENTIALLY_DYNAMIC] == pytest.approx(0.9, abs=1e-6)
    assert lat[DYNAMIC] == pytest.approx(0.9, abs=1e-6)


def test_state_confusion(result):
    c = result.confusion
    # pred frames sit 1 ms before each grid time, so the effective lag is 0.801 s:
    # GT1 keeps the previous label on grid frames t_tr + {0.0, ..., 0.8} = 9 frames
    assert c[POTENTIALLY_DYNAMIC][DYNAMIC] == 9
    assert c[DYNAMIC][POTENTIALLY_DYNAMIC] == 9
    assert c[POTENTIALLY_DYNAMIC]['missed'] == 20
    n_dyn = sum(c[DYNAMIC].values())
    assert result.state_recall[DYNAMIC] == pytest.approx((n_dyn - 9) / n_dyn)
    assert math.isnan(result.state_recall[STATIC])


def test_static_predictions_ignored():
    gt, pred = build()
    only_static = pred[pred['state'] == STATIC]
    r = evaluate(gt, only_static, EvalConfig())
    assert r.tp == 0 and r.fp == 0 and r.fn == r.num_gt


def test_csv_roundtrip(tmp_path):
    gt, pred = build()
    p = tmp_path / 'tracks.csv'
    write_tracks(pred, p)
    back = read_tracks(p)
    assert len(back) == len(pred)
    r1 = evaluate(gt, pred, EvalConfig())
    r2 = evaluate(gt, back, EvalConfig())
    assert (r1.tp, r1.fp, r1.fn, r1.idsw, r1.idtp) == (r2.tp, r2.fp, r2.fn, r2.idsw, r2.idtp)
    assert r1.motp == pytest.approx(r2.motp, abs=1e-4)


def test_matches_py_motmetrics(result):
    """Independent reference: same sampled frames through py-motmetrics."""
    mm = pytest.importorskip('motmetrics')
    from harness.metrics import make_grid, sample_gt, sample_pred
    gt, pred = build()
    cfg = EvalConfig()
    grid = make_grid(gt, cfg)
    acc = mm.MOTAccumulator(auto_id=True)
    for g, p in zip(sample_gt(gt, grid, cfg), sample_pred(pred, grid, cfg)):
        ga = np.array([[x, y] for _, x, y, _ in g]).reshape(-1, 2)
        pa = np.array([[x, y] for _, x, y, _ in p]).reshape(-1, 2)
        d = np.sqrt(mm.distances.norm2squared_matrix(ga, pa, max_d2=cfg.match_dist ** 2))
        acc.update([a[0] for a in g], [a[0] for a in p], d)
    s = mm.metrics.create().compute(acc, metrics=['mota', 'motp', 'idf1', 'num_switches'], name='x')
    assert result.mota == pytest.approx(s['mota']['x'])
    assert result.motp == pytest.approx(s['motp']['x'])
    assert result.idf1 == pytest.approx(s['idf1']['x'])
    assert result.idsw == s['num_switches']['x']


def test_gt_dropout_frames_skipped():
    gt, pred = build()
    # GT 2 drops out for t in [12, 13): those frames must not create FPs for its (correct) prediction
    t = (gt['t_ns'] - T0) / 1e9
    gt_drop = gt[~((gt['track_id'] == 2) & (t >= 12) & (t < 13))]
    r = evaluate(gt_drop, pred, EvalConfig())
    base = evaluate(gt, pred, EvalConfig())
    assert r.num_frames_skipped == 10          # grid frames 12.0..12.9 (a GT sample exists again at 13.00)
    assert r.fp == base.fp
    off = evaluate(gt_drop, pred, EvalConfig(skip_missing_gt=False))
    assert off.fp > base.fp


def test_roi_drops_far_predictions():
    gt, pred = build()
    r = evaluate(gt, pred, EvalConfig(roi=(-1, 15, -1, 6)))
    assert r.fp == 0                            # the fake object at (20, 20) is outside
