"""The mocap<->odom alignment must recover a known T_base_JO, per-sequence
T_mocap_odom and clock offset, including for a sequence where the robot stands
still (T_mocap_odom then comes from the shared T_base_JO), and must flag the
straight-line-only case where T_base_JO's translation is not observable."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from harness.gt_mocap import (body_speed, compose, fit_alignment, gt_tracks,  # noqa: E402
                              tracks_to_mocap, wrap)
from harness.tracks import DYNAMIC, POTENTIALLY_DYNAMIC, make_tracks  # noqa: E402

T_BJ = np.array([0.12, -0.05, 0.3])
DELTA = 0.37
RNG = np.random.default_rng(0)


def robot_odom(kind, T=40.0):
    t = np.arange(0, T, 0.025)
    if kind == 'still':
        x, y, yaw = 0.02 * np.ones_like(t), -0.01 * np.ones_like(t), 0.05 * np.ones_like(t)
    else:
        straight = kind.startswith('line')
        yaw = (0.0 * t if straight else 0.4 * np.sin(0.2 * t)) + (0.3 if kind.endswith('b') else 0.0)
        v = np.clip(0.3 * np.sin(0.35 * t) + 0.1 * np.sin(1.3 * t), 0, None)   # stop-and-go
        x = np.cumsum(v * np.cos(yaw)) * 0.025
        y = np.cumsum(v * np.sin(yaw)) * 0.025
    return np.stack([1000.0 + t, x, y, yaw], axis=1)


def mocap_jo(odom, T_mo, noise=0.003):
    t_m = np.arange(odom[0, 0] + 0.5, odom[-1, 0] - 0.5, 0.1)
    p = np.stack([np.interp(t_m, odom[:, 0], odom[:, i]) for i in (1, 2, 3)], axis=1)
    jo = compose(compose(np.broadcast_to(T_mo, p.shape), p), np.broadcast_to(T_BJ, p.shape))
    jo[:, :2] += RNG.normal(0, noise, (len(jo), 2))
    return np.column_stack([t_m + DELTA, jo[:, 0], jo[:, 1], np.zeros(len(jo)), jo[:, 2]])


@pytest.fixture(scope='module')
def fit():
    T = {'a': np.array([3.0, 1.0, 0.8]), 'b': np.array([-2.0, 4.0, -1.2]), 'still': np.array([1.0, -1.0, 2.0])}
    seqs = {}
    for k, T_mo in T.items():
        o = robot_odom(k)
        seqs[k] = (o, mocap_jo(o, T_mo))
    return T, fit_alignment(seqs)


def test_base_jo(fit):
    _, f = fit
    assert np.allclose(f['T_base_JO'][:2], T_BJ[:2], atol=0.01)
    assert abs(wrap(f['T_base_JO'][2] - T_BJ[2])) < 0.01


def test_delta(fit):
    _, f = fit
    assert set(f['moving']) == {'a', 'b'}
    assert f['t_base_JO_observable']
    for k in ('a', 'b', 'still'):
        assert f['delta'][k] == pytest.approx(DELTA, abs=0.02)


def test_mocap_odom(fit):
    T, f = fit
    for k in T:
        assert np.allclose(f['T_mocap_odom'][k][:2], T[k][:2], atol=0.02), k
        assert abs(wrap(f['T_mocap_odom'][k][2] - T[k][2])) < 0.01, k
        assert f['rms_xy'][k] < 0.01


def test_straight_line_flags_unobservable_translation():
    T = {'linea': np.array([3.0, 1.0, 0.8]), 'lineb': np.array([-2.0, 4.0, -1.2])}
    seqs = {k: (o := robot_odom(k), mocap_jo(o, T_mo)) for k, T_mo in T.items()}
    f = fit_alignment(seqs, t_base_jo_prior=T_BJ[:2])      # prior = true (measured) translation
    assert not f['t_base_JO_observable']
    assert abs(wrap(f['T_base_JO'][2] - T_BJ[2])) < 0.01   # body yaw stays observable
    for k in T:
        assert f['delta'][k] == pytest.approx(DELTA, abs=0.02)
        assert np.allclose(f['T_mocap_odom'][k][:2], T[k][:2], atol=0.02), k
    # with a wrong prior the residual stays ~0 (unobservable!) but T_mocap_odom is biased by it
    g = fit_alignment(seqs, t_base_jo_prior=(0.0, 0.0))
    for k in T:
        assert g['rms_xy'][k] < 0.01
        assert np.linalg.norm(g['T_mocap_odom'][k][:2] - T[k][:2]) == pytest.approx(np.linalg.norm(T_BJ[:2]), abs=0.02)


def test_tracks_to_mocap_roundtrip(fit):
    T, _ = fit
    tr = make_tracks([dict(t_ns=10**12, track_id=4, x=1.0, y=2.0, yaw=0.1, vx=1.0, vy=0.0, state=DYNAMIC)])
    out = tracks_to_mocap(tr, T['a'], DELTA)
    exp = compose(T['a'], np.array([1.0, 2.0, 0.1]))
    assert out.loc[0, 'x'] == pytest.approx(exp[0]) and out.loc[0, 'y'] == pytest.approx(exp[1])
    assert np.hypot(out.loc[0, 'vx'], out.loc[0, 'vy']) == pytest.approx(1.0)
    assert out.loc[0, 't_ns'] == 10**12 + round(DELTA * 1e9)


def test_gt_state_from_speed():
    t = np.arange(0, 20, 0.1)
    x = np.where(t < 10, 0.5 * t, 5.0)            # 0.5 m/s then still
    body = np.column_stack([t, x, np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)])
    v = body_speed(body)
    assert np.nanmax(np.abs(v[(t > 1) & (t < 9)] - 0.5)) < 1e-6
    gt, _ = gt_tracks({'person': body}, dyn_speed=0.3)
    s = dict(zip(gt['t_ns'] // 10**8, gt['state']))
    assert s[50] == DYNAMIC and s[150] == POTENTIALLY_DYNAMIC
    assert gt['t_ns'].min() >= 0.5e9 - 1  # first 0.5 s has no centred window -> dropped
