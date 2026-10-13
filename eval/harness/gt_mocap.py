"""Motion-capture ground truth: GT tracks with derived dynamic state, and the
mocap <-> odom alignment needed to compare them with a detector's output.

GT state (BENCHMARK_PLAN.md, "Etichetta dinamico"): a body is DYNAMIC at t if
its horizontal speed, as the centred difference over [t - w, t + w] (w = 0.5 s
by default, i.e. smoothed over a 1 s window), exceeds ``dyn_speed`` (0.3 m/s by
default, swept in the sensitivity analysis); otherwise it is POTENTIALLY_DYNAMIC
(every GT body here is a person).

Alignment model (SE(2), per sequence k):

    T_mocap_JO(t + delta) = T_mocap_odom[k] . T_odom_base(t) . T_base_JO

- T_odom_base(t): the robot's odometry (the same /odometry/filtered every method consumes)
- T_mocap_JO:     the robot's own rigid body in the mocap system
- T_base_JO:      where that rigid body sits on the robot: one constant for all sequences
- delta:          mocap clock - robot clock

With the robot moving, T_mocap_odom, delta and T_base_JO are all observable; they
are estimated jointly over every moving sequence (T_base_JO shared). With the
robot standing still, T_mocap_odom follows from the robot's mocap pose and the
already-estimated T_base_JO, and delta is taken from the moving sequences.
The alignment uses ONLY robot poses, never the detector output, so it cannot
favour any method.
"""

import numpy as np
from scipy.optimize import least_squares

from .tracks import DYNAMIC, POTENTIALLY_DYNAMIC, make_tracks


# ── SE(2) helpers ───────────────────────────────────────────────────────────

def se2(x, y, th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def se2_params(T):
    return np.array([T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def interp_pose(traj, t):
    """traj [t, x, y, yaw] -> poses at t (NaN outside the span)."""
    x = np.interp(t, traj[:, 0], traj[:, 1], left=np.nan, right=np.nan)
    y = np.interp(t, traj[:, 0], traj[:, 2], left=np.nan, right=np.nan)
    yaw = np.interp(t, traj[:, 0], np.unwrap(traj[:, 3]), left=np.nan, right=np.nan)
    return np.stack([x, y, yaw], axis=1)


def compose(a, b):
    """a, b: (..., 3) SE(2) params -> a . b"""
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([a[..., 0] + c * b[..., 0] - s * b[..., 1],
                     a[..., 1] + s * b[..., 0] + c * b[..., 1],
                     a[..., 2] + b[..., 2]], axis=-1)


def inverse(a):
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([-(c * a[..., 0] + s * a[..., 1]), s * a[..., 0] - c * a[..., 1], -a[..., 2]], axis=-1)


# ── GT tracks ───────────────────────────────────────────────────────────────

def body_speed(body, half_window=0.5, max_gap=0.3):
    """Centred-difference horizontal speed over [t-w, t+w] at each sample (NaN where undefined)."""
    t = body[:, 0]
    tp, tm = t + half_window, t - half_window
    xp, yp = np.interp(tp, t, body[:, 1]), np.interp(tp, t, body[:, 2])
    xm, ym = np.interp(tm, t, body[:, 1]), np.interp(tm, t, body[:, 2])
    v = np.hypot(xp - xm, yp - ym) / (2 * half_window)
    # undefined if the window leaves the data or spans a tracking dropout
    gaps = np.diff(t) > max_gap
    gap_t0, gap_t1 = t[:-1][gaps], t[1:][gaps]
    bad = (tm < t[0]) | (tp > t[-1])
    for a, b in zip(gap_t0, gap_t1):
        bad |= (tm < b) & (tp > a)
    v[bad] = np.nan
    return v


def gt_tracks(bodies, dyn_speed=0.3, half_window=0.5, max_gap=0.3, cls='person'):
    """{name: [t, x, y, z, yaw]} -> unified GT tracks (mocap frame); track_id = index in sorted names.

    Samples whose speed is undefined (sequence edges, dropouts) are dropped, so the
    evaluator treats them like missing GT."""
    rows, ids = [], {}
    for gid, name in enumerate(sorted(bodies)):
        b = bodies[name]
        ids[gid] = name
        v = body_speed(b, half_window, max_gap)
        for (t, x, y, z, yaw), s in zip(b, v):
            if not np.isfinite(s):
                continue
            rows.append(dict(t_ns=int(round(t * 1e9)), track_id=gid, x=x, y=y, z=z, yaw=yaw,
                             state=DYNAMIC if s > dyn_speed else POTENTIALLY_DYNAMIC,
                             score=s, **{'class': cls}))
    return make_tracks(rows), ids


# ── alignment ───────────────────────────────────────────────────────────────

def _residuals(mo, delta, bJ, odom, jo, w_yaw=0.5):
    pred = compose(compose(np.broadcast_to(mo, (len(jo), 3)), interp_pose(odom, jo[:, 0] - delta)),
                   np.broadcast_to(bJ, (len(jo), 3)))
    r = np.concatenate([pred[:, 0] - jo[:, 1], pred[:, 1] - jo[:, 2], w_yaw * wrap(pred[:, 2] - jo[:, 4])])
    return np.nan_to_num(r)  # samples outside the odometry span contribute nothing


def _init_mo(odom, jo, delta, bJ):
    """Closed-form-ish init: align first JO pose."""
    p_ob = interp_pose(odom, jo[:, 0] - delta)
    ok = np.isfinite(p_ob).all(axis=1)
    i = np.flatnonzero(ok)[0]
    return compose(compose(jo[i, [1, 2, 4]], inverse(bJ)), inverse(p_ob[i]))


def is_moving(odom, min_extent=0.5):
    return max(np.ptp(odom[:, 1]), np.ptp(odom[:, 2])) >= min_extent


def yaw_range(traj, yaw_col):
    return float(np.ptp(np.unwrap(traj[:, yaw_col])))


def estimate_delta(odom, jo, delta_grid=np.arange(-2.0, 2.0001, 0.01), rate=20.0):
    """Clock offset from the robot's SPEED profiles (rotation/offset invariant, so it
    stays observable for pure straight-line motion): argmax of their correlation."""
    def speed(traj):
        t = np.arange(traj[0, 0], traj[-1, 0], 1.0 / rate)
        x, y = np.interp(t, traj[:, 0], traj[:, 1]), np.interp(t, traj[:, 0], traj[:, 2])
        k = int(rate * 0.25)
        v = np.hypot(np.gradient(x), np.gradient(y)) * rate
        return t, np.convolve(v, np.ones(k) / k, 'same')
    to, vo = speed(odom)
    tj, vj = speed(jo)
    best, best_d = -np.inf, 0.0
    for d in delta_grid:
        v = np.interp(tj - d, to, vo, left=np.nan, right=np.nan)
        ok = np.isfinite(v)
        if ok.sum() < rate * 5:
            continue
        a, b = v[ok] - v[ok].mean(), vj[ok] - vj[ok].mean()
        c = (a @ b) / np.sqrt((a @ a) * (b @ b) + 1e-12)
        if c > best:
            best, best_d = c, d
    return float(best_d), float(best)


def fit_alignment(seqs, t_base_jo_prior=(0.0, 0.0), min_yaw_range=0.3):
    """seqs: {name: (odom [t,x,y,yaw], jo [t,x,y,z,yaw])}.

    - delta: per moving sequence, initialised from speed-profile correlation and refined
      in the pose least squares; median of those for the stationary sequences.
    - T_base_JO yaw: shared, from the moving sequences (direction of motion vs body yaw).
    - T_base_JO translation: estimated only if some moving sequence rotates by at least
      ``min_yaw_range`` rad; otherwise it is NOT observable (it is absorbed by each
      sequence's T_mocap_odom translation) and is fixed to ``t_base_jo_prior``
      (e.g. measured on the robot) -- flagged in the result as 't_base_JO_observable'.

    Returns dict with 'T_base_JO', 'delta' (per seq + 'median'), 'T_mocap_odom' per
    seq, residual RMS per seq, 'moving', 't_base_JO_observable', 'delta_corr'."""
    moving = [k for k, (o, _) in seqs.items() if is_moving(o)]
    if not moving:
        raise ValueError('need at least one sequence with the robot moving')
    observable = any(yaw_range(seqs[k][0], 3) >= min_yaw_range for k in moving)

    out = {'delta': {}, 'delta_corr': {}, 'T_mocap_odom': {}, 'rms_xy': {}, 'rms_yaw': {},
           'moving': moving, 't_base_JO_observable': observable}
    for k in moving:
        out['delta'][k], out['delta_corr'][k] = estimate_delta(*seqs[k])
    out['delta']['median'] = float(np.median([out['delta'][k] for k in moving]))

    # joint LSQ over the moving sequences: per-seq T_mocap_odom, shared T_base_JO
    # (yaw always; translation only if observable)
    def unpack(p):
        if observable:
            bJ = p[:3]
            rest = p[3:]
        else:
            bJ = np.array([t_base_jo_prior[0], t_base_jo_prior[1], p[0]])
            rest = p[1:]
        return bJ, [rest[4 * i: 4 * i + 3] for i in range(len(moving))], [rest[4 * i + 3] for i in range(len(moving))]

    def joint(p):
        bJ, mos, ds = unpack(p)
        return np.concatenate([_residuals(mo, d, bJ, *seqs[k]) for mo, d, k in zip(mos, ds, moving)])

    best = None
    for yaw0 in np.linspace(-np.pi, np.pi, 8, endpoint=False):   # body yaw is multimodal: multi-start
        bJ = np.array([t_base_jo_prior[0], t_base_jo_prior[1], yaw0])
        x = np.concatenate([bJ if observable else bJ[2:]] +
                           [np.append(_init_mo(seqs[k][0], seqs[k][1], out['delta'][k], bJ), out['delta'][k])
                            for k in moving])
        sol = least_squares(joint, x, loss='huber', f_scale=0.1)
        if best is None or sol.cost < best.cost:
            best = sol
    bJ, mos, ds = unpack(best.x)
    bJ = bJ.copy()
    bJ[2] = wrap(bJ[2])
    out['T_base_JO'] = bJ
    for mo, d, k in zip(mos, ds, moving):
        out['T_mocap_odom'][k] = mo
        out['delta'][k] = float(d)
    out['delta']['median'] = float(np.median([out['delta'][k] for k in moving]))

    # stationary sequences: T_mocap_odom from the robot's own (averaged) pose
    for k, (odom, jo) in seqs.items():
        if k in moving:
            continue
        d = out['delta']['median']
        p_ob = interp_pose(odom, jo[:, 0] - d)
        ok = np.isfinite(p_ob).all(axis=1)
        mo = compose(compose(jo[ok][:, [1, 2, 4]], inverse(np.broadcast_to(bJ, (ok.sum(), 3)))), inverse(p_ob[ok]))
        out['T_mocap_odom'][k] = np.array([np.median(mo[:, 0]), np.median(mo[:, 1]),
                                           np.arctan2(np.median(np.sin(mo[:, 2])), np.median(np.cos(mo[:, 2])))])
        out['delta'][k] = d

    for k, (odom, jo) in seqs.items():
        r = _residuals(out['T_mocap_odom'][k], out['delta'][k], bJ, odom, jo, w_yaw=1.0).reshape(3, -1)
        valid = np.isfinite(interp_pose(odom, jo[:, 0] - out['delta'][k])).all(axis=1)
        out['rms_xy'][k] = float(np.sqrt(np.mean(r[0][valid] ** 2 + r[1][valid] ** 2)))
        out['rms_yaw'][k] = float(np.sqrt(np.mean(r[2][valid] ** 2)))
    return out


def tracks_to_mocap(tracks, T_mocap_odom, delta):
    """Map unified tracks from odom frame/robot clock into mocap frame/clock (x, y, yaw, vx, vy)."""
    df = tracks.copy()
    real = df['track_id'] >= 0
    p = np.stack([df.loc[real, 'x'], df.loc[real, 'y'], df.loc[real, 'yaw'].fillna(0.0)], axis=1)
    q = compose(np.broadcast_to(T_mocap_odom, p.shape), p)
    df.loc[real, 'x'], df.loc[real, 'y'], df.loc[real, 'yaw'] = q[:, 0], q[:, 1], q[:, 2]
    c, s = np.cos(T_mocap_odom[2]), np.sin(T_mocap_odom[2])
    vx, vy = df.loc[real, 'vx'].to_numpy(), df.loc[real, 'vy'].to_numpy()
    df.loc[real, 'vx'], df.loc[real, 'vy'] = c * vx - s * vy, s * vx + c * vy
    df['t_ns'] = df['t_ns'] + int(round(delta * 1e9))
    return df
