"""Tracking + dynamic-state metrics on a fixed evaluation grid.

Every method is sampled on the SAME time grid (``rate_hz`` over the GT's time
span), so a method that publishes faster or slower is neither rewarded nor
penalised for it: at each grid time the latest prediction frame no older than
``max_pred_age`` is used (none -> the method reported nothing at that time).
GT tracks are linearly interpolated onto the grid; their state is held from the
previous GT sample.

Per grid frame, GT and predictions are matched one-to-one (Hungarian) on 2D
centre distance gated at ``match_dist``. From these matches:

- CLEAR MOT (Bernardin & Stiefelhagen 2008): MOTA, MOTP [m], FP, FN, IDSW.
- IDF1 (Ristani et al. 2016): global GT-id <-> pred-id assignment over all
  frames, pairs within ``match_dist`` count as co-occurrences.
- State: confusion matrix (GT state x predicted state, plus 'missed' for
  unmatched GT), per-state recall P(pred state == GT state | GT state),
  precision/F1 on matched pairs, multiclass MCC on matched pairs.
- Transition latency: for each GT state change that then holds for at least
  ``min_hold`` s, the time until the matched prediction first reports the new
  state (censored after ``max_latency`` s or at the next GT transition).

Grid frames where a GT track is temporarily missing inside its own time span
(e.g. a mocap marker dropout) are skipped entirely when ``skip_missing_gt``:
otherwise a correct detection there would be counted as a false positive.
With ``roi = (xmin, xmax, ymin, ymax)`` (GT frame), predictions outside it are
ignored, e.g. outside the motion-capture volume where no GT can exist.

``label_mode`` picks the question the evaluation answers (same outputs, three readings):

- ``full``          D / PD / S as labelled (this system's own task).
- ``dynamic_only``  "does it flag what MOVES?" — GT positives are only the D phases, predictions only the D
                    ones; a standing person (GT PD) labelled D is a false positive, labelled PD costs nothing.
                    Comparable with moving-object segmentation (Dynablox, M-detector, LV-DOT dynamic).
- ``merged``        "does it find and keep people / movable objects, moving or not?" — D and PD are one label
                    on both sides. Comparable with person/object trackers (AB3DMOT, JRDB-style GT).

Only predictions whose state is in ``pred_states`` are considered objects of
interest (default: dynamic + potentially dynamic; a method's 'static' tracks
are ignored, as the task is detecting dynamic / potentially dynamic obstacles).
"""

from dataclasses import dataclass, field, asdict

import numpy as np
from scipy.optimize import linear_sum_assignment

from .tracks import DYNAMIC, POTENTIALLY_DYNAMIC, STATIC, STATES, objects

MISSED = 'missed'


@dataclass
class EvalConfig:
    rate_hz: float = 10.0
    match_dist: float = 1.0        # [m] 2D centre-distance gate
    max_pred_age: float = 0.25     # [s] how long a prediction frame stays valid
    max_gt_gap: float = 0.3        # [s] GT samples further apart are not interpolated across
    pred_states: tuple = (DYNAMIC, POTENTIALLY_DYNAMIC)
    min_hold: float = 1.0          # [s] a GT transition must hold this long to be scored
    max_latency: float = 5.0       # [s] latency censoring horizon
    t_start: float = None          # [s] relative to the first GT sample, optional
    t_end: float = None
    skip_missing_gt: bool = True   # skip frames where a GT track has a gap inside its span
    roi: tuple = None              # (xmin, xmax, ymin, ymax) in GT frame; predictions outside ignored
    label_mode: str = 'full'       # 'full' | 'dynamic_only' | 'merged' — see the module docstring


@dataclass
class EvalResult:
    num_frames: int = 0
    num_frames_skipped: int = 0
    num_gt: int = 0
    num_pred: int = 0
    tp: int = 0
    fp: int = 0
    fn: int = 0
    idsw: int = 0
    mota: float = float('nan')
    motp: float = float('nan')
    idtp: int = 0
    idf1: float = float('nan')
    confusion: dict = field(default_factory=dict)        # gt_state -> {pred_state|missed: count}
    state_recall: dict = field(default_factory=dict)     # gt_state -> P(correct label | gt state)
    state_precision: dict = field(default_factory=dict)  # pred_state -> on matched pairs
    state_f1: dict = field(default_factory=dict)
    mcc: float = float('nan')
    latencies: list = field(default_factory=list)        # dicts: gt_id, t, from, to, latency (nan = censored)
    latency_mean: dict = field(default_factory=dict)     # 'to_dynamic' / 'to_potentially_dynamic' -> mean [s]
    latency_censored: dict = field(default_factory=dict)

    def to_dict(self):
        return asdict(self)


# ── sampling ────────────────────────────────────────────────────────────────

def make_grid(gt, cfg):
    t = gt['t_ns'].to_numpy()
    t0, t1 = t.min() / 1e9, t.max() / 1e9
    if cfg.t_start is not None:
        t0 = t0 + cfg.t_start
    if cfg.t_end is not None:
        t1 = min(t1, t.min() / 1e9 + cfg.t_end)
    return np.arange(t0, t1 + 1e-9, 1.0 / cfg.rate_hz)


def sample_gt(gt, grid, cfg):
    """-> list (per grid frame) of list of (gt_id, x, y, state)."""
    out = [[] for _ in grid]
    for gid, g in objects(gt).groupby('track_id'):
        g = g.sort_values('t_ns')
        ts = g['t_ns'].to_numpy() / 1e9
        xs, ys, st = g['x'].to_numpy(), g['y'].to_numpy(), g['state'].to_numpy()
        idx = np.searchsorted(ts, grid, side='right') - 1     # previous sample
        for k, (t, i) in enumerate(zip(grid, idx)):
            if i < 0:
                continue
            if i == len(ts) - 1:
                if t - ts[i] > 1e-6:
                    continue
                out[k].append((int(gid), xs[i], ys[i], st[i]))
                continue
            if ts[i + 1] - ts[i] > cfg.max_gt_gap:
                continue
            a = (t - ts[i]) / (ts[i + 1] - ts[i])
            out[k].append((int(gid), xs[i] + a * (xs[i + 1] - xs[i]), ys[i] + a * (ys[i + 1] - ys[i]), st[i]))
    return out


def sample_pred(pred, grid, cfg):
    """-> list (per grid frame) of list of (pred_id, x, y, state)."""
    out = [[] for _ in grid]
    if pred.empty:
        return out
    ft = np.unique(pred['t_ns'].to_numpy())
    obj = objects(pred)
    obj = obj[obj['state'].isin(cfg.pred_states)]
    if cfg.roi is not None:
        xmin, xmax, ymin, ymax = cfg.roi
        obj = obj[(obj['x'] >= xmin) & (obj['x'] <= xmax) & (obj['y'] >= ymin) & (obj['y'] <= ymax)]
    by_t = {t: list(zip(g['track_id'].astype(int), g['x'], g['y'], g['state']))
            for t, g in obj.groupby('t_ns')}
    ft_s = ft / 1e9
    idx = np.searchsorted(ft_s, grid, side='right') - 1
    for k, (t, i) in enumerate(zip(grid, idx)):
        if i < 0 or t - ft_s[i] > cfg.max_pred_age:
            continue
        out[k] = by_t.get(ft[i], [])
    return out


# ── evaluation ──────────────────────────────────────────────────────────────

def _match(gts, preds, gate):
    if not gts or not preds:
        return [], np.zeros((len(gts), len(preds)))
    g = np.array([[x, y] for _, x, y, _ in gts])
    p = np.array([[x, y] for _, x, y, _ in preds])
    d = np.linalg.norm(g[:, None, :] - p[None, :, :], axis=2)
    cost = np.where(d <= gate, d, 1e6)
    r, c = linear_sum_assignment(cost)
    return [(i, j, d[i, j]) for i, j in zip(r, c) if d[i, j] <= gate], d


def _mcc(conf_matched, labels):
    C = np.array([[conf_matched[a].get(b, 0) for b in labels] for a in labels], dtype=float)
    n = C.sum()
    if n == 0:
        return float('nan')
    tk, pk = C.sum(axis=1), C.sum(axis=0)
    c = np.trace(C)
    den = np.sqrt((n * n - (pk * pk).sum()) * (n * n - (tk * tk).sum()))
    return float((c * n - (tk * pk).sum()) / den) if den > 0 else float('nan')


def evaluate(gt, pred, cfg=None):
    cfg = cfg or EvalConfig()
    grid = make_grid(gt, cfg)
    G = sample_gt(gt, grid, cfg)
    P = sample_pred(pred, grid, cfg)
    if cfg.skip_missing_gt:
        spans = {gid: (g['t_ns'].min() / 1e9, g['t_ns'].max() / 1e9)
                 for gid, g in objects(gt).groupby('track_id')}
        keep = []
        for k, t in enumerate(grid):
            present = {g[0] for g in G[k]}
            keep.append(all(gid in present for gid, (a, b) in spans.items() if a <= t <= b))
        keep = np.array(keep, dtype=bool)
    else:
        keep = np.ones(len(grid), dtype=bool)
    res = EvalResult(num_frames=int(keep.sum()), num_frames_skipped=int((~keep).sum()))
    grid = grid[keep]
    G = [g for g, k in zip(G, keep) if k]
    P = [p for p, k in zip(P, keep) if k]
    G_states = G  # original GT states, for transition latencies
    # Relabel AFTER the dropout skip above, so that e.g. a GT person's PD phase in dynamic_only mode is a
    # legitimate "nothing to flag here" frame, not a frame skipped as if the motion capture had lost them.
    if cfg.label_mode == 'dynamic_only':
        G = [[g for g in gs if g[3] == DYNAMIC] for gs in G]
        P = [[p for p in ps if p[3] == DYNAMIC] for ps in P]
    elif cfg.label_mode == 'merged':
        G = [[(i, x, y, DYNAMIC) for i, x, y, _ in gs] for gs in G]
        P = [[(i, x, y, DYNAMIC) for i, x, y, _ in ps] for ps in P]
    elif cfg.label_mode != 'full':
        raise ValueError(f'unknown label_mode {cfg.label_mode!r}')

    labels = list(STATES)
    conf = {a: {b: 0 for b in labels + [MISSED]} for a in labels}
    last_match = {}                      # gt_id -> pred_id (CLEAR MOT id-switch bookkeeping)
    cooc = {}                            # (gt_id, pred_id) -> frames within gate (IDF1)
    gt_count, pred_count = {}, {}
    dist_sum = 0.0
    matched_state = []                   # per frame: {gt_id: pred_state}

    for gts, preds in zip(G, P):
        res.num_gt += len(gts)
        res.num_pred += len(preds)
        for gid, *_ in gts:
            gt_count[gid] = gt_count.get(gid, 0) + 1
        for pid, *_ in preds:
            pred_count[pid] = pred_count.get(pid, 0) + 1
        m, d = _match(gts, preds, cfg.match_dist)
        for i, j in zip(*np.nonzero(d <= cfg.match_dist)):
            key = (gts[i][0], preds[j][0])
            cooc[key] = cooc.get(key, 0) + 1
        frame_state = {}
        matched_g = set()
        for i, j, dist in m:
            gid, pid = gts[i][0], preds[j][0]
            res.tp += 1
            dist_sum += dist
            if gid in last_match and last_match[gid] != pid:
                res.idsw += 1
            last_match[gid] = pid
            conf[gts[i][3]][preds[j][3]] += 1
            frame_state[gid] = preds[j][3]
            matched_g.add(i)
        for i, g in enumerate(gts):
            if i not in matched_g:
                conf[g[3]][MISSED] += 1
        matched_state.append(frame_state)
        res.fn += len(gts) - len(m)
        res.fp += len(preds) - len(m)

    if res.num_gt:
        res.mota = 1.0 - (res.fn + res.fp + res.idsw) / res.num_gt
    if res.tp:
        res.motp = dist_sum / res.tp

    # IDF1: maximise co-occurring frames under a one-to-one id assignment
    gids, pids = sorted(gt_count), sorted(pred_count)
    if gids and pids:
        W = np.array([[cooc.get((g, p), 0) for p in pids] for g in gids], dtype=float)
        r, c = linear_sum_assignment(-W)
        res.idtp = int(W[r, c].sum())
    denom = res.num_gt + res.num_pred
    if denom:
        res.idf1 = 2.0 * res.idtp / denom

    # state metrics
    res.confusion = conf
    for s in labels:
        n = sum(conf[s].values())
        res.state_recall[s] = conf[s][s] / n if n else float('nan')
        col = sum(conf[a][s] for a in labels)
        row = sum(conf[s][b] for b in labels)
        prec = conf[s][s] / col if col else float('nan')
        rec_m = conf[s][s] / row if row else float('nan')
        res.state_precision[s] = prec
        res.state_f1[s] = (2 * prec * rec_m / (prec + rec_m)
                           if col and row and (prec + rec_m) > 0 else float('nan'))
    res.mcc = _mcc(conf, labels)

    res.latencies = _transition_latencies(grid, G_states, matched_state, cfg) if cfg.label_mode != 'merged' else []
    for target in (DYNAMIC, POTENTIALLY_DYNAMIC):
        ls = [l['latency'] for l in res.latencies if l['to'] == target]
        ok = [x for x in ls if np.isfinite(x)]
        res.latency_mean[f'to_{target}'] = float(np.mean(ok)) if ok else float('nan')
        res.latency_censored[f'to_{target}'] = f'{len(ls) - len(ok)}/{len(ls)}'
    return res


def _transition_latencies(grid, G, matched_state, cfg):
    # per GT id: state on the grid (None where absent)
    series = {}
    for k, gts in enumerate(G):
        for gid, _, _, st in gts:
            series.setdefault(gid, [None] * len(grid))[k] = st
    out = []
    hold_n = int(round(cfg.min_hold * cfg.rate_hz))
    for gid, s in series.items():
        changes = [k for k in range(1, len(s)) if s[k] is not None and s[k - 1] is not None and s[k] != s[k - 1]]
        for n, k in enumerate(changes):
            new = s[k]
            if new not in (DYNAMIC, POTENTIALLY_DYNAMIC):
                continue
            end_hold = k + hold_n
            if end_hold > len(s) or any(x != new for x in s[k:end_hold]):
                continue
            horizon = min(len(grid), k + int(round(cfg.max_latency * cfg.rate_hz)) + 1)
            if n + 1 < len(changes):
                horizon = min(horizon, changes[n + 1])
            lat = float('nan')
            for j in range(k, horizon):
                if matched_state[j].get(gid) == new:
                    lat = grid[j] - grid[k]
                    break
            out.append({'gt_id': gid, 't': float(grid[k]), 'from': s[k - 1], 'to': new, 'latency': lat})
    return out
