#!/usr/bin/env python3
"""
Figures for the report and the paper (PNG + PDF in eval/figures/<lang>/), from results already computed:

  timeline   per-frame label given to the person by every method, against the GT state (r9a, one sequence)
  labels     D F1, PD F1, false dynamics on static objects per method (results/labels_r9.json)
  stopgo     id kept across stop/restart, person tracked while standing (results/stopgo_r9.json)
  map        top-down view of the GLIM map at body height, per rejection variant (bags/replay/map_*), ghost
             voxels highlighted; and the person-removed / static-kept bars (results/map_r9a_*.json)
  ablation   semantics on/off in the tracker (results/ablation_semantics_r9a.json)

usage: python3 eval/make_figures.py [--lang it|en] [--only timeline map ...]
"""

import argparse
import json
import statistics
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import baseline_eval as be  # noqa: E402
from harness.metrics import EvalConfig, _match, make_grid, sample_gt, sample_pred  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
RES = REPO / 'eval' / 'results'
# state colours: the ones of the videos and report pages (D red, PD orange); "missed" a light neutral
C_D, C_PD, C_MISS, C_S = '#d62728', '#ff9f1c', '#e4e6ea', '#8a8f98'
C_ME, C_V1, C_OTHER = '#1f4e79', '#6b7785', '#b4bcc6'   # v2 = accent, everyone else = neutral
INK, INK2 = '#1d232b', '#5b6573'

T = {
    'it': dict(gt='Verità (mocap)', time='tempo [s]', D='D: si muove', PD='PD: ferma, può ripartire', miss='non trovata',
               f1D='F1 di D (persona che cammina)', f1PD='F1 di PD (persona ferma)', fdyn='Oggetti fermi presi per dinamici [al min]',
               idkept='Stesso ID quando si ferma e riparte', tracked='Persona seguita mentre è ferma',
               removed='Persona tolta dalla mappa', removed_static='Persona ferma tolta (1942, 1949)', kept='Struttura statica conservata',
               none='Nessuna rimozione', v2d='SINS, solo D', v2dpd='SINS, D+PD', v1dpd='v1, D+PD', na='n/d (nessun ID)',
               noise='rumore di GLIM', found='Persona trovata, ferma o no', abl_full='SINS completo', abl_nosem='senza semantica', abl_nosem_nopd='senza semantica né PD'),
    'en': dict(gt='Ground truth (mocap)', time='time [s]', D='D: moving', PD='PD: standing, may move', miss='missed',
               f1D='D F1 (walking person)', f1PD='PD F1 (standing person)', fdyn='Static objects flagged dynamic [per min]',
               idkept='Same ID across stop and restart', tracked='Person tracked while standing',
               removed='Person removed from the map', removed_static='Standing person removed (1942, 1949)', kept='Static structure kept',
               none='No rejection', v2d='SINS, D only', v2dpd='SINS, D+PD', v1dpd='v1, D+PD', na='n/a (no ID)',
               noise='GLIM noise', found='Person found, moving or not', abl_full='SINS full', abl_nosem='no semantics', abl_nosem_nopd='no semantics, no PD'),
}
NAMES = {'v2': 'SINS', 'v1': 'jo_detector v1', 'lvdot': 'LV-DOT', 'dynablox': 'Dynablox',
         'mdetector': 'M-detector', 'xurgbd': 'Xu et al.', 'xuyolo': 'Xu et al. + YOLO', 'unimt': 'UniMT + SimpleTrack',
         'eagermot': 'EagerMOT'}


def style():
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.edgecolor': '#c3c9d1',
                         'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2, 'axes.titlesize': 10,
                         'axes.titleweight': 'bold', 'axes.titlecolor': INK, 'axes.spines.top': False,
                         'axes.spines.right': False, 'savefig.dpi': 200, 'savefig.bbox': 'tight'})


def save(fig, out, name):
    out.mkdir(parents=True, exist_ok=True)
    for ext in ('png', 'pdf'):
        fig.savefig(out / f'{name}.{ext}')
    plt.close(fig)
    print('wrote', out / f'{name}.png')


def colour(m):
    return C_ME if m == 'v2' else C_V1 if m == 'v1' else C_OTHER


def hbars(ax, names, vals, colours, fmt, title, xmax=None, note=None):
    y = np.arange(len(names))[::-1]
    for yi, v, c in zip(y, vals, colours):
        if v is None or v != v:
            ax.text(0.01 * (xmax or 1), yi, note or 'n/a', va='center', fontsize=8, color=INK2)
            continue
        ax.barh(yi, v, height=0.62, color=c)
        ax.text(v + 0.01 * (xmax or max(x for x in vals if x == x and x is not None)), yi, fmt(v),
                va='center', fontsize=8, color=INK)
    ax.set_yticks(y, names)
    ax.set_title(title, loc='left')
    ax.set_xlim(0, xmax or 1.15 * max(x for x in vals if x is not None and x == x))
    ax.tick_params(axis='y', length=0)
    ax.grid(axis='x', color='#eceef1', lw=0.8)
    ax.set_axisbelow(True)


# ── timeline ────────────────────────────────────────────────────────────────

def person_labels(gt, pred, cfg):
    grid = make_grid(gt, cfg)
    G, P = sample_gt(gt, grid, cfg), sample_pred(pred, grid, cfg)
    gts, lab = [], []
    for g, p in zip(G, P):
        g = [x for x in g if x[0] >= 0]
        gts.append(g[0][3] if g else None)
        m, _ = _match(g, p, cfg.match_dist)
        lab.append(p[m[0][1]][3] if m else 'missed')
    return grid - grid[0], gts, lab


def fig_timeline(lang, out, seq, methods):
    t = T[lang]
    res = RES / 'own_r9a'
    S = json.loads((res / 'summary.json').read_text())
    al = json.loads((res / 'alignment_v2.json').read_text())
    cfg = EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'])
    gt = read_tracks(res / seq / 'gt' / 'tracks.csv')
    rows = []
    for m in methods:
        tt, g, lab = person_labels(gt, be.predictions(m, 'r9', 'a', seq, res, al), cfg)
        rows.append((NAMES[m], lab))
    rows.insert(0, (t['gt'], g))
    col = {'dynamic': C_D, 'potentially_dynamic': C_PD, 'missed': C_MISS, None: '#ffffff', 'static': C_S}
    fig, ax = plt.subplots(figsize=(7.2, 0.34 * len(rows) + 0.9))
    dt = tt[1] - tt[0]
    for k, (name, lab) in enumerate(rows):
        yk = len(rows) - 1 - k
        i = 0
        while i < len(lab):  # runs of the same label as one bar
            j = i
            while j + 1 < len(lab) and lab[j + 1] == lab[i]:
                j += 1
            ax.broken_barh([(tt[i], tt[j] - tt[i] + dt)], (yk - 0.38, 0.76), color=col[lab[i]], lw=0)
            i = j + 1
    ax.axhline(len(rows) - 1.5, color='#9aa3ad', lw=0.8)
    ax.set_yticks(range(len(rows))[::-1], [r[0] for r in rows])
    for lbl in ax.get_yticklabels():
        if lbl.get_text() == NAMES['v2']:
            lbl.set_fontweight('bold'); lbl.set_color(C_ME)
    ax.set_xlim(0, tt[-1] + dt)
    ax.set_xlabel(t['time'])
    ax.tick_params(axis='y', length=0)
    ax.spines['left'].set_visible(False)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=C_D, label=t['D']), Patch(color=C_PD, label=t['PD']), Patch(color=C_MISS, label=t['miss'])],
              ncol=3, loc='upper left', bbox_to_anchor=(0, 1.02 + 0.35 / len(rows)), frameon=False, fontsize=8, handlelength=1.2)
    ax.set_title(f'{seq}', loc='right', fontsize=8, color=INK2, fontweight='normal')
    save(fig, out, f'timeline_{seq[-4:]}')


# ── per-label scores ────────────────────────────────────────────────────────

def mean_over_reps(per, k):
    xs = [v[k] for v in per.values() if v[k] == v[k]]
    return statistics.fmean(xs) if xs else float('nan')


def fig_labels(lang, out):
    t = T[lang]
    L = json.loads((RES / 'labels_r9.json').read_text())
    ms = [m for m in ['v2', 'v1', 'lvdot', 'dynablox', 'mdetector', 'xurgbd', 'xuyolo', 'unimt', 'eagermot'] if m in L]
    fig, axs = plt.subplots(1, 3, figsize=(10.5, 3.0), sharey=True)
    names = [NAMES[m] for m in ms]
    cols = [colour(m) for m in ms]
    pct = lambda v: f'{100 * v:.1f}%'
    hbars(axs[0], names, [mean_over_reps(L[m], 'f1_D') for m in ms], cols, pct, t['f1D'], 1.12)
    hbars(axs[1], names, [mean_over_reps(L[m], 'f1_PD') if m in ('v1', 'v2') else float('nan') for m in ms], cols, pct,
          t['f1PD'], 1.12, note='0 (no PD)' if lang == 'en' else '0 (nessun PD)')
    hbars(axs[2], names, [mean_over_reps(L[m], 'false_dyn_per_min') for m in ms], cols, lambda v: f'{v:.1f}', t['fdyn'])
    for ax in axs[:2]:
        ax.xaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1.0))
    fig.tight_layout(w_pad=1.5)
    save(fig, out, 'labels')


def fig_stopgo(lang, out):
    t = T[lang]
    D = json.loads((RES / 'stopgo_r9.json').read_text())
    ms = [m for m in ['v2', 'v1', 'lvdot', 'dynablox', 'mdetector', 'xuyolo', 'unimt', 'eagermot'] if m in D]
    avg = lambda m, k: None if D[m]['a'][k] is None else statistics.fmean(D[m][r][k] for r in D[m])
    fig, axs = plt.subplots(1, 2, figsize=(8.0, 2.8), sharey=True)
    names, cols = [NAMES[m] for m in ms], [colour(m) for m in ms]
    pct = lambda v: f'{100 * v:.0f}%'
    hbars(axs[0], names, [avg(m, 'id_kept') for m in ms], cols, pct, t['idkept'], 1.15, note=t['na'])
    hbars(axs[1], names, [avg(m, 'tracked_standing') for m in ms], cols, pct, t['tracked'], 1.15)
    for ax in axs:
        ax.xaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1.0))
    fig.tight_layout(w_pad=1.5)
    save(fig, out, 'stopgo')


# ── map ─────────────────────────────────────────────────────────────────────

MAP_SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']
MAP_VARIANTS = ['none', 'v2d', 'v2dpd', 'v1dpd', 'dynablox', 'lvdot']


def map_name(lang, v):
    return T[lang].get(v, NAMES.get(v, v))


def fig_map_bars(lang, out):
    t = T[lang]
    R = {s: json.loads((RES / f'map_r9a_{s}.json').read_text()) for s in MAP_SEQS}
    vs = MAP_VARIANTS[1:]
    g = lambda v, seqs: sum(R[s][v]['ghost_voxels'] for s in seqs)
    still = ['20260519_1942', '20260519_1949']
    removed = [1 - g(v, MAP_SEQS) / g('none', MAP_SEQS) for v in vs]
    removed_still = [1 - g(v, still) / g('none', still) for v in vs]
    kept = [statistics.fmean(R[s][v]['static_kept'] for s in MAP_SEQS) for v in vs]
    noise = statistics.fmean(R[s]['none2']['static_kept'] for s in MAP_SEQS)
    names = [map_name(lang, v) for v in vs]
    cols = [C_ME if v.startswith('v2') else C_V1 if v == 'v1dpd' else C_OTHER for v in vs]
    cols[0] = '#7fa3c4'  # v2 D only: same family as v2, lighter
    fig, axs = plt.subplots(1, 3, figsize=(10.5, 2.6), sharey=True)
    pct = lambda v: f'{100 * v:.1f}%'
    hbars(axs[0], names, removed, cols, pct, t['removed'], 1.15)
    hbars(axs[1], names, removed_still, cols, pct, t['removed_static'], 1.15)
    hbars(axs[2], names, kept, cols, pct, t['kept'], 1.15)
    axs[2].axvline(noise, color=INK2, lw=0.8, ls='--')
    axs[2].text(noise, len(vs) - 0.45, t['noise'] + ' ', fontsize=7, color=INK2, ha='right', va='bottom')
    for ax in axs:
        ax.xaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1.0))
    fig.tight_layout(w_pad=1.5)
    save(fig, out, 'map_bars')


def fig_map_topdown(lang, out, seqs=('20260519_1942', '20260519_1950'), variants=('none', 'v2d', 'v2dpd', 'dynablox', 'lvdot')):
    import map_eval as me
    from scipy.spatial import cKDTree
    fig, axs = plt.subplots(len(seqs), len(variants), figsize=(2.3 * len(variants), 2.5 * len(seqs)), squeeze=False)
    for r, seq in enumerate(seqs):
        runs = [f'{v}=' + str(REPO / 'bags' / 'replay' / f'map_{v}_{seq}') for v in variants]
        maps, _, person, _ = me.aligned_maps(seq, runs, 'none')
        lo, hi = person.min(0) - 1.6, person.max(0) + 1.6
        side = max(hi - lo)
        c = (lo + hi) / 2
        lo, hi = c - side / 2, c + side / 2
        tree = cKDTree(person)
        R = json.loads((RES / f'map_r9a_{seq}.json').read_text())
        for k, v in enumerate(variants):
            ax = axs[r][k]
            pm = maps[v]
            m = (pm[:, 2] >= 0.2) & (pm[:, 2] <= 1.8) & (pm[:, 0] > lo[0]) & (pm[:, 0] < hi[0]) & (pm[:, 1] > lo[1]) & (pm[:, 1] < hi[1])
            cells = np.unique(np.floor(pm[m, :2] / 0.05).astype(np.int64), axis=0) * 0.05 + 0.025
            d, _ = tree.query(cells)
            ghost = d <= 0.8
            ax.scatter(cells[~ghost, 0], cells[~ghost, 1], s=0.6, c='#7d8794', lw=0, rasterized=True)
            ax.scatter(cells[ghost, 0], cells[ghost, 1], s=0.9, c=C_D, lw=0, rasterized=True)
            ax.plot(person[:, 0], person[:, 1], color=INK, lw=0.6, alpha=0.6)
            ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect('equal')
            ax.set_xticks([]); ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_visible(True); sp.set_color('#c3c9d1')
            n = R[v]['ghost_voxels']
            ax.set_title(f"{map_name(lang, v)}\n{n} voxel" + ('' if lang == 'en' else ''), fontsize=8,
                         color=C_ME if v == 'v2dpd' else INK)
            if k == 0:
                still = seq.endswith(('1942', '1949'))
                lab = (('standing person' if still else 'walking person') if lang == 'en'
                       else ('persona ferma' if still else 'persona che cammina'))
                ax.set_ylabel(f'{seq[-4:]}: {lab}', fontsize=8, color=INK2)
        # 1 m scale bar
        ax = axs[r][0]
        ax.plot([lo[0] + 0.3, lo[0] + 1.3], [lo[1] + 0.3] * 2, color=INK, lw=1.5)
        ax.text(lo[0] + 0.8, lo[1] + 0.4, '1 m', ha='center', fontsize=7, color=INK)
    fig.tight_layout(h_pad=1.6, w_pad=0.3)
    save(fig, out, 'map_topdown')


def fig_ablation(lang, out):
    t = T[lang]
    p = RES / 'ablation_semantics_r9a.json'
    if not p.exists():
        print('no ablation results yet')
        return
    A = json.loads(p.read_text())
    SG = json.loads((RES / 'stopgo_ablation_r9a.json').read_text()) if (RES / 'stopgo_ablation_r9a.json').exists() else {}
    cfgs = [c for c in ('v2', 'nosem', 'nosem_nopd') if c in A]
    names = [t['abl_full'] if c == 'v2' else t[f'abl_{c}'] for c in cfgs]
    cols = [C_ME, '#7fa3c4', C_OTHER][:len(cfgs)]
    # false dynamics on static objects are the same in all three (1.25/min): not a panel
    panels = [('f1_D', t['f1D'], True), ('f1_PD', t['f1PD'], True), ('merged_recall', t['found'], True)]
    if SG:
        panels.append(('id_kept', t['idkept'], True))
    fig, axs = plt.subplots(2, 2, figsize=(8.4, 3.6), sharey=True)
    axs = axs.flat
    for ax, (k, title, is_pct) in zip(axs, panels):
        vals = [(SG[c]['a'][k] if k == 'id_kept' else A[c][k]) for c in cfgs]
        vals = [0.0 if v != v else v for v in vals]
        hbars(ax, names, vals, cols, (lambda v: f'{100 * v:.1f}%') if is_pct else (lambda v: f'{v:.2f}'), title,
              1.15 if is_pct else None)
        if is_pct:
            ax.xaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1.0))
    fig.tight_layout(w_pad=1.2)
    save(fig, out, 'ablation')


def fig_camera(lang, out, seq='20260519_1950', times=(10.0, 21.0, 45.0),
               methods=('v2', 'lvdot', 'dynablox', 'mdetector', 'unimt')):
    """Front-camera frames at a few instants (s from the scored start), each captioned with the label every method
    gives the person at that instant (same matching as the timeline)."""
    from rosbags.highlevel import AnyReader
    t = T[lang]
    res = RES / 'own_r9a'
    S = json.loads((res / 'summary.json').read_text())
    al = json.loads((res / 'alignment_v2.json').read_text())
    cfg = EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'])
    gt = read_tracks(res / seq / 'gt' / 'tracks.csv')
    grid = make_grid(gt, cfg)
    labs = {}
    for m in methods:
        tt, g, labs[m] = person_labels(gt, be.predictions(m, 'r9', 'a', seq, res, al), cfg)
    short = {'dynamic': ('D', C_D), 'potentially_dynamic': ('PD', C_PD), 'missed': ('–' if lang == 'it' else '–', '#9aa3ad')}
    want = [grid[0] + x for x in times]
    imgs = [None] * len(want)
    best = [1e9] * len(want)
    with AnyReader([REPO / 'bags' / 'bags_benchmark' / f'duccio_test_{seq}_0']) as r:
        con = [c for c in r.connections if c.topic == '/front_camera/camera/color/image_raw']
        for c, ts, raw in r.messages(con):
            tsec = ts / 1e9
            hit = [k for k, w in enumerate(want) if abs(tsec - w) < best[k]]
            if not hit:
                if tsec > max(want) + 1:
                    break
                continue
            msg = r.deserialize(raw, c.msgtype)
            im = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
            if msg.encoding.lower().startswith('bgr'):
                im = im[:, :, ::-1]
            for k in hit:
                best[k], imgs[k] = abs(tsec - want[k]), im.copy()
    fig, axs = plt.subplots(1, len(times), figsize=(3.6 * len(times), 3.9))
    for ax, x, im in zip(axs, times, imgs):
        i = int(np.argmin(np.abs(tt - x)))
        ax.imshow(im)
        ax.set_xticks([]); ax.set_yticks([])
        gs = g[i]
        ax.set_title(f"t = {x:.0f} s   {t['gt'].split(' (')[0]}: {short.get(gs, ('?', INK))[0]}", fontsize=9, loc='left')
        for k, m in enumerate(methods):  # one row per method, label in its state colour
            txt, c = short[labs[m][i]]
            y = -0.06 - 0.085 * k
            ax.text(0.0, y, NAMES[m], transform=ax.transAxes, fontsize=7.5, color=C_ME if m == 'v2' else INK2,
                    va='top', fontweight='bold' if m == 'v2' else 'normal')
            ax.text(0.62, y, txt if txt != '–' else t['miss'], transform=ax.transAxes, fontsize=7.5, color=c,
                    va='top', fontweight='bold')
    fig.tight_layout(w_pad=0.8)
    fig.subplots_adjust(bottom=0.36)
    save(fig, out, f'camera_{seq[-4:]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lang', default='it', choices=['it', 'en'])
    ap.add_argument('--only', nargs='*', default=['timeline', 'labels', 'stopgo', 'camera', 'map', 'ablation'])
    args = ap.parse_args()
    style()
    out = REPO / 'eval' / 'figures' / args.lang
    if 'timeline' in args.only:
        for seq in ('20260519_1950', '20260519_1947', '20260519_1952'):
            fig_timeline(args.lang, out, seq, ['v2', 'v1', 'lvdot', 'dynablox', 'mdetector', 'unimt', 'eagermot', 'xuyolo'])
    if 'labels' in args.only:
        fig_labels(args.lang, out)
    if 'stopgo' in args.only:
        fig_stopgo(args.lang, out)
    if 'camera' in args.only:
        fig_camera(args.lang, out)
    if 'map' in args.only:
        fig_map_bars(args.lang, out)
        fig_map_topdown(args.lang, out)
    if 'ablation' in args.only:
        fig_ablation(args.lang, out)


if __name__ == '__main__':
    main()
