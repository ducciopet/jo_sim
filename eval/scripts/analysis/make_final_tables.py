#!/usr/bin/env python3
"""Final comparison tables for the status page (27/11): per dataset, SINS (reps) vs every baseline found, pooled over the
dataset's windows: MOTA, IDF1, HOTA (+DetA/AssA) from coda_eval.py JSONs (metrics.merged, hota_alpha sums), mean and
min-max over reps, plus the claim verdict of bootstrap_claims.py (exact sign + permutation on window means, cluster
bootstrap only with >= 10 sequences). Writes an HTML fragment (one <section> per dataset, with the metric formulas
above each table) and a JSON with the numbers.

usage: make_final_tables.py --spec spec.json --out tables.html
spec.json: [{"name": "CODa 2/3/18 (mai viste)", "dir": "<results dir with <TAG>_<window>.json>",
             "windows": [...], "sins": ["SF","SFr2","SFr3"], "baselines": {"EMMC": ["EMMC"], "Dynablox": ["DBXF","DBXF2","DBXF3"]},
             "note": "..."}, ...]
Pure analysis of existing JSONs.
"""
import argparse
import html
import itertools
import json
import os
import re
import sys
from math import comb
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from harness.metrics import hota_from_sums   # noqa: E402

FORMULAS = """<div class="formulas">
<p><b>MOTA</b> = 1 − (FN + FP + IDSW) / GT — quanti errori si fanno in tutto, rispetto al numero di oggetti veri: mancati
(FN), falsi allarmi (FP) e scambi di identità (IDSW), sommati su tutti i fotogrammi. 100 = perfetto; può essere negativo.
Un oggetto è "trovato" se il centro previsto è entro 1 m da quello vero.</p>
<p><b>IDF1</b> = 2·IDTP / (GT + PRED) — quanto a lungo ogni oggetto vero resta associato alla stessa traccia: misura la
continuità delle identità, non solo il trovarle.</p>
<p><b>HOTA</b> = media su α di √(DetA·AssA) — bilancia trovare gli oggetti (<b>DetA</b> = TP/(TP+FN+FP)) e mantenerne
l'identità (<b>AssA</b>), con soglie di somiglianza α da 0,05 a 0,95; qui la somiglianza è 1 − distanza/1 m.</p>
<p><b>Recall</b> = TP / GT — quanti oggetti veri vengono trovati; <b>Precisione</b> = TP / (TP + FP) — quante delle
cose segnalate esistono davvero. Un oggetto vero e uno previsto si abbinano se i centri distano meno di 1 m (non la
sovrapposizione dei box: i box di oggetti visti in parte sono più piccoli del vero).</p>
<p><b>F1 dello stato</b> = 2·P·R / (P + R) per lo stato <b>D</b> (si muove) e <b>PD</b> (fermo ma mobile), sugli oggetti
abbinati: misura se il metodo dice giusto <i>che cosa sta facendo</i> l'oggetto, non solo se lo trova.</p>
<p><b>Verdetto</b>: "migliore"/"peggiore" solo se l'intervallo di confidenza al 95% (bootstrap sulle sequenze) esclude lo
zero e i test esatti (segni e permutazione sulle finestre, ripetizioni mediate) danno p &lt; 0,05; altrimenti
"comparabile". Con meno di 10 sequenze nessun intervallo: valgono i test esatti per finestra.</p>
</div>"""


def load(d, tag, w):
    f = Path(d) / f'{tag}_{w}.json'
    if not f.exists():
        return None
    J = json.load(open(f))['metrics']
    m = dict(J['merged'])
    m['confusion'] = J.get('full', {}).get('confusion', {})
    return m


def state_f1(ms):
    """F1 of the D and PD labels from the summed confusion of the 'full' level (matched pairs; recall counts misses)."""
    C = {}
    for m in ms:
        for g, row in m['confusion'].items():
            for p, n in row.items():
                C.setdefault(g, {}).setdefault(p, 0)
                C[g][p] += n
    out = {}
    for s in ('dynamic', 'potentially_dynamic'):
        tp = C.get(s, {}).get(s, 0)
        row = sum(v for k, v in C.get(s, {}).items())          # GT rows of that state, misses included
        col = sum(C.get(g, {}).get(s, 0) for g in C)            # matched predictions with that label
        pr = tp / col if col else np.nan
        rc = tp / row if row else np.nan
        out[s] = 100 * 2 * pr * rc / (pr + rc) if col and row and (pr + rc) > 0 else np.nan
    return out


def pooled(d, tags, W):
    reps = []
    for t in tags:
        ms = [load(d, t, w) for w in W]
        if any(m is None for m in ms):
            continue
        s = {k: sum(m[k] for m in ms) for k in ('fp', 'fn', 'idsw', 'num_gt', 'idtp', 'num_pred')}
        h = {k: list(np.sum([m['hota_alpha'][k] for m in ms if m.get('hota_alpha')], axis=0)) for k in ('tp', 'fn', 'fp', 'assa_num')} \
            if all(m.get('hota_alpha') for m in ms) else None
        hv = hota_from_sums(h) if h else (np.nan, np.nan, np.nan)
        f1 = state_f1(ms)
        tp = s['num_gt'] - s['fn']
        reps.append(dict(mota=100 * (1 - (s['fp'] + s['fn'] + s['idsw']) / s['num_gt']),
                         recall=100 * tp / s['num_gt'], precision=100 * tp / max(1, tp + s['fp']),
                         f1d=f1['dynamic'], f1pd=f1['potentially_dynamic'],
                         idf1=200 * s['idtp'] / max(1, s['num_gt'] + s['num_pred']),
                         hota=100 * hv[0], deta=100 * hv[1], assa=100 * hv[2],
                         err=[(m['fp'] + m['fn'] + m['idsw']) for m in ms], gt=[m['num_gt'] for m in ms]))
    return reps


def verdict(a, b, W):
    ea = np.mean([r['err'] for r in a], axis=0)
    eb = np.mean([r['err'] for r in b], axis=0)
    gt = np.array(a[0]['gt'], float)
    dm = (eb - ea) / gt
    wins, losses = int((dm > 0).sum()), int((dm < 0).sum())
    n = wins + losses
    p_sign = min(1.0, 2 * sum(comb(n, j) for j in range(max(wins, losses), n + 1)) / 2 ** n) if n else 1.0
    diff = eb - ea
    flips = np.array(list(itertools.product([1, -1], repeat=len(W)))) if len(W) <= 18 else \
        np.random.default_rng(0).choice([1, -1], size=(100000, len(W)))
    p_perm = float(np.mean(np.abs(flips @ diff) >= abs(diff.sum()) - 1e-9))
    seqs = sorted({re.sub(r'_w\d+$', '', w) for w in W})
    ci = None
    if len(seqs) >= 10:
        rng = np.random.default_rng(0)
        cl = [[k for k, w in enumerate(W) if re.sub(r'_w\d+$', '', w) == s] for s in seqs]
        bs = []
        for _ in range(10000):
            idx = np.concatenate([cl[c] for c in rng.integers(0, len(cl), len(cl))])
            bs.append(100 * ((eb[idx].sum() - ea[idx].sum()) / gt[idx].sum()))
        ci = np.percentile(bs, [2.5, 97.5])
    if p_sign < 0.05 and p_perm < 0.05 and (ci is None or (ci[0] > 0 if wins > losses else ci[1] < 0)):
        v = 'migliore' if wins > losses else 'peggiore'
    else:
        v = 'comparabile'
    return dict(verdict=v, wins=wins, n=n, p_sign=round(p_sign, 3), p_perm=round(p_perm, 3),
                ci=None if ci is None else [round(float(ci[0]), 1), round(float(ci[1]), 1)])


def fmt(reps, k):
    v = [r[k] for r in reps if np.isfinite(r[k])]
    if not v:
        return '–'
    s = f'{np.mean(v):.1f}'.replace('.', ',')
    return s + (f' <small>[{min(v):.1f}–{max(v):.1f}]</small>'.replace('.', ',') if len(v) > 1 else '')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--spec', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    spec = json.load(open(a.spec))
    parts, data = [], []
    for ds in spec:
        W = ds['windows']
        s = pooled(ds['dir'], ds['sins'], W)
        if not s:
            parts.append(f'<section><h3>{html.escape(ds["name"])}</h3><p>Risultati di SINS non ancora disponibili.</p></section>')
            continue
        rows = [('SINS', s, None)]
        for name, tags in ds['baselines'].items():
            b = pooled(ds['dir'], tags, W)
            if b:
                rows.append((name, b, verdict(s, b, W)))
        t = ['<div class="tablewrap"><table><tr><th>Metodo</th><th class="n">MOTA</th><th class="n">IDF1</th>'
             '<th class="n">HOTA</th><th class="n">DetA</th><th class="n">AssA</th><th class="n">Recall</th><th class="n">Precisione</th>'
             '<th class="n">F1 D</th><th class="n">F1 PD</th><th>SINS rispetto a lui</th></tr>']
        for name, r, v in rows:
            vt = '' if v is None else f'{v["verdict"]} ({v["wins"]}/{v["n"]} finestre' + \
                (f', IC {v["ci"][0]:+.1f}…{v["ci"][1]:+.1f}' if v['ci'] else '') + ')'
            t.append(f'<tr{" class=me" if name == "SINS" else ""}><td>{html.escape(name)}</td><td class="n">{fmt(r, "mota")}</td>'
                     f'<td class="n">{fmt(r, "idf1")}</td><td class="n">{fmt(r, "hota")}</td><td class="n">{fmt(r, "deta")}</td>'
                     f'<td class="n">{fmt(r, "assa")}</td><td class="n">{fmt(r, "recall")}</td><td class="n">{fmt(r, "precision")}</td>'
                     f'<td class="n">{fmt(r, "f1d")}</td><td class="n">{fmt(r, "f1pd")}</td><td>{html.escape(vt)}</td></tr>')
        t.append('</table></div>')
        parts.append(f'<section><h3>{html.escape(ds["name"])}</h3>{FORMULAS}{"".join(t)}'
                     f'<p class="note">{html.escape(ds.get("note", ""))}</p></section>')
        data.append(dict(name=ds['name'], rows=[dict(name=n, reps=r, verdict=v) for n, r, v in rows]))
    Path(a.out).write_text('\n'.join(parts))
    Path(a.out).with_suffix('.json').write_text(json.dumps(data, indent=1, default=float))
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
