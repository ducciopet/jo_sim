#!/usr/bin/env python3
"""
Plain-language benchmark page: v2 vs v1 on the lab sequences, a few numbers explained in words.

Reads eval/results/own_<C>_reps.json (campaign_report.py) and own_<C>_static.json (static_eval.py), copies the
top-down videos from eval/videos/<C>/ and writes eval/report_simple/index.html (static HTML, no chart library).
The detailed page stays eval/make_report_page.py.

Usage
  python3 eval/make_simple_report.py --campaign r9
"""

import argparse
import datetime
import html
import json
import shutil
import statistics
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SEQS = [('20260519_1942', 'fermo', 'ferma'), ('20260519_1945', 'fermo', 'cammina'),
        ('20260519_1947', 'fermo', 'cammina e si ferma'), ('20260519_1949', 'avanti e indietro', 'ferma'),
        ('20260519_1950', 'avanti e indietro', 'cammina e si ferma'), ('20260519_1952', 'avanti e indietro', 'cammina')]
# v2 score along the development (eval/STATUS.md): r1–r7 single runs, r8–r9 mean of 3 runs
HISTORY = [
    ('r1', 'prima versione, primo confronto con v1', '≈ 72'),
    ('r2', 'niente più doppioni su una persona ferma', '79,6'),
    ('r3', 'messa a punto con l\'analisi degli errori', '87,0'),
    ('r4', 'la classe "persona" passa al nuovo track se l\'identità si scambia', '91,5'),
    ('r5', 'i muri non tagliano più le persone vicine', '93,1'),
    ('r6', 'conferma più rapida degli oggetti fermi', '93,3'),
    ('r7', 'persona ferma riconosciuta dal movimento visto prima; YOLO pronto da subito', '97,4'),
    ('r8', 'nuova associazione (Hungarian); prima campagna ripetuta 3 volte', '96,5'),
    ('r9', 'la persona non perde più il suo numero uscendo dal campo della camera', '98,1'),
]
VIDEO_NOTE = {
    '20260519_1947': 'Quando il tempo in alto segna circa 28 s la persona esce di lato dal campo della camera: v2 la segue con lo stesso numero.',
    '20260519_1950': 'Quando il tempo in alto segna circa 38 s la persona passa accanto a un oggetto fermo: v2 non si confonde.',
}


def git(path, *a):
    return subprocess.run(['git', '-C', str(path), *a], capture_output=True, text=True).stdout.strip()


def mean(xs):
    return statistics.fmean(xs)


def num(v, nd=1):
    return f'{v:.{nd}f}'.replace('.', ',')


def headline(L, S, m):
    """plain-language numbers for one method: mean over the repetitions"""
    reps, minutes = L[m]['reps'], 2890 / 10 / 60
    P = [L[m]['per_rep'][r] for r in reps]
    gt = [sum(p[f'conf_{g}_{k}'] for g in ('D', 'PD') for k in ('D', 'PD', 'S', 'miss')) for p in P]
    return {
        'found': 100 * mean([1 - p['FN'] / n for p, n in zip(P, gt)]),
        'state': 100 * mean([(p['conf_D_D'] + p['conf_PD_PD']) /
                             (p['conf_D_D'] + p['conf_D_PD'] + p['conf_PD_D'] + p['conf_PD_PD']) for p in P]),
        'idf1': 100 * mean([p['IDF1'] for p in P]),
        'mota': 100 * mean([p['MOTA'] for p in P]),
        'alarms': mean([p['FP'] / minutes for p in P]),
        'static_alarms': mean([S[m][r]['false_dyn_per_min'] for r in reps]) if S.get(m) else None,
        'lat': mean([p['lat_to_D_s'] for p in P]),
        'seq': {s: per_seq(L[m]['per_seq_rep'][s], reps) for s, *_ in SEQS},
        'reps': len(reps),
    }


def per_seq(PS, reps):
    """found [%], false alarms per minute and IDF1 [%] of one sequence, mean over the repetitions"""
    out = {'found': [], 'alarms': [], 'idf1': []}
    for r in reps:
        p = PS[r]
        gt = sum(p[f'conf_{g}_{k}'] for g in ('D', 'PD') for k in ('D', 'PD', 'S', 'miss'))
        out['found'].append(100 * (1 - p['FN'] / gt))
        out['alarms'].append(p['FP'] / (gt / 600.0))   # one person per frame at 10 Hz: gt frames / 600 = minutes
        out['idf1'].append(100 * p['IDF1'])
    return {k: mean(v) for k, v in out.items()}


def bar_row(label, hint, v2, v1, unit='%', nd=1, maxv=100.0):
    def bar(cls, v, who):
        w = max(0.0, min(100.0, 100 * v / maxv))
        return (f'<div class="bar {cls}"><span class="who">{who}</span><span class="track"><span class="fill" '
                f'style="width:{w:.1f}%"></span></span><span class="val">{num(v, nd)}{unit}</span></div>')
    return (f'<div class="metric"><div class="q"><h3>{label}</h3><p>{hint}</p></div>'
            f'<div class="bars">{bar("v2", v2, "v2")}{bar("v1", v1, "v1")}</div></div>')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9')
    args = ap.parse_args()
    c = args.campaign
    L = json.loads((REPO / 'eval' / 'results' / f'own_{c}_reps.json').read_text())['full']
    sp = REPO / 'eval' / 'results' / f'own_{c}_static.json'
    S = json.loads(sp.read_text()) if sp.exists() else {}
    v2, v1 = headline(L, S, 'v2'), headline(L, S, 'v1')
    out = REPO / 'eval' / 'report_simple'
    (out / 'videos').mkdir(parents=True, exist_ok=True)
    vids = []
    for s, robot, person in SEQS:
        src = REPO / 'eval' / 'videos' / c / f'{c}a_{s[-4:]}.mp4'
        if src.exists():
            shutil.copy(src, out / 'videos' / src.name)
            vids.append((s, robot, person, f'videos/{src.name}'))

    percent = ''.join([
        bar_row('La persona viene segnalata', 'Quota dei momenti in cui il metodo segnala la persona dove si trova davvero.',
                v2['found'], v1['found']),
        bar_row('Lo stato è giusto', 'Quando la segnala, dice correttamente se si sta muovendo o se è ferma.',
                v2['state'], v1['state']),
        bar_row('La persona tiene lo stesso numero', 'Quanto a lungo la persona resta lo stesso oggetto per il robot, '
                'senza cambiare identità (IDF1).', v2['idf1'], v1['idf1']),
    ])
    amax = max(v1['alarms'], v2['alarms']) * 1.05
    lower = ''.join([
        bar_row('Falsi allarmi al minuto', 'Segnalazioni dove non c\'è nessuna persona: oggetti fermi presi per persone '
                'in movimento, oppure la stessa persona segnalata due volte.', v2['alarms'], v1['alarms'], unit='', maxv=amax),
        bar_row('Ritardo nell\'accorgersi del movimento', 'Secondi tra il momento in cui la persona parte e quello in cui il '
                'metodo la segnala come in movimento.', v2['lat'], v1['lat'], unit=' s', nd=2,
                maxv=max(v1['lat'], v2['lat']) * 1.05),
    ])
    seq_rows = ''.join(
        f'<tr><td>{s[-4:]}</td><td>robot {robot}, persona {person}</td>'
        f'<td class="n">{num(v1["seq"][s]["found"])}%</td><td class="n strong">{num(v2["seq"][s]["found"])}%</td>'
        f'<td class="n">{num(v1["seq"][s]["alarms"])}</td><td class="n strong">{num(v2["seq"][s]["alarms"])}</td></tr>'
        for s, robot, person in SEQS)
    hist_rows = ''.join(f'<tr{" class=now" if k == c else ""}><td>{k}</td><td>{html.escape(t)}</td><td class="n">{v}%</td></tr>'
                        for k, t, v in HISTORY)
    options = ''.join(f'<option value="{f}"{" selected" if s.endswith("1947") else ""} data-note="{html.escape(VIDEO_NOTE.get(s, ""))}">'
                      f'{s[-4:]} · robot {robot}, persona {person}</option>' for s, robot, person, f in vids)
    first = next((v for v in vids if v[0].endswith('1947')), vids[0] if vids else None)
    ratio = v1['alarms'] / v2['alarms'] if v2['alarms'] else float('nan')
    page = TEMPLATE.format(
        mota2=num(v2['mota']), mota1=num(v1['mota']), found2=num(v2['found']), found1=num(v1['found']), idf2=num(v2['idf1'], 0), idf1=num(v1['idf1'], 0),
        al2=num(v2['alarms']), al1=num(v1['alarms']), ratio=f'{ratio:.0f}', reps=v2['reps'],
        percent=percent, lower=lower, seq_rows=seq_rows, hist_rows=hist_rows, options=options,
        first_src=first[3] if first else '', first_note=html.escape(VIDEO_NOTE.get(first[0], '')) if first else '',
        commit_v2=git(REPO / 'jo_detector_v2', 'rev-parse', '--short', 'HEAD'),
        commit_v1=git(REPO / 'jo_detector', 'rev-parse', '--short', 'HEAD'), campaign=c,
        generated=datetime.date.today().strftime('%d/%m/%Y'))
    (out / 'index.html').write_text(page)
    print('wrote', out / 'index.html', 'videos:', len(vids))


TEMPLATE = """<title>Benchmark jo_detector</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
/* Layout: one reading column; each result is a question in words with two bars under it, v2 above v1. */
:root {{
  --bg: #f5f7f9; --surface: #ffffff; --ink: #14181d; --ink-2: #4a525c; --ink-3: #7a838e; --rule: #dde2e8;
  --track: #e8ecf0; --v2: #2a78d6; --v1: #eb6834;
  --f-body: "IBM Plex Sans", "Segoe UI", system-ui, sans-serif;
  --f-num: "IBM Plex Mono", ui-monospace, Menlo, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #121418; --surface: #1a1d22; --ink: #eef0f2; --ink-2: #b9c0c8; --ink-3: #8a929c; --rule: #2c3138;
  --track: #262b31; --v2: #3987e5; --v1: #d95926; color-scheme: dark }} }}
:root[data-theme="dark"] {{
  --bg: #121418; --surface: #1a1d22; --ink: #eef0f2; --ink-2: #b9c0c8; --ink-3: #8a929c; --rule: #2c3138;
  --track: #262b31; --v2: #3987e5; --v1: #d95926; color-scheme: dark }}
body {{ background: var(--bg); color: var(--ink); font: 16px/1.6 var(--f-body);
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); }}
main {{ max-width: 760px; margin: 0 auto; padding-block: 40px 72px; display: grid; gap: 48px; }}
h1, h2, h3 {{ margin: 0; text-wrap: balance; font-weight: 600; }}
h1 {{ font-size: clamp(28px, 5vw, 38px); line-height: 1.15; }}
h2 {{ font-size: 22px; }}
h3 {{ font-size: 16px; }}
p {{ margin: 0; }}
section {{ display: grid; gap: 16px; min-width: 0; }}
.eyebrow {{ font: 500 12px var(--f-num); letter-spacing: .08em; text-transform: uppercase; color: var(--ink-3); }}
.lede {{ color: var(--ink-2); font-size: 17px; max-width: 65ch; }}
.sub {{ color: var(--ink-2); max-width: 65ch; }}
.verdict {{ background: var(--surface); border: 1px solid var(--rule); border-radius: 10px; padding: 20px 22px;
  display: grid; gap: 10px; }}
.verdict p {{ font-size: 18px; line-height: 1.5; }}
.verdict b {{ font-weight: 600; font-variant-numeric: tabular-nums; }}
.legend {{ display: flex; gap: 20px; font-size: 14px; color: var(--ink-2); flex-wrap: wrap; }}
.legend span {{ display: inline-flex; align-items: center; gap: 8px; }}
.dot {{ width: 12px; height: 12px; border-radius: 3px; display: inline-block; }}
.metrics {{ display: grid; gap: 0; border-top: 1px solid var(--rule); }}
.metric {{ display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1.15fr); gap: 12px 28px; padding-block: 18px;
  border-bottom: 1px solid var(--rule); align-items: center; }}
.metric .q {{ display: grid; gap: 4px; min-width: 0; }}
.metric .q p {{ color: var(--ink-2); font-size: 14px; }}
.bars {{ display: grid; gap: 8px; min-width: 0; }}
.bar {{ display: grid; grid-template-columns: 22px minmax(0, 1fr) 74px; gap: 10px; align-items: center; }}
.who {{ font: 500 13px var(--f-num); color: var(--ink-2); }}
.track {{ height: 14px; background: var(--track); border-radius: 3px; overflow: hidden; }}
.fill {{ display: block; height: 100%; border-radius: 3px; }}
.bar.v2 .fill {{ background: var(--v2); }}
.bar.v1 .fill {{ background: var(--v1); }}
.val {{ font: 500 15px var(--f-num); font-variant-numeric: tabular-nums; text-align: right; }}
.bar.v2 .val {{ color: var(--ink); }}
.bar.v1 .val {{ color: var(--ink-2); }}
.note {{ font-size: 14px; color: var(--ink-3); }}
.tablewrap {{ overflow-x: auto; border: 1px solid var(--rule); border-radius: 10px; background: var(--surface); }}
table {{ border-collapse: collapse; width: 100%; font-size: 15px; }}
th, td {{ padding: 10px 14px; text-align: left; border-bottom: 1px solid var(--rule); }}
tr:last-child td {{ border-bottom: 0; }}
th {{ font-size: 13px; font-weight: 500; color: var(--ink-3); }}
td.n, th.n {{ text-align: right; font-family: var(--f-num); font-variant-numeric: tabular-nums; white-space: nowrap; }}
td.strong {{ font-weight: 500; color: var(--ink); }}
tr.now td {{ font-weight: 600; }}
.video {{ display: grid; gap: 12px; }}
.video .bar2 {{ display: flex; gap: 12px; align-items: center; flex-wrap: wrap; }}
select {{ flex: 1 1 220px; min-width: 0; width: 100%; font: 15px var(--f-body); color: var(--ink); background: var(--surface); border: 1px solid var(--rule);
  border-radius: 8px; padding: 8px 10px; max-width: 100%; }}
select:focus-visible {{ outline: 2px solid var(--v2); outline-offset: 2px; }}
video {{ width: 100%; max-width: 100%; border-radius: 10px; border: 1px solid var(--rule); background: #fff; }}
ul {{ margin: 0; padding-left: 20px; display: grid; gap: 8px; max-width: 65ch; }}
dl {{ margin: 0; display: grid; grid-template-columns: max-content minmax(0, 1fr); gap: 8px 18px; max-width: 70ch; }}
dt {{ font-family: var(--f-num); font-weight: 500; }}
dd {{ margin: 0; color: var(--ink-2); }}
footer {{ font: 13px var(--f-num); color: var(--ink-3); }}
@media (max-width: 620px) {{
  .metric {{ grid-template-columns: minmax(0, 1fr); }}
  dl {{ grid-template-columns: minmax(0, 1fr); gap: 2px; }}
  dd {{ margin-bottom: 8px; }}
  table {{ font-size: 14px; }}
  th, td {{ padding: 8px 10px; }}
  .wide {{ display: none; }}
}}
</style>

<main>
  <header style="display:grid;gap:14px">
    <div class="eyebrow">jo_detector · prova in laboratorio</div>
    <h1>Il nuovo rilevatore (v2) contro quello attuale (v1)</h1>
    <p class="lede">Il robot deve accorgersi delle persone vicine, dire se si stanno muovendo e seguirle nel tempo. Abbiamo
      registrato 6 brevi prove in laboratorio con una persona, mentre un sistema di motion capture misurava al centimetro
      dove si trovava davvero: è il riferimento con cui si confrontano i due metodi. Ogni metodo è stato fatto girare
      {reps} volte sulle stesse registrazioni; i numeri sono la media.</p>
  </header>

  <section class="verdict" aria-label="In breve">
    <div class="eyebrow">In breve</div>
    <p>v2 trova la persona il <b>{found2}%</b> del tempo (v1 il {found1}%), la segue con la stessa identità
      (<b>{idf2}%</b> contro {idf1}%) e dà circa <b>{ratio} volte meno falsi allarmi</b> ({al2} al minuto contro {al1}).</p>
  </section>

  <section>
    <h2>I risultati</h2>
    <div class="legend"><span><i class="dot" style="background:var(--v2)"></i>v2, nuovo</span>
      <span><i class="dot" style="background:var(--v1)"></i>v1, attuale</span></div>
    <p class="sub">Qui conta la barra più lunga:</p>
    <div class="metrics">{percent}</div>
    <p class="sub">Qui conta la barra più corta:</p>
    <div class="metrics">{lower}</div>
  </section>

  <section>
    <h2>Perché non un punteggio unico</h2>
    <p class="sub">Il punteggio usato di solito per il tracking (MOTA) somma persone mancate, falsi allarmi e cambi di
      identità. È pensato per scene con molte persone: qui ce n'è una sola, quindi un cambio di identità pesa quasi zero
      e il MOTA misura solo "trovarla e non segnalare altro". Per questo le misure sopra sono tenute separate. Per
      confronto con la letteratura: MOTA <b>{mota2}%</b> per v2 e {mota1}% per v1. Con più persone, nelle prossime
      registrazioni, si userà anche HOTA, lo standard attuale.</p>
  </section>

  <section>
    <h2>Prova per prova</h2>
    <p class="sub">Quanto spesso la persona viene trovata e quanti falsi allarmi al minuto, in ciascuna registrazione.
      v1 dà falsi allarmi in tutte le prove (da 16 a 103 al minuto), v2 al massimo 6. Col robot in movimento v1 perde
      più spesso la persona (la trova tra l'82 e l'89% del tempo), v2 resta sopra il 93%.</p>
    <div class="tablewrap"><table>
      <tr><th>Prova</th><th>Robot / persona</th><th class="n">Trovata v1</th><th class="n">Trovata v2</th>
        <th class="n">Allarmi/min v1</th><th class="n">Allarmi/min v2</th></tr>
      {seq_rows}
    </table></div>
  </section>

  <section class="video">
    <h2>Guardalo succedere</h2>
    <p class="sub">Vista dall'alto, velocità doppia. Il cerchio tratteggiato è la posizione vera della persona; i
      rettangoli sono ciò che segnala ciascun metodo, col suo numero: rosso = in movimento, arancio = ferma. Lo spicchio
      azzurro è ciò che vede la camera.</p>
    <div class="bar2"><label for="vsel">Prova</label><select id="vsel">{options}</select></div>
    <p class="note" id="vnote">{first_note}</p>
    <video id="vid" controls muted playsinline preload="metadata" src="{first_src}"></video>
  </section>

  <section>
    <h2>Come ci siamo arrivati</h2>
    <p class="sub">Ogni "campagna" è un giro completo sulle 6 prove dopo una modifica di v2. Fino a r7 è un singolo
      giro, da r8 la media di 3 giri: per questo r8 sembra più basso di r7, che era stato un giro fortunato. La colonna è
      il MOTA, l'unica misura registrata per tutte le campagne.</p>
    <div class="tablewrap"><table>
      <tr><th>Campagna</th><th>Cosa è cambiato in v2</th><th class="n">MOTA</th></tr>
      {hist_rows}
    </table></div>
  </section>

  <section>
    <h2>Cosa tenere a mente</h2>
    <ul>
      <li>Sono 6 prove brevi (circa 5 minuti in tutto) con una sola persona. Con più persone il compito è più difficile.</li>
      <li>v2 è stato sviluppato guardando proprio queste prove, quindi questi numeri dicono che le modifiche funzionano,
        ma non sono ancora il risultato da pubblicare: servono registrazioni nuove.</li>
      <li>Prossimi passi: nuova campagna in laboratorio con più persone e confronto con altri metodi pubblicati
        (LV-DOT, Dynablox, M-detector) sulle stesse registrazioni.</li>
    </ul>
  </section>

  <section>
    <h2>Le parole usate</h2>
    <dl>
      <dt>In movimento</dt><dd>la persona si sposta a più di 0,3 m/s (etichetta D, "dynamic").</dd>
      <dt>Ferma</dt><dd>la persona è ferma ma potrebbe ripartire in ogni momento (etichetta PD, "potentially dynamic").</dd>
      <dt>Statico</dt><dd>tutto il resto: mobili, muri, oggetti (etichetta S). Il robot non lo segnala.</dd>
      <dt>Falso allarme</dt><dd>una segnalazione senza una persona vera entro 1 m.</dd>
      <dt>Identità</dt><dd>il numero che il robot assegna a ogni oggetto seguito; se cambia, il robot crede che sia arrivata
        una persona nuova.</dd>
    </dl>
  </section>

  <footer>Campagna {campaign} · v2 {commit_v2} · v1 {commit_v1} · aggiornato il {generated} · dettagli completi in eval/STATUS.md</footer>
</main>
<script>
const sel = document.getElementById('vsel'), vid = document.getElementById('vid'), note = document.getElementById('vnote');
if (sel) sel.addEventListener('change', () => {{
  const o = sel.selectedOptions[0]; vid.src = sel.value; note.textContent = o.dataset.note || '';
}});
</script>
"""

if __name__ == '__main__':
    main()
