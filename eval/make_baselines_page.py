#!/usr/bin/env python3
"""
Plain-language page: which literature methods are ready to compare against, how they were set up, and how to rebuild
them quickly. Reads eval/results/baselines_<C>.json (eval/baseline_eval.py); writes eval/report_baselines/index.html.

Usage
  python3 eval/make_baselines_page.py --campaign r9
"""

import argparse
import datetime
import html
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

METHODS = [
    # key, name, paper, sensors, state, folder, image, notes (list of short sentences)
    ('v2', 'jo_detector v2', 'questo lavoro', 'LiDAR, RGB-D', 'pronto', '–', '–', []),
    ('v1', 'jo_detector v1', 'predecessore, deriva da LV-DOT', 'LiDAR, RGB-D', 'pronto', '–', '–', []),
    ('lvdot', 'LV-DOT', 'Xu et al., 2025', 'LiDAR, RGB-D', 'pronto', 'lvdot/', 'lvdot-noetic:upstream', [
        'Codice originale (commit 449bf2c), ROS1 Noetic con GPU.',
        'Adattati solo topic, intrinseci delle camere, posizione dei sensori (URDF) e quota del pavimento.',
        'Non dà un numero d\'identità stabile: si confronta solo sul rilevamento.']),
    ('dynablox', 'Dynablox', 'Schmid et al., RA-L 2023', 'LiDAR', 'pronto', 'dynablox/', 'dynablox-noetic:upstream', [
        'Codice originale (commit 44a1537) con le sue dipendenze (voxblox).',
        'Una patch che esporta soltanto l\'ID dei cluster che già traccia, e una che toglie due dipendenze usate solo '
        'dalle loro demo.',
        'Adattato il modello del sensore al VLP-16. Deterministico: le ripetizioni coincidono.']),
    ('mdetector', 'M-detector', 'Wu et al., Nat. Comm. 2024', 'LiDAR', 'pronto', 'mdetector/', 'mdetector-noetic:upstream', [
        'Codice originale (commit 0645dab); TBB di sistema invece del percorso degli autori.',
        'Vuole un\'odometria per ogni scansione nel frame del LiDAR: l\'adattatore la ricampiona (senza, i muri '
        'sembravano muoversi).',
        'Dà punti, non oggetti: un unico raggruppamento comune (DBSCAN 0,5 m) li trasforma in box, senza identità.']),
    ('xurgbd', 'Xu et al. (solo depth)', 'Xu et al., RA-L 2023', 'RGB-D', 'pronto', 'xu_rgbd/', 'xu-rgbd-noetic:upstream', [
        'Codice originale (commit 4b4be99), variante di default senza YOLO. Stessi adattamenti di LV-DOT.',
        'Traccia la persona fusa con gli oggetti vicini (box larghi quasi 3 m) e quasi mai la dichiara in '
        'movimento: per questo esiste anche la variante con YOLO qui sotto.']),
    ('xuyolo', 'Xu et al. (con YOLO)', 'Xu et al., RA-L 2023', 'RGB-D, colore', 'pronto', 'xu_rgbd/', 'xu-rgbd-noetic:upstream', [
        'Stesso codice, con il modulo YOLO degli autori (launch detector_with_learning_module).',
        'YOLO vuole la depth allineata al colore, che le bag non registrano: un adattatore la ricostruisce come fa '
        'la RealSense (riproiezione della depth nella camera colore) e porta il colore a 848×480.',
        'YOLO gira su CPU, come nel codice originale. Nessun numero d\'identità stabile.']),
    ('unimt', 'UniMT + SimpleTrack', 'He et al., 2025', 'LiDAR, camera', 'pronto', 'unimt/', 'unimt-noetic:upstream', [
        'Codice e pesi originali: rilevatore appreso LiDAR+camera e tracker SimpleTrack; pesi "wheelchair", '
        'addestrati con un LiDAR a 16 linee come il nostro.',
        'Adattati solo topic, camera e posizione dei sensori. Lavora su una scansione ogni due (5 al secondo), come '
        'il nodo originale.',
        'Guarda solo in avanti (fino a 14 m davanti al LiDAR): la persona di lato o dietro non la vede. '
        'Riconosce persone, non il movimento: ogni box conta come "in movimento".']),
    ('jrmot', 'JRMOT', 'Shenoi et al., IROS 2020', 'LiDAR, camera', 'bloccato', '–', '–', [
        'Manca il rilevatore 2D, serve Gurobi (licenza legata alla macchina) e i messaggi SPENCER.',
        'Sui dataset ufficiali conviene EagerMOT o SPENCER al suo posto.']),
]


def pct(v):
    return '–' if v is None else f'{100 * v:.1f}'.replace('.', ',') + '%'


def num(v):
    return '–' if v is None else f'{v:.1f}'.replace('.', ',')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9')
    args = ap.parse_args()
    res = json.loads((REPO / 'eval' / 'results' / f'baselines_{args.campaign}.json').read_text())

    def mean(m, lvl, k):
        reps = res.get(m, {}).get(lvl)
        if not reps:
            return None
        xs = [r[k] for r in reps.values() if r[k] is not None]
        return sum(xs) / len(xs) if xs else None

    rows = ''
    for key, name, paper, sensors, state, *_ in METHODS:
        strong = ' class="me"' if key == 'v2' else ''
        rows += (f'<tr{strong}><td>{html.escape(name)}<span class="paper">{html.escape(paper)}</span></td><td>{sensors}</td>'
                 f'<td class="n">{pct(mean(key, "merged", "recall"))}</td>'
                 f'<td class="n">{pct(mean(key, "dynamic_only", "recall"))}</td>'
                 f'<td class="n">{num(mean(key, "dynamic_only", "false_alarms_per_min"))}</td>'
                 f'<td class="n">{pct(mean(key, "merged", "IDF1"))}</td></tr>')
    cards = ''
    for key, name, paper, sensors, state, folder, image, notes in METHODS[2:]:
        badge = {'pronto': 'ok', 'da fare': 'todo', 'bloccato': 'stop'}[state]
        li = ''.join(f'<li>{html.escape(n)}</li>' for n in notes)
        where = (f'<p class="where">Cartella <code>eval/baselines/{folder}</code> · immagine <code>{image}</code></p>'
                 if folder != '–' else '')
        cards += (f'<article class="card"><header><h3>{html.escape(name)}</h3><span class="badge {badge}">{state}</span></header>'
                  f'<p class="meta">{html.escape(paper)} · {sensors}</p><ul>{li}</ul>{where}</article>')
    page = TEMPLATE.format(rows=rows, cards=cards, campaign=args.campaign,
                           generated=datetime.date.today().strftime('%d/%m/%Y'))
    out = REPO / 'eval' / 'report_baselines'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'index.html').write_text(page)
    print('wrote', out / 'index.html')


TEMPLATE = """<title>Baseline jo_detector</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
/* Layout: one reading column; a results table first, then one card per method, then the rebuild recipe. */
:root {{
  --bg: #f5f7f9; --surface: #ffffff; --ink: #14181d; --ink-2: #4a525c; --ink-3: #7a838e; --rule: #dde2e8;
  --track: #e8ecf0; --accent: #2a78d6; --ok: #1f7a3a; --todo: #8a6100; --stop: #b3261e;
  --f-body: "IBM Plex Sans", "Segoe UI", system-ui, sans-serif;
  --f-num: "IBM Plex Mono", ui-monospace, Menlo, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #121418; --surface: #1a1d22; --ink: #eef0f2; --ink-2: #b9c0c8; --ink-3: #8a929c; --rule: #2c3138;
  --track: #262b31; --accent: #3987e5; --ok: #5cc27a; --todo: #e0a040; --stop: #ef6b63; color-scheme: dark }} }}
:root[data-theme="dark"] {{
  --bg: #121418; --surface: #1a1d22; --ink: #eef0f2; --ink-2: #b9c0c8; --ink-3: #8a929c; --rule: #2c3138;
  --track: #262b31; --accent: #3987e5; --ok: #5cc27a; --todo: #e0a040; --stop: #ef6b63; color-scheme: dark }}
body {{ background: var(--bg); color: var(--ink); font: 16px/1.6 var(--f-body);
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); }}
main {{ max-width: 820px; margin: 0 auto; padding-block: 40px 72px; display: grid; gap: 44px; }}
h1, h2, h3 {{ margin: 0; text-wrap: balance; font-weight: 600; }}
h1 {{ font-size: clamp(28px, 5vw, 38px); line-height: 1.15; }}
h2 {{ font-size: 22px; }}
h3 {{ font-size: 17px; }}
p {{ margin: 0; max-width: 68ch; }}
section {{ display: grid; gap: 14px; min-width: 0; }}
.eyebrow {{ font: 500 12px var(--f-num); letter-spacing: .08em; text-transform: uppercase; color: var(--ink-3); }}
.lede, .sub {{ color: var(--ink-2); }}
.lede {{ font-size: 17px; }}
code {{ font: 13.5px var(--f-num); background: var(--track); padding: 1px 5px; border-radius: 4px; }}
.tablewrap {{ overflow-x: auto; border: 1px solid var(--rule); border-radius: 10px; background: var(--surface); }}
table {{ border-collapse: collapse; width: 100%; font-size: 15px; }}
th, td {{ padding: 10px 12px; text-align: left; border-bottom: 1px solid var(--rule); vertical-align: top; }}
tr:last-child td {{ border-bottom: 0; }}
th {{ font-size: 12.5px; font-weight: 500; color: var(--ink-3); }}
td.n, th.n {{ text-align: right; font-family: var(--f-num); white-space: nowrap; }}
tr.me td {{ font-weight: 600; }}
.paper {{ display: block; font-size: 13px; color: var(--ink-3); font-weight: 400; }}
.cards {{ display: grid; gap: 12px; }}
.card {{ background: var(--surface); border: 1px solid var(--rule); border-radius: 10px; padding: 16px 18px; display: grid; gap: 8px; }}
.card header {{ display: flex; align-items: center; justify-content: space-between; gap: 12px; }}
.card .meta {{ color: var(--ink-3); font-size: 14px; }}
.card ul {{ margin: 0; padding-left: 20px; display: grid; gap: 4px; color: var(--ink-2); }}
.card .where {{ font-size: 14px; color: var(--ink-2); }}
.badge {{ font: 500 12.5px var(--f-num); padding: 2px 9px; border-radius: 999px; border: 1px solid currentColor; white-space: nowrap; }}
.badge.ok {{ color: var(--ok); }} .badge.todo {{ color: var(--todo); }} .badge.stop {{ color: var(--stop); }}
pre {{ margin: 0; background: var(--surface); border: 1px solid var(--rule); border-radius: 10px; padding: 14px 16px;
  overflow-x: auto; font: 13.5px/1.55 var(--f-num); }}
ol, ul.plain {{ margin: 0; padding-left: 22px; display: grid; gap: 8px; max-width: 68ch; color: var(--ink-2); }}
ol b, ul.plain b {{ color: var(--ink); font-weight: 600; }}
footer {{ font: 13px var(--f-num); color: var(--ink-3); }}
</style>

<main>
  <header style="display:grid;gap:14px">
    <div class="eyebrow">jo_detector · metodi di confronto</div>
    <h1>I metodi della letteratura, pronti per il confronto</h1>
    <p class="lede">Quattro metodi pubblicati girano sulle nostre registrazioni di laboratorio con il loro codice originale,
      cambiando solo come ricevono i dati. Qui sotto: come vanno, come sono stati preparati e come rifarli in poche ore,
      per esempio su un dataset ufficiale come JRDB.</p>
  </header>

  <section>
    <h2>Come vanno sulle 6 prove di laboratorio</h2>
    <p class="sub">Media di 3 giri per metodo. <b>Ferma o no</b>: quanto spesso la persona viene segnalata, che cammini
      o stia ferma. <b>Mentre cammina</b>: quanto spesso viene segnalata come "in movimento" quando cammina.
      <b>Allarmi/min</b>: segnalazioni "in movimento" sbagliate al minuto, compresa la persona ferma presa per in
      movimento. <b>IDF1</b>: quanto la persona tiene lo stesso numero, solo per i metodi che ne danno uno stabile.</p>
    <div class="tablewrap"><table>
      <tr><th>Metodo</th><th>Sensori</th><th class="n">Ferma o no</th><th class="n">Mentre cammina</th>
        <th class="n">Allarmi/min</th><th class="n">IDF1</th></tr>
      {rows}
    </table></div>
    <p class="sub">Nessun metodo esterno distingue una persona ferma che può ripartire (PD) da un oggetto statico: i
      metodi solo LiDAR la perdono quando si ferma, LV-DOT la tiene "in movimento" e accumula falsi allarmi. Sono numeri
      sulle prove usate per sviluppare v2: il confronto da pubblicare va rifatto su dati nuovi.</p>
  </section>

  <section>
    <h2>Metodo per metodo</h2>
    <div class="cards">{cards}</div>
  </section>

  <section>
    <h2>Come sono stati preparati</h2>
    <ol>
      <li><b>Codice originale.</b> Nessuna modifica a rilevamento, tracking o classificazione. Si adattano solo topic,
        parametri delle camere, posizione dei sensori sul robot e quota del pavimento (righe marcate <code># JO</code>);
        ogni altra modifica è una patch documentata in <code>patches/</code>.</li>
      <li><b>Stesse condizioni di v1 e v2.</b> Stessa odometria, metodo avviato 12 s prima della registrazione,
        riproduzione a velocità reale.</li>
      <li><b>Un container ROS1 per metodo.</b> Le registrazioni ROS2 vengono convertite al volo e cancellate dopo l'uso.</li>
      <li><b>Stessa valutazione.</b> L'uscita di ogni metodo diventa lo stesso formato di tracce; i metodi che danno
        punti invece di oggetti passano da un unico raggruppamento comune.</li>
    </ol>
  </section>

  <section>
    <h2>Rifarli da capo</h2>
    <p class="sub">Dalla cartella del repository <code>jo_sim_lv-dot</code>. Le istruzioni complete, con i valori di ogni
      parametro, sono in <code>eval/baselines/README_baselines.md</code>.</p>
    <pre>docker build -t &lt;immagine&gt; eval/baselines/&lt;cartella&gt;
bash eval/baselines/&lt;cartella&gt;/run_&lt;metodo&gt;.sh 20260519_1950 prova --keep-input
setsid nohup bash eval/baselines/&lt;cartella&gt;/run_&lt;metodo&gt;_campaign.sh \\
     &gt; bags/&lt;metodo&gt;_out/campaign.log 2&gt;&amp;1 &lt; /dev/null &amp;
python3 eval/baseline_eval.py --campaign r9 --reps a b c</pre>
    <p class="sub">Per un dataset ufficiale cambia solo l'ingresso: topic e parametri dei sensori nei file
      <code># JO</code>, una conversione della bag e la ground truth del dataset al posto del motion capture.</p>
  </section>

  <section>
    <h2>Trappole già incontrate</h2>
    <ul class="plain">
      <li><b>Odometria e scansioni.</b> M-detector accoppia la n-esima scansione con la n-esima posa: con l'odometria a
        40 Hz i muri sembravano muoversi. Serve una posa per scansione.</li>
      <li><b>Uscite vuote.</b> Dynablox non pubblica nulla quando non vede moto: un topic assente non è un errore.</li>
      <li><b>Compilazioni silenziose.</b> Un errore nascosto da un filtro sull'output: ogni Dockerfile controlla che
        l'eseguibile esista.</li>
      <li><b>Spazio su disco.</b> Le build lasciano decine di GB di cache: <code>docker builder prune -f</code>.</li>
      <li><b>Fermare i processi.</b> <code>pkill -f</code> con un pattern presente nel comando stesso chiude anche la
        shell: usare la forma <code>'[p]attern'</code>.</li>
    </ul>
  </section>

  <footer>Campagna {campaign} · aggiornato il {generated} · dati eval/results/baselines_{campaign}.json</footer>
</main>
"""

if __name__ == '__main__':
    main()
