# SINS: passaggio di consegne (aggiornato il 30/10/2026 sera)

Per chi riprende il lavoro da un altro account Claude. Leggi questo file, poi `eval/STATUS.md` (le sezioni in fondo
sono le più recenti), poi `PAPER_PLAN.md`. Con l'utente si parla in **italiano**.

## 1. Il progetto in due righe

**SINS (Still Is Not Static)**, prima "jo_detector v2": rilevatore e tracker online di ostacoli per robot mobile,
LiDAR + camere RGB-D + YOLO, con tre stati: **D** (si muove), **PD** (fermo ma può muoversi: persona ferma, bici
parcheggiata, veicolo fermo) e **S** (statico). Obiettivo: paper RA-L + IROS 2027 (submission ~22/02/2027).

## 2. Dove sta il codice

| Cosa | Dove |
|---|---|
| Repository principale | `~/Desktop/jo_sim_sins`, branch `sins` |
| SINS (pacchetto ROS 2) | submodule `sins/` (github.com/ducciopet/sins), branch `main` |
| Nodi C++ | `sins/src/preprocessing_node.cpp` (nuvole, foglie YOLO), `dbscan_detector_node.cpp` (cluster, fusione, YOLO→cluster), `tracker_node.cpp` (tracce, D/PD/S) |
| Nodi YOLO | `sins/scripts/yolo_seg_track_detector.py` (maschere), `yolo_semantic_seg_detector.py` (riquadri) |
| Launch | `sins/launch/run_detector.launch.py` (`env:=indoor|outdoor|coda`, `camera_names:=a,b`) |
| Profili | `sins/cfg/indoor/`, `sins/cfg/outdoor/` (**le modifiche di metodo per scene all'aperto vanno qui**), `sins/cfg/coda/` è GENERATO |
| Profilo di un dataset | `eval/configs/sins/<dataset>/overrides.yaml` (solo sensore: camere, portata, ring spacing) + `python3 eval/configs/sins/make_cfg.py <dataset>` |
| Valutazione | `eval/` sull'host (Python), runner nel container `jo_sim_sins` |
| Compilare SINS | `docker exec -u ros jo_sim_sins bash -c "source /opt/ros/*/setup.bash; cd ~ && colcon build --packages-select sins --install-base local_install --cmake-args -DCMAKE_BUILD_TYPE=Release"` |

Il container `jo_sim_sins` ha l'orologio un'ora indietro (UTC). `bags/` sul host = `/home/ros/bags` nel container.
`bags/replay/run_method.sh` e `run_coda.sh` NON sono nel git: le copie aggiornate sono in `eval/scripts/`.
**Non modificare script mentre una run li sta eseguendo** (bash li legge a pezzi: è già successo, run corrotte).

## 3. Regole dell'utente (dalla memoria di Claude, copia in `~/Desktop/claude_account_backup/sins_handoff/memoria_claude/`)

- Niente tuning di pesi/Kalman sulle 6 sequenze di laboratorio; solo correzioni fisiche, con parametro che le
  spegne, verificate A/B. In generale: prove con **3 ripetizioni** (il rumore fra run è ±3 MOTA, ±6 IDF1).
- Report semplici in italiano; ogni artifact copiato in `~/Desktop/claude_account_backup/artifacts/`.
- Raccomandazioni dirette, non menu di opzioni. `git reset --hard` senza chiedere.
- Uccidere i processi di debug a fine lavoro (ATTENZIONE: `pkill -f <pattern>` può uccidere la propria shell se il
  pattern compare nel comando: meglio PID espliciti).
- Il framework DEVE lavorare **online** (tempo reale, 10 Hz): niente precalcolo offline di depth o altro.
- **Un solo YOLO per camera** (richiesta esplicita del 30/10).
- Per scene outdoor si modificano i file YAML **outdoor**; in laboratorio (indoor) SINS deve restare identico.
- Non cercare credenziali; il NAS lo monta l'utente.
- Commit con `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## 4. Stato attuale: CODa (dataset outdoor pubblico)

CODa (UT Austin): LiDAR Ouster OS1-128 + ZED (cam3, con depth) + coppia stereo cam0/cam1. Usiamo la sequenza 0,
tre finestre (w1 cortile con rastrelliera e furgone, w2 corridoio interno, w3 viale): 850 fotogrammi annotati a
10 Hz (400 + 250 + 200 = 1 min 25 s), più 200 scansioni di rodaggio per finestra (in tutto 2 min 25 s di dati).
**Sono lo split di tuning**: i numeri del paper vanno presi sulla sequenza 1 (ancora da convertire).

### Pipeline CODa
- Conversione: `eval/datasets/coda_to_bag.py` (bag sensore + odom + GT), `eval/datasets/coda_stereo_bag.py`
  (cam0/cam1, un fotogramma per scansione LiDAR, bag `bags/coda/coda0_<w>_stereo`).
- Run: `STEREO=1 bash eval/scripts/coda_ab.sh <TAG> [w1 w2 w3]` (rigenera cfg/coda, gira nel container, valuta).
  `STEREO=1` aggiunge la seconda camera `stereo_camera` con depth calcolata **online** da
  `eval/adapters/stereo_depth_node.py` (SGBM, 28 ms/coppia).
- Valutazione: `eval/coda_eval.py` (regole: oggetti fermi solo nel campo camera, ≥1 punto LiDAR, 1,2–25 m, match
  1 m), `eval/coda_pool.py` (aggrega; `per_label` D/PD con precisione e falsi/min), `eval/coda_compare.py`
  (8 metodi → `eval/results/coda/compare.json`, `eval/tables/coda_compare.csv`).
- Diagnostica: `eval/diagnostics/coda_still_funnel.py` (dove si perde ogni oggetto fermo, stadio per stadio),
  `eval/diagnostics/coda_yolo_seen.py`. Video: `eval/make_coda_video.py`.

### Configurazione outdoor attuale = "M2" (commit sins `61b1e1f`, repo `081b219`)
Tutto in `sins/cfg/outdoor/` (indoor invariato):
- foglie YOLO da **maschere** (`yolo_leaf_method: mask`, letto dal profilo), **un solo YOLO per camera**
  (yolo26s-seg, 1280 px, classi dinamiche dello YAML; launch `single_yolo:=true` di default);
- maschera senza foglia → riquadro della stessa rilevazione (`yolo_leaf_mask_box_fallback`);
- foglie LiDAR oltre la depth senza muri, gruppo più vicino che copre ≥ metà del riquadro;
- memoria di classe degli oggetti fermi (`track_class_anchor_*`, 5 s, 2 conferme);
- veicoli: cluster fino a 6 m, classe veicolo solo se alto ≥ 1,1 m (nessun limite per persone/bici/animali);
- moto confermato (velocità stabile o punti) per tutte le tracce con classe (`track_classified_corroboration_all`);
- depth per camera (`<cam>.depth_max_value`: ZED 4,5 m, stereo 6 m misurato contro il LiDAR).

### Risultati (media di 3 ripetizioni, seq 0)
| | Prima (29/10) | G1 (2 YOLO/camera) | **M2 (attuale)** |
|---|---|---|---|
| Fermi come PD | 3,5% | 42,2% | 46,1% |
| Precisione PD | 39% | 68,5% | 64,9% |
| PD sbagliati/min | 17 | 97 | 122 |
| Chi si muove: trovato / precisione | 82,8 / 86,3% | 84,9 / 77,7% | 83,4 / 82,7% |
| Segnalazioni sbagliate/min | 96 | 231 | 219 |

Altri metodi su CODa: CenterPoint+SimpleTrack (pesi CODa degli autori di UniMT; CMDT CODa non pubblicato),
EagerMOT (CenterPoint + YOLO), Dynablox, M-detector (anche a 0,25x), LV-DOT. Solo SINS ha lo stato PD.

### Provato e scartato (3 ripetizioni, opzioni rimaste spente nel codice)
YOLO più grande (m/l); voxel solo vicino al sensore (`lidar_processed_voxel_max_range`); 3 conferme nella memoria;
taglio dei cluster che contengono la foglia (`yolo_refine_nested`); associazione per punti condivisi
(`yolo_refine_assoc: points`, solo run singole: molti PD falsi sui paletti); `track_first_class_weak_keeps_streak`.

### Dove si perde il resto (analisi stadio per stadio, G1)
PD ok 55% · YOLO non vede 13% · foglia mancante 8% · foglia accanto 5% · cluster senza classe 8% · D sbagliato 4%
· tracker/cluster/preprocessing ~6%. Errori: paletti segnati PD (~60/min), cose non annotate da CODa.
Il modello con maschere trova più bici del modello a riquadri ma con riquadri piccoli: il fallback LiDAR→2D del
tracker dà "bicycle" ai paletti.

## 5. Prossimi passi (in ordine)
1. **Provare `track_lidar_fallback: false`** in `sins/cfg/outdoor/tracker.yaml` (il codice è già compilato; cambiare
   uno YAML non richiede ricompilare: l'installazione colcon è a link simbolici e `coda_ab.sh` rigenera `cfg/coda`).
   Atteso: meno PD sui paletti. 3 ripetizioni (`N1`, `N1r2`, `N1r3`), confronto con M2 (comandi in §8).
   **Criterio**: si tiene se i PD sbagliati/min calano chiaramente oltre il rumore fra ripetizioni e PD trovati,
   chi-si-muove trovato e precisione restano entro il rumore. Se si tiene: commit in `sins/`, commit del puntatore
   del submodule nel repo, `eval/coda_compare.py` (righe 23-24, oggi `M2`) puntato su `N1`, rilancio del confronto,
   aggiornamento di `eval/STATUS.md` e della pagina "SINS a confronto". Se no: `git -C sins checkout cfg/outdoor/tracker.yaml`.
2. **Bloccare la configurazione** (= un commit di `sins/` con nome, es. tag git `coda-frozen`, da non cambiare più fino
   alla fine della valutazione held-out) e **convertire CODa sequenza 1** → confronto completo lì (numeri del paper).
   Finestre: non c'è uno strumento; scegliere 2-3 tratti di 400-600 scansioni dove `3d_bbox/os1/1/` ha annotazioni
   continue e ci sono sia persone in movimento sia oggetti fermi (bici, veicoli), più 200 scansioni di rodaggio prima.
   Disco: per finestra ~2,4 GB (bag sensore) + ~2,9 GB (bag stereo) = ~5,3 GB; con 14 GB liberi ci stanno 2 finestre:
   liberare spazio prima (es. vecchie bag in `bags/coda/runs/` con TAG scartati, ~100-170 MB l'una con i topic
   intermedi). Tutti i metodi di confronto vanno rifatti sulle nuove finestre (deterministici: 1 run; M-detector anche a
   0,25x; SINS 3 ripetizioni).
3. JRDB (stesso VLP-16 del nostro robot, persone ferme fra la folla: banco giusto per il PD).
4. SemanticKITTI con depth stereo (immagini in download, vedi §6).
5. CROSSLAB 29/10 (nuove prove Qualisys, controprova del laboratorio).
6. Aggiornare l'artifact "SINS a confronto" con M2 (oggi mostra G1).

## 6. Dati e trasferimenti
- NAS montato su `/mnt/kloster-reichenau` (non in fstab: dopo un riavvio lo rimonta l'utente). **Pieno: ~2 GB
  liberi**; cancellare non libera spazio finché non si svuota il cestino `#recycle` (amministratore).
- `datasets_SINS/`: CODa seq 0/1, JRDB train (27), SemanticKITTI (22 seq. LiDAR+label), registrazioni Marina di Pisa
  e San Rossore (16 zip, sha256 tutti OK), CROSSLAB (motion capture + 2/5 zip verificati).
- `~/transfer_crosslab.sh` (log `~/rclone-crosslab.log`) aspetta spazio per i 3 zip mancanti (~85 GB).
- Download immagini KITTI (`eval/datasets/remote_zip.py`, coda `~/kitti_color_queue.sh`, log `~/kitti_color.log`):
  **in pausa** con SIGSTOP (PID nel processo `remote_zip.py`; riprendere con `kill -CONT <pid>` quando c'è spazio).

## 7. Artifact (copie locali in `~/Desktop/claude_account_backup/artifacts/`)
- SINS a confronto: https://claude.ai/artifact/2GJ3QbsawkCTCjpxh4bP46 (lab + CODa, video, tabelle)
- Baseline di SINS: https://claude.ai/artifact/XGLMmauGC6XHbai8ygpQaz
- Come funziona SINS: https://claude.ai/artifact/HBGNJo3pUHZNipahHnqXpE
- SINS, punto della situazione (questa pagina in HTML): https://claude.ai/artifact/ES11EkRzwFGcXAZJBtZJrw (copia `artifacts/SINS_punto_della_situazione.html`)

## 8. Ricette (comandi esatti, da `~/Desktop/jo_sim_sins`)

**Attenzione: gli artifact su claude.ai sono privati di questo account.** Un altro account NON li apre: usa le copie
in `~/Desktop/claude_account_backup/artifacts/` e questo file. Per pubblicarli di nuovo, ripubblicare le copie HTML
(ogni pagina con figure e video è una cartella completa: `artifacts/SINS_a_confronto/index.html`, `artifacts/Baseline_di_SINS/index.html`).

Ambiente:
- Container principale `jo_sim_sins` (immagine `jo-sim_sins:latest`), avviato con `docker compose up` dal repo
  (vedi `README.md`). Baseline in immagini a parte: `dynablox-noetic`, `mdetector-noetic`, `lvdot-noetic`,
  `unimt-noetic` (usata anche per CenterPoint), `xu-rgbd-noetic`. EagerMOT gira sull'host:
  `eval/baselines/eagermot/.venv`.
- `bags/replay/` (dentro il container `/home/ros/bags/replay`) contiene gli script di esecuzione FUORI dal git;
  se mancano: `cp eval/scripts/run_method.sh eval/scripts/run_coda.sh eval/adapters/stereo_depth_node.py
  eval/adapters/realsense_bag_adapter.py bags/replay/`.
- NAS (lo monta l'utente, credenziali sue): `sudo mount -t cifs "//konstanz.local/Kloster Reichenau"
  /mnt/kloster-reichenau -o username=...,uid=1000,gid=1000,file_mode=0664,dir_mode=0775,vers=3.0`.

Prossimo passo 1 (fallback del tracker spento), 3 ripetizioni e confronto con M2:

    sed -i 's|^    track_lidar_fallback: true |    track_lidar_fallback: false|' sins/cfg/outdoor/tracker.yaml
    for t in N1 N1r2 N1r3; do STEREO=1 bash eval/scripts/coda_ab.sh $t; done
    cd eval && python3 -c "import sys,numpy as np; sys.path.insert(0,'.'); from coda_pool import pool
    for n in ['M2','N1']:
        rs=[pool(t) for t in (n,n+'r2',n+'r3')]; pdl=[r['per_label']['potentially_dynamic'] for r in rs]
        print(n,'PD rec %.3f prec %.3f falsePD/min %.0f'%tuple(np.mean([[p['recall'],p['precision'],p['false_min']] for p in pdl],0)))"

Ogni run: ~1,5 min per finestra; risultati in `eval/results/coda/<TAG>_w<1-3>.json`, bag e log in
`bags/coda/runs/<TAG>_v2_coda0_<w>{,_logs}`. I TAG sono descritti in `eval/STATUS.md` (sezione "Registro delle
prove CODa"); `python3 eval/coda_pool.py <TAG> ...` stampa i numeri di ogni run. **Fra una run e l'altra non modificare gli script di esecuzione né ricompilare.**

Sequenza 1 di CODa (held-out): copiare `eval/datasets/convert_coda0.sh` cambiando sequenza e finestre (scegliere
finestre di 400-600 scansioni con annotazioni, più 200 di rodaggio prima; le annotazioni sono in
`/mnt/kloster-reichenau/datasets_SINS/CODa/3d_bbox/os1/1/`), poi per ogni finestra
`python3 eval/datasets/coda_stereo_bag.py <CODa root> bags/coda/coda1_<w>_gt.json bags/coda/coda1_<w>_stereo`.
Questi file hanno `coda0_` scritto dentro e vanno parametrizzati sulla sequenza (`coda_eval.py` invece no: prende i
percorsi da riga di comando): `eval/scripts/run_coda.sh` (e la copia in `bags/replay/`), `eval/scripts/coda_ab.sh`,
`eval/coda_compare.py`, `eval/make_coda_video.py`, `eval/coda_diag.py`, `eval/datasets/convert_coda0.sh`,
`eval/datasets/coda_stereo_bag.py` (solo nell'esempio d'uso), `eval/baselines/unimt/{make_coda_manifest.py,run_unimt_coda.sh}`,
`eval/baselines/eagermot/export_inputs_coda.py`, `eval/baselines/{dynablox,mdetector,lvdot}/run_*_coda.sh`,
`eval/baselines/mdetector/make_input_coda.py`, `eval/baselines/lvdot/make_param_coda.py`. Baseline sulla stessa finestra: `eval/baselines/dynablox/run_dynablox_coda.sh
coda1_w1 dynabloxa_coda1_w1` (idem `mdetector/run_mdetector_coda.sh`, `lvdot/run_lvdot_coda.sh`); CenterPoint +
SimpleTrack: `eval/baselines/unimt/make_coda_manifest.py` poi `run_unimt_coda.sh` (scansioni copiate in locale,
il NAS è lento); EagerMOT: `eval/baselines/eagermot/export_inputs_coda.py` + `run_eagermot_jo.py`.

Riprendere le immagini KITTI quando il NAS ha spazio: `kill -CONT $(pgrep -f "python3 eval/datasets/remote_zip.py")`.

Trappole già incontrate: `pkill -f <pattern>` uccide anche la propria shell se il pattern è nel comando (usare PID
espliciti); il container ha l'orologio un'ora indietro; il disco locale ha ~14 GB liberi (ogni bag stereo ~2-3 GB).
