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

## 3. Regole dell'utente (dalla memoria di Claude, copia in `memoria_claude/`)

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
tre finestre annotate (w1 cortile con rastrelliera e furgone, w2 corridoio interno, w3 viale), 2 min 45 s.
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
1. **Provare `track_lidar_fallback: false`** in `cfg/outdoor/tracker.yaml` (già compilato, non ancora testato):
   atteso meno PD sui paletti. `STEREO=1 bash eval/scripts/coda_ab.sh N1` ×3 rep, confronto con M2.
2. Bloccare la configurazione e **convertire CODa sequenza 1** (finestre da scegliere sulle annotazioni; serve anche
   la bag stereo) → confronto completo lì (numeri del paper). Disco locale: ~14 GB liberi.
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
