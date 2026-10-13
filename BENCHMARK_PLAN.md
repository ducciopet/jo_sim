# Benchmark plan — onboard_detector v1/v2 vs letteratura, su più dataset

Specifica per Claude Code in locale. Obiettivo: produrre le tabelle di confronto per il paper, cioè
**N algoritmi × M dataset**, con un protocollo unico, riproducibile e documentato.
Lavora una fase alla volta. A fine fase aggiorna `eval/STATUS.md` con cosa funziona, cosa no e i numeri ottenuti.

---

## Regole che valgono sempre

1. **Niente tuning sui dati di valutazione.** Per ogni dataset fissa subito uno split `tune/` vs `eval/`
   (scritto in `eval/splits.yaml`). I parametri di *ogni* metodo, v2 compreso, si toccano solo guardando `tune/`.
2. **Stessi input per tutti.** Stessa odometria, stessi pesi YOLO dove un metodo usa YOLO (v1, v2, LV-DOT),
   stesse sequenze, stessi frame valutati.
3. **Parametri per metodo fissi tra dataset.** Cambia solo la config del sensore (topic, intrinseci,
   estrinseci, numero di fasci). Qualsiasi altra modifica va motivata in `eval/configs/<metodo>/<dataset>/NOTES.md`.
4. **Tracciabilità.** Ogni run scrive un `manifest.json`: git hash del metodo, config usata, sequenza,
   hardware, data. Senza manifest il run non conta.
5. **Non modificare la logica dei baseline.** Solo adattatori di I/O. Se un baseline va patchato per
   girare, la patch va in `eval/baselines/<nome>/patches/` con una spiegazione.

---

## Algoritmi

| ID | Metodo | Sensori | Repo | ROS | Output nativo |
|---|---|---|---|---|---|
| `v2` | onboard_detector_v2 | LiDAR + RGB-D multi-camera | questo repo | ROS2 | `jo_msgs/ObstacleArray` su `/onboard_detector/tracked_dynamic_obstacles` |
| `v1` | onboard_detector (v1) | LiDAR + RGB-D | repo interno | ROS2 | stesso topic/tipo di v2 |
| `lvdot` | LV-DOT (Xu et al., 2025) | LiDAR + RGB-D | github.com/Zhefan-Xu/LV-DOT | ROS1 Noetic | MarkerArray `dynamic_bboxes`, `tracked_bboxes` + srv `GetDynamicObstacles` |
| `xu_rgbd` | onboard_detector (Xu et al., RA-L 2023) | solo RGB-D | github.com/Zhefan-Xu/onboard_detector | ROS1 Noetic | come LV-DOT |
| `dynablox` | Dynablox (Schmid et al., RA-L 2023) | solo LiDAR | github.com/ethz-asl/dynablox | ROS1 Noetic | punti dinamici |
| `mdetector` | M-detector (Wu et al., Nat. Comm. 2024) | solo LiDAR | github.com/hku-mars/M-detector | ROS1 (richiede livox_ros_driver anche con LiDAR non-Livox) | punti dinamici |
| `ab3dmot` | AB3DMOT (Weng et al., IROS 2020) | tracker puro | github.com/xinshuoweng/AB3DMOT | Python | track 3D |

Note:
- **v1 deriva da LV-DOT.** Il pacchetto si chiama `onboard_detector` e contiene `dynamicDetector.cpp` con
  `lidarPoseCB` e `classificationCB`. Annota in `eval/STATUS.md` quali parti di v1 differiscono da LV-DOT,
  perché il paper deve dichiararlo.
- **`ab3dmot` riceve in input le detection *pre-tracking* di v2** (DBSCAN + YOLO fusi, topic
  `/onboard_detector_v2/dbscan_detector/fused/clusters` o equivalente). Serve a isolare il contributo del `tracker_node`.
- **Metodi punto-a-punto (`dynablox`, `mdetector`)**: converti i punti dinamici in box con *un unico*
  clustering (DBSCAN, stessi parametri per entrambi, in `eval/adapters/points_to_boxes.py`), poi box orientati
  minimi. Così LV-DOT li ha confrontati. Riporta anche il caso "senza tracking" e dichiaralo.
- **Licenze**: M-detector è GPL-2.0. Va usato solo come eseguibile esterno, il suo codice non entra nel repo.

---

## Dataset

| ID | Dataset | Sensori utili | GT | Accesso |
|---|---|---|---|---|
| `jrdb` | JRDB (Martín-Martín et al., TPAMI 2021) | 2× Velodyne-16, anello RGB, ZED RGB-D, odometria ruote | box 3D persone con ID | registrazione sul sito JRDB; la GT del test set è nascosta → usa **solo il train** e dividilo in tune/eval |
| `coda` | CODa (Zhang et al., IJRR 2024) | Ouster OS1-128, stereo RGB, Azure Kinect, ZED 2i | box 3D, 53 classi, instance ID persistenti (gap ≤ 3 s) | devkit `github.com/ut-amrl/coda-devkit` |
| `doals` | DOALS (Pfreundschuh et al., ICRA 2021) | solo LiDAR (OS1-64) | punti dinamici | script `dynablox_ros/scripts/download_doals_data.sh` |
| `own` | bag del lab (`bags/*_validation_lab` ecc.) | il vostro stack | da annotare | locale |

Note:
- **Odometria**: se un dataset non fornisce un'odometria utilizzabile come `/odometry/filtered`, generala
  **una volta** con un LIO/LO (KISS-ICP è il più semplice, oppure GLIM come sul robot). Salvala nella bag e
  dalla a *tutti* i metodi.
- **Depth**: verifica subito se le bag JRDB contengono la depth ZED. Se manca, usa per v1/v2/LV-DOT la
  depth ottenuta proiettando il LiDAR sull'immagine, e dichiaralo. Così non è più un confronto RGB-D puro.
- **Etichetta "dinamico" nella GT**: deriva lo stato dal GT stesso. Una track GT è *dynamic* nel frame t se
  la sua velocità orizzontale, lisciata su una finestra di ±0.5 s, supera 0.3 m/s. Rendi la soglia un
  parametro e riporta una sensitivity analysis. *Potentially dynamic* = classe GT in {person, rider, ...}
  ma ferma.
- **DOALS** non ha box. Serve per (a) controllare che Dynablox e M-detector, nel nostro harness, riproducano
  i numeri pubblicati (IoU punto-a-punto), e (b) il confronto LiDAR-only di v1/v2 a livello di punti.

---

## Formato unico dei risultati

Ogni metodo, tramite il suo adattatore, produce `results/<dataset>/<sequence>/<method>/tracks.csv`:

```
t_ns,track_id,x,y,z,l,w,h,yaw,vx,vy,vz,state,class,score
```

- Coordinate nel **frame globale dell'odometria** comune (vedi sopra).
- `state` ∈ {`dynamic`, `potentially_dynamic`, `static`}. I metodi che non distinguono usano `dynamic`/`static`.
- `track_id = -1` se il metodo non traccia.
- Più `runtime.csv`: `t_ns,stage,ms`, e `manifest.json`.

GT convertita nello stesso formato: `gt/<dataset>/<sequence>/tracks.csv`.

Struttura:
```
eval/
  STATUS.md  splits.yaml
  datasets/{jrdb,coda,doals}/   # convert_to_ros2bag.py, gt_to_csv.py, README.md
  adapters/                     # <method>_to_csv.py, points_to_boxes.py
  baselines/<nome>/             # Dockerfile, run.sh, patches/
  configs/<metodo>/<dataset>/
  metrics/                      # detection.py, tracking.py, dynamic_cls.py, report.py
  run_matrix.py                 # lancia metodo × dataset × sequenza
results/                        # non versionato (.gitignore), solo i CSV finali in eval/tables/
```

---

## Metriche

1. **Detection di ostacoli dinamici**: precision/recall/F1 a IoU 3D 0.3/0.5/0.7 + curva F1-vs-IoU.
   È lo stesso protocollo di LV-DOT, quindi confrontabile con il loro paper. Più errore di posizione medio [m].
2. **Tracking**: HOTA, MOTA, IDF1, ID switch (TrackEval o py-motmetrics; matching su distanza dei centri
   ≤ 1 m in BEV oppure IoU 3D ≥ 0.25, dichiarando quale).
3. **Classificazione dinamico/potenzialmente-dinamico**: P/R/F1 per stato, calcolati solo sulle coppie
   associate correttamente, + latenza di commutazione (GT diventa dynamic → metodo dice dynamic).
4. **Runtime**: ms per frame per stadio, CPU% e memoria, sulla stessa macchina. Indica GPU e CPU.
5. **Statistica**: media ± dev. std. per sequenza; bootstrap CI al 95% sulle sequenze per le differenze v2 vs altri.

---

## Fasi

**Fase 0 — Inventario (prima di scrivere codice)**
- Stampa: GPU/driver/CUDA, distro ROS2, Docker + nvidia-container-toolkit, spazio libero.
- Verifica che v1 e v2 compilino e girino su una bag del lab.
- Committa la cartella `eval/` esistente (`extract_obstacles.py`, `eval_tracking.py`), che oggi non è nel repo pubblico.
- Scrivi in `eval/STATUS.md` cosa manca e stima dello spazio necessario per i dataset.

**Fase 1 — Harness**
- Formato unico, convertitori GT, `metrics/*`, `report.py` che genera tabelle LaTeX + CSV.
- Test su dati sintetici: GT nota + predizioni perturbate in modo noto (offset, ID swap iniettati,
  ritardo di classificazione iniettato). Ogni metrica deve recuperare il valore iniettato.

**Fase 2 — v2 su una sequenza JRDB, end-to-end**
- Converti una sequenza (ROS1 → ROS2 con `rosbags-convert`), crea `cfg/jrdb/` (remap topic, intrinseci,
  estrinseci, numero camere), lancia v2, estrai, valuta. Obiettivo: un primo numero, anche brutto.
- Poi v1 sulla stessa sequenza.

**Fase 3 — Baseline ROS1**
- Un container Docker Noetic per ciascuno (`lvdot`, `xu_rgbd`, `dynablox`, `mdetector`).
  Niente bridge: bag ROS2 → bag ROS1 offline, `rosbag play` + `rosbag record` dei topic di output nel
  container, poi estrazione con `rosbags` in Python.
- **Sanity check obbligatorio**: Dynablox su DOALS deve riprodurre i numeri del suo paper entro qualche punto
  percentuale. Se non li riproduce, l'harness ha un bug: fermati e indaga prima di andare avanti.
- Anche LV-DOT: falla girare sulla sua bag demo (`corridor_demo.bag`) e confronta qualitativamente con il loro video.

**Fase 4 — Matrice completa**
- `run_matrix.py` su tutti metodo × dataset (eval split). Logga i fallimenti, non nasconderli: un metodo
  che non può girare su un dataset (es. `xu_rgbd` senza depth) va in tabella come "n/a" con il motivo.

**Fase 5 — Ablation di v2**
- Varianti da `tracker_node.cpp`: senza soppressione da disocclusione, dt nominale invece che reale,
  senza class-continuity, senza YOLO (LiDAR-only), solo depth (camera-only). Richiede flag nel codice,
  non solo config: aggiungili.

**Fase 6 — Report**
- `eval/tables/*.tex` + `eval/figures/*.pdf` (F1-vs-IoU, runtime breakdown), e `eval/RESULTS.md` che
  riassume, per ogni numero, il run e il manifest da cui viene.

---

## Domande da fare all'utente quando servono (non indovinare)

- Quante sequenze per dataset sono sostenibili come tempo di calcolo.
- Se annotare anche le bag del lab, e con che strumento.
- Venue target (cambia il peso tra ablation e confronto esterno).
