# Benchmark status

Vedi `../BENCHMARK_PLAN.md` per il protocollo e `../PAPER_PLAN.md` per il piano del paper.

---

## Fase 0: inventario (2026-10-13)

### Hardware e ambiente

| Voce | Valore |
|---|---|
| GPU | NVIDIA GeForce RTX 4060 Laptop, 8 GB |
| Driver / CUDA | 580.178.04 / CUDA 13.0 (driver); torch 2.14.0+cu130 nel container, `cuda.is_available() = True` |
| CPU / RAM | 20 thread / 31 GB |
| ROS | ROS 2 Jazzy (container `jo-sim_lv_dot:latest`) |
| Docker | 29.8.0, nvidia-container-toolkit installato |
| Disco libero su `/` | **62 GB** su 503 GB (88% usato) |

### Build

- `jo_detector` (v1) e `jo_detector_v2` compilano nel container (`colcon build`, nessun errore,
  solo warning `-Wmaybe-uninitialized` in `static_structures_node.cpp:884`).
- **L'immagine Docker è vecchia** (creata il 2026-09-14). Il suo `/entrypoint.sh` compila ancora
  `onboard_detector` e non compila affatto v2; mancava anche `lap`, già aggiunto al Dockerfile del
  repo il 2/10. Il container è stato ricreato il 2026-10-13 con i mount `jo_detector*` e i due
  pacchetti sono stati compilati a mano. **Da fare: `docker compose build`.**
- Nel container in esecuzione sono stati installati a mano `lap` e `rosbags`, che andranno persi
  alla prossima ricreazione. Dopo il rebuild dell'immagine `lap` sarà incluso; `rosbags` va aggiunto
  al Dockerfile se serve dentro il container.

### Conflitto numpy nell'immagine (preesistente, bloccante per v1)

Il Dockerfile installa `numpy>=2` via pip, mentre `python3-scipy` e `cv_bridge` (apt/ROS) sono compilati
contro numpy 1.x. Conseguenze:
- il nodo `yolov11_detector_node.py` di v1 va in crash all'import di `cv_bridge`, quindi **v1 non ha YOLO**;
- `motmetrics` (che importa scipy) non funziona nel container, perciò `eval_tracking.py` va eseguito sull'host.

I nodi YOLO di v2 non usano `cv_bridge` e funzionano.
Possibili soluzioni (decisione aperta): togliere `cv_bridge` dal nodo YOLO di v1 (conversione
numpy diretta, come fa v2), oppure creare un venv separato per l'harness.

### Harness esistente

- `jo_detector_v2/eval/` (`extract_obstacles.py`, `eval_tracking.py`, `GT_SCHEMA.md`) è già
  committato e pushato nel submodule v2 (commit `b588136`).
- Il test di regressione sintetico **passa** sull'host: MOTA 100%, IDF1 100%, debounce latency 1.50 s.
- `eval/detector_eval_pipeline.py` (copiato da `bags/`, che è in `.gitignore`) valuta la classificazione D/PD/S
  contro il GT Qualisys, con allineamento spaziale e temporale stimato. Esempio: `eval/scripts/run_qualisys_example.sh`.

### Smoke test su bag del lab

Script: `eval/scripts/smoke_test.sh <v1|v2> [bag] [durata_s]`, da copiare in `bags/smoke/` (montato nel container);
riepilogo con `eval/scripts/summarize_obstacles.py <bag_registrata>`. Esegue GLIM + EKF (`localization_detector.launch.py`)
\+ il detector, headless, su `ROS_DOMAIN_ID=77`, e registra l'output.

| Metodo | Bag | Esito |
|---|---|---|
| v2 | `20260910_124939_validation_lab`, 80 s | **OK**: 787 msg a 9,9 Hz, 64 track ID, 1445 osservazioni D + 634 PD; ICP valido su entrambe le camere; YOLO-sem a circa 19 ms/frame |
| v1 | `bags_benchmark/duccio_test_20260519_1942_0`, 65 s | **Nessun output.** Il detector resta su "Waiting for LiDAR to depth camera extrinsic calibration" anche se `velodyne→camera_refined` è presente su `/tf`. Il flag viene impostato solo nel callback sincronizzato con l'odometria: va indagata la catena sync/odometria. In più YOLO è in crash (vedi numpy). |
| v1 | `validation_lab` | non compatibile con la config attuale: v1 si aspetta `depth/image_rect_raw`, la bag ha solo `aligned_depth_to_color/zstd` |

### Dati disponibili localmente

| Cartella | Contenuto | Note |
|---|---|---|
| `bags/bags_benchmark/` (37 GB) | 6 sequenze, 49–71 s, 19/05/2026 | solo camera frontale, depth raw, **nessuna odometria e `/tf` vuoto** |
| `bags/qualysis/` (3 MB) | GT mocap delle stesse 6 sequenze, `/rigid_bodies` circa 10 Hz | body `person` + `JO`; una sola persona |
| `bags/20260910_124939_validation_lab` (4,8 GB) | 99 s, 2 camere, depth allineata zstd | nessun GT |
| `bags/indoor_test`, `bags/outdoor_test` (34 GB) | 12 sequenze brevi (21–53 s), aprile 2026 | nessun GT |

### Stima dello spazio per i dataset esterni

Ordini di grandezza, da verificare sulle pagine di download prima di scaricare:
- **JRDB**: centinaia di GB (bag ROS1 + immagini a 360°); la sola partizione train supera comunque lo spazio libero.
- **CODa**: nell'ordine dei TB nella versione completa; servirebbe un sottoinsieme di sequenze.
- **DOALS**: decine di GB.

**Con 62 GB liberi non si scarica nessuno dei dataset con box 3D.** Serve un disco esterno o lo
storage del laboratorio prima della Fase 2.

### Cosa manca per chiudere la Fase 0

1. Rebuild dell'immagine Docker e risoluzione del conflitto numpy/cv_bridge.
2. Far produrre output a v1 su una bag del lab (catena sync/odometria).
3. Decidere dove salvare i dataset esterni.
