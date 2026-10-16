# Baseline esterni: come ricostruirli in fretta

Tutto quello che serve per rifare da zero il confronto con i metodi della letteratura sulle bag di laboratorio, e per
portarli su un dataset ufficiale (JRDB, THÖR-MAGNI, CODa, GrandTour) cambiando solo l'ingresso.

## Regole comuni

- **Codice originale.** Logica di rilevamento, tracking e classificazione mai toccata. Si adatta solo l'I/O: righe
  marcate `# JO` nei file di parametri (topic, intrinseci, estrinseci, modello del sensore, quota del pavimento).
  Ogni modifica al codice è una patch in `<metodo>/patches/` con il perché.
- **Stesso protocollo di v1/v2.** Odometria condivisa (`bags/replay/odom_<seq>`, `/odometry/filtered`, frame `odom`,
  pose di `base_link`), detector avviato 12 s prima della bag, riproduzione 1×, registrazione dell'uscita.
- **ROS1 in Docker.** Tutti i baseline sono ROS1 Noetic: ogni metodo ha un'immagine; le bag ROS2 vengono convertite
  al volo con `rosbags-convert --dst-typestore ros1_noetic` e cancellate dopo l'uso (6,6 GB l'una con le immagini).
- **Sensori del robot JO** (dalle bag): VLP-16 su `velodyne` (base_link + 0,03, 0,029, 0,6 m); RealSense frontale,
  depth 848×480 fx=fy=436,965 cx=431,921 cy=240,134, colore 1280×720 fx=644,308 fy=643,637 cx=650,24 cy=374,757;
  base_link → camera_link (0,336, 0,014, 0,278), colore −0,059 m in y; pavimento a −0,373 m da base_link.
- **Valutazione.** `python3 eval/baseline_eval.py --campaign r9 --reps a b c` (livelli "solo movimento" e "D e PD
  insieme"; MODA, recall, falsi allarmi/min, IDF1 dove ci sono ID). Adattatori: `eval/adapters/marker_adapter.py`
  (box pubblicati come MarkerArray LINE_LIST), `eval/adapters/points_adapter.py` (punti dinamici → DBSCAN 2D,
  eps 0,5 m, 5 punti). Allineamento al mocap: lo stesso di v2 (`results/own_r9<rep>/alignment_v2.json`).

## Comandi (uguali per ogni metodo)

```
docker build -t <immagine> eval/baselines/<cartella>                       # una volta
bash eval/baselines/<cartella>/run_<metodo>.sh 20260519_1950 prova --keep-input   # prova su una sequenza
setsid nohup bash eval/baselines/<cartella>/run_<metodo>_campaign.sh \
     > bags/<metodo>_out/campaign.log 2>&1 < /dev/null &                   # 6 sequenze × 3, riprendibile (.done)
touch bags/<metodo>_out/STOP                                                # fermare tra una run e l'altra
python3 eval/baseline_eval.py --campaign r9 --reps a b c --methods <metodo>
```

## Schede

### LV-DOT (Xu et al. 2025) — LiDAR + RGB-D + YOLO
- Sorgente: github.com/Zhefan-Xu/LV-DOT, commit `449bf2c`. Cartella `lvdot/`, immagine `lvdot-noetic:upstream`
  (osrf/ros:noetic-desktop + vision-msgs, pcl-ros, gazebo-msgs, libpcl-dev, torch 2.4.1 cu121, ultralytics 8.3.40,
  numpy 1.24.4; serve `--gpus all`).
- Config `detector_param_jo.yaml`: odometria (`localization_mode: 1`), topic, intrinseci, `image_cols 848`,
  estrinseci URDF base_link→depth/colore/velodyne, `ground_height −0,173`, `roof_height 1,627`.
- Il nodo YOLO legge il colore da un topic fisso: remap nel launch `run_detector_jo.launch` (senza rviz).
- Uscita `/onboard_detector/dynamic_bboxes`: nessun ID persistente (l'id del marker è l'indice) → solo rilevamento.
- Noto: il thread di visualizzazione di YOLO va in errore (cv_bridge con numpy nuovo); non tocca le detection.

### Dynablox (Schmid et al., RA-L 2023) — solo LiDAR
- Sorgente: github.com/ethz-asl/dynablox, commit `44a1537`, dipendenze da `https.rosinstall` (voxblox …).
  Cartella `dynablox/`, immagine `dynablox-noetic:upstream` (osrf/ros:noetic-desktop-full, catkin build).
- Patch: output-only (`export_cluster_id.patch`: il marker porta l'ID del cluster tracciato) e build-only (tolte
  dal package.xml le dipendenze `ouster_ros`, `lidar_undistortion`, usate solo dalle demo DOALS).
- Config `motion_detector_jo.yaml`: modello VLP-16 (1824 colonne, 16 righe, 30°), `global_frame_name: odom`.
  TF: `/tf` della bag di odometria (odom→base_link) + static base_link→velodyne nel launch.
- Uscita `/motion_detector/visualization/clusters` (pubblicata solo se non vuota: topic assente = niente moto).
- Deterministico: le ripetizioni coincidono.

### M-detector (Wu et al., Nat. Comm. 2024) — solo LiDAR
- Sorgente: github.com/hku-mars/M-detector, commit `0645dab`. Cartella `mdetector/`, immagine
  `mdetector-noetic:upstream` (desktop-full + libtbb-dev + Livox-SDK + livox_ros_driver, dipendenza di build).
- Patch build-only: tolte le righe CMake che compilano TBB da un percorso della macchina degli autori.
- Ingresso: `make_input.py` scrive la nuvola grezza e **un'odometria per nuvola** nel frame velodyne, interpolata al
  timestamp della nuvola (il nodo accoppia la n-esima nuvola con la n-esima odometria, come esce da FAST-LIO).
- Config `jo.yaml` da `nuscenes0.yaml` (LiDAR rotante più vicino): topic, `frame_dur 0.1`, FOV ±15°,
  `ver_resolution_max 0.035`, ingombro del robot attorno al LiDAR.
- Uscita `/m_detector/point_out` (punti dinamici, frame odom) → DBSCAN comune → box senza ID.

### Xu et al. (RA-L 2023) — solo RGB-D, con e senza YOLO
- Sorgente: github.com/Zhefan-Xu/onboard_detector, commit `4b4be99`. Cartella `xu_rgbd/`, immagine
  `xu-rgbd-noetic:upstream` (FROM lvdot-noetic, workspace sostituito).
- Variante di default (`run_xu_rgbd.sh`, `run_detector_jo.launch`, prefisso `xurgbd`): solo depth.
- Variante con modulo YOLO (`run_xu_rgbd_yolo.sh`, `run_detector_jo_yolo.launch` = upstream
  `detector_with_learning_module.launch`, prefisso `xuyolo`). Il modulo vuole colore + depth allineata al colore,
  letti con un'unica dimensione d'immagine (quella della depth). Adattatore `make_input_yolo.py`: colore ridotto a
  848×480 (intrinseci scalati in `detector_param_jo_yolo.yaml`), depth riproiettata nella camera colore come
  l'`align` di librealsense (offset URDF depth→colore 0,059 m, vince la depth più vicina), pubblicata su
  `/camera/aligned_depth_to_color/image_raw`. Il rilevatore YOLO upstream gira su CPU (come nel codice).
- Config `detector_param_jo.yaml`: come LV-DOT per depth/colore/odometria/pavimento.
- Uscita `/onboard_detector/dynamic_bboxes`, senza ID persistenti.

### UniMT + SimpleTrack (He et al. 2025) — LiDAR + camera, appreso
- Sorgente: github.com/TossherO/ros_packages, commit `aee3167` (nodo `perception_test/scripts/infer2.py`). Cartella
  `unimt/` (`git clone` in `unimt/ros_packages_src/` prima del build: il Dockerfile lo copia), immagine `unimt-noetic:upstream`: CUDA 12.1 devel Ubuntu 20.04 + ROS Noetic
  ros-base; Python 3.10 standalone via `uv` (deadsnakes non serve più focal) con torch 2.1.2 cu121, mmcv 2.1.0,
  mmdet 3.2.0, mmdet3d 1.4.0, spconv-cu120 2.3.6, flash-attn 2.6.3 (wheel), numpy 1.26.4. rospy (puro Python)
  usato da Python 3.10; cv_bridge non serve.
- Pesi (Google Drive degli autori, `gdown --folder https://drive.google.com/drive/folders/1PXP8glbf5VoRDix-hlE0TBfx-48icKI7 -O eval/baselines/unimt/ckpts`,
  non versionati): si usa
  `wheelchair.pth` montato come `src/detection/ckpts/cmdt_wheelchair.pth` (config `cmdt_wheelchair.py`,
  ottimizzato da `cmdt_coda_16lines`: LiDAR a 16 linee come il nostro VLP-16, camera 640×480).
- Nodo `infer_jo.py` = copia di `infer2.py`, solo I/O (`# JO`): topic delle bag, immagine senza cv_bridge,
  intrinseci colore della bag (1280×720; il `ResizeCropFlipImage` del pipeline la porta a 480×640 e aggiorna
  `cam2img`), `lidar2cam` dall'URDF, ego2global = posa del velodyne in odom, niente predizione di traiettoria.
  Uscita `/unimt/tracks`: MarkerArray LINE_LIST, id = ID di traccia SimpleTrack, solo tracce `alive` (come upstream).
- Classifica persone e ciclisti, non il moto: tutti i box contano come "dinamici" (stesso trattamento di LV-DOT).
- Range del modello solo in avanti (x 0–14,4 m, y ±9,6 m nel frame LiDAR).
- Runtime: il run script aspetta la riga `finish init` (caricamento del modello ~20 s) e poi i soliti 12 s;
  `data_dict` definito prima dei subscriber; `buff_size=2**24` sui subscriber di nuvola e immagine (con
  `queue_size=1` rospy scarta i messaggi grandi: senza, 1,8 Hz invece dei 5 Hz di progetto). Una scansione su due,
  come upstream; ~60 ms per frame su GPU.

### JRMOT (Shenoi et al., IROS 2020) — bloccato
- github.com/StanfordVL/JRMOT_ROS: manca il rilevatore 2D (va collegato), serve Gurobi (licenza legata alla macchina),
  messaggi SPENCER, pesi F-PointNet su JRDB. Alternativa sui dataset: EagerMOT o SPENCER.

## Trappole incontrate
- Docker: `python3.10` della venv va chiamato con un wrapper, non un symlink (il symlink salta la venv →
  "externally-managed-environment"); il nodo YOLO di Xu ha shebang `python`: symlink a python3 nel container.
- `pkill -f <pattern>` uccide anche la shell che lo lancia se il pattern compare nel comando: usare `'[p]attern'`.
- Disco: le build lasciano GB di cache, `docker builder prune -f` (36 GB liberati).
- Un errore di compilazione nascosto da `| tail`: nei Dockerfile si controlla che l'eseguibile esista (`test -x`).
- DNS di Docker Hub a volte fallisce: i build hanno 2–3 tentativi.
