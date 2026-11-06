# Fedeltà dei baseline (audit, 06/11/2026, PC del laboratorio)

Obiettivo: ogni baseline deve girare nelle condizioni in cui ha ottenuto i punteggi del suo paper ed essere integrato
correttamente; ogni scostamento è dichiarato qui e nella tabella dei risultati. SINS gira sempre online a 10 Hz con il
profilo congelato. I baseline offline possono girare offline o rallentati, ma va scritto.

Stato delle verifiche:
- [V] verificato su codice, config o dati di questo repo;
- [P] dal paper, da ricontrollare sul PDF prima di pubblicare;
- [?] da verificare.

## 1. Problema trasversale: lo split di CODa e i pesi appresi [V]

Lo split ufficiale di CODa (`CODa/metadata/<seq>.json`, `ObjectTracking`) è **casuale per fotogramma dentro ogni
sequenza**: circa 70% training, 15% validation, 15% testing, in **tutte** le sequenze annotate. Fotogrammi
3d_bbox delle nostre finestre, per split (train / val / test):

| Seq | Finestra | train | val | test |
|---|---|---|---|---|
| 10 | w1 / w2 / w3 / w4 / w5 | 145/135/93/140/68 | 25/35/21/30/13 | 29/30/16/30/19 |
| 11 | w1 / w2 / w3 | 348/71/141 | 78/14/28 | 74/15/31 |
| 12 | w1 / w2 / w3 | 147/138/183 | 24/28/48 | 28/33/39 |
| 16 | w1 | 519 | 94 | 87 |

Conseguenza:
- CenterPoint, addestrato dagli autori UniMT su `coda_infos_train.pkl` (`unimt/centerpoint_coda_cfg.py`), ha visto
  circa il 70% dei fotogrammi valutati, comprese le held-out 11 e 12. Lo stesso vale per EagerMOT, che usa quei box 3D.
- "Held-out" vale per SINS (nessun parametro scelto su 11/12), **non** per i metodi appresi su CODa.
- Non basta escludere singole sequenze: lo split è dentro ogni sequenza.
- **Correzione proposta:** valutare tutti i metodi (SINS compreso) sui soli fotogrammi `testing` dello split ufficiale,
  circa 15% delle finestre, non usati in addestramento né in validazione. È lo stesso sottoinsieme per tutti. Va
  aggiunta in `eval/coda_eval.py` un'opzione `--frames testing`, con la lista dei fotogrammi da `metadata/<seq>.json`.
  Il punteggio sull'intera finestra resta come cifra secondaria, con la nota "CenterPoint/EagerMOT: train set".
- Il paper CODa conferma lo split "70%, 15%, 15% di ogni sequenza annotata". Nella pipeline ufficiale (coda-models)
  il `val` usato durante l'addestramento carica il **test**: i fotogrammi `testing` non sono in addestramento, ma
  possono essere serviti a scegliere il checkpoint. Da dichiarare nella tabella.

## 2. Schede

### Dynablox (Schmid et al., RA-L 2023) — LiDAR, geometrico, online
- Paper (arXiv 2304.10049, letto 06/11): DOALS, Ouster OS1-64 a 10 Hz, 8 sequenze in 4 ambienti; 10 fotogrammi
  annotati per sequenza. Metrica: **IoU per punto** dei punti dinamici, media sui 10 fotogrammi. Tabella I:
  **83,8%** a raggio pieno, **86,0%** entro 20 m (Station 87,3, Shopville 87,8, HG 86,0, Niederdorf 83,1).
  Runtime 17 FPS (58 ms) su un NUC con CPU AMD 4800U. Parametri del paper: voxel 0,2 m, τ_s = 2, τ_w = 5,
  τ_c = 20 (cluster minimo, in voxel), integrazione 20 m. La fonte delle pose su DOALS non è dichiarata; i dati
  nuovi usano FAST-LIO2. Il de-skew non è dichiarato.
- Config ufficiale `default.yaml` (main): `max_range 20`, `min_cluster_size 20`, `counter_to_reset 150`,
  `temporal_buffer 2`, `burn_in_period 5`, voxel 0,2 [V, README]. Il nostro `motion_detector_coda.yaml` ha gli
  stessi valori, tranne **`max_range 25`** (scelto per uguagliare il raggio di valutazione) [V].
  Il `min_cluster_size 20` è quello del paper, non cambiato [V].
- Ingresso qui: odometria condivisa (non FAST-LIO), nessun de-skew [V].
- Deviazioni da dichiarare o correggere:
  1. `max_range` 25 invece di 20: riportarlo a 20 (valore del paper e del default), oppure dichiararlo;
  2. pose dall'odometria del dataset invece che da FAST-LIO2: nessuna correzione, si dichiara;
  3. patch di uscita per l'ID del cluster (non cambia il rilevamento).
- Riproduzione (fase B): DOALS (`download_doals_data.sh`, projects.asl.ethz.ch, registrazione da verificare),
  script `evaluate_data.py` nel repo. Obiettivo: 86,0% di IoU entro 20 m (tolleranza pochi punti).

### M-detector (Wu et al., Nat. Commun. 2024) — LiDAR, geometrico, online
- Paper (PMC10771495, letto in parte 06/11): KITTI, nuScenes, Waymo, AVIA-Indoor (119 sequenze, 51 min).
  Metrica: IoU per punto "evento / non evento". Unico numero leggibile nel testo: **KITTI, modalità frame-out,
  IoU 0,746**. Gli altri sono nella Fig. 5a, da leggere sul PDF [?]. Pose: **FAST-LIO2 per tutti i metodi**; le nuvole
  entrano già compensate dal moto (topic `/cloud_registered_body`). Runtime su i7-10700: 87,3 ms per scansione
  KITTI (periodo 100 ms), quindi appena in tempo reale.
- Repo: config per kitti, nuscenes, waymo, avia, mid360; launch `detector_<dataset>.launch`, `cal_recall.launch`
  per l'IoU; bag degli autori su Google Drive, dati del paper su Figshare (10.6084/m9.figshare.24481966).
- Qui `coda.yaml` parte da `nuscenes0.yaml` (`dataset: 1`, 32 linee a 20 Hz) [V]. Per l'OS1-128 a 10 Hz è più
  vicina la config KITTI (64 linee, 10 Hz, ~120k punti per scansione, come i 131k di CODa): **da rifare con base
  kitti** e dichiarare.
- Ingresso qui: odometria condivisa interpolata, una posa per nuvola, nuvola **non** compensata [V]. Nel paper le
  nuvole arrivano compensate da FAST-LIO2: deviazione da correggere (de-skew con l'odometria) oppure da dichiarare.
- A 1x su questo hardware perde metà delle scansioni da 131k punti (il paper stesso è a 87 ms su 100 ms per i 120k
  di KITTI) [V]. Usare **0,25x**, dichiarato "offline, rallentato 4x", e verificare le coppie nuvola/posa.
- Uscita a punti → DBSCAN comune (eps 0,5 m; min 5 punti nel lab, 20 su CODa): post-processing nostro, uguale per
  tutti i metodi a punti, da dichiarare [V].
- Riproduzione (fase B): KITTI frame-out 0,746. Servono le etichette degli autori (Figshare o Drive) e FAST-LIO2.
  SemanticKITTI è già sul NAS, ma il paper **non** riporta un numero SemanticKITTI leggibile: usarlo solo se le
  etichette degli autori coincidono con le classi "moving" di SemanticKITTI [?].

### LV-DOT (Xu et al., arXiv 2502.20607, 2025) — LiDAR + RGB-D + YOLO, online
- Paper (letto 06/11): quadricottero con **Livox Mid-360 + RealSense D435i (RGB-D attiva)**, pose da Fast-LIO2,
  Jetson Orin NX; valutazione **solo al chiuso** (7 scenari + 2 prove con motion capture, 635 fotogrammi annotati,
  dataset non pubblico). KITTI/nuScenes scartati dagli autori perché all'aperto. Metriche: precisione/recall/F1 dei
  box 3D dinamici a IoU 0,3/0,5/0,7. LV-DOT completo: **P 74,5% R 88,3% F1 0,794** (IoU 0,3); 0,764 (0,5); 0,641
  (0,7). Tempi: LiDAR 8,3 ms, depth 10,2 ms, YOLO 34,2 ms. LiDAR ridotto a **N_max = 3000 punti** prima del DBSCAN.
- Nel lab le condizioni sono vicine a quelle del paper: al chiuso, VLP-16 (16 linee, 360°) al posto del Mid-360,
  D435 RGB-D attiva, odometria condivisa al posto di Fast-LIO2 [V]. Config upstream, solo I/O e altezze del
  pavimento/soffitto adattate [V].
- Su CODa è **fuori dalle condizioni del paper**: all'aperto, depth stereo ZED (non RGB-D attiva), OS1-128.
  Recall 1-4% [V]. **Proposta: escluderlo da CODa e JRDB** (come già per JRDB), con la motivazione "il paper lo
  valuta solo al chiuso con RGB-D attiva". In alternativa la modalità solo-LiDAR di LV-DOT (prevista dal README) si
  potrebbe usare su CODa, ma resta all'aperto: decisione dell'utente.
- Riproduzione (fase B): impossibile (dataset non pubblico). C'è solo il bag demo `corridor_demo.bag` (cmu.box.com),
  senza etichette. Fedeltà = config upstream intatta + condizioni del lab vicine a quelle del paper.

### CenterPoint + SimpleTrack (UniMT, He et al. 2025) — appreso
- Pesi CenterPoint CODa dal repo degli autori (TossherO/ros_packages), config `centerpoint_coda_cfg.py`, nessuna chiave
  mancante [V]. Il repo non contiene la creazione dei `coda_infos_*.pkl`: quei nomi vengono dalla pipeline ufficiale
  ut-amrl/coda-models, dove **`val` carica `coda_infos_test.pkl`** ("hack bc test split in coda is used as
  validation"). Quindi anche i fotogrammi `testing` possono essere stati usati per scegliere il checkpoint [P].
  Restano comunque mai visti in addestramento, e sono l'unico sottoinsieme non addestrato.
- Paper CODa (arXiv 2309.13549, Tabella IV): CenterPoint (voxel) **AP_BEV 82,08 / AP_3D 76,92**, PV-RCNN 92,08 /
  91,11; mAP su auto, pedoni, ciclisti, IoU 0,7/0,5/0,5, media easy/medium/hard, split **medium**, LiDAR 128 linee.
  Questi pesi però sono di UniMT, non degli autori CODa: il loro numero non è dichiarato [?].
- Qui: script upstream degli autori per CODa, soglia 0,1, SimpleTrack `coda_configs/diou.yaml` [V].
- Deviazioni: contaminazione train (sezione 1); classifica oggetti, non moto (tutte le tracce "dinamiche"); offline.
- Riproduzione (fase B): mAP sui fotogrammi `testing`/val CODa con la valutazione KITTI-style. Obiettivo: quello che
  dichiara UniMT [?], altrimenti l'ordine di grandezza della Tabella IV (AP_3D ~77). Richiede l'immagine
  `unimt-noetic` e i pesi (Google Drive degli autori, senza registrazione).

### EagerMOT (Kim et al., ICRA 2021) — tracker su rilevatori appresi
- Paper [P]: KITTI (HOTA) e nuScenes (AMOTA), con PointGNN/CenterPoint 3D + rilevatori 2D. Codice `2792a76`,
  parametri pedoni `NUSCENES_BEST_PARAMS` [V].
- Qui: box 3D CenterPoint CODa (score > 0,3) + YOLO della replay SINS [V].
- Deviazioni: eredita la contaminazione di CenterPoint (sezione 1); il rilevatore 2D è YOLO, non quello del paper
  (sono intercambiabili per design: dichiarare).
- Riproduzione (fase B): nuScenes val con i box CenterPoint pubblicati (dataset grande, registrazione nuScenes →
  da elencare per l'utente) oppure KITTI tracking (registrazione KITTI) [?].

### Xu et al. RGB-D (RA-L 2023) — solo lab (CODa non ha RGB-D attiva; JRDB non ha depth)
- Solo per le 6 sequenze Qualisys, già fedeli (config upstream, I/O adattato) [V].

## Riepilogo deviazioni (06/11)

| Metodo | Dataset | Deviazione | Azione |
|---|---|---|---|
| Dynablox | CODa | `max_range` 25 invece di 20 (paper, default) | riportare a 20 |
| Dynablox | JRDB | `max_range` 15 | dichiarare (raggio di valutazione 15 m) |
| Dynablox | lab, JRDB | LiDAR a 16 linee: il paper è su OS1-64; LV-DOT stesso nota che Dynablox soffre con LiDAR radi | dichiarare |
| Dynablox, M-detector | tutti | pose dall'odometria del dataset, non FAST-LIO2; nuvole non compensate | de-skew con l'odometria o dichiarare |
| M-detector | CODa | config base nuScenes (32 linee, 20 Hz) per un OS1-128 a 10 Hz | rifare con base KITTI |
| M-detector | CODa | 1x perde scansioni | 0,25x, dichiarato "offline 4x" |
| M-detector | tutti | punti → DBSCAN comune (nostro) | dichiarare |
| LV-DOT | CODa, JRDB | all'aperto, senza RGB-D attiva (paper: solo al chiuso, D435i) | **escluso** (decisione utente 06/11) |
| CenterPoint, EagerMOT | CODa | ~70% dei fotogrammi valutati in training; `testing` forse usato come validazione | valutare sui fotogrammi `testing` |
| CenterPoint, EagerMOT | tutti | classificano oggetti, non moto | tutte le tracce "dinamiche", dichiarare |
| SINS | – | modalità solo camera non disponibile senza modifiche | da provare con bassa priorità (utente 06/11): ingresso alternativo dietro un flag spento di default, a fine lavoro |

## 3. Sensori: dataset × metodi, e SINS per modalità

SINS gira con tutti i sensori che il dataset offre. Il confronto si fa anche a parità di sensori, per separare il
guadagno della fusione da quello del metodo.

| Dataset | Sensori disponibili | SINS (pieno) | Dynablox, M-detector | Xu RGB-D | LV-DOT | CenterPoint+ST, EagerMOT |
|---|---|---|---|---|---|---|
| Lab Qualisys (6 seq) | VLP-16 + RealSense D435 (RGB-D) | LiDAR + RGB-D | LiDAR | RGB-D | LiDAR + RGB-D (sensori del paper) | LiDAR + RGB (pesi wheelchair, 16 linee) |
| CODa 0/10/16, 11/12 | OS1-128 + stereo (cam0/cam1) + ZED (solo seq 0, 1, 10-12) | LiDAR + stereo (STEREO=1) e ZED dove c'è | LiDAR | escluso: nessuna RGB-D attiva | **deviazione**: depth stereo, non RGB-D attiva | LiDAR (CenterPoint) + YOLO (EagerMOT) |
| JRDB | VLP-16 x2 + camere RGB, nessuna depth | LiDAR + RGB | LiDAR | escluso | escluso (nessuna depth) | da valutare |
| Marina di Pisa street (secondario) | LiDAR + camera frontale | LiDAR + camera | LiDAR | – | – | – |

Modalità di SINS (verificato sul codice, 06/11):
- **Solo LiDAR**: la pipeline parte dalla nuvola (`preprocessing_node` sincronizza LiDAR e odometria; le camere
  arricchiscono i cluster in modo asincrono). Probabilmente basta una configurazione: `enable_yolo26:=false` e
  nessun topic camera (oppure `camera_names` con una camera senza dati nella bag). Attenzione: `camera_names:=''`
  vuoto viene ignorato dal launch, che usa la lista dello YAML. **Da provare** con una run di controllo: SINS non deve
  bloccarsi e non deve ricevere dati camera.
- **Solo camera / RGB-D**: **non supportata senza modifiche al codice.** Senza LiDAR il `preprocessing_node` non
  produce nulla (callback sincronizzato su LiDAR + odometria). Per farla servirebbe un ingresso alternativo, per
  esempio la nuvola della depth al posto del LiDAR: è una modifica di SINS, quindi **decisione dell'utente**. Finché non
  c'è, il confronto con i metodi solo camera (Xu RGB-D, YOLO+ByteTrack+depth) si fa con SINS pieno, dichiarandolo.
- **LiDAR + RGB-D**: la configurazione normale, contro LV-DOT, EagerMOT e CenterPoint+SimpleTrack.

Regola dei sensori anche per i baseline: LV-DOT vuole una RGB-D attiva. Su CODa ha solo depth stereo (ZED o
cam0/cam1): va dichiarato, oppure LV-DOT va escluso da CODa.

## 4. Infrastruttura sul PC del laboratorio

- Le immagini Docker dei baseline (`dynablox-noetic`, `mdetector-noetic`, `lvdot-noetic`, `unimt-noetic`, ...) **non
  esistono** su questo PC e vanno ricostruite. Sul disco restano ~10-30 GB, quindi una alla volta, cancellando la
  cache di build dopo ogni immagine (solo la nostra, mai immagini o container altrui).
- I pesi UniMT/CenterPoint (Google Drive degli autori) non sono su questo PC [?]: vanno riscaricati sul NAS.

## 5. Prossimi passi

1. Opzione `--frames testing` nel valutatore CODa e rivalutazione di tutte le run esistenti (economico, nessuna
   nuova run).
2. Verifiche [?] sul de-skew CODa, `min_cluster_size` upstream e config M-detector per 128 linee.
3. Run di controllo SINS solo LiDAR (configurazione, nessuna modifica al codice).
4. Fase B: M-detector su SemanticKITTI (dati già sul NAS), Dynablox su DOALS, CenterPoint mAP su CODa val.
