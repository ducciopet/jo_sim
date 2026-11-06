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
- [?] Verificare nel repo UniMT (TossherO) come sono stati costruiti `coda_infos_train/val.pkl`: se seguono lo split
  ufficiale, i fotogrammi `testing` sono puliti. Se invece fondono train e val, serve comunque `testing`.

## 2. Schede

### Dynablox (Schmid et al., RA-L 2023) — LiDAR, geometrico, online
- Paper [P]: dataset DOALS (Pfreundschuh et al. 2021; Ouster OS1-64, ambienti urbani e al chiuso, etichette di punti
  dinamici su fotogrammi campione) più un dataset CARLA proprio. Metriche: IoU e precisione/recall per punto
  (e per oggetto). Online in tempo reale su CPU, con pose da odometria/SLAM e nuvole **de-skew** (le demo DOALS usano
  `lidar_undistortion`).
- Config ufficiale: `dynablox_ros/config/motion_detector/default.yaml` (commit `44a1537`). Qui: sensore e frame
  adattati (VLP-16 nel lab, OS1-128 su CODa), `max_range 25` su CODa (stesso raggio della valutazione).
- Ingresso qui: odometria condivisa; nessun de-skew (tolta la dipendenza `lidar_undistortion`). Su CODa le nuvole
  sono già de-skew dal dataset? [?] (le OS1 di CODa sono "3d_raw"; controllare se sono compensate).
- Deviazioni:
  1. Senza de-skew nelle sequenze con robot in moto [?];
  2. `min_cluster_size` 20 sia su CODa sia nel lab: confrontare con `default.yaml` [?] (la nota chiedeva se è stato
     cambiato);
  3. patch di uscita per esportare l'ID del cluster (non cambia il rilevamento) [V].
- Riproduzione (fase B): DOALS è pubblico (ETH, senza registrazione) [?]. Rifare l'IoU per punto di una sequenza.

### M-detector (Wu et al., Nat. Commun. 2024) — LiDAR, geometrico, online
- Paper [P]: KITTI, SemanticKITTI, nuScenes, Waymo, più dataset AVIA (Livox). Metrica: IoU per punto dell'etichetta
  dinamica. Online a 10 Hz con odometria di FAST-LIO, una posa per scansione.
- Config ufficiali per KITTI (64 linee), nuScenes (32), Waymo (64), AVIA. Qui `coda.yaml` parte da `nuscenes0.yaml`
  (`dataset: 1`), con FOV e numero di punti dell'OS1-128 [V]. Per un LiDAR a 128 linee forse è più vicina la config
  KITTI/Waymo: da decidere confrontando le risoluzioni verticali [?].
- Ingresso qui: odometria condivisa, una posa per nuvola interpolata al timestamp (`make_input*.py`) [V].
- Deviazioni:
  1. A 1x perde metà delle scansioni da 131k punti e poi accoppia nuvole e pose sbagliate (recall 5,6% contro 43,0%
     a 0,25x) [V]. Usare **0,25x** (dichiarato: "offline, rallentato 4x") e verificare che ogni nuvola abbia la sua
     posa (`check_run.py` sulle coppie);
  2. uscita a punti → DBSCAN comune (eps 0,5 m; min 5 punti nel lab, 20 su CODa) [V]: è un post-processing nostro
     uguale per tutti i metodi a punti, da dichiarare;
  3. patch solo di build (TBB) [V].
- Riproduzione (fase B): dati e etichette degli autori (KITTI/SemanticKITTI pubblici; SemanticKITTI è già sul NAS in
  `datasets_SINS/SemanticKITTI`). Rifare l'IoU su una sequenza SemanticKITTI con la config KITTI.

### LV-DOT (Xu et al., 2025) — LiDAR + RGB-D + YOLO, online
- Paper [P]: robot propri (LiDAR a basso numero di linee / Livox + RealSense D435i, al chiuso e all'aperto, su
  Jetson). Metriche: precisione/recall di rilevamento, errore di posizione e velocità. Codice commit `449bf2c`.
- Qui: nel lab sensori molto vicini a quelli del paper (VLP-16 + D435) [V]. Su CODa: depth della ZED (stereo,
  960x540), LiDAR OS1-128, `ground/roof_height` di default, calibrazione per sequenza (`make_param_coda.py`) [V].
- Deviazioni: su CODa quasi zero rilevazioni (recall 1-4%) [V]. Probabili cause, da verificare in quest'ordine [?]:
  1. depth ZED stereo (rumorosa e senza `depth_scale` RealSense) invece di una RGB-D attiva;
  2. 131k punti per scansione contro i ~20k del sensore per cui è tarato (downsampling, tempo di calcolo);
  3. altezze pavimento/soffitto: con l'origine odom a terra, `ground_height 0.2` può tagliare i piedi.
  Se resta fuori dalle sue condizioni, va **escluso da CODa** con motivazione ("nessuna RGB-D attiva in CODa"), come
  già per JRDB.
- Riproduzione (fase B): i dati del paper non sono pubblici [?]. Fedeltà = config upstream intatta sul lab.

### CenterPoint + SimpleTrack (UniMT, He et al. 2025) — appreso
- CenterPoint (Yin et al., CVPR 2021): nuScenes/Waymo, mAP/NDS. Su CODa: pesi degli autori UniMT
  (`centerpoint_coda_cfg.py`, mmdet3d, CenterHead standard, nessuna chiave mancante) [V]. SimpleTrack (Pang et al.,
  ECCVW 2022): AMOTA/MOTA su nuScenes/Waymo, offline.
- Qui: script upstream `infer.py` degli autori per CODa, soglia 0,1, SimpleTrack `coda_configs/diou.yaml` [V].
- Deviazioni:
  1. **contaminazione train** (sezione 1) [V];
  2. classifica oggetti, non moto: tutte le tracce contano come "dinamiche" [V];
  3. offline (frame dal manifest), dichiarato [V].
- Riproduzione (fase B): mAP CenterPoint sullo split val CODa contro il numero di UniMT / del paper CODa [P].
  Richiede `coda_infos_val.pkl` (script degli autori) e la valutazione mmdet3d.

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
