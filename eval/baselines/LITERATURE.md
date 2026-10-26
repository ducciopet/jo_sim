# Metodi con cui confrontare SINS (aggiornato 2026-10-26)

Legenda. **Sensori**: cosa usa il metodo. **Approccio**: *geometrico* (regole, mappe, occlusioni, nessun
addestramento) o *appreso* (rete addestrata su un dataset). **Stati**: che cosa dice della persona: solo "in moto"
(una persona ferma sparisce o diventa statica), "persona" (la segue, ma non dice se si muove), oppure D/PD/S.
**Codice**: verificato il 26/10/2026 (repository pubblico). **Qui**: stato nel nostro benchmark.

| Metodo | Sensori | Approccio | Uscita | Stati | ID | Codice | Qui |
|---|---|---|---|---|---|---|---|
| **SINS** (nostro) | LiDAR + RGB-D | geometrico + YOLO | tracce 3D | **D / PD / S** | sì | sì | – |
| *LiDAR, geometrici* | | | | | | | |
| Dynablox (Schmid et al., RA-L 2023) | LiDAR | geometrico, online | cluster in moto | in moto | sì | sì | fatto |
| M-detector (Wu et al., Nat. Commun. 2024) | LiDAR | geometrico, online | punti in moto | in moto | no | sì | fatto |
| DUFOMap (Duberg et al., RA-L 2024) | LiDAR | geometrico (ray casting) | mappa pulita | in moto | no | sì (KTH-RPL/dufomap) | candidato per la mappa |
| ERASOR (Lim et al., RA-L 2021) | LiDAR | geometrico, offline | mappa pulita | in moto | no | sì | candidato per la mappa |
| Removert (Kim e Kim, IROS 2020) | LiDAR | geometrico, offline | mappa pulita | in moto | no | sì | candidato per la mappa |
| BeautyMap (Jia et al., RA-L 2024) | LiDAR | geometrico, offline | mappa pulita | in moto | no | non verificato | – |
| *LiDAR, appresi (MOS)* | | | | | | | |
| LMNet / LiDAR-MOS (Chen et al., RA-L 2021) | LiDAR | appreso (range image) | punti in moto | in moto | no | sì (PRBonn/LiDAR-MOS) | – |
| MotionSeg3D (Sun et al., IROS 2022) | LiDAR | appreso | punti in moto | in moto | no | sì (haomo-ai/MotionSeg3D) | – |
| 4DMOS (Mersch et al., RA-L 2022) | LiDAR | appreso (conv. sparse 4D) | punti in moto | in moto | no | sì (PRBonn/4DMOS) | **consigliato** |
| MapMOS (Mersch et al., RA-L 2023) | LiDAR | appreso, con mappa | punti in moto | in moto | no | sì (PRBonn/MapMOS) | **consigliato** |
| InsMOS (Wang et al., IROS 2023) | LiDAR | appreso, per istanze | punti + istanze in moto | in moto | no | annunciato (nubot-nudt/InsMOS) | – |
| MF-MOS (Cheng et al., ICRA 2024) | LiDAR | appreso | punti in moto | in moto | no | sì (SCNU-RISLAB/MF-MOS) | – |
| *LiDAR, appresi, persone* | | | | | | | |
| Person-MinkUNet (Jia e Leibe, 2021) | LiDAR (JRDB, 16 linee) | appreso | box di persone | persona | no | non trovato | – |
| *Camera / RGB-D* | | | | | | | |
| Xu et al. (RA-L 2023) | RGB-D (+ YOLO) | geometrico + YOLO | box dinamici | in moto | no | sì | fatto (2 varianti) |
| YOLO + ByteTrack (Zhang et al., ECCV 2022) + depth | RGB-D | appreso | tracce 2D portate in 3D | persona | sì | sì | candidato semplice |
| *Ibridi LiDAR + camera* | | | | | | | |
| LV-DOT (Xu et al., 2025) | LiDAR + RGB-D | geometrico + YOLO | box dinamici | in moto | no | sì | fatto |
| UniMT + SimpleTrack (He et al., 2025) | LiDAR + camera | appreso (rilevatore CMDT) | tracce di persone | persona | sì | sì | fatto |
| EagerMOT (Kim et al., ICRA 2021) | box 3D da LiDAR + box 2D da camera | tracker geometrico su rilevatori appresi | tracce 3D | persona | sì | sì (aleksandrkim61/EagerMOT) | fatto |
| DeepFusionMOT (Wang et al., RA-L 2022) | LiDAR + camera | associazione appresa | tracce 3D | persona | sì | sì (wangxiyang2022/DeepFusionMOT) | candidato |
| JRMOT (Shenoi et al., IROS 2020) | LiDAR + camera | appreso | tracce di persone | persona | sì | sì, ma serve Gurobi | bloccato |
| SPENCER (Linder et al., ICRA 2016) | laser 2D + RGB-D | rilevatori + tracker IMM | tracce di persone | persona | sì | sì (spencer-project, ROS 1) | candidato (fetta 2D del LiDAR) |
| Chen, Huang e Cho (arXiv 2607.06896, 2026) | LiDAR + fisheye | griglia di occupazione + KF + YOLOv8 | tracce | "temporaneamente statico" | ? | no | solo da citare |

## Cosa ne esce

- Nessun metodo con codice disponibile ha uno stato come PD: o segnalano solo ciò che si muove (tutti i MOS, LiDAR e
  camera), o seguono le persone senza dire se si muovono (UniMT, EagerMOT, JRMOT, SPENCER). L'unico vicino, Chen et al.
  2026, non ha codice.
- I MOS appresi (4DMOS, MapMOS, MF-MOS, ...) sono addestrati su SemanticKITTI: LiDAR a 64 linee, strade all'aperto.
  Sul nostro VLP-16 al chiuso partono svantaggiati; vanno provati con i pesi pubblicati e dichiarato il dominio.
- Per la mappa SLAM i confronti giusti sono i metodi di pulizia della mappa (DUFOMap, ERASOR, Removert): lavorano
  offline su tutta la sequenza, quindi hanno un vantaggio che SINS, online, non ha.

## Ordine consigliato

1. ~~EagerMOT~~ fatto (26/10): uguale a UniMT, perché usa lo stesso rilevatore 3D.
2. **4DMOS o MapMOS**: il MOS LiDAR appreso di riferimento; stesso trattamento di M-detector (punti → DBSCAN comune).
3. **DUFOMap** nell'esperimento mappa, accanto a Dynablox.
4. **YOLO + ByteTrack + depth**: il baseline "solo camera appreso" che un revisore chiede per primo; facile.
5. DeepFusionMOT e SPENCER solo se avanza tempo.
