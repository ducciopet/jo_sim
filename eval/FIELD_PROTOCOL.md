# Protocollo esperimenti sul campo — spiaggia di sassi e parco nazionale

Obiettivo: dati con cui (1) misurare le etichette **D / PD / S** di v2 e (2) confrontare v2 con altri algoritmi
(v1, LV-DOT, Dynablox, M-detector, ...) sulle **stesse bag**. Senza mocap, la ground truth (GT) viene da: coreografia
scritta + punti misurati a terra + annotazione offline indipendente dai metodi (+ RTK se disponibile).

---

## 1. Definizioni delle etichette (da fissare prima, valgono per tutte le prove)

| Etichetta | Definizione GT |
|---|---|
| **D** (dynamic) | persona (o oggetto) in movimento, velocità ≥ 0,3 m/s |
| **PD** (potentially dynamic) | **persona** ferma (in piedi, seduta, accovacciata) |
| **S** (static) | oggetto non-persona fermo (zaino, cassa, sedia, borsa frigo, sassi grandi, alberi) |

Per ogni persona la GT è una sequenza di fasi (ferma/cammina) con istanti di inizio e fine: è il formato a
*waypoint* già previsto (`jo_detector_v2/eval/GT_SCHEMA.md`). Per gli oggetti S basta la posizione misurata.

---

## 2. Ground truth senza mocap: cosa preparare

In ordine di affidabilità:

1. **RTK sulle persone (fortemente consigliato, se si trova anche un solo ricevitore in più).** Il robot ha già RTK
   (topic `/rtcm`, NTRIP): un rover RTK su uno zaino/asta della persona dà la posizione a ~2 cm a 5–10 Hz, già nello
   stesso riferimento del GNSS del robot. In spiaggia e nel parco il cielo è libero. Anche una sola persona con RTK
   per prova rende misurabili posizione, velocità e latenze come con il mocap.
2. **Coni a terra in posizioni misurate.** Griglia di coni attorno alla posizione di partenza del robot (es. a 2, 4,
   6 m, davanti/lati/dietro), misurata con rotella o distanziometro laser rispetto a un segno sotto il robot. Le fasi
   "ferma" avvengono **sui coni**: la posizione GT è esatta senza annotare nulla.
3. **Annotazione offline** delle fasi di cammino tra coni e degli istanti di partenza/arresto, guardando LiDAR grezzo e
   video, **senza** mostrare l'output di nessun metodo (altrimenti la GT favorisce chi l'ha prodotta).
4. **Camera esterna su treppiede** (anche un telefono) che inquadra tutta l'area dall'alto/di lato, con orologio
   visibile: serve per annotare gli istanti e risolvere i casi dubbi.

**Sincronizzazione**, all'inizio di ogni prova: l'operatore legge ad alta voce l'ID prova vicino al microfono del
telefono/camera, poi **tutte le persone fanno un salto insieme** (evento netto e visibile nel LiDAR e nel video).
Gli istanti di partenza/arresto delle fasi li **chiama ad alta voce** un coordinatore con un conto alla rovescia:
così sono anche nel video.

**Abbigliamento**: pettorine/gilet di colori diversi e numerate (P1, P2, ...) per riconoscere le identità nel video.

---

## 3. Cosa registrare (una bag per prova)

- Entrambe le camere: colore (compresso), **depth raw e allineata**, `camera_info`, `extrinsics/depth_to_color`.
- LiDAR (`/velodyne_packets` e `/velodyne_points`), IMU completa, GNSS/RTK (`/gnss`, `/rtcm`, `/nmea`), `/tf_static`,
  `/robot_description`, odometria/comandi se disponibili.
- Niente output dei detector nella bag "sensori": i metodi si fanno girare dopo, tutti sugli stessi dati.
- **Nome bag**: `<ambiente>_<prova>_<rip>` (es. `spiaggia_P4_r2`); tutto annotato nel foglio di registro (§7).
- **Spazio**: la bag di laboratorio registra ~51 MB/s → **~180 GB per ora**. Prove da 60–90 s; verificare lo spazio
  sul robot/SSD esterno prima di partire (40 prove × 75 s ≈ 150 GB per ambiente).
- **All'inizio della giornata, per ambiente**: 60 s di robot fermo davanti a oggetti ben strutturati (per la
  calibrazione LiDAR–camera) e 60 s di scena **vuota** con robot fermo e poi in movimento.

---

## 4. Matrice delle prove (per ambiente)

Ogni prova: 60–90 s, **2–3 ripetizioni**, robot **fermo** e **in movimento** (rettilineo lento ~0,3 m/s; in alcune
prove anche rotazione sul posto — utile anche per la calibrazione del sistema).

| ID | Scenario | Cosa misura |
|---|---|---|
| **E0** | scena vuota | falsi D/PD al minuto (non serve GT di posizione) |
| **S1** | solo oggetti statici (zaino, cassa, sedia...) a distanze note | S non devono diventare D/PD |
| **P1** | 1 persona davanti: ferma 10 s sul cono → cammina al cono successivo → ferma 10 s (×3) | PD↔D, latenza di partenza/arresto |
| **P2** | 1 persona fa il giro completo del robot a ~2 m, con soste di 5 s davanti, di lato, dietro | uscita/rientro dal FOV, mantenimento della classe dietro il robot |
| **P3** | 1 persona già ferma **dietro** il robot dall'inizio, poi entra nel FOV | persona mai vista dalla camera |
| **P4** | 2 persone che si incrociano lungo il raggio del robot (una passa davanti all'altra) | occlusioni, ID switch |
| **P5** | 1 persona che cammina, si gira e si ferma **accanto** a un oggetto (masso, albero, cassa) | "scivolamento" del track sull'oggetto |
| **P6** | gruppo di 3–4 persone: alcune ferme, altre che camminano | scene affollate |
| **P7** | 1 persona seduta / accovacciata / chinata | PD con forma non standard |
| **P8** | robot che si avvicina a una persona ferma (da 6 m a 1 m) | box tagliata da vicino, distanza minima |

Totale indicativo: 10 scenari × 2 modalità robot × 2 ripetizioni ≈ 40 prove per ambiente (~1 h di registrazione
netta). Se il tempo manca, priorità: **E0, P1, P2, P4, P5, S1**.

---

## 5. Note specifiche per ambiente

- **Spiaggia di sassi**: terreno irregolare e in pendenza (verifica il filtro del pavimento); sole forte = depth
  RealSense più rumorosa (annota l'ora e le condizioni di luce); niente muri. Usa massi grandi o casse come oggetti S.
- **Parco nazionale**: vegetazione mossa dal vento = il test più duro per i falsi D su oggetti S (registra anche E0 con
  vento); alberi e cespugli come ingombri vicini alle persone (P5); pendenze. Verificare se servono permessi per
  riprese/registrazioni.

---

## 6. Metriche che si potranno calcolare

| Metrica | Prove | GT necessaria |
|---|---|---|
| Falsi D/PD al minuto, falsi D/PD su oggetti S | E0, S1 | solo l'elenco degli oggetti (e posizioni misurate) |
| Confusione D/PD/S frame per frame, recall D e PD | tutte | fasi delle persone (coreografia + annotazione) |
| Latenza di partenza/arresto | P1, P2, P8 | istanti delle transizioni (chiamate + video) |
| MOTA, IDF1, ID switch (gate 1–1,5 m, più largo del mocap per l'incertezza dell'annotazione) | P1–P8 | posizioni (coni + annotazione, o RTK) |
| Errore di posizione (MOTP) | solo con RTK, o soste sui coni | RTK / coni |

**Confronto con altri algoritmi**: tutte le bag si riproducono con la stessa odometria condivisa (come in laboratorio)
per ogni metodo. LV-DOT/v1/v2 producono D/PD/S; Dynablox e M-detector (solo LiDAR) producono punti dinamici → solo la
parte **D** del confronto (dichiarandolo). Con l'RTK si possono confrontare anche posizioni e ID.

---

## 7. Foglio di registro (una riga per prova)

| Bag | Ora | Ambiente | Prova | Rip. | Robot (fermo/rettilineo/rotazione) | Persone (P1..Pn) | Oggetti S e posizioni | Luce/vento | RTK su | Note |
|---|---|---|---|---|---|---|---|---|---|---|

## 8. Checklist del giorno

- [ ] Spazio disco per ~200 GB per ambiente; batterie (robot, laptop, camere, RTK)
- [ ] Sincronizzazione orologi (robot, PC, telefono della camera esterna)
- [ ] Rotella/distanziometro, 12–16 coni, nastro per il segno sotto il robot
- [ ] Pettorine numerate, oggetti S (zaino, cassa, sedia)
- [ ] Treppiede + telefono per la camera esterna
- [ ] Ricevitore RTK in più (se disponibile) e supporto da zaino
- [ ] Consenso delle persone riprese (se il dataset va pubblicato: volti visibili nelle camere)
- [ ] Per ogni ambiente: calibrazione (60 s), E0 (60 s), poi le prove
- [ ] Dopo ogni prova: controllare che la bag sia completa (durata, topic, nessun messaggio perso)
