# Benchmark status

Vedi `../BENCHMARK_PLAN.md` per il protocollo, `../PAPER_PLAN.md` per il piano del paper e `README.md` per l'uso dell'harness.

---

## Fase 0: inventario (2026-10-13), praticamente chiusa

### Hardware e ambiente

| Voce | Valore |
|---|---|
| GPU | NVIDIA GeForce RTX 4060 Laptop, 8 GB |
| Driver / CUDA | 580.178.04 / CUDA 13.0; torch 2.14.0+cu130 nel container, `cuda.is_available() = True` |
| CPU / RAM | 20 thread / 31 GB |
| ROS | ROS 2 Jazzy (container `jo-sim_lv_dot:latest`) |
| Docker | 29.8.0, nvidia-container-toolkit installato |
| Disco libero su `/` | **62 GB** su 503 GB |

### Cosa è stato risolto

| Problema | Soluzione |
|---|---|
| Container con i vecchi mount `onboard_detector*` | ricreato; v1 e v2 compilati a mano |
| `/entrypoint.sh` dell'immagine compila ancora `onboard_detector` e non compila v2 | **da fare: `docker compose build`** (il Dockerfile del repo è già corretto) |
| `lap` assente nell'immagine, quindi i nodi YOLO di v2 andavano in crash | installato a mano nel container (temporaneo, è già nel Dockerfile) |
| `numpy>=2` contro `cv_bridge` per numpy 1.x, quindi il YOLO di v1 andava in crash | `cv_bridge` rimosso dal nodo YOLO di v1 (jo_detector `e09610a`); nessun nodo usa scipy |
| v1 non produceva output su bag: transport `compressedDepth`/`compressed` scritte nel codice | rese parametri (default invariati), con argomenti di launch |
| la calibrazione ICP di v1 usava gli intrinseci colore sulla depth raw, quindi 0,5–1,1 m di errore | argomento `calibration_camera_info_topic` (jo_detector `ee170f3`) |
| v2 non girava sulle bag con una sola camera | argomento `camera_names` (jo_detector_v2 `ba0353a`) |
| alle bag di maggio mancano estrinseco, immagine compressa e nuvola di depth che v2 si aspetta | `adapters/realsense_bag_adapter.py`; estrinseco verificato identico al driver |
| GLIM non è deterministico, quindi ogni run avrebbe un'odometria diversa | odometria generata **una volta** per sequenza (`scripts/make_odom.sh`) e riprodotta a tutti i metodi |
| `motmetrics` non funziona nel container (scipy di sistema per numpy 1.x) | harness eseguito sull'host; nostre metriche verificate identiche a motmetrics |

### Ancora aperto

1. `docker compose build` (lungo, 23 GB di immagine con 62 GB liberi).
2. Disco per JRDB/CODa: con 62 GB liberi non si scarica nessuno dei due.
3. La calibrazione online one-shot di v2 impiega circa 20 s; v1 va in timeout a 7 s e usa la media dei risultati validi.
   Sulle sequenze da 50–70 s questo pesa, da cui la variante "dopo 25 s" qui sotto.

---

## Fase 1: harness, fatta

- `harness/`: formato unico, metriche (CLEAR MOT, IDF1, confusione D/PD/S, recall per stato, latenza di transizione)
  su griglia comune a 10 Hz, GT da mocap, allineamento mocap↔odom.
- **18 test sintetici verdi**: le metriche recuperano offset, ID switch, ritardo di classificazione, FP e FN
  iniettati; MOTA, MOTP, IDF1 e IDSW coincidono con `py-motmetrics`; l'allineamento recupera trasformazioni e
  offset di clock noti.
- `report.py`: CSV + LaTeX in `tables/`; l'aggregato somma i conteggi, non media le percentuali.

---

## E1: lab con GT Qualisys, primi numeri (2026-10-13)

Dati: 6 sequenze del 19/05/2026 (49–71 s, in totale 2890 frame valutati a 10 Hz), una persona, solo camera frontale.

| Seq. | Robot | Persona |
|---|---|---|
| 1942 | fermo | ferma |
| 1945 | fermo | cammina |
| 1947 | fermo | cammina e si ferma |
| 1949 | avanti e indietro | ferma |
| 1950 | avanti e indietro | mista |
| 1952 | avanti e indietro | cammina |

Allineamento: residuo 0–11 mm, offset di clock −0,18 s (correlazione delle velocità 0,99).

**Aggregato** (soglia GT 0,3 m/s, gate 1 m; dettaglio in `tables/own_mocap.csv`). Protocollo attuale: detector
avviato 12 s prima della bag (niente secondi persi al caricamento di YOLO), odometria condivisa. v1 è stato rifatto
con lo stesso protocollo.

| | MOTA | IDF1 | MOTP | FP | FN | IDSW | Recall D | Recall PD | latenza →D |
|---|---|---|---|---|---|---|---|---|---|
| v1 | 54,7% | 48,8% | 0,16 m | 168 | 1107 | 35 | 88,4% | 31,0% | 0,73 s |
| v2 (`c2ca9fd`, duplicati corretti) | 79,6% | 70,2% | 0,11 m | 128 | 454 | 7 | 93,9% | 72,0% | 0,36 s |
| v2 (`7a91090`, messa a punto) | 87,0% | 79,3% | 0,10 m | 36 | 335 | 5 | 91,8% | 82,1% | 0,45 s |
| v2 (`889bae0`, passaggio di classe + box persona) | 91,5% | 79,2% | 0,09 m | 55 | 187 | 5 | 95,6% | 87,4% | 0,25 s |
| v2 (`5677ac5`, muri limitati) | 93,1% | 87,3% | 0,09 m | 43 | 152 | 5 | 95,0% | 90,1% | 0,29 s |
| **v2 (`52a5573`, + `first_class_static_confirm_ticks` 5)** | **93,3%** | 74,4% | **0,09 m** | **30** | 159 | **4** | 94,1% | **90,5%** | 0,29 s |

Escludendo i primi 25 s (`tables/own_mocap_after25s.csv`): v1 MOTA 49,5%, IDF1 54,8%; v2 (`52a5573`) MOTA 93,5%, IDF1 82,4%.

**IDF1 molto variabile tra run complete** (87,3% in r5, 74,4% in r6 con lo stesso detector a monte): con una sola
persona per sequenza, un unico cambio di ID a metà sequenza dimezza l'IDF1 di quella sequenza. In r6 è successo nella
1945 e nella 1950: la persona, senza classe e dietro il robot, inverte la direzione accanto a un ingombro, il suo track
scivola sull'ingombro e lei riparte con un ID nuovo. Rigiocando il solo tracker sugli input r6 il risultato è identico
con il vecchio e il nuovo valore del parametro, quindi dipende dalle detection della singola run, non dalla modifica.
Per il paper l'IDF1 va riportato come media su più run.
Sensibilità alla soglia GT (0,2–0,5 m/s): recall PD di v2 (`52a5573`) tra 74% e 93%, di v1 tra 26% e 32%.
Variabilità tra run della pipeline completa: pochi punti di MOTA/IDF1 per sequenza (es. 1950 IDF1 68,8% nel run completo
contro 92,5% nel replay solo-tracker sugli input di un altro run). Il replay solo-tracker
(`scripts/run_tracker_only.sh`) è invece deterministico: stessi input, stesso output.

**Messa a punto di v2 (2026-10-13, `7a91090`)**
Metodo: per ogni FN/FP la causa (`analyze_errors.py`), più la diagnostica `tracker_debug_lifecycle:=true` per seguire i
singoli track; parametri confrontati con il replay solo-tracker sugli stessi input registrati.

| Problema osservato | Correzione |
|---|---|
| 117 FP su 118 erano lampi "dynamic" su track senza classe | track senza classe: 8 tick di movimento prima di "dynamic" (classificati: 3). Sweep 5/6/8: FP 73/53/32, FN 322/323/337 |
| persona ferma che non diventava mai PD: il track si "stabiliva" prima che YOLO la vedesse e poi serviva movimento per classificarlo | prima classifica anche da fermo se l'evidenza depth-leaf (maschera YOLO + depth) si ripete per 10 tick; la via debole (LiDAR-only) richiede ancora movimento |
| classe persa uscendo dal FOV (al bordo YOLO vede solo parte della persona) | decadimento in FOV a 3 tick (prima 9) ma solo con il centro ad almeno il 10% dal bordo; nella fascia di bordo dopo 15 tick |
| classe cancellata a persone ferme dietro il robot (box LiDAR che si restringe da vicino) | trigger di restringimento solo dentro il FOV |
| oggetto statico al bordo che prendeva "person" da una box YOLO della persona vicina | la via LiDAR-only prende una box 2D solo se nessun detection coperto dalla camera la spiega meglio, e solo nella zona affidabile dell'immagine |
| primi 4–6 s persi in ogni run (caricamento YOLO durante la bag) | protocollo: detector avviato prima della bag |
| 2 modelli YOLO per camera | segmentazione del pavimento spenta (serviva solo a RViz); GSeg3D era già spento |

Il decadimento a 5 tick invece di 3 dà risultati identici su questi dati.

Variante successiva (`jo_detector_v2`, commit dopo `7a91090`): il debounce lungo vale solo per i track senza classe
**dentro il FOV**; fuori FOV tornano 3 tick, per non ritardare chi si muove dietro il robot. Replay solo-tracker sugli
stessi input del run finale: latenza →D 0,42 → 0,33 s, recall D 92,2% → 93,6%, FN 331 → 312, FP 35 → 62 (tutti senza
classe e fuori FOV), MOTA 87,2% → 86,9%. La tabella sopra è ancora quella del run completo precedente a questa variante.

Debounce dei track senza classe fuori FOV portato a 5 tick (`1a1baad`). Replay solo-tracker (fuori FOV 3 / 5 / 8):
MOTA 86,9 / 87,3 / 87,2%, FP 62 / 47 / 35, FN 312 / 318 / 331, IDSW 5 / 3 / 5, latenza →D 0,33 / 0,35 / 0,42 s.

**Analisi dei casi semplici** (1942, 1945, 1949; poi tutte), sui frame in cui la persona è associata:
- **Bias di posizione radiale**: il centro della box sta 7–12 cm più vicino al robot del centro mocap, in tutte le
  sequenze e sia con robot fermo sia in moto. Coerente con il LiDAR che vede solo la superficie anteriore della
  persona; non separabile da dove sta il rigid body sulla persona.
- **Altezza della box sottostimata da vicino e dietro il robot**: mediana 1,6–1,7 m a 2–2,7 m davanti (1942, 1949), ma
  0,9 m nella 1947 (persona a 1,0–1,5 m) e 1,06 m nella 1952 (persona dietro a circa 2 m, solo LiDAR). Geometria del
  sensore (campo verticale del VLP-16), non un errore del tracker.
- **Velocità**: persona ferma stimata 0,03–0,08 m/s (p90 ≤ 0,16), ben sotto la soglia di 0,35; persona che cammina
  sovrastimata di 0,03–0,1 m/s rispetto al mocap, in parte perché la GT è lisciata su 1 s.
- **Stabilità**: nelle sequenze semplici un solo ID per la persona e nessun cambio di stato spurio; i cambi D/PD nella
  1947/1950 sono quasi tutti transizioni vere riconosciute con qualche decimo di ritardo, più un solo lampo PD→D→PD di
  0,1 s su una persona ferma (1950, 17,2 s).
- **Partenza**: prima uscita 0–2,6 s dopo la comparsa della persona; i casi lenti coincidono con la calibrazione
  LiDAR–camera iniziale (circa 3 s) o con una persona ferma dietro il robot (senza classe, quindi non pubblicata).
- **Ipotesi da verificare**: con il robot in moto (1949) l'errore trasversale sale a 6 cm (1–2 cm a robot fermo):
  possibile piccolo sfasamento temporale tra LiDAR e odometria.

**Passaggio di classe e box persona (`889bae0`)**, validati con la pipeline completa (riga in tabella):
- *passaggio di classe*: un track nato da una detection che un track classificato vicino (< 0,5 m) ha scartato per
  un'altra eredita la classe se entro 3 s quel track la perde (scambio d'identità, decadimento) o muore. Nella 1947
  la persona che esce di lato dal FOV ora resta `person` dietro il robot (MOTA 1947: 69% → 93%); il passaggio scatta
  una sola volta in tutte le sequenze, proprio in quel caso.
- *box persona in uscita*: base sul pavimento (`base_footprint`), altezza minima 1,7 m, centro spostato di 0,07 m lungo
  il raggio del LiDAR. Bias radiale residuo −5…+5 cm (prima −7…−12), MOTP 0,10 → 0,09 m.
- Osservato e **non** cambiato: l'EKF di `jo_navigation` gira in `two_d_mode` (beccheggio e rollio sempre 0). Col
  robot in moto l'8% dei punti della persona finisce sotto il pavimento (2% da fermo). Va corretto nella
  localizzazione, non nel detector.

**Analisi dei parametri (2026-10-13, sera)**

Metodo, pensato per limitare l'overfitting su 6 sequenze con una sola persona:
1. si toccano solo parametri legati a una causa d'errore osservata (`analyze_errors.py`);
2. `sweep_tracker.py`: replay solo-tracker sugli stessi input (run r5), griglia grossolana di valori fisicamente
   sensati, validazione leave-one-sequence-out (il valore scelto su 5 sequenze, misurato sulla sesta);
3. il replay solo-tracker **non è perfettamente deterministico** (ordine di arrivo dei messaggi tra topic):
   quattro ripetizioni del default danno MOTA 92,28 / 92,91 / 92,91 / 93,08%, quindi differenze sotto ~1 punto
   non sono credibili senza ripetizioni.

| Parametro | Valori (default in grassetto) | MOTA | Esito |
|---|---|---|---|
| `track_dynamic_speed_thresh` | 0,25 / **0,35** / 0,45 | 93,0 / 92,3–93,1 / 90,7 | 0,25 dentro il rumore, 0,45 peggio: resta 0,35 |
| `track_min_hits` | 2 / **4** / 6 | 87,8 / 92,9 / 93,7 | leave-one-out 87,4% (instabile): resta 4 |
| `track_max_missed_classified` | 14 / **21** / 35 | 93,2 / 92,9 / 92,8 | ottimo piatto: resta 21 |
| `track_first_class_static_confirm_ticks` | 5 / **10** / 20 | 93,4 / 93,1 / 91,9 | 5 confermato su 3 ripetizioni (93,36 / 93,5 / 93,5): **adottato** |

**Detector (preprocessing/DBSCAN/YOLO)**: nessuno sweep, per scelta. Dopo la correzione dei muri, gli errori rimasti
sono quasi tutti (a) fuori FOV, dove serve la camera posteriore, oppure (b) nei primi 1,7–2,5 s di sequenza, prima
della prima classificazione. Nessuno dei due dipende da soglie di DBSCAN o di YOLO: abbassare la soglia di YOLO o i
punti minimi di DBSCAN aggiungerebbe rischio di FP senza un errore osservato da correggere.

Correzione strutturale trovata con questa analisi (`5677ac5`): i muri venivano rimossi come **piani infiniti**, quindi
anche i punti sul prolungamento di un muro lontano; una persona ferma sulla linea di un muro a 5 m perdeva il 60% dei
punti e la detection, e la "fetta" spezzava le persone in frammenti che diventavano track duplicati. Ora la rimozione
è limitata all'estensione del muro (+0,2 m). Pipeline completa: MOTA 91,5 → 93,1%, IDF1 79,2 → 87,3%.

**Altezza nell'associazione** (jo_detector_v2, commit dopo `52a5573`): il track tiene una media mobile della **cima**
delle box associate; i candidati pagano 0,5 per metro di differenza (rifiuto oltre 1 m). Replay solo-tracker sugli input
r6, 2 ripetizioni: FP 34/32 → 15/11, IDF1 74,4 → 80,8%, MOTA 93,2/93,3 → 93,8/93,9%, FN invariati. Lo scambio d'ID della
1950 sparisce; quello della 1945 resta (l'ingombro ha una cima simile).

Politica semantica attuale: in FOV ci si fida di YOLO (classe tolta dopo 3 tick senza conferma, 15 nella fascia di
bordo); fuori FOV la classe resta finché il track vive, salvo scambio d'identità (box cresciuta oltre il 50% della
taglia vista con YOLO per 5 tick).

**Cosa resta (FN 335, FP 36)**
- **Persona dietro il robot senza camera posteriore** (≈ 230 frame di FN, soprattutto 1947): se arriva lì perdendo
  l'identità del track, il nuovo track non ha mai classe, quindi se è ferma non viene pubblicata e se cammina diventa
  dynamic solo dopo 0,8 s. È in buona parte un limite del sensore (queste bag hanno solo la camera frontale).
- **Scambio d'identità all'uscita laterale dal FOV** (1947, 29 s): il track `person` viene associato a un oggetto statico
  vicino e la persona vera nasce come track nuovo senza classe. Probabile causa: la penalità di continuità di classe
  usa la distanza dall'*ultima osservazione* del track, che per una persona in moto favorisce l'oggetto rimasto vicino a
  dove lei era. Non corretto: tocca la logica anti-scambio-ID e va affrontato a parte.
- **Rischio di overfitting**: tutte le scelte sono state fatte guardando queste 6 sequenze. Ora sono di fatto lo split
  `tune/`: i numeri del paper vanno presi sulla nuova campagna mocap (PAPER_PLAN §4).

**Limiti da dichiarare**
- Una sola persona, 6 sequenze, circa 5 minuti: è un pilota, non l'E1 del paper (vedi PAPER_PLAN §4).
- Il robot non ruota mai, quindi la posizione del rigid body Qualisys rispetto a `base_link` non è osservabile:
  fissata a 0. Questo può introdurre un bias costante di qualche cm in MOTP (identico per i due metodi). Va misurata
  sul robot e passata con `--t-base-jo`.
- Le calibrazioni LiDAR–camera online variano tra sequenze (x da 0,04 a 0,31 m per v2), anche questo da riportare.
  Alternativa più pulita per isolare il tracker: una calibrazione unica condivisa da tutti i metodi.
- v2 riceve la depth tramite l'adattatore (deproiezione), v1 legge la depth raw: stessa informazione, percorso diverso.

## Analisi finale (2026-10-14): v1 corretto, tre correzioni a v2, nuova associazione

**v1 era penalizzato dalla pipeline del benchmark.** Sulla 1942 faceva MOTA 0%: YOLO vedeva la persona, i box LiDAR e
depth c'erano, ma il track restava "non confermato" per tutta la sequenza (in v1 un oggetto fermo si conferma solo con
l'aggancio YOLO, e l'aggancio non riusciva mai: 0 accettazioni su 1262 tentativi, rifiutate da `boxTooTall`). Causa:
`wall_detector_node` stimava il pavimento a +0,28 m invece di −0,4 m, perché deproiettava la depth **raw** (848×480) con
gli intrinseci della camera **a colori** (il suo `camera_info_topic` di default; sul robot la depth è allineata al
colore e va bene). Tutto ciò che stava sotto 0,28 m veniva tagliato come pavimento. Corretto in `jo_detector`:
argomento `wall_camera_info_topic` (default uguale a `calibration_camera_info_topic`, che le bag di benchmark passano
già come depth `camera_info`). Inoltre v1 girava con la config **outdoor** (imposta dal launch) mentre v2 con quella
**indoor**: ora c'è `env:=indoor|outdoor` (default outdoor, invariato sul robot) e il benchmark usa indoor per tutti
(`scripts/run_v1_batch.sh`). Pavimento stimato ora −0,30…−0,44 m su tutte le sequenze.

**v2, tre correzioni** (tutte guidate da `analyze_errors.py` + `tracker_debug_lifecycle`, misurate con replay
solo-tracker sugli stessi input, `sweep_tracker.py --tag`):

| Problema | Causa trovata | Correzione | Replay solo-tracker |
|---|---|---|---|
| Persona ferma davanti alla camera che perdeva la classe per ~0,8 s (1942, 1949) | la depth delle bag ha buchi fino a 0,63 s (25 buchi > 0,3 s nella 1942); nel tick senza depth il box è solo-LiDAR, molto più stretto della taglia di riferimento (costruita su box LiDAR+camera), e il trigger "restringimento = cambio d'identità" scattava | il restringimento conta solo se il box del tick ha punti della camera | MOTA 93,91 → 94,53, FN 160 → 144, FP 12 → 10 |
| Persona mai vista dalla camera (dietro il robot) statica appena si ferma o inverte, e track che muore per invecchiamento statico (1945, 1952; anche uno scambio di ID) | il PD esisteva solo per track con classe semantica | **PD guadagnato col moto**: un track senza classe pubblicato dynamic per 20 tick diventa PD quando si ferma, resta tale finché vive, lo perde dopo 3 tick nel FOV affidabile senza YOLO (`track_motion_pd_min_dynamic_ticks`) | MOTA 94,43 → 96,02, FN 144 → 101, FP 13 → 10; piatto tra 10 e 30 tick, LOSO 95,95 |
| Ogni sequenza: persona statica per i primi ~2,5 s | la prima inferenza YOLO dura ~2 s (inizializzazione CUDA al primo frame) | inferenza di riscaldamento all'avvio del nodo (`predict` su immagine nera, ByteTrack non toccato) | solo pipeline completa (il replay del tracker non usa YOLO) |

**Nuova associazione** (`track_assignment_method`: `greedy_kuhn` default, `hungarian` = matching di peso massimo
vero). Tre confronti sugli stessi input:

| Input / codice | greedy_kuhn MOTA / IDF1 / IDSW | hungarian MOTA / IDF1 / IDSW |
|---|---|---|
| r6, prima delle correzioni | 93,91 / 80,82 / 4 | 94,05 / 80,87 / 2 |
| r6, con la correzione del restringimento | 94,53 / 81,19 / 4 | 94,60 / 81,21 / 2 |
| r7, codice finale | 97,40 / 88,94 / 6 | 97,44 / 88,92 / 4 |

Differenze di MOTA dentro il rumore del replay (~0,6), ma `hungarian` ha sempre meno scambi di ID e il leave-one-out
lo sceglie in 5 sequenze su 6. Con una sola persona per sequenza l'associazione ha pochi casi ambigui: il confronto
vero va fatto su scene con più persone.

**Pipeline completa, r7** (`jo_detector_v2` con le tre correzioni, `greedy_kuhn`; v1 corretto, config indoor;
`tables/own_r7_{full,dynamic_only,merged}.csv`):

| Livello | MOTA v1 | MOTA v2 | IDF1 v1 | IDF1 v2 | FP v1 / v2 | FN v1 / v2 |
|---|---|---|---|---|---|---|
| D/PD/S completo | 81,7% | **97,4%** | 65,1% | **88,9%** | 269 / 13 | 243 / 56 |
| Solo moto | 76,6% | **89,9%** | 75,5% | **94,0%** | 183 / 58 | 115 / 69 |
| D e PD unite | 81,7% | **97,4%** | 65,1% | **88,9%** | 269 / 13 | 243 / 56 |

Recall D 91,2 / 94,7%, recall PD 83,3 / 95,0%, latenza →D 0,68 / 0,28 s, MOTP 0,13 / 0,09 m (v1 / v2).
Il v1 precedente (stessa pipeline ma depth interpretata male, config outdoor) faceva 54,7%: **quel numero non va più
usato**.

**Cosa resta in v2 (FN 56, FP 13)**
- 1952 (28 FN): persona ferma dietro il robot che la camera non ha mai visto e che non ha camminato abbastanza per
  guadagnare il PD; limite del sensore (serve la camera posteriore).
- 1947 (17 FN, IDF1 58%): all'uscita laterale dal FOV il box passa da LiDAR+camera a solo-LiDAR, il centro salta e il
  track si perde; il passaggio di classe salva la classe ma non l'ID. Possibile correzione: nell'associazione,
  confrontare il box con la parte LiDAR del track quando la camera sparisce.
- I buchi della depth registrata (fino a 0,63 s) sono un problema della registrazione: per le prove sul campo
  controllare il frame rate della depth nella bag (`ros2 bag info`), non solo che il topic ci sia.
