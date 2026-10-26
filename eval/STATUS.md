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

**Aggiornamento (2026-10-14, sera):** `track_assignment_method` ha ora default `hungarian` (codice e
`cfg/{indoor,outdoor}/tracker.yaml`, `jo_detector_v2` `6ff364b`); `greedy_kuhn` resta per l'ablation. I numeri r7
sopra sono stati misurati con `greedy_kuhn`; con `hungarian` sugli stessi input: MOTA 97,44, IDF1 88,92, IDSW 4.

---

## Campagna r8 (2026-10-14, completata): `hungarian` a pipeline completa, 3 ripetizioni

Obiettivo: numeri da paper con media ± deviazione standard (l'IDF1 cambia di vari punti tra run) e l'ablation
dell'associazione a parità di input.

- Codice: `jo_detector_v2` `6ff364b` (default `hungarian`), `jo_detector` `e2bd34a` (config indoor).
- `scripts/run_campaign.sh` (copia in `bags/replay/`): per ripetizione a, b, c e per sequenza, v2 e v1 a pipeline
  completa (`r8<rep>_{v2,v1}_<seq>`, v2 registra anche gli input del tracker), poi `greedy_kuhn` col solo tracker
  sugli input di ciascun run v2 (`r8<rep>g_v2_<seq>`). 54 job, circa 1,5 min l'uno.
- **Ripresa**: ogni job finito lascia `<out>.done`; rilanciare lo script salta i job finiti e rifà da capo quello
  interrotto. Stato e orari in `bags/replay/r8_state.log`.
  - interrompere: `docker exec jo_sim_lv-dot-jo-sim_lv-dot-1 bash -c "pkill -f '[r]un_campaign.sh'; pkill -f '[r]un_tracker_only.sh'; pkill -f '[r]un_method.sh'; bash /home/ros/bags/replay/cleanup.sh"`
    (le parentesi quadre servono: senza, `pkill -f` uccide anche la shell che lo esegue e il resto non parte)
    (oppure `touch bags/replay/r8.stop` per fermarsi alla fine del job corrente)
  - riprendere: `rm -f bags/replay/r8.stop; docker exec jo_sim_lv-dot-jo-sim_lv-dot-1 bash -c "CAMPAIGN=r8 bash /home/ros/bags/replay/run_campaign.sh"`
- Valutazione (sull'host, anche su campagna parziale: aggrega solo le ripetizioni complete per metodo):
  `python3 eval/campaign_report.py --campaign r8 --reps a b c` → `results/own_r8<rep>/`,
  `tables/own_r8<rep>_<livello>.*`, `tables/own_r8_<livello>_reps{,_perseq}.csv`, `.tex`.
- Video top-down GT vs track: `python3 eval/make_video.py --results eval/results/own_r8a --seq <seq> --methods v1 v2 --out eval/videos/<nome>.mp4`.

**Stato al 2026-10-14 (interrotta su richiesta, 17:22 ora del container):** le 3 ripetizioni a pipeline completa sono
fatte per v2 e v1, tranne v1 rep c 1942 da rifare: la bag era stata riprodotta solo per 32 s su 64, per errori DDS
dopo che l'interfaccia `10.42.0.95` (hotspot) è sparita. Run invalidato (`.done` tolto, log in
`r8c_v1_20260519_1942_logs_truncated`). Gli script usano ora `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` e
`campaign_report.py` esclude i run che coprono meno del 97% della bag. Ablation `greedy_kuhn`: 8 replay su 18 fatti
(rep a completa, rep b 1942 e 1945). Alla ripresa lo script rifà nell'ordine v1 rep c 1942 e i 10 replay mancanti
(circa 15 min).

Risultati parziali (D/PD/S, media ± std; v1 su 3 ripetizioni compreso il run troncato, da ricalcolare):
v2 MOTA 96,46 ± 0,26%, IDF1 80,7 ± 6,7%, IDSW 3,0 ± 0. Gli scambi di ID di v2 sono sempre negli stessi punti:
1947 a 28,3 s (uscita laterale dal FOV, 3 run su 3), 1945 a 18,7 s (2 su 3), 1950 a circa 38 s (3 su 3; in rep b
l'ID torna dopo 0,2 s). Pagina report: `eval/make_report_page.py` + `report_template.html` + `report_r8_notes.html`
(i segnaposto `__...__` nelle note vanno riempiti coi numeri finali), video in `eval/videos/r8/`.

### Risultati finali r8 (campagna completa, 54 job; pagina: https://claude.ai/artifact/XvJ67raKwMDW5NQ4qMcnx3)

Media ± std su 3 ripetizioni (`tables/own_r8_{full,dynamic_only,merged}_reps{,_perseq}.csv`, `.tex`); ablation
`greedy_kuhn` col solo tracker sugli input registrati di ciascun run v2.

| Livello D/PD/S | MOTA | IDF1 | IDSW | FP | FN | Recall D | Recall PD | →D |
|---|---|---|---|---|---|---|---|---|
| v1 | 81,1 ± 2,3% | 60,4 ± 0,9% | 26,0 ± 8,5 | 287,7 | 231,7 | 92,1% | 83,6% | 0,55 s |
| **v2 `hungarian`** | **96,5 ± 0,3%** | **80,7 ± 6,7%** | **3,0 ± 0** | **32,7** | **66,7** | **94,1%** | **95,2%** | **0,27 s** |
| v2 `greedy_kuhn` | 96,5 ± 0,2% | 80,7 ± 6,7% | 3,3 ± 0,6 | 33,0 | 66,0 | 94,0% | 95,2% | 0,26 s |

Solo moto: v1 73,3 ± 2,1%, v2 88,3 ± 0,7% di MOTA (IDF1 48,6 / 81,1%).

- **v2 stabile sul MOTA** (96,3–96,8%), **IDF1 binario per sequenza** (76,9 / 88,4 / 76,9%): gli scambi sono sempre
  1947 a 28,3 s (3/3, uscita laterale dal FOV), 1945 a 18,7 s (2/3), 1950 a circa 38 s (3/3; in rep b l'ID torna
  dopo 0,2 s). Il r7 (IDF1 88,9%) era un run fortunato.
- **Associazione**: `hungarian` e `greedy_kuhn` identici a parità di input, salvo uno scambio in meno con
  `hungarian` (1949, rep c). Con una persona sola non ci sono casi ambigui: il confronto vero richiede più persone.
  Default `hungarian` confermato (mai peggio).
- **v1**: FP concentrati nella 1947 (circa 120 per run) e nella 1950 (circa 100 in 2 run su 3).
- **Run scartato**: v1 rep c 1942 riprodotto solo per metà (vedi sopra), rifatto: MOTA v1 81,1% invece del 77,7%
  con il run troncato.
- Video dall'alto (rep a, v1 contro v2) in `videos/r8/`; pagina rigenerabile con `make_report_page.py --campaign r8`.

**Prossimi passi**: correzione della 1947 (associazione sulla parte LiDAR del track quando la camera sparisce);
diagnosi dello scambio deterministico della 1950 a 38 s; poi stop alla messa a punto su queste 6 sequenze e
campagna sul campo (`FIELD_PROTOCOL.md`) con più persone.

---

## Correzione della 1947 (2026-10-15, `jo_detector_v2` `df56a77`)

**Causa** (replay solo tracker con `track_debug_lifecycle`, GT riportata nel frame odom): a 28,85 s la persona che
esce di lato dal FOV viene spezzata dalla fusione in due box, uno solo-LiDAR su di lei (`L`, 0,1 m dalla GT) e uno
solo-camera a 0,4 m sullo sfondo (`F`, profondità della maschera YOLO sull'armadio dietro) che porta la classe
`person`. Il track #20 prende il frammento `F` (bonus di classe +0,25; il box `L` paga la penalità di continuità di
classe e una velErr di 4,1 perché il centro precedente era quello del box fuso, spostato verso lo sfondo), al tick
dopo scivola sull'armadio e la persona rinasce come track nuovo senza classe. Non era quindi un problema di
centro LiDAR/fuso nel gating, come ipotizzato, ma un frammento solo-camera che vince l'associazione.

**Correzione**: `track_camera_fragment_dist` (0,6 m, 0 la disattiva): un candidato senza punti LiDAR viene scartato
per un track che nello stesso tick ha un candidato con punti LiDAR entro quella distanza. Una persona nel raggio
del LiDAR è sempre vista dal LiDAR, quindi il box solo-camera accanto è un frammento.

**Verifica**: A/B col solo tracker sugli input registrati dei 18 run v2 di r8 (`bags/replay/f{x,0}{a,b,c}_v2_*`,
`results/fix1947_{a,b,c}`); il run col parametro a 0 riproduce r8 esattamente.

| D/PD/S, 3 ripetizioni | MOTA | IDF1 | IDSW | FP | FN |
|---|---|---|---|---|---|
| senza (0) | 96,40 ± 0,27% | 80,7 ± 6,6% | 3,0 | 34,3 | 66,7 |
| **con (0,6 m)** | **97,17 ± 0,32%** | **90,8 ± 6,6%** | **2,0** | **22,0** | **57,7** |

1947: IDF1 57–58% → 99,6–100% e IDSW 1 → 0 in 3 run su 3; nessun'altra sequenza cambia. Gli scambi rimasti sono
1945 a 18,7 s (2 su 3) e 1950 a circa 38 s (3 su 3). I numeri r8 a pipeline completa restano quelli della campagna;
la correzione entrerà nella prossima campagna a pipeline completa.

**Prossimi passi**: ~~diagnosi dello scambio della 1950 a 38 s~~ (fatto, sotto), poi campagna sul campo.

## Correzione della 1950 (2026-10-15, `jo_detector_v2` `4a41b17`)

**Causa** (stesso metodo, log dei candidati esteso per un momento anche ai track non classificati): #594 è un track
non classificato e "statico" di un oggetto alto vicino a (2,9, 1,0) in odom. Quando la persona (#22, `person`) gli
passa accanto, i box fusi persona+oggetto lo trascinano verso di lei in passi da 0,1–0,2 m (ognuno sotto i gate).
Al tick decisivo #22 ha come candidati il box `person` (+0,35) e un frammento (−0,72); #594 ha solo il box `person`
(−0,47). L'assegnazione massimizza prima il numero di coppie e poi il punteggio (con `hungarian` per via dei
costi FORBIDDEN, con `greedy_kuhn` per costruzione), quindi sceglie le due coppie scadenti: #594 prende la persona
e diventa `person`, #22 finisce sul frammento e muore. Per questo succede in 3 run su 3 con entrambi i metodi.

**Correzione**: `track_class_ownership` (default true): un box che porta la classe X non può andare a un track non
classificato che lo valuta peggio del miglior track già classificato X. Il track di una seconda persona, nuova e
non ancora classificata, non perde il proprio box, perché su quello ha il punteggio più alto.

**Verifica**: solo tracker sugli input dei 18 run v2 di r8 (`bags/replay/fo{a,b,c}_v2_*`, `results/fix1950_{a,b,c}`),
contro i run con la sola correzione della 1947 (`fx`).

| D/PD/S, 3 ripetizioni | MOTA | IDF1 | IDSW | FP | FN |
|---|---|---|---|---|---|
| r8 (nessuna correzione) | 96,40 ± 0,27% | 80,7 ± 6,6% | 3,0 | 34,3 | 66,7 |
| frammenti (`df56a77`) | 97,17 ± 0,32% | 90,8 ± 6,6% | 2,0 | 22,0 | 57,7 |
| **frammenti + classe (`4a41b17`)** | **97,24 ± 0,36%** | **94,9 ± 3,1%** | **2,0** | **20,0** | **57,7** |

1950: IDF1 68% → 99% in 3 run su 3; nessun'altra sequenza cambia. Gli IDSW non scendono perché in a e b resta un
lampo di un tick (0,1 s): #594, trascinato e quindi pubblicato `dynamic`, passa a 4 cm dalla persona e la metrica
conta un andata e ritorno (2 IDSW). Resta da capire il trascinamento dei track statici da parte dei box fusi
(un'àncora di posizione per i track fermi da molto tempo, da valutare con cautela: una persona non classificata
ferma che poi cammina deve restare libera di muoversi).

**Scambi rimasti**: 1945 a 18,7 s (2 run su 3, persona dietro il robot senza classe) e il lampo della 1950.
Entrambe le correzioni vanno confermate in una campagna a pipeline completa prima di usarle nei numeri del paper.

**Decisione (2026-10-15)**: niente sweep dei pesi dell'associazione, del costo di non-assegnazione o dei parametri
del filtro di Kalman su queste 6 sequenze (regola in PAPER_PLAN §1). Lo scambio della 1945 e il lampo della 1950
si guardano sui dati nuovi, non qui.

---

## Campagna r9 (avviata il 2026-10-15): le due correzioni a pipeline completa

- Codice: `jo_detector_v2` `4a41b17` (frammenti + classe). v1 non è cambiato: i run v1 sono quelli di r8, collegati
  come `r9<rep>_v1_*`. Niente ablation greedy (`V1=0 GREEDY=0`, nuove opzioni di `scripts/run_campaign.sh`).
- 18 job v2 (3 ripetizioni × 6 sequenze), stato in `bags/replay/r9_state.log`.
- Interrompere: `touch bags/replay/r9.stop` (alla fine del job in corso) oppure il `pkill` della sezione r8.
- Riprendere: `rm -f bags/replay/r9.stop; docker exec -d jo_sim_lv-dot-jo-sim_lv-dot-1 bash -c "cd /home/ros/bags/replay && CAMPAIGN=r9 V1=0 GREEDY=0 bash run_campaign.sh > r9_campaign.log 2>&1"`
- Valutazione: `python3 eval/campaign_report.py --campaign r9 --reps a b c`.
- Istruzioni complete per chi riprende (anche da un altro account Claude):
  `~/Desktop/claude_account_backup/jo_detector_v2_handoff/LEGGIMI_PRIMA.md`.

### Risultati r9 (completata il 2026-10-15, 18 job, nessun run troncato)

Media ± std su 3 ripetizioni a pipeline completa (`tables/own_r9_{full,dynamic_only,merged}_reps{,_perseq}.csv`, `.tex`;
v1 identico a r8 perché i run sono gli stessi).

| Livello D/PD/S | MOTA | IDF1 | IDSW | FP | FN | Recall D | Recall PD | →D |
|---|---|---|---|---|---|---|---|---|
| v1 | 81,1 ± 2,3% | 60,4 ± 0,9% | 26,0 ± 8,5 | 287,7 | 231,7 | 92,1% | 83,6% | 0,55 s |
| v2 r8 (`6ff364b`) | 96,5 ± 0,3% | 80,7 ± 6,7% | 3,0 ± 0 | 32,7 | 66,7 | 94,1% | 95,2% | 0,27 s |
| **v2 r9 (`4a41b17`)** | **98,1 ± 0,1%** | **99,05 ± 0,05%** | **2,7 ± 3,1** | **9,7** | **42,7** | **96,1%** | **95,2%** | **0,26 s** |

- La pipeline completa conferma le due correzioni (e va oltre il solo tracker: 98,1 contro 97,2 di MOTA).
- Nessuna perdita d'identità persistente in nessun run: IDF1 per sequenza 96–100%. Anche lo scambio della 1945
  a 18,7 s non compare più in nessuna delle 3 ripetizioni (non era un obiettivo delle correzioni; da verificare sui
  dati nuovi se è un effetto della correzione della classe o fortuna).
- Gli IDSW rimasti (0 / 6 / 2 per ripetizione) sono tutti lampi di andata e ritorno di un tick, a IDF1 99%:
  1950 (b, c) e 1949 (b, 4 IDSW). Per la regola sulle 6 sequenze non si correggono qui; nel paper riportare IDF1
  insieme agli IDSW, perché gli IDSW contano i lampi come scambi.

### Label S sulle sequenze di laboratorio (2026-10-15, `eval/static_eval.py`)

Assunzione: nell'area del mocap tutto ciò che non è la persona è statico; dal lato dei metodi è S tutto ciò che non è
pubblicato come D/PD (v2 non pubblica i track statici). S→D/PD = track D/PD a più di 1 m dalla persona e non abbinati
(esatti); persona→S = i "mancati"; S→S stimato con i cluster del detector nell'area del mocap (circa 5 per frame,
stesso numero per tutti i metodi). Tabelle `tables/own_{r8,r9}_static.csv`, dati `results/own_{r8,r9}_static.json`.

| | Precision S | Recall S | Falsi dinamici/min | Precision D con la riga S |
|---|---|---|---|---|
| v2 r9 | 99,72% | 99,94% | **1,8** | 95,7% |
| v2 r8 | 99,57% | 99,81% | 6,2 | 94,5% |
| v1 | 98,50% | 99,10% | 28,7 | 83,5% |

Precision e recall di S sono schiacciate verso il 100% (circa 5 oggetti statici per frame contro una persona): nel paper
riportare i falsi dinamici al minuto, non l'F1 di S.

---

## Fase 3: baseline esterni sulle bag di laboratorio (avviata il 2026-10-15)

**LV-DOT** (Xu et al. 2025, github.com/Zhefan-Xu/LV-DOT commit `449bf2c`), ROS1 Noetic in Docker
(`eval/baselines/lvdot/Dockerfile`, immagine `lvdot-noetic:upstream`). Logica originale, solo I/O adattato:
- `eval/baselines/lvdot/detector_param_jo.yaml`: topic, intrinseci dalle `camera_info` delle bag, estrinseci URDF
  (base_link → camera depth/colore, → velodyne), `ground_height`/`roof_height` spostati di −0,373 m perché l'origine
  dell'odometria è base_link (stessi 0,2 m / 2,0 m dal pavimento dei default). Tutti gli altri parametri ai default.
- stessa odometria condivisa di v1/v2 (`bags/replay/odom_<seq>`), detector avviato 12 s prima della bag, play 1×.
- le bag ROS2 vengono convertite in ROS1 al volo (`rosbags-convert`, ~30 s, 6,6 GB, cancellate dopo l'uso).
- uscita: `/onboard_detector/dynamic_bboxes` (MarkerArray) → `eval/adapters/lvdot_adapter.py`. LV-DOT non ha ID
  persistenti né lo stato PD: si confrontano solo metriche di rilevamento (MODA, recall, falsi allarmi/min) ai livelli
  "solo movimento" e "D e PD insieme" (`eval/baseline_eval.py`).
- campagna: `bash eval/baselines/lvdot/run_lvdot_campaign.sh` (3 ripetizioni × 6 sequenze, ~3 min l'una, riprendibile:
  `bags/lvdot_out/lvdot<rep>_<seq>.done`; fermarla con `touch bags/lvdot_out/STOP`; log `bags/lvdot_out/campaign.log`).
- valutazione: `python3 eval/baseline_eval.py --campaign r9 --reps a b c` → `results/baselines_r9.json`,
  `tables/baselines_r9.csv`.
- prova sulla 1950: al livello "D e PD insieme" LV-DOT trova la persona il 94,8% del tempo (v2 99,6%), ma la tiene
  "dinamica" anche da ferma: al livello "solo movimento" 427 falsi allarmi contro 13 di v2.

Prossimi: Dynablox e M-detector (solo LiDAR, punti dinamici → box con lo stesso clustering per entrambi).

**Metriche con una sola persona (decisione del 2026-10-15).** Con un solo oggetto nella GT il MOTA coincide in pratica
col MODA (un ID switch vale 1/2890: per v2 r9 2,7 IDSW = 0,09 punti) e non misura l'associazione; scende sotto zero
quando i falsi allarmi superano i frame (LV-DOT −138% sulla 1950). Sulle prove con una persona si riportano quindi,
separate: recall, falsi allarmi al minuto (e su oggetti statici), F1 per stato e latenza, IDF1 e numero di ID dati alla
persona, MOTP. Il MOTA resta come riga di confronto con la nota "single subject: MOTA ≈ MODA". Sulle prove con più
persone e su JRDB/CODa: HOTA come metrica principale, più IDF1 e MOTA.

### Risultati LV-DOT (campagna completa, 18 run, 2026-10-15)

Media ± std su 3 ripetizioni, stesse bag e odometria di r9 (`results/baselines_r9.json`, `tables/baselines_r9.csv`;
falsi dinamici su oggetti statici da `static_eval`, `results/own_r9_static_lvdot.json`).

| | Persona trovata (D e PD insieme) | Falsi allarmi/min (D e PD insieme) | di cui su oggetti statici | MODA solo movimento | IDF1 |
|---|---|---|---|---|---|
| **v2** | **98,5 ± 0,1%** | **2,0** | **1,8** | **91,7%** | **99,1%** |
| v1 | 92,0 ± 1,8% | 59,7 | 28,7 | 73,9% | 60,4% |
| LV-DOT | 96,4 ± 1,8% | 79,2 | 5,6 | −54,2% | – (nessun ID persistente) |

- LV-DOT trova la persona più spesso di v1 (96,4 contro 92,0%) ma meno di v2 (98,5%).
- I suoi falsi allarmi sono quasi tutti **doppioni sulla persona** (box multipli entro 1 m): 79,2/min in totale, solo
  5,6/min su oggetti statici (v2 1,8, v1 28,7).
- Al livello "solo movimento" crolla (MODA −54%, 402 falsi allarmi/min) perché tiene la persona "dinamica" anche da
  ferma: non ha lo stato PD, e il suo `dynamic_consistency`/`frames_force_dynamic` mantiene l'etichetta.
- Variabilità tra run bassa (std ≤ 4 punti).

### Dynablox (campagna completa, 18 run) e note su M-detector (2026-10-15 sera)

Dynablox (solo LiDAR, `eval/baselines/dynablox/`, patch output-only per gli ID dei cluster): deterministico (le 3
ripetizioni danno lo stesso risultato: elabora ogni nuvola in ordine). Pubblica solo quando vede moto: nella 1942
(tutto fermo) il topic manca, trattato come uscita vuota (`marker_adapter`).

| | Solo movimento: recall | falsi allarmi/min | IDF1 | Ferma o no: recall |
|---|---|---|---|---|
| v2 | 96,1% | 11,9 | 95,8% | 98,5% |
| Dynablox | 96,6% | 68,1 | 64,7% | 52,7% |
| LV-DOT | 94,3% | 402 | – | 96,4% |

Dynablox trova la persona che cammina quanto v2, ma con 6 volte i falsi allarmi e meno identità; la persona ferma
la perde (52,7%), come atteso da un metodo di sola segmentazione del moto.

M-detector: accoppia la n-esima nuvola con la n-esima odometria (upstream FAST-LIO ne pubblica una per scansione);
la nostra odometria a 40 Hz andava quindi ricampionata: `make_input.py` scrive un'odometria per nuvola, interpolata
al suo timestamp, nel frame velodyne. Prima della correzione i muri risultavano dinamici (recall 0); dopo, sulla 1950:
solo movimento recall 89,5%, 32 falsi allarmi. Uscita usata: `/m_detector/point_out` (punti), raggruppati con il
DBSCAN comune di `points_adapter.py` (eps 0,5 m, 5 punti), senza ID.

### Confronto completo con 5 metodi della letteratura (2026-10-15/16, 3 ripetizioni ciascuno)

`results/baselines_r9.json`, `tables/baselines_r9.csv`; pagina: https://claude.ai/artifact/XGLMmauGC6XHbai8ygpQaz.

| Metodo | Sensori | Ferma o no: recall | Solo movimento: recall | Falsi allarmi/min (movimento) | IDF1 |
|---|---|---|---|---|---|
| **v2** | LiDAR + RGB-D | **98,5%** | 96,1% | **11,9** | **99,1%** |
| v1 | LiDAR + RGB-D | 92,0% | 92,1% | 49,3 | 60,4% |
| LV-DOT | LiDAR + RGB-D | 96,4% | 94,3% | 402 | – |
| Dynablox | LiDAR | 52,7% | 96,6% | 68,1 | 51,1% |
| M-detector | LiDAR | 48,6% | **97,9%** | 44,4 | – |
| Xu et al. 2023 | RGB-D | 5,9% | 3,1% | 37,6 | – |
| Xu et al. 2023 + YOLO | RGB-D + colore | 4,9% | 2,9% | 33,1 | – |
| UniMT + SimpleTrack | LiDAR + camera (appreso) | 47,9% | 14,5% | 248 | 54,7% |

- I due metodi solo LiDAR trovano la persona che cammina quanto o più di v2, ma con 4–6 volte i falsi allarmi, e la
  perdono quando si ferma. LV-DOT la tiene "in movimento" da ferma. Nessuno ha lo stato PD.
- Xu et al. (variante di default, senza YOLO) traccia la persona fusa con gli oggetti vicini (box larghi 2,9 m in
  media) e quasi mai la dichiara dinamica: probabilmente il vincolo di forma `target_object_size` scarta i box fusi.
  Da verificare con il modulo YOLO (serve la depth allineata al colore) prima di usarlo nel paper.
- (16/10) Xu con modulo YOLO (depth allineata ricostruita da `make_input_yolo.py`): YOLO vede la persona (1950: tra i
  box tracciati 308/555 frame), ma il metodo quasi mai la classifica dinamica → nessun miglioramento. Il limite è la
  classificazione del moto, non l'adattamento.
- (16/10) UniMT + SimpleTrack (pesi wheelchair, LiDAR 16 linee): 0 falsi allarmi a livello "D e PD insieme", e trova
  quasi tutte le persone nel suo campo (solo avanti, x 0–14,4 m): 1942 98%, 1949 100%, 1947 60% (persona davanti
  63%), 1950 58% (61%); 1945 e 1952 la persona è sempre dietro al robot → 0. Non classifica il moto: ogni box è
  "dinamico", da qui 248 allarmi/min sul livello solo movimento.
- (16/10) Due campagne in parallelo falsano la riproduzione (disco): ora `run_queue.sh` + `check_run.py` su ogni run;
  scartate e rifatte 2 run Xu+YOLO e 1 UniMT, le 76 precedenti sono valide.
- JRMOT bloccato (vedi `eval/baselines/README_baselines.md`).

## Esperimento mappa (2026-10-26): quanto della persona finisce nella mappa SLAM

GLIM da solo su ogni sequenza (`scripts/run_glim_map.sh`), con la rimozione dinamica BBOX alimentata dagli ostacoli
che ogni metodo ha davvero pubblicato (run r9a; per Dynablox e LV-DOT i loro box, tutti DYNAMIC, convertiti da
`map_obstacles_from_baseline.py`). Campagne `scripts/run_map_campaign.sh` e `run_map_campaign2.sh`, valutazione
`map_eval.py` (per sequenza, `results/map_r9a_<seq>.json`) e `map_table.py` (`tables/map_r9a.csv`).

- **Voxel fantasma**: voxel da 10 cm occupati entro 0,8 m dalla traiettoria mocap della persona, 0,2–1,8 m dal
  pavimento. 0,8 m perché `T_mocap_odom` non osserva l'offset base_link → marker del robot: a robot fermo il corpo
  della persona sta fino a ~0,6 m dalla posizione mocap (1942). Entro 0,8 m ci finisce anche un po' di struttura
  statica, quindi la quota rimossa è una stima per difetto.
- **Statico conservato**: quota dei voxel a più di 1,5 m dalla persona, nella mappa senza rimozione, che restano.
- Pavimento: livello più denso sotto base_link nella mappa di riferimento, lo stesso per tutte le varianti (la stima
  per singola mappa saltava tra pavimento e un ripiano ~1 m più su, e falsava 1945 e 1952 nella prima valutazione).

| Variante | 1942 (ferma) | 1945 | 1947 | 1949 (ferma) | 1950 | 1952 | Persona rimossa | Statico conservato (media / min) |
|---|---|---|---|---|---|---|---|---|
| nessuna rimozione | 140 | 7633 | 3153 | 185 | 4028 | 8083 | – | 100% |
| nessuna rimozione, ripetuta (rumore) | 140 | 7627 | 3152 | 184 | 4023 | 8079 | 0,1% | 98,7% / 97,3% |
| v2 solo D | 140 | 1097 | 766 | 185 | 1070 | 1315 | 80,3% | 97,8% / 95,0% |
| **v2 D+PD** | **0** | 689 | 196 | **2** | 303 | 808 | **91,4%** | 97,1% / 94,5% |
| v1 D+PD | 105 | 761 | 291 | 124 | 666 | 1967 | 83,1% | 97,3% / 94,3% |
| Dynablox | 141 | 525 | 707 | 184 | 960 | 705 | 86,1% | 98,6% / 97,0% |
| LV-DOT | 102 | 754 | 198 | 4 | 337 | 801 | 90,5% | 97,8% / 95,8% |

- Con la persona ferma (1942, 1949) solo PD la toglie dalla mappa: v2 solo D e Dynablox la lasciano tutta. LV-DOT la
  toglie in 1949 perché la tiene "in movimento" anche da ferma, lo stesso comportamento che gli costa 402 falsi
  allarmi/min nel benchmark.
- Con la persona che cammina v2 D+PD toglie più di v2 solo D in ogni sequenza: PD copre gli istanti in cui rallenta o
  si ferma. Dynablox fa meglio di v2 su 1945 e 1952.
- Costo: v2 D+PD conserva in media l'1,6% di struttura statica in meno rispetto al rumore di GLIM (box gonfiati e
  tracce PD su oggetti statici). Piccolo, ma è un costo da dichiarare nel paper.

## Sosta e ripartenza, ablazione della semantica, figure (2026-10-26)

**Identità nelle soste** (`stopgo_eval.py`, `results/stopgo_r9.json`): 16 eventi sulle 6 prove (8 soste D→PD, 8
ripartenze PD→D, fasi di almeno 1 s). "Stesso ID" = l'ultimo ID dato alla persona nel secondo prima coincide con il
primo dei 2 s dopo; "seguita da ferma" = quota dei frame PD in cui la persona è trovata (qualsiasi label).

| Metodo | Stesso ID | Seguita da ferma |
|---|---|---|
| **v2** | **93,8%** (15/16) | **93,1%** |
| v1 | 87,5% | 85,4% |
| Dynablox | 62,5% (ottimista: numera i cluster da 0, con una persona l'ID coincide quasi sempre) | 15,1% |
| UniMT + SimpleTrack | 37,5% | 72,0% |
| LV-DOT / M-detector / Xu+YOLO | n/d (nessun ID stabile) | 93,1% / 6,8% / 6,2% |

L'unico evento perso da v2: la partenza di 1952 a 1,8 s, persona dietro al robot e ancora senza traccia.

**Ablazione "semantica spenta"** (`ablation_semantics.py`, solo tracker sugli input r9a, deterministico;
`results/ablation_semantics_r9a.json`, `stopgo_ablation_r9a.json`). `nosem`: whitelist delle classi dinamiche
vuota (`track_semantic_dynamic_classes:=['__none__']`), quindi nessuna traccia ha mai una classe; `nosem_nopd`:
anche il PD da solo movimento spento (`track_motion_pd_min_dynamic_ticks:=0`). Il rilevatore resta identico.

| Variante | F1 D | F1 PD | Trovata, ferma o no | IDF1 | Stesso ID nelle soste | Falsi dinamici S/min |
|---|---|---|---|---|---|---|
| v2 (solo tracker) | 96,2% | 96,4% | 98,5% | 99,1% | 93,8% | 1,25 |
| senza semantica | 93,9% | 2,5% | 44,3% | 58,8% | 50,0% | 1,25 |
| senza semantica né PD | 93,9% | 0 | 42,8% | 57,2% | 43,8% | 1,25 |

Senza semantica la persona ferma viene trattata come oggetto fermo e eliminata: è la semantica, non il movimento
passato, a tenere in piedi lo stato PD. Il replay solo-tracker di v2 riproduce i numeri della pipeline completa.

**Figure e video**: `make_figures.py --lang it|en` → `eval/figures/<lang>/` (PNG + PDF): timeline delle label
(1947, 1950, 1952), foto della camera con le label di ogni metodo, barre D/PD/S, sosta e ripartenza, mappa vista
dall'alto e barre della mappa, ablazione. `make_video.py --map` ora accetta i baseline e disegna la mappa statica:
`eval/videos/compare_r9a_1950.mp4`, `compare_r9a_1947.mp4`. Pagina: https://claude.ai/artifact/2GJ3QbsawkCTCjpxh4bP46.
Nelle foto si vede il volto della persona: per il paper serve il consenso o una sfocatura.

## Nome del metodo e prova fuori dal campo visivo (2026-10-26)

**Nome**: da oggi il metodo si chiama **SINS (Still Is Not Static)**. Nelle figure, nei video e nelle pagine compare
come SINS; nel codice restano il pacchetto `jo_detector_v2`, i topic `/jo_detector_v2/...` e le chiavi `v2` dei
risultati (le bag registrate li contengono: un cambio va fatto su un branch dedicato, con remap).

**Dentro e fuori dal campo della camera** (`fov_eval.py`, `results/fov_r9.json`, 3 ripetizioni). Camera colore
frontale 89,6° (da camera_info), entro 6 m; posizione della persona nel frame del robot dal mocap. Fuori dal
campo: 1945 e 1952 sempre, 1947 e 1950 il 44–45% del tempo, 1942 e 1949 mai; in tutto 1550 frame su 2890.
Mentre cammina fuori dal campo la persona gira in mediana di 35–56°/s (percorsi curvi; uno zig-zag vero e proprio
non è mai stato registrato: va nella nuova campagna).

| Metodo | Dove | Cammina: trovata / come D | Ferma: trovata / come PD | Cambi di ID per ripetizione |
|---|---|---|---|---|
| SINS | nel campo | 98,6% / 91,7% | 99,6% / 98,7% | 2,7 (lampi di un tick) |
| SINS | fuori | 98,9% / 96,6% | 93,7% / 83,5% | 0 |
| v1 | nel campo | 97,0% / 75,8% | 91,7% / 87,6% | 21,7 |
| v1 | fuori | 94,2% / 93,8% | 84,0% / 70,4% | 4,3 |
| Dynablox | nel campo | 91,7% / 91,7% | 9,8% / – | 4,0 |
| Dynablox | fuori | 97,0% / 97,0% | 39,3% / – | 40,0 |

Fuori dal campo SINS segue chi cammina come dentro; il punto debole è la persona che si ferma fuori dal campo
(PD 83,5% contro 98,7%), perché lì nessuna classe YOLO la sostiene: regge solo se la classe era già stata vista.

## EagerMOT (2026-10-26)

`eval/baselines/eagermot/`: codice originale (commit 2792a76, in `eagermot_src/`, ignorato da git), fusione 3D-2D e
tracking invariati, parametri pedoni di nuScenes (`NUSCENES_BEST_PARAMS`). Box 3D: CMDT, il rilevatore di UniMT,
pubblicato prima del suo tracker (`/unimt/detections`, campagna `PREFIX=unimtdet`); box 2D: lo YOLO della replay SINS
della stessa ripetizione. `export_inputs.py` (python di sistema) → `run_eagermot_jo.py` (venv `.venv`) →
`bags/eagermot_out/eagermot<rep>_<seq>.csv`. Nessuno stato di moto: ogni traccia conta come "dinamica", come UniMT.

| | Ferma o no: recall | Solo movimento: recall | Allarmi/min (movimento) | IDF1 | Stesso ID nelle soste | Nel campo / fuori |
|---|---|---|---|---|---|---|
| EagerMOT | 47,6% | 13,9% | 248 | 53,9% | 37,5% | trovata 97,9% / 4,1% |
| UniMT + SimpleTrack | 47,9% | 14,5% | 248 | 54,7% | 37,5% | 97,4% / 5,1% |

Stesso rilevatore, due tracker diversi, stesso risultato: il limite è CMDT, che guarda solo avanti (x 0–14,4 m).
