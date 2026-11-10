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
| ROS | ROS 2 Jazzy (container `jo_sim_sins`, immagine `jo-sim_sins:latest`) |
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
| v2 non girava sulle bag con una sola camera | argomento `camera_names` (`sins` `ba0353a`) |
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

Variante successiva (`sins` (allora `sins`), commit dopo `7a91090`): il debounce lungo vale solo per i track senza classe
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

**Altezza nell'associazione** (`sins`, commit dopo `52a5573`): il track tiene una media mobile della **cima**
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

**Pipeline completa, r7** (`sins` con le tre correzioni, `greedy_kuhn`; v1 corretto, config indoor;
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
`cfg/{indoor,outdoor}/tracker.yaml`, `sins` `6ff364b`); `greedy_kuhn` resta per l'ablation. I numeri r7
sopra sono stati misurati con `greedy_kuhn`; con `hungarian` sugli stessi input: MOTA 97,44, IDF1 88,92, IDSW 4.

---

## Campagna r8 (2026-10-14, completata): `hungarian` a pipeline completa, 3 ripetizioni

Obiettivo: numeri da paper con media ± deviazione standard (l'IDF1 cambia di vari punti tra run) e l'ablation
dell'associazione a parità di input.

- Codice: `sins` `6ff364b` (default `hungarian`), `jo_detector` `e2bd34a` (config indoor).
- `scripts/run_campaign.sh` (copia in `bags/replay/`): per ripetizione a, b, c e per sequenza, v2 e v1 a pipeline
  completa (`r8<rep>_{v2,v1}_<seq>`, v2 registra anche gli input del tracker), poi `greedy_kuhn` col solo tracker
  sugli input di ciascun run v2 (`r8<rep>g_v2_<seq>`). 54 job, circa 1,5 min l'uno.
- **Ripresa**: ogni job finito lascia `<out>.done`; rilanciare lo script salta i job finiti e rifà da capo quello
  interrotto. Stato e orari in `bags/replay/r8_state.log`.
  - interrompere: `docker exec jo_sim_sins bash -c "pkill -f '[r]un_campaign.sh'; pkill -f '[r]un_tracker_only.sh'; pkill -f '[r]un_method.sh'; bash /home/ros/bags/replay/cleanup.sh"`
    (le parentesi quadre servono: senza, `pkill -f` uccide anche la shell che lo esegue e il resto non parte)
    (oppure `touch bags/replay/r8.stop` per fermarsi alla fine del job corrente)
  - riprendere: `rm -f bags/replay/r8.stop; docker exec jo_sim_sins bash -c "CAMPAIGN=r8 bash /home/ros/bags/replay/run_campaign.sh"`
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

## Correzione della 1947 (2026-10-15, `sins` `df56a77`)

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

## Correzione della 1950 (2026-10-15, `sins` `4a41b17`)

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

- Codice: `sins` `4a41b17` (frammenti + classe). v1 non è cambiato: i run v1 sono quelli di r8, collegati
  come `r9<rep>_v1_*`. Niente ablation greedy (`V1=0 GREEDY=0`, nuove opzioni di `scripts/run_campaign.sh`).
- 18 job v2 (3 ripetizioni × 6 sequenze), stato in `bags/replay/r9_state.log`.
- Interrompere: `touch bags/replay/r9.stop` (alla fine del job in corso) oppure il `pkill` della sezione r8.
- Riprendere: `rm -f bags/replay/r9.stop; docker exec -d jo_sim_sins bash -c "cd /home/ros/bags/replay && CAMPAIGN=r9 V1=0 GREEDY=0 bash run_campaign.sh > r9_campaign.log 2>&1"`
- Valutazione: `python3 eval/campaign_report.py --campaign r9 --reps a b c`.
- Istruzioni complete per chi riprende (anche da un altro account Claude):
  `~/Desktop/claude_account_backup/sins_handoff/LEGGIMI_PRIMA.md`.

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

**Nome**: da oggi il metodo si chiama **SINS (Still Is Not Static)**, ovunque. Dal 27/10 anche nel codice: pacchetto
ROS `sins` (cartella `sins/`, repo `github.com/ducciopet/sins`, già `sins`), topic `/sins/...`. Le bag
registrate prima hanno i topic `/jo_detector_v2/...`: `run_tracker_only.sh` li rimappa e i lettori (`static_eval.py`,
`analyze_errors.py`) accettano entrambi i nomi. Le chiavi `v2` dei risultati restano (sono nomi di colonne e file).

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

## Fase 4: dataset pubblici, CODa outdoor con LiDAR a 128 linee (2026-10-29)

Dati sul kloster (`/mnt/kloster-reichenau/datasets_SINS/`, va montato a mano dopo ogni riavvio): CODa seq 0 e 1,
JRDB train (27 sequenze), SemanticKITTI (scansioni + etichette, senza immagini). Split: CODa seq 0 = tuning,
seq 1 + JRDB = held-out (mai guardati durante il tuning).

**Pipeline.** `eval/datasets/convert_coda0.sh` → `coda_to_bag.py`: 3 finestre annotate di seq 0 (w1 200–799,
w2 4550–4999, w3 5400–5799, 20 s di rodaggio), bag sensori + bag odometria + GT (`<w>_gt.csv` con classe, punti LiDAR
nel box `npts`, `in_fov` della camera). SINS: `eval/scripts/coda_ab.sh <TAG>` (rigenera `cfg/coda` con
`eval/configs/sins/make_cfg.py coda` da `cfg/outdoor` + `eval/configs/sins/coda/overrides.yaml`, gira nel container,
valuta). Valutazione `eval/coda_eval.py`, aggregato `coda_pool.py`, diagnosi `coda_diag.py`, confronto `coda_compare.py`.

**Protocollo (fissato prima dei confronti, uguale per tutti i metodi).**
- GT = tutti gli agenti mobili annotati (pedoni, bici, monopattini, veicoli di servizio, la statua "Horse"); box senza
  punti LiDAR ignorati (criterio KITTI/nuScenes). Il flag `isOccluded` di CODa NON si usa: descrive la camera (gli
  operatori a 1–2 m dal robot sono "Full" con migliaia di punti).
- Oggetti fermi (PD) contano solo nel campo della camera (fuori, nessun metodo con una camera frontale può dire che
  una bici parcheggiata non è un paletto); quelli in movimento contano ovunque.
- Esclusi GT e predizioni entro 1,2 m dal LiDAR: l'operatore che tiene il robot cammina a 0,9 m dietro il sensore
  (offset fisso, 6 cm di std), annotato come pedone in w1 e w3 ma non in w2, visto solo in una fascia di 0,7 m.
- Predizioni con centro oltre 2,5 m sopra il suolo del robot scartate (chiome, tetti): aiuta i baseline, non SINS
  (taglia già a 3 m).
- Raggio 25 m (= `lidar_range` di cfg/coda). Match 1 m.

**Errori trovati e corretti (miei, non del metodo).** TF camera color→depth nel convertitore in verso opposto a quello
che SINS ripubblica (anello nel TF tree, w2/w3 senza camera): ora depth→color come il driver RealSense. SVD del fit del
suolo a matrici piene (29 GB di RAM). Stampi GT via float (solo nella diagnosi).

**Tuning outdoor (modifiche fisiche, A/B su seq 0, ripetizioni per il rumore: ±3 MOTA, ±6 IDF1 tra run identiche).**
1. Nuovo parametro `yolo_leaf_lidar_beyond_depth` (preprocessing): i punti dell'ultima scansione LiDAR oltre
   `depth_max_value` entrano nelle leaves YOLO. Motivo: la depth della ZED in CODa è affidabile solo fino a ~4 m
   (misurato contro il LiDAR: errore mediano 0,13 m a 2–4 m, 0,48 a 4–6, 1,9 a 8–10), quindi le bici a 10–15 m che
   YOLO vede non ricevevano mai la classe. `true` in `cfg/outdoor`, `false` in `cfg/indoor` (lab invariato).
   Aggregato 3 finestre: recall 44,8→48,8%, IDF1 52,0→59,0%, MOTA 34,3→37,5%.
2. `lidar_eps_vertical_angle_deg: 0.33` per CODa (spaziatura delle linee dell'OS1-128 misurata; base VLP-16 2,0):
   media su 2 ripetizioni MOTA 35,5→37,8%, FP/min 135→113, IDF1 invariato.
3. Preprocessing con 131 072 punti/scansione: dopo il voxel a due bande e il taglio suolo/tetto restano 24–32 mila
   punti, il downsampling adattivo interviene di rado (foglia 0,10 m); 9–18 ms per scansione, DBSCAN 29–37 ms
   (nuovo log `lidar point counts` del preprocessing). `depth_max_value` 4,5 m confermato dalla misura sopra.

Limite noto: bici parcheggiate a 12–15 m nel campo della camera trovate poco (la leaf da box YOLO di un oggetto sottile
prende il muro dietro, che ha più punti).

**Baseline su CODa** (stesso container e logica originale del lab, solo il modello del sensore OS1-128):
`eval/baselines/dynablox/run_dynablox_coda.sh` (+ `motion_detector_coda.yaml`), `mdetector/run_mdetector_coda.sh`
(+ `coda.yaml`, `make_input_coda.py`; M-detector a 1x perde metà delle scansioni da 131k punti e accoppia nuvole e
odometria sbagliate, quindi anche una run a 0,25x), `lvdot/run_lvdot_coda.sh` (+ `make_param_coda.py`). UniMT/EagerMOT
non applicabili senza riaddestramento (rilevatore allenato sul VLP-16 del lab). Pose di CODa verificate (scarto mediano
6–10 cm tra scansioni a 0,5 s, sfasamento migliore = 0). Il raggruppamento dei punti dinamici di M-detector usa
min 20 punti (default Dynablox) invece dei 5 del lab, per la densità 8 volte maggiore.

**Confronto su CODa seq 0** (3 finestre, 1 650 frame; SINS media di 3 run B1/B1r2/B1r3; `results/coda/compare.json`,
`tables/coda_compare.csv`; pagina https://claude.ai/artifact/XGLMmauGC6XHbai8ygpQaz#coda):

| Metodo | Trovati (D+PD) | In movimento: recall / precisione | FP/min | MOTA (movimento) | IDF1 |
|---|---|---|---|---|---|
| SINS | 47,5% | 82,8% / 86,3% | 96 | 69,4% | 55,4% |
| Dynablox | 47,1% | 87,4% / 8,5% | 5 880 | −852% | 12,8% |
| M-detector 1x | 5,6% | 10,3% / 29,5% | 155 | −24% | – |
| M-detector 0,25x | 43,0% | 79,9% / 17,5% | 2 360 | −375% | – |
| LV-DOT | 1,3% | 2,5% / 71,0% | 6 | −1% | – |

ATTENZIONE: seq 0 = split di tuning di SINS. Prossimi: convertitore seq 1 (finestre sulle annotazioni) e JRDB
(held-out), stesse regole; lì i numeri del paper.

### 30/10: CODa, bici parcheggiate e confronto a 8 metodi (seq 0, split di tuning)
- Bici ferme non classificate: YOLO le vedeva, la classe non arrivava al LiDAR. Correzioni (solo profilo outdoor, indoor
  invariato per costruzione): foglie LiDAR dalla nuvola senza muri (`lidar_leaf_cloud_` dopo `isNearAnyWall`);
  `yolo_leaf_nearest_cluster` + `yolo_leaf_min_box_coverage 0.5` per foglie fatte per lo più di punti oltre la depth;
  `detection_frame_stride 1` su CODa (ZED a 7,5 Hz); `detection_inference_size 1280` outdoor (bici/immagine 0,9 -> 1,6).
  Provata e scartata `track_first_class_weak_keeps_streak` (nessun guadagno chiaro, più falsi PD).
- Risultato SINS C7 (3 rep): moving rec 84,6% prec 83,4%; PD trovati 3,5% -> 13,1%, falsi PD 17 -> 143/min (3/4 su
  paletti "Bollard" annotati, il resto non annotato: persona dietro vetro, operatore). FP totali 96 -> 240/min.
  Limite: bici in rastrelliera frammentate in molte tracce.
- Nuovi baseline: CenterPoint (pesi CODa degli autori UniMT, `centerpoint_coda_cfg.py`, CenterHead standard, nessuna
  chiave mancante) + SimpleTrack (`eval/baselines/unimt/infer_coda.py`, `run_unimt_coda.sh`, manifest
  `make_coda_manifest.py`); EagerMOT (`eagermot/export_inputs_coda.py`, 3D CenterPoint > 0,3 + YOLO). CMDT CODa non
  pubblicato; `unimt_coda`/`cmt_coda` richiedono codice non rilasciato. Pesi addestrati su CODa train: favoriti.
- Valutazione: metriche per label (`fp_by_state` in harness/metrics.py, `per_label` in coda_pool.py). Video
  `eval/make_coda_video.py`. Risultati `eval/results/coda/compare.json`, `eval/tables/coda_compare.csv`.
- (30/10 pomeriggio) Regola: ogni classe YOLO dello YAML -> D se si muove, PD se fermo. Su CODa falliva per oggetti
  fermi lontani. Aggiunti (profilo outdoor, indoor invariato): `yolo_leaf_method:=mask` di default nei run CODa
  (`coda_ab.sh`), cfg/outdoor/detection.yaml = yolo26s-seg, stesse classi del rilevatore a box, 1280 px; test di
  copertura (>= 50% larghezza) anche sulle maschere; memoria di classe degli oggetti fermi nel tracker
  (`track_class_anchor_ttl 5 s`, `_min_votes 2`, `_radius 0.6`, solo da tracce ferme); `max_box_extent [6,6,3.5]`,
  `max_box_volume 40` (furgone 3,2 m scartato). Risultato E4 (3 rep): PD 28,0% prec 53,6% falsi PD 114/min; moving
  83,3/80,3. Per oggetto: bici/monopattini 29-68% PD, furgone 55% PD (14% D: centro del cluster che si sposta), bici
  coperte in fondo alla rastrelliera ~0.
- NAS al 100% (9,8 GB liberi): download KITTI in pausa (kill -STOP sul python di remote_zip), CROSSLAB 2/5 zip.
- (30/10 sera) Configurazione CODa finale G1 (3 rep): seconda camera `stereo_camera` = CODa cam0/cam1 (un fotogramma per
  scansione; `eval/datasets/coda_stereo_bag.py` -> bags/coda/coda0_<w>_stereo), depth online da
  `eval/adapters/stereo_depth_node.py` (SGBM 3-way a mezza risoluzione, 28 ms/coppia; errore vs LiDAR 2% fino a 6 m,
  ZED 10% a 4-6 m) -> `depth_max_value` per camera (nuovo `<cam>.depth_min_value/depth_max_value`, stereo 6 m, ZED 4,5).
  `run_coda.sh`/`run_method.sh` (copie in eval/scripts/): STEREO=1, EXTRA_BAGS, ADAPTER_CAMS. Altri cambi outdoor:
  altezza minima per classe veicolo 1,1 m (`yolo_class_min_height_*`, `track_class_min_height_*`; nessun limite per
  persone/bici/animali), corroborazione del moto per tracce con classe >= 2 m (`track_classified_corroboration_min_size`).
  Scartati (3 rep): voxel solo < 8 m (`lidar_processed_voxel_max_range`, PD prec 59 -> 48%), YOLO m/l (+4% bici, 2x tempo).
  Risultati: PD 42,2% prec 68,5% falsi PD 97/min; moving 84,9/77,7; FP tot 231/min; IDF1 55,7%. Latenza attesa
  camere nel clustering 77 ms (45 con una). Bici 25-62% PD, monopattini 59-91%, furgone 58% PD / 5% D.
- CROSSLAB: 2/5 zip sul NAS, verificati. NAS a 3 GB liberi: KITTI in pausa (SIGSTOP), servono ~90 GB (cestino NAS).
- (30/10 notte) Limiti di G1 (oggetti fermi annotati): ~52% PD, 24% nessuna traccia (bici in rastrelliera), 14% traccia
  senza classe (1/4 statua), 6% a 1-2 m, 4% D. FP: paletti PD (~60/min), non annotati D (~45), strutture D (~30),
  centri a 1-2 m (~25). Scartati (3 rep): `track_class_anchor_min_votes 3` (PD 42 -> 39%, falsi PD invariati),
  `yolo_refine_nested` (taglio di cluster che contengono la foglia: PD +1,6 pt, precisione PD 69 -> 60%). Entrambi
  restano come opzione spenta. `yolo_leaf_method` ora nello YAML del profilo (outdoor mask, indoor box): il launch lo
  legge se l'argomento è vuoto; coda_ab.sh non lo passa più.
- (30/10 notte) Analisi stadio per stadio degli oggetti fermi (run K1 = G1 con tutti i topic intermedi;
  `eval/diagnostics/coda_still_funnel.py`, statua esclusa): PD ok 54,7%; YOLO a box non lo vede 13,2%; preprocessing
  <5 punti 1,8%; nessuna foglia sopra 13,7% (7,9% nessuna foglia dinamica entro 2 m, 5,2% foglia accanto); clustering
  2,8%; cluster senza classe 8,1%; tracker 1,6%; classe ma D 4,0%. Box e segmentazione (K2, w1) vedono ciascuno l'80%,
  insieme l'89%, e sono in disaccordo sul 18%: le foglie (mask) vengono solo dalla segmentazione. Leve: unione
  foglie box+maschera; associazione YOLO->cluster nello spazio immagine (proiezione dei cluster nella maschera) al
  posto di istogramma di profondità + IoU 3D; moto da punti per gli oggetti classificati fermi.

### Registro delle prove CODa (TAG di `eval/scripts/coda_ab.sh`, seq 0, tutte con 3 finestre salvo nota)
| TAG | Cosa | Esito |
|---|---|---|
| B1, B1r2, B1r3 | profilo outdoor del 29/10 | riferimento "prima" |
| D1-D3, K1, K2, L0* | run di diagnosi con topic intermedi (una finestra o tutte) | solo analisi |
| C1-C7 | foglie senza muri, YOLO 7,5 Hz, gruppo più vicino, copertura riquadro, YOLO 1280 | C7 tenuto |
| E1-E4 | memoria di classe, maschere YOLO, limiti dimensione veicoli, copertura su maschere | E4 tenuto |
| F1-F3 | voxel solo vicino (F2: scartato), altezza minima veicoli 1,1 m e conferma moto oggetti grandi | F3 tenuto |
| G1 x3 | F3 + seconda camera stereo (STEREO=1) | tenuto (2 YOLO per camera) |
| H1 x3 | memoria con 3 conferme | scartato |
| I1 x3 | taglio dei cluster che contengono la foglia (`yolo_refine_nested`) | scartato |
| M1 x3 | un solo YOLO per camera (maschere) + conferma moto per tutte le tracce con classe | |
| M2 x3 | M1 + riquadro di riserva quando la maschera non dà foglia | **configurazione attuale** |
| M3 | M2 + associazione per punti (`yolo_refine_assoc: points`) | interrotta (2 finestre), scartata dalle run singole L0* |
| N1 x3 | M2 + `track_lidar_fallback: false` (niente classe 2D ai cluster solo LiDAR) | scartato: PD sbagliati/min 235 vs 122, precisione PD 51% vs 65%, PD trovati 46% invariati; moving 84,3/81,2 vs 83,3/82,6 |

- (02/11) CODa seq 1 (held-out) controllata: solo due tratti annotati, 1750-2249 (2 pedoni) e 2900-3399 (3 pedoni + 1
  cavallo), nessuna bici/veicolo. Troppo povera come banco PD: proposta JRDB come held-out principale, seq 1 solo
  controprova sulle persone in movimento.

### JRDB held-out (02/11)
- Configurazione bloccata: tag `coda-frozen` su sins (M2 + correzione del profilo indoor: `yolo_class_min_height_*: []`
  faceva abortire dbscan_detector_node all'avvio con env:=indoor dal 30/10, commit sins `8019cab`; outdoor invariato).
- Pipeline JRDB: `eval/datasets/jrdb/` (README con tutte le scelte), depth online dal LiDAR
  (`eval/adapters/lidar_depth_node.py`, hook `LIDAR_DEPTH` in run_method.sh), `eval/scripts/{run_jrdb,jrdb_ab}.sh`,
  `coda_pool.py --jrdb`. 6 sequenze (4 indoor, 2 outdoor), scelte dalle etichette di azione JRDB prima di ogni run.
- (02/11) PROSSIMO FILONE (richiesta dell'utente): riconoscere e rimuovere il suolo su terreno irregolare (pendenze,
  sassi, dislivelli) invece di un unico piano RANSAC. Bag di prova: Marina di Pisa 14/10, missione "Slope recognition"
  (spiaggia di sassi, 5 min, 21 GB): registrazioni_robot/marina_di_pisa_20261014/2026-10-14_11-42-41_mdp-seaside_duccio-petreni.zip
  (bag 20261014_094511_mdp_slopes). Modifica di metodo: dopo le run held-out a configurazione bloccata, solo profilo outdoor.
- (02/11 pomeriggio) Prima run JRDB (J1, solo 1 ripetizione, fermata): indoor quasi vuoto (huangb 0,1%, tressider 0%,
  packard 23%), outdoor ~55%. Causa: il pavimento di static_structures viene SOLO dalle camere (RANSAC, primo piano
  orizzontale); con camere orizzontali e depth dal LiDAR diventava il soffitto o il piano dei tavoli e tagliava le
  persone. Fatto: (1) base_link JRDB sul montaggio fisso 1,24 m (jrdb_reframe.py; forbes 0,71 m e bytes 0,43 m erano
  fit sbagliati su scale/tavoli); (2) opzione `floor_lowest_below_sensor` (spenta di default, accesa nei profili
  JRDB): A/B su bytes-cafe (debug) recall 17,7 -> 40,0%, ma il pavimento resta a 0,3-0,8 m: in un bar affollato il
  pavimento non lo vede nessun sensore (VLP-16 a 1,24 m lo tocca solo oltre 4,6 m). Serve una stima del suolo che
  usi anche il LiDAR e il piano su cui poggia il robot = il filone "suolo irregolare". Run J1 da NON usare.

### Suolo irregolare (dal 02/11, anticipato prima delle run JRDB held-out)
Vincolo dell'utente: il pavimento viene dalle camere perché il LiDAR ha un grande cono d'ombra intorno al robot;
il LiDAR può aggiungersi solo lontano. Piano: modello del suolo a griglia (altezza per cella) che fonde camere (vicino),
LiDAR (lontano) e il terreno sotto le ruote (cono d'ombra), cresciuto dal robot con limite di pendenza; switch, indoor
spento. Oggi `heightAboveGround()` = MINIMO su tutti i piani delle camere: un solo piano sbagliato (tavolo) taglia
tutto sotto di lui intorno al robot.
- Metrica: `eval/diagnostics/coda_ground_eval.py` (etichette di terreno per punto di CODa, 3d_semantic, 200 scansioni
  in w2/w3; run con MORE_TOPICS=/sins/lidar/processed). Riferimento GR0 (= M2): suolo rimasto 758 / 3956 punti per
  scansione (10,6% / 17,2% dei voxel di terreno, w2 / w3); punti degli agenti rimasti 71% / 71%, ultimi 50 cm 48% / 55%.
- Bag dei sassi: `eval/datasets/extract_topics.py` (estrazione in streaming dallo zip, solo i topic di SINS, ~6,7 GB).
- Prototipo griglia (`eval/diagnostics/ground_grid_proto.py`, offline, causale; ingressi = scansione LiDAR intera +
  depth delle camere della run GR1 + odometria). Griglia 0,25 m in odom, crescita dal robot (celle sotto il robot =
  quello che vedono i sensori, altrimenti la quota del robot), punto più basso valido della cella accettato se entro
  0,10 m + 0,25 m*tan(20 deg) dal vicino (sopra: oggetto, sotto: riflesso), superficie = mediana dei 10 cm più bassi,
  memoria 30 s, margine 0,15 m. Metrica gambe = punti degli agenti 0,25-0,75 m sopra la base della box.
  Piani SINS (GR1) vs griglia: w2 suolo rimasto 7,9 -> 5,9%, gambe tagliate 49,0 -> 4,7%, agenti tagliati 28,3 -> 3,1%;
  w3 suolo 19,6 -> 15,5%, gambe 0,5 -> 4,3%, agenti 3,5 -> 5,8%. Scartati: riferimento al vicino più basso (w2 suolo 47%),
  pendenza 30 deg (gambe w2 19%). Attenzione: /sins/lidar/voxelized_sensor NON ha il suolo (fascia tagliata per i muri).
- Bag dei sassi estratta: bags/mdp/mdp_slopes (7,1 GB, 292 s; senza /odometry/filtered: c'è /odometry/local e glim).
- (02/11) Marina di Pisa "mdp_slopes": odometria. /odometry/local è 2D; GLIM pubblica PIÙ stime sullo stesso
  /glim_ros/odom (anche nella registrazione a bordo) e il flusso principale deriva 26 m (mediana) rispetto all'RTK;
  l'EKF di make_odom.sh dà z=0 e 284 km. GNSS RTK fisso 1166/1169 fix (estratto in bags/mdp/mdp_slopes_gnss): percorso
  107 m, dislivello 1,24 m. KISS-ICP: xy 0,41 m mediana, ma z fino a 4 m. Riferimento = `eval/datasets/rtk_odom_bag.py`
  (posizione RTK, roll/pitch IMU, heading KISS-ICP) -> bags/mdp/mdp_slopes_odom_rtk.
- (02/11 sera) Griglia del suolo in C++ (`sins/include/sins/ground_grid.hpp`, `ground_model: grid` in cfg/outdoor e nei
  profili JRDB; indoor del laboratorio = piani). Costo: scansione preprocessing 11-15 -> 17-24 ms su OS1-128 (griglia
  50x50 m), dentro il budget di 100 ms: si tiene all'istante k. GG1 x3 vs M2 x3 (CODa seq 0): PD trovati 46,1 -> 59,8%,
  precisione PD 64,9 -> 49,4%, PD sbagliati/min 122 -> 319, moving 83,4/82,6 -> 84,8/80,6, FP/min tot 219 -> 425,
  MOTA 41,6 -> 29,3%, IDF1 56,3 -> 49,4%. Suolo (etichette): w2 suolo rimasto 10,6 -> 5,4%, parte bassa degli agenti
  48 -> 68%; w3 falsi D/min 127 -> 65. L'aumento dei PD falsi è quasi tutto in w1 (255 -> 670/min): archetti a U delle
  rastrelliere, paletti, rastrelliera al muro, accanto alle bici (classe "bicycle" da YOLO). I piani li tagliavano
  insieme a gambe e ruote (pavimento troppo alto): la griglia ha scoperto il problema già noto dei paletti PD.
  Immagini: eval/results/ground/coda_w1_fp/. Video del suolo: ~/Desktop/suolo_video/.
- (02/11 sera) Altezza minima per la classe bicycle (outdoor, dbscan + fallback tracker): GH1 0,7 m x3: PD 39,8% prec
  69,1% falsi PD 93/min, FP tot 180, MOTA 44,5 IDF1 57,5 (troppe bici in rastrelliera perse); GH2 0,6 m x3 (altezze
  osservate: bici vere mediana 0,80, archetti falsi 0,55): PD 50,8% prec 64,0% (F1 PD 0,57 vs M2 0,54), falsi PD
  148/min, moving 84,9/79,3, FP tot 270 (M2 219, rumore 172-322), MOTA 39,5 IDF1 55,0. Bloccata GH2: tag sins
  `heldout-v2`. Lanciate le run JRDB held-out J2 x3 (6 sequenze) + prova su bytes-cafe (DBG2).
- (02/11 ~19:00) FERMATO su richiesta dell'utente (porta il PC a casa). Stato per ripartire:
  * JRDB held-out heldout-v2: ripetizione J2 GIRATA su tutte le 6 sequenze (bags/jrdb/runs/J2_*.done), VALUTATE solo
    forbes, huangb, packard2 (eval/results/jrdb/J2_*.json). Da fare: valutare J2 tressider2/clark0/clark1
    (`python3 eval/coda_eval.py --run bags/jrdb/runs/J2_v2_jrdb_<s> --gt bags/jrdb/<s>_gt.csv --odom bags/jrdb/<s>_odom
    --range 15 --out eval/results/jrdb/J2_<s>.json`), poi J2r2 e J2r3 (`bash eval/scripts/jrdb_ab.sh J2r2` ecc.).
    Prova di debug DBG2 su bytes-cafe fatta (eval/results/jrdb/DBG2_bytes.json).
  * Analisi PD: script pronto `eval/diagnostics/pd_funnel.py <TAG>`; run di diagnosi da lanciare:
    `MORE_TOPICS="/sins/lidar/processed /sins/front_camera/semantic_leaves /sins/stereo_camera/semantic_leaves
    /sins/dbscan_detector/lidar/detections3d" STEREO=1 bash eval/scripts/coda_ab.sh K3` poi `python3 eval/diagnostics/pd_funnel.py K3`
    (script mai eseguito: possibile qualche errore da correggere al primo giro).
  * Copia locale CODa per la metrica del suolo senza NAS: INTERROTTA (bags/coda/raw_seq0, 126/850 scansioni). Da
    completare in laboratorio (serve il NAS) oppure cancellare; gli script NON la usano ancora (leggono dal NAS).
- (02/11 sera, a casa) JRDB held-out heldout-v2, ripetizione J2 (6 sequenze, valutazione completa): tutte le persone
  rec 43,6% prec 86,3%; PD (persone ferme) rec 37,9% prec 81,3%, mai trovate 59,5%; moving rec 48,6% prec 74,1%;
  IDSW 6537 (folla). Per sequenza (rec/prec): forbes 34/82, huangb 29/88 (J1 0%), packard2 54/85 (J1 23%), tressider2
  40/95 (J1 0%), clark0 58/83, clark1 60/75. La griglia risolve il crollo indoor dei piani. FP/min JRDB non confrontabili
  con CODa (15 Hz, ~30 persone/fotogramma). Liberati 1,5 GB (run GG1*, K1, copia CODa interrotta). Lanciate K3
  (diagnosi CODa) + pd_funnel e le ripetizioni J2r2/J2r3.
- (02/11 sera) Analisi PD (`eval/diagnostics/pd_funnel.py K3`, K3 = heldout-v2 con topic intermedi; testo in
  eval/results/pd_analysis/). Oggetti fermi in vista camera (745 righe): PD ok 63,4%; YOLO non lo vede 16,9% (5 punti
  = statua "Horse"); foglia ok + cluster ok ma classe non passa (stadio 5) 10,1%; clustering 3,8%; foglia accanto /
  1-2 punti 3,1%; tracker 2,6%. Stadio 5 = 75 casi: 56 bici + 14 monopattini (foglia "bicycle") + 5 furgone; solo 16
  cluster sotto 0,6 m (filtro bici); gli altri cluster 0,6-1,1 m alti, 0,5-2 m lunghi, a 0,24 m dall'oggetto, 38 punti
  di foglia: rifiutati dall'associazione per IoU 3D delle box (cluster bici+rastrelliera più grande della foglia).
  PD falsi (184 righe, 15 tracce): quasi tutti UNA traccia 0,43x0,18x0,54 m accanto al furgone con classe "truck" dalla
  MEMORIA di classe (classAnchor non controllava l'altezza minima della classe: corretto in tracker_node.cpp).
  In coda (chain_gp): ricompilazione dopo J2r2/J2r3, poi GH3 x3 (correzione memoria) e GP1 x3 (+ yolo_refine_assoc: points,
  ora protetto dal filtro altezza bici 0,6 m che fermava i paletti).
- (02/11 notte) Correzione memoria di classe (classAnchor rispetta track_class_min_height_*), GH3 x3 vs GH2 x3:
  PD 50,8 -> 47,7%, precisione PD 64,0 -> 76,7%, falsi PD/min 148 -> 63, F1 PD 0,566 -> 0,588; moving 84,9/79,3 ->
  87,1/84,7; FP/min tot 270 -> 148 (rep 127-178 vs 216-322); MOTA 39,5 -> 50,1; IDF1 55,0 -> 61,1. TENUTA: tag sins
  `heldout-v3`. Su JRDB non cambia nulla (la classe person non ha altezza minima): le run J2 x3 restano valide.
  GP1 (+ yolo_refine_assoc: points) x3: PD 52,0% prec 65,8%, falsi PD 133/min, FP tot 235: scartata (torna iou).
  JRDB heldout-v2 x3 (J2, J2r2, J2r3): PD rec 37,9/37,2/37,3% prec 81,3/80,4/82,3%, mai trovate ~60%; moving rec
  48,6/48,7/49,0% prec 74-76%; tutte rec ~43% prec ~86%, MOTA 32-33%, IDF1 33%.
- (02/11 notte) JRDB, diagnosi su bytes-cafe (debug; `pd_funnel.py --jrdb`, criterio "visto da YOLO" corretto:
  riquadro YOLO per metà dentro la box proiettata, non IoU: le persone sedute sono viste solo sopra il tavolo).
  KB2 (indoor, riquadri): PD ok 55%, YOLO non vede 19%, foglia accanto 8,5%, classe non passa 7,3%, clustering 3,7%.
  Profilo outdoor intero (KB3): PD rec 59 prec 68 (peggio). Prove 3 rep su bytes: KB2 indoor PD 53,0/83,5% F1 0,648
  FP 1017/min MOTA 45,1 IDF1 44,3; KB4 + foglie da maschere 50,9/90,5% F1 0,651 FP 467 MOTA 48,3 IDF1 47,6;
  KB5 + riquadro di riserva 51,6/93,5% F1 0,665 FP 291 MOTA 50,2 IDF1 47,4 (TENUTA); KB6 + YOLO 768 px 46,9/92,4%
  (scartata); KB7 + gruppo più vicino 52,4/92,4% FP 372 (uguale, scartata). detection_decode_reduced 1 non cambia nulla.
  Bloccata come tag sins `heldout-v4`; lanciate J3 x3 sulle 6 sequenze held-out.
- (03/11 notte) JRDB held-out heldout-v4 (J3 x3) vs heldout-v3 (J2 x3): PD rec 37,5 -> 34,4%, prec 81,3 -> 81,7%,
  F1 0,513 -> 0,484; moving 48,8/74,7 -> 47,9/75,8; MOTA 32,7 -> 30,5, IDF1 33,2 -> 33,4. forbes 33 -> 25%, huangb
  28 -> 22% di persone trovate, altre invariate. Il guadagno su bytes-cafe (prec 84 -> 94%) NON si trasferisce:
  sovra-adattamento a UNA sequenza di debug. Proposta: numeri JRDB = heldout-v3 (J2), v4 riportata come prova negativa;
  per tarare su JRDB serve uno split di tuning di 4-5 sequenze JRDB train non usate (conversione dal NAS, in laboratorio).
- (03/11 sera/notte) Analisi PD su GH3 (= heldout-v3): rifatta con la run di diagnosi K4 (GH3 + topic intermedi: le run
  GH3 non li hanno, e pd_funnel contava "preprocessing <5 punti" 27% = artefatto). K4: PD ok 54,5%, YOLO non vede 15,6%,
  classe non passa (stadio 5) 13,4%, foglia accanto 5%, clustering 4%, tracker 5%. YOLO non vede = 39 righe del cavallo
  (dietro un muro nell'immagine: invisibile alla camera) + bici in rastrelliera a 15-18 m (65 px). Offline (stesso modello
  yolo26s-seg a 1280): su 77 riquadri mancati 18 hanno punteggio >= 0,5, 48 tra 0,3 e 0,5, 11 sotto 0,3. Lo stadio 5
  sono in gran parte le STESSE bici lontane (YOLO intermittente vicino a 0,5): nessun cluster entro 3 m prende la classe.
  Nuovo parametro yolo_seg_track_detector `class_score_thresholds` (una sola inferenza alla soglia più bassa, le altre
  classi scartate sotto score_threshold): cfg/outdoor `["bicycle:0.3"]`.
  T1 x3 vs GH3 x3: PD rec 47,7 -> 60,2% (±1,2), prec PD 76,7 -> 81,7%, falsi PD/min 63 -> 57; MOTA 50,1 -> 53,2;
  IDF1 61,1 -> 62,9; moving rec 87,1 -> 88,1, prec 84,7 -> 81,8 (falsi "dinamico"/min 85 -> 109, soprattutto w2: tracce
  1,3-1,6 m a 0-3 m dal robot, persone non annotate/duplicati; GH3 oscilla già 45-58 su w2). TENUTA (commit sins a78dd21).
  Profilo JRDB indoor riportato a heldout-v3 (la v4 era rimasta attiva nei file). JRDB: J4 x3 solo clark0/clark1 (outdoor,
  unico profilo che cambia; indoor = J2). Baseline su JRDB: Dynablox e M-detector (solo LiDAR, VLP-16 15 Hz,
  eval/baselines/{dynablox,mdetector}/*jrdb*); LV-DOT escluso (JRDB non ha camera di profondità), CenterPoint/EagerMOT
  esclusi (pesi addestrati su CODa OS1-128, non trasferibili al VLP-16 senza riaddestrare). Confronto:
  `python3 eval/coda_compare.py --jrdb`.
- (03/11 notte) JRDB J4 x3 (clark0/clark1, profilo outdoor con bici da 0,3) vs J2: PD rec 50,6 -> 50,9%, prec 49,5 ->
  47,6%, falsi tot +9% (righe 4266 -> 4650, rep sovrapposte; parte bici vere non annotate: JRDB annota solo persone),
  MOTA 34,1 -> 32,4. Tenuta comunque (guadagno grande su CODa, costo piccolo su JRDB). SINS JRDB = J5 (link: indoor J2,
  outdoor J4) = tag sins heldout-v5. Confronto JRDB (eval/results/jrdb/compare.json, tables/jrdb_compare.csv):
  SINS moving rec 49,6% prec 76,2%, PD 37,5%/80,9%, tutte rec 43,1% MOTA 32,5 IDF1 33,3; Dynablox moving 39,3/44,8, tutte
  13,3%, MOTA 7,3, IDF1 7,9; M-detector moving 35,0/57,6, tutte 10,6%, MOTA -1,6. CODa compare aggiornato a T1 (SINS
  moving 88,1/81,8, PD 60,2/81,7, FP/min 166, MOTA 53,2, IDF1 62,9; CenterPoint+SimpleTrack 91,7/63,2 MOTA 29,2;
  EagerMOT 91,1/70,5 MOTA 28,4). Pagina "SINS a confronto" aggiornata (sezione JRDB).
- (03/11) Diagnosi K5 = T1 (heldout-v5) + topic intermedi, CODa: PD ok 54,5% (K4) -> 71,0%; YOLO non vede 15,6 -> 8,2%
  (resta il cavallo dietro il muro, 39 righe); passo 5 (foglia bicycle sul cluster, cluster senza classe) 13,4 -> 8,3%;
  clustering 4 -> 5%. Falsi PD 51 -> 50 righe. Passo 5 restante: ~1/3 cluster sotto 0,6 m (soglia anti-rastrelliera
  voluta), il resto respinto dall'associazione IoU (motivo non registrato: serve log non limitato in yolo_refine).
  Analisi: eval/results/pd_analysis/K5_heldout-v5.txt. Pulizia disco: tolti input tmp LV-DOT/UniMT/Dynablox e run scartate
  (J3, KB*, DBG2, GP1, GR0/1, K3, K4): 17 GB liberi.
- (03/11) Diagnosi K6 (= K5 + nuovo parametro yolo_refine_log_all: ogni scarto di yolo_refine nel log, con leaf_pos;
  default false) e attribuzione per posizione e tempo (pd_funnel scrive <TAG>_stage5.csv). Passo 5 = 72 righe, tutte in w1
  nelle rastrelliere a x -13..-27: ~15% "too low" (bici viste <= 0,6 m nella rastrelliera dietro: soglia voluta),
  ~40% foglia ACCEPTED nello stesso istante ma su un cluster vicino (bici a 0,5 m l'una dall'altra, una foglia -> un
  cluster), il resto foglia centrata altrove (maschera su piu' bici) o cluster gia' preso dall'altra camera. Il ritardo
  camera-LiDAR non c'entra (86-92% delle scansioni con foglia entro 0,08 s). Nessuna correzione: e' un problema di
  rastrelliera densa, nested/points (che lo attaccano) avevano gia' quasi raddoppiato i falsi PD.
- (03/11 pomeriggio) SPLIT DI TUNING JRDB (scelto prima di ogni run, per persone ferme entro 15 m, luoghi diversi dalle
  held-out; eval/datasets/jrdb/convert_jrdb_tune.sh): indoor bytes, gateslab, nvidia, stlc; outdoor tressider1 (16/03_1),
  memorial, hewlett. TU0 x3 (profili attuali): ALL rec 50,5% FP/min 2301 MOTA 22,1% | MOVING rec 47,8% prec 80,7%.
  TU1 x3 (indoor v4: foglie da maschere + riquadro) sulle 4 indoor vs TU0 sulle stesse: ALL rec 46,5 -> 42,8%, prec
  84,4 -> 89,1%, FP/min 1075 -> 656, MOTA 35,0 -> 35,6%, moving rec 45,1 -> 42,9%: nessun guadagno netto, v4 resta fuori
  (conferma il held-out). Baseline sul tuning: Dynablox ALL rec 23,8% MOTA 18,0%, moving 42,4/54,7%; M-detector ALL rec
  23,0% MOTA -4,4%, moving 52,6/67,1% (MOTA moving -25%).
- (03/11) JRDB-PanoTrack: labels_3d_pseudo_panoptic_CW (1 Hz, per punto, upper poi lower, label = inst<<16 | classe;
  verifica: 90-100% dei punti "pedestrian" su persone annotate). eval/datasets/jrdb/jrdb_pano_ignore.py -> bags/jrdb/<seq>
  _pano.csv (bici/monopattini, auto, golf cart, veicoli grandi, carrelli, skateboard, animali; esclusi entro 0,8 m da una
  persona annotata); coda_eval --extra-ignore (0,3 m, +-0,5 s); coda_compare --pano. Oggetti mobili non-persona solo in
  hewlett (21 bici), tressider1 (5), memorial (3), clark1 (3); indoor nessuno. Tuning con pano: SINS MOTA 22,1 -> 29,5%,
  FP/min 2301 -> 1583; held-out invariato (MOTA 32,5%). Residuo su hewlett (TU0): FP su vegetazione 33%, non etichettato
  36%, bici residue 22%, lampioni 9% -> errori veri di SINS all'aperto (bici/persona YOLO su alberi e pali).
- (03/11) CODa: seq 0 (tuning) e' tra le piu' povere (1,7 pedoni/fotogramma). Seq 1 inutile come held-out (2 pedoni +
  statua del cavallo): conversione interrotta e cancellata. RGB-D ZED esiste solo per seq 0, 1, 10, 11, 12 -> HELD-OUT
  CODa = 10, 11, 12 (19/23/21 pedoni, bici, monopattini, veicoli di servizio, 28 auto), solo finestre annotate + 20 s
  (eval/datasets/coda_select/sel_*.txt, download_coda_heldout.sh via remote_zip --names, 30 GB sul NAS imperatore).
- (03/11 sera) Rete: Wi-Fi debole (-71 dBm, 120-440 ms al router) -> download CODa quasi tutto fallito e lettura NAS
  ~0,4 MB/s; passati su Ethernet (LAN 192.168.178.53): ~20 MB/s. Download rifatto (resumable), completo 17:14.
- (03/11 sera) CODa seq 10 con heldout-v5 (H1 x3) e tutte le baseline (coda_compare.py --seq10, compareS10.json):
  SINS ALL rec 47,1% FP/min 1465 MOTA -0,5% | moving rec 49,2% prec 68,4%; Dynablox moving 76,0/26,4 MOTA -140;
  M-detector 12,4/39,5; LV-DOT 3,7/80,7 (quasi nessun dinamico); CenterPoint+SimpleTrack 88,0/46,3 MOTA -14,8;
  EagerMOT 85,9/58,9 MOTA 24,8 (CenterPoint addestrato su CODa: forse seq 10 nel suo training). Due cause:
  (1) dataset: auto parcheggiate non annotate (w1: GT 0,7 auto/frame, CenterPoint 3,9; 45% dei falsi SINS su di esse);
  (2) SINS: funnel pedoni (scratchpad ped_funnel.py) seq 10 vs seq 0: uscita ok 55% vs 94%; cluster mancante 5%;
  cluster senza traccia 10%; TRACCIA ESISTENTE MA "static" 30% vs 3%. Quelle tracce camminano (GT 1-1,5 m/s, KF
  >1 m/s nel 65%, eta' >2 s nel 43%), 83% fuori FOV camera (senza classe -> PATH 3: serve kf_vel_confident o point
  vote, 5 tick consecutivi, gate dimensioni, blocco disocclusione — 38 log/s throttled di "disocclusion artifact" in w4).
  Pedoni a <0,8 m da un altro: 2% trovati (fusione cluster). DECISIONE utente: seq 10 = secondo TUNING CODa,
  held-out = solo 11, 12 (chain_heldout.sh non cancella i bag della 10). EagerMOT: export corretto per topic
  /yolo_seg_track (prima sceglieva tutti i messaggi). LV-DOT: calibrazione per sequenza.
- (03/11 notte) HELD-OUT CODa UFFICIALE (seq 11+12, heldout-v5 congelato, compareH.json): SINS moving rec 46,5% prec
  77,8% MOTA 32,1% (tutte: rec 50,5% FP/min 1245 MOTA 15,6%); CenterPoint+SimpleTrack 90,0/45,0 MOTA -20,6;
  EagerMOT 66,4/57,9 MOTA 17,3; Dynablox 66,7/21,7 MOTA -181; M-detector 22,7/62,8; LV-DOT 3,0/98,0 MOTA 0.
- (03/11 notte) Tuning seq 0 + 10 sul blocco disocclusione (tracker: log diagnostico track_log_motion_block, off).
  D1: 86% dei blocchi su tracce in moto senza classe = disocclusione; occlusore a ~6 m, traccia bloccata a ~17 m.
  A1 track_disocclusion_lateral_margin 0,2 (cono limitato alla sagoma): seq10 moving rec 49 -> 49%, seq0 88 -> 83%: NO.
  A2 track_disocclusion_require_unseen (vista al tick prima = non nascosta): seq10 moving rec 50 -> 62%, MOTA 25 -> 29;
  seq0 prec 82 -> 70%, MOTA 68 -> 49 (pali/tronchi parzialmente coperti 'si muovono'): NO. A3 = A2 + innovazione
  <= 0,3 m: uguale ad A2: NO. Etichettatura dei casi esentati (D5): solo 2-4% su pedoni annotati, nessuna feature
  (dimensioni, altezza, distanza dietro l'occlusore, eta', innovazione) li separa. Profilo resta heldout-v5; i tre
  parametri restano nel codice, default off. Latenza: un confronto per traccia, trascurabile.
- (03-04/11) Perche' SINS trova poche persone su CODa held-out: funnel su 11+12 (H1): output ok 63%, traccia c'e' ma
  'static' 24% (83% si muove >1 m/s, 85% fuori FOV), cluster senza traccia 10%, nessun cluster 3%. Recall per FOV
  camera: dentro 88% (3434 righe), fuori 49% (5928 righe, 63% delle persone). Il collo di bottiglia e' decidere il
  moto senza YOLO fuori dal FOV della sola camera frontale, non vedere le persone.
  Nuovo parametro track_disocclusion_travel_window (+ _min_dist 1,5 m, _min_straightness 0,8; default off): una traccia
  che negli ultimi W s si e' spostata >= min_dist quasi in linea retta non e' bloccata dalla disocclusione.
  B1 (W=2 s) x3: seq10 moving rec 49 -> 56%, MOTA 25 -> 30, ALL MOTA ~0 -> 7; seq0 moving rec 88 -> 82%, prec 82 -> 86,
  MOTA 68 -> 68 (calo concentrato in w2, non spiegato dai log: la regola scatta ~35 volte). B2 (3 s, 2,5 m, 0,9) x3:
  seq10 moving rec 51%, MOTA 27; seq0 MOTA 66: troppo poco.
  B1 come candidato v6 su held-out 11+12 (valutazione unica, criterio fissato prima: moving MOTA >= v5): moving rec
  46,5 -> 50,3%, prec 77,8 -> 74,0%, MOTA 32,1 -> 31,1, FP/min 1245 -> 1425. NON adottato: profilo resta heldout-v5.
- (04/11) Diagnosi tick per tick (D7 = v5 + track_debug_csv, nuovo parametro diagnostico: una riga CSV per traccia per
  tick con stato, flag di moto, dimensioni) sui pedoni in movimento della seq 10 (tuning): trovati 52%; persi per
  disocclusione 23%, traccia appena nata non confermata 9%, traccia in coasting 7%, nessun cluster 5%, altro 4%.
  Script in eval/diagnostics/coda_walkers/ (attrib.py, occl3.py). Dei 39k blocchi di disocclusione solo il 4% cade su
  pedoni; tra quelli con evidenza di moto, sagoma persona (h 1,0-1,9 m, impronta 0,3-0,9 m) + velocita' KF 0,8-2,2 m/s
  => 82% pedoni (contro 11%), copre il 68% dei pedoni bloccati.
  Nuovo parametro track_disocclusion_person_exempt (+ _min/_max_speed/height/footprint; default off). P1 x3:
  seq10 moving rec 49 -> 60%, MOTA 25 -> 31, ALL MOTA ~0 -> 11, FP/min 1465 -> 1390; seq0 moving rec 88 -> 84,
  MOTA 68 -> 67, FP/min 166 -> 158 (calo da un solo episodio in w2: persona a 0-5 m fusa con un'altra in un cluster
  di 3,5 m, la traccia nuova rinasce piu' tardi; D8/D9).
  Held-out 11+12 (seconda valutazione dopo B1, stesso criterio moving MOTA >= v5): moving rec 46,5 -> 50,8%, prec 77,8
  -> 75,2%, MOTA 32,1 -> 32,6; ALL rec 50,5 -> 54,4%, ALL MOTA 15,6 -> 14,3, FP/min 1245 -> 1438. ADOTTATO come
  heldout-v6 (guadagno modesto; il +11 punti di recall in tuning diventa +4 su held-out). Latenza: qualche confronto
  per traccia, trascurabile. Prossime cause: tracce non confermate/coasting (16%, frammentazione vicino al robot).
- (04/11) Regola classi (utente): etichetta PD solo per classi che la GT annota. CODa: track_semantic_dynamic_classes
  senza dog/cat/cow/sheep; profilo coda10 (eval/configs/sins/coda10, scelto da run_coda.sh per le finestre 10_*):
  anche senza car/bus/truck (auto parcheggiate non annotate). C0 = v6 + regola, x3: seq10 moving rec 58%, prec 72%,
  MOTA 34 (v6: 31), ALL MOTA 18 (v6: 11), FP/min 1090 (v6: 1390); seq0 moving prec 86%, MOTA 71 (v6: 67).
  Rotture di traccia (parametri nuovi, default off): track_publish_coasting_ticks 3 (C1) e track_inherit_motion_dist
  1,0 m / _age 1,0 s (C2), insieme C3 x3: seq10 moving rec 66% ma MOTA 30, seq0 MOTA 51: NO. C1 (1 rip.): seq0 MOTA 62;
  C2 (1 rip.): seq10 MOTA 36, seq0 MOTA 63: NO.
  Veicoli in movimento: il gate dimensioni blocca ogni oggetto non classificato piu' grande di una persona. Righe GT in
  movimento: seq0 ~66 veicoli, seq10 ~43, seq11 ~785, seq12 ~2970 (quasi meta' del moto in seq12). Nuovo
  track_size_gate_travel_window (+ _min_dist 2 m al netto dello spostamento del sensore, rettilineita' 0,7+0,05*impronta
  <= 0,95; default off). V1 x3: seq10 moving MOTA 31,5 (C0 33,6), seq0 67,0 (C0 70,8), FP seq0 +16%: i grandi oggetti
  dinamici sono pochi (~60 righe/rip. su seq10, alcuni auto vere non annotate); il beneficio non e' misurabile sulle
  sequenze di tuning (poche auto in moto). NON adottato finche' non c'e' una sequenza di tuning con traffico.
  Profilo corrente: C0 (heldout-v6 + regola classi), non ancora valutato sull'held-out.
- (04/11 pomeriggio) ERRORE NELLA GT CODa: la lista classi di convert_coda0.sh / convert_coda_heldout.sh aveva "Truck",
  classe che CODa non ha; Pickup Truck, Delivery Truck, Utility Vehicle, Bus e Cart annotati restavano fuori dalla GT
  (contati come falsi). Seq 10 (finestre valutate): +1170 box (Pickup 499, Delivery 330, Utility 341); seq 11: +2880
  (Service Vehicle era gia' dentro); seq 12: +216; seq 0: nessuna. Corretto (lista classi nei due script), GT rigenerata
  con coda_to_bag.py --gt-only (n_color/n_depth del json conservati), vecchie GT in bags/coda/gt_v1/, vecchi risultati
  in eval/results/coda/gt_v1/. Conseguenza: le "auto parcheggiate non annotate" della seq 10 SONO annotate (10_w1: GT
  0,7 -> ~4,2 veicoli/fotogramma, CenterPoint 3,9): la premessa del profilo coda10 era sbagliata (C0 su seq 10 con GT
  nuova: PD F1 ~1%, moving rec 54% prec 73% MOTA 31,5 media 3 rip.). Da rifare: C0 su seq 10 con profilo coda.
  Held-out ricalcolato (nessuna run nuova): SINS v6 moving rec 48,8% prec 76,1% MOTA 32,1, ALL MOTA 18,0, FP/min 1285;
  v5 44,2/78,0/30,7, ALL 17,8; CenterPoint+ST ALL MOTA 23,9 (era 11,0), moving -8,4; EagerMOT ALL 26,6, moving 17,3.
- (04/11 pomeriggio) Sequenza di tuning con veicoli: sondaggio su CODa (annotazioni ogni 50 fotogrammi, pose dense):
  veicoli in moto rari ovunque; migliore seq 16 (16 coppie a 5 s con veicoli in moto, 830 di pedoni), senza ZED
  (solo stereo cam0/cam1 -> run_coda.sh usa stereo_camera come unica camera se n_color = 0). Finestre 2900-3799,
  8450-9249, 10320-11559, 14700-15849 (eval/datasets/coda_select/sel_16.txt, download sul NAS in corso, ~23 GB).
- (04/11 pomeriggio) Marina di Pisa "street" (14/10, 18 min, robot FERMO sul marciapiede, auto e pedoni che passano):
  estratta in bags/mdp/mdp_street (extract_topics.py --image-hz 10 --min-free-gb, camere a 10 Hz per lo spazio),
  odometria KISS-ICP bags/mdp/mdp_street_odom (spostamento nullo). Camera posteriore puntata a terra: solo frontale.
  Riferimento automatico degli oggetti IN MOVIMENTO (eval/datasets/static_sensor_movers.py: sfondo = voxel occupati
  >= 10% delle scansioni in 180 s, cluster, tracking offline, spostamento netto >= 2 m): 201 oggetti, 45 veicoli
  (lunghezza mediana 3,8 m, ~10,6 m dal robot = la strada) e 156 persone/bici; campione di 8 controllato sulle
  immagini, tutti veri. Profili eval/configs/sins/mdp_street (base outdoor, sola camera frontale, depth compressedDepth
  senza nuvola nativa, YOLO stride 1) e mdp_street_v1 (+ V1). make_cfg.py: ora sostituisce anche un blocco annidato.
  Risultati street (media 3 rip., coda_eval dynamic_only entro 15 m, riferimento automatico sopra):
    A (outdoor)  rec 37,9% prec 82,3% MOTA 28,8 | rec veicoli 5,7%  persone 43,3%
    V1           rec 43,9% prec 80,8% MOTA 32,4 | rec veicoli 7,9%  persone 49,9%
    V2           rec 46,2% prec 80,9% MOTA 34,2 | rec veicoli 34,9% persone 48,2%
  (variazione tra rip. < 1 punto). Diagnosi V1: veicoli clusterizzati al 94% ma tracciati al 20%: gate di associazione
  tarati sulle persone (8 m/s, innovazione 0,6 m, direzione 1,2 rad) scartano le auto a 5-14 m/s. V2 = V1 + gate per
  oggetti grandi in tracker_node (track_large_object_footprint, -1 = spento; >= 2 m su traccia E rilevamento:
  velocita' 20 m/s, direzione 15 rad = di fatto libera, innovazione 2 m). Latenza scansione->tracce invariata (mediana
  98,0 vs 98,1 ms, p95 114,6 entrambe). Persone -1,7 punti, ripetibile: da capire (gruppi >= 2 m con gate larghi?).
  Controllo visivo del riferimento su 12 fotogrammi proiettati sulla camera (scratchpad gt_overlay.py): veicoli 6/6
  giusti, persone quasi tutte, 1 errore (pezzo d'auto etichettato persona). Il riferimento usa lo stesso lidar di SINS:
  valido per confronti A/B, non per numeri assoluti nel paper.
  Bag sensori spostata sul NAS (datasets_SINS/derived/mdp_street, md5 verificato) per far posto alla seq 16.
- (04/11 sera) run_coda.sh: tolto l'instradamento delle finestre 10_* sul profilo coda10 (premessa sbagliata, vedi GT
  sopra); il profilo resta in eval/configs ma non e' usato. convert_coda_heldout.sh: ONLY=wK converte una sola finestra.
- (05/11) TRACKING/MOTO SENZA SOGLIE PER TIPO (richiesta utente: meno hardcoded). Regole nuove in tracker_node (tutte
  default off, sins c807344): W1 track_two_point_init (velocita' KF dalle prime due osservazioni: prima tutto cio' che
  andava > ~3 m/s rinasceva a ogni scansione), W2 track_size_gate_body_lengths 1.0 (size gate passato dopo aver
  percorso la propria lunghezza), N track_motion_min_net_footprints 0.5 (moto solo dopo che il centro ha lasciato il
  punto di nascita di max(0,3 m, mezzo ingombro)), C track_motion_consistency_window 1 s / ratio 0.5 (velocita' KF
  confermata dallo spostamento osservato), J track_jitter_footprint_scale 1.0 (gate di associazione meno mezzo
  ingombro della traccia), M track_fragment_merge_margin 0.3 (pezzi dentro l'ingombro previsto di una traccia in moto
  fusi prima dell'associazione). Profili coda_w1/w/w1n/wn/wnc/wncjm, mdp_street_idem. W NON usa i gate V2 per oggetti grandi.
  Street (media 3 rip., dynamic_only 15 m; rec / prec / MOTA | rec veicoli, persone):
    V2 46,2/80,9/34,2 | 34,9 48,2;  W1 53,9/82,9/41,6 | 32,2 57,6;  W 58,7/82,3/44,8 | 39,9 61,9
    (solo rip. 2-3:) WN 57,5/82,7/44,3 | 36,0 61,2;  W1N 51,8/83,4/40,4 | 27,5 56,0. Latenza scan->tracce 98,3 / p95 115 ms.
  CODa tuning, moving only, media 3 rip. (rec/prec/MOTA), K0 = profilo coda attuale con GT nuova:
    seq 0:  K0 85,6/85,5/70,8  W1 86,5/74,7/57,0  W 85,0/68,9/46,3  W1N 84,5/80,1/63,3  WN 83,9/77,9/59,9
    seq 10: K0 54,0/70,4/29,4  W1 61,2/68,5/30,6  W 60,8/66,6/27,2  W1N 59,5/69,9/31,6  WN (r1-2) 60,2/68,5/29,2
    seq 16: K0 62,7/93,6/57,3  W1 67,9/93,5/61,6  W 73,9/89,9/62,7  W1N 67,1/94,0/61,5  WN 73,6/91,2/63,9
  Seq 0: FP = frammenti fermi (centro sposta < 0,5 m, KF 0,5-0,8 m/s) dichiarati dinamici: l'init a due punti da' loro
  velocita'; N dimezza l'aumento (FP righe 59 K0 -> 208 W1 -> 138 W1N). C pensata per il resto (non ancora provata).
  Auto street (scratchpad -> eval/scripts/analysis/car_clusters.py, car_gaps.py): 33% dei fotogrammi auto in >= 2
  cluster; salto del centro > 0,6 m nel 24% delle scansioni (gate persona 0,6/1,0 m); tra i pezzi distacco tipico 0,45 m
  (5-10 m) e 0,80 m (10-15 m) contro eps 0,33/0,40: eps spiega ~1/3-1/2, il resto sono buchi veri -> J + M. Eps far
  0,45 -> ~0,7 m nel profilo street ancora da provare.
  Run perse per errori DDS UDP (meta' scansioni): street wn/w1n rip. 1, CODa KWNr3 10_w4/10_w5 -> da rifare.
  PASSAGGIO AL PC DEL LABORATORIO (05/11 sera): esperimenti interrotti; in coda eval/scripts/analysis/lab_resume.sh
  (build, KWNr3 10_w4/w5, CODa KWNC/KWNCJM x3, street wnc/wncjm x3 + wn/w1n rip. 1, punteggi). Dati sul NAS
  datasets_SINS/derived/{coda_windows,mdp_street_work}; setup docker/setup_lab_pc.sh (DDS solo localhost).
  Prossimo: scegliere tra WN, WNC, WNCJM (CODa 0/10/16 + street), poi held-out 11-12 col profilo congelato.
- 06/11 PC DEL LABORATORIO (i7-13650HX, RTX 4060 Laptop, 31 GB RAM, disco ~10-20 GB liberi, PC condiviso):
  WN / WNC / WNCJM x3 rifatti tutti qui (tag LWN/LWNC/LWNCJM [r2|r3], street in eval/results/mdp_lab/), script
  locale eval/scripts/analysis/lab_local.sh (non committato). lab_resume.sh fermato dopo KWNr3: replay dal NAS (CIFS,
  1 Gb/s = 117 MB/s contro ~87 MB/s chiesti da una finestra CODa+stereo) perdeva scansioni (10_w4: 267/400), quindi
  ogni finestra viene copiata in locale, girata 9 volte e cancellata. KWNr3_10_w4/w5 NON validi.
  Latenza CODa su questo PC ~190 ms mediana (p90 340-400) contro 98 sul portatile: il ritardo nasce in dbscan_detector
  (detections3d gia' a ~184 ms, YOLO 35-59 ms) e si accumula nelle parti affollate -> qui non tiene i 10 Hz su CODa
  (profilo energetico "performance" durante le run). Street: 96 ms / p90 109, come il portatile.
  Riproducibilita': LWN vs WN del portatile seq 0 60,1 vs 59,9; seq 10 33,2 vs 29,2; seq 16 70,0 vs 63,9; street 43,2 vs 44,3.
  CODa moving only, media 3 rip. (rec/prec/MOTA, pooled per sequenza) [min-max MOTA]:
    seq 0:  WN 83,3/78,4/60,1 [57-64]  WNC 84,8/83,4/67,8 [66-71]  WNCJM 48,8/74,5/34,7 [6-52]
    seq 10: WN 63,9/68,5/33,2 [31-35]  WNC 61,2/69,8/34,0 [34-34]  WNCJM 46,3/66,2/22,1 [22-22]
    seq 16: WN 74,3/95,1/70,0 [69-71]  WNC 72,8/95,3/68,6 [68-70]  WNCJM 58,2/94,3/54,2 [51-56]
  Street 15 m (rec/prec/MOTA | rec veicoli, persone): WN 57,0/81,9/43,2 | 35,3 60,7;  WNC 54,5/83,1/42,5 | 32,0 58,3;
    WNCJM 55,0/84,7/44,3 | 38,4 57,9.
  WNCJM su CODa e' instabile e peggiore (seq 0 w1 r3 recall 0, w2 recall 29-46): J/M rompono i pedoni -> scartare o
  limitare J/M ai veicoli. WNC: +7,7 MOTA su seq 0 (meno FP), pari su 10, -1,4 su 16, -0,7 street.
  LWN w3 r1 aveva 176/400 tracce: rifatta (400/400), la vecchia e' LWN_w3_scartata_176su400.json.
  Prossimo: decidere WN vs WNC (o C solo CODa), J/M solo per veicoli sulla street; poi held-out 11-12.
- 06/11 PORTATILE, WNC vs WNCE (= WNC + lidar_eps_far 0,45 -> 0,7), CODa 0/10/16 x3 (tag KWNC/KWNCE [r2|r3]),
  script eval/scripts/analysis/chain_e.sh. Street interrotta (l'utente porta via il portatile): solo mdp_street_wnc_r1.
  Decisione 06/11: WNC profilo di riferimento, J/M scartate (niente regole per tipo di cluster).
  CODa moving only, media 3 rip. (rec/prec/MOTA) [min-max MOTA]:
    seq 0:  WNC 84,0/81,9/65,1 [61-68]  WNCE 78,4/81,8/60,2 [57-62]   (WN portatile 83,9/77,9/59,9)
    seq 10: WNC 53,6/69,8/28,3 [26-30]  WNCE 53,3/70,6/29,1 [26-31]
    seq 16: WNC 68,1/93,6/61,8 [61-63]  WNCE 61,6/93,1/55,2 [50-60]
  WNC conferma sul portatile il guadagno su seq 0 (+5,2 MOTA vs WN). WNCE peggiore (-4,9 seq 0, -6,6 seq 16) e
  rallenta DBSCAN: latenza tracce mediana CODa WNC ~200 ms (p90 400-1050), WNCE 377-575 ms (p90 1100-1700), con
  scansioni perse (w1 497 vs 553). Nota: anche il portatile su CODa (128 linee + stereo) sta a ~200 ms, non 98:
  i 98 ms (p90 112) valgono per la street (VLP-16). Il PC del laboratorio quindi su CODa e' come il portatile.
  Prossimo: confermare con LWNCE (lab, CODa + street), poi held-out 11-12 col profilo congelato (WNC salvo sorprese).
- 07/11 PC DEL LABORATORIO, LWNCE + held-out 11/12 (tag LWNCE, LHK = K0 coda, LHW = coda_wnc congelato, stereo, x3).
  WNCE vs WNC, CODa moving only (rec/prec/MOTA) [min-max MOTA]:
    seq 0:  WNC 84,8/83,4/67,8 [66-71]  WNCE 81,8/79,8/60,7 [58-65]
    seq 10: WNC 61,2/69,8/34,0 [34-34]  WNCE 60,4/69,0/29,9 [27-31]
    seq 16: WNC 72,8/95,3/68,6 [68-70]  WNCE 65,2/91,6/57,8 [57-58]
  Street 15 m: WNC 54,5/83,1/42,5 | veicoli 32,0 persone 58,3;  WNCE 54,9/84,2/43,7 | veicoli 37,3 persone 57,9.
  Latenza tracce mediana/p90: CODa WNC 194/355, WNCE 174/424 ms; street 96/109 vs 95/108.
  DECISIONE: WNCE peggiore su tutte e 3 le sequenze CODa (-4 / -7 / -11 MOTA), aiuta solo i veicoli della street
  (+1,2 MOTA, +5 recall veicoli) -> profilo congelato = WNC (coda_wnc / mdp_street_wnc). Conferma il portatile.
  HELD-OUT (nessun parametro toccato guardando 11/12), moving only (rec/prec/MOTA) [min-max MOTA]:
    seq 11: K0 60,9/77,9/41,8 [38-46]  WNC 61,2/79,5/43,5 [43-44]
    seq 12: K0 36,7/71,1/19,8 [19-22]  WNC 38,8/68,9/19,6 [19-20]
  Latenza tracce mediana/p90: seq 11 K0 169/448, WNC 169/388; seq 12 K0 222/501, WNC 225/523 ms.
  Lettura: sulle sequenze mai viste WNC e' pari o appena meglio di K0 (+1,7 MOTA su 11, pari su 12) e piu' stabile
  tra le ripetizioni; il grosso guadagno di seq 0 (+7,7) non si generalizza -> WNC non e' overfitting dannoso ma
  nemmeno un salto. Seq 12 e' difficile per entrambi (recall ~37-39%). Finestre convertite salvate su NAS
  derived/coda_windows (coda11_w1-3, coda12_w1-3). Profilo energetico tornato su balanced.
  Prossimo: fasi B/C/D dei baseline (FIDELITY.md).
- 09/11 PC DEL LABORATORIO, fase D CODa: Dynablox e M-detector con le condizioni corrette dall'audit (FIDELITY.md) su
  tutte le 15 finestre (tuning 0/10/16, held-out 11/12), 1 ripetizione (deterministici), script
  eval/scripts/analysis/lab_baselines_coda.sh, risultati eval/results/coda/cmpF{a,t}_<metodo>_<finestra>.json
  (a = tutti i fotogrammi annotati, t = solo fotogrammi 'testing' di CODa), bag su NAS derived/baselines_out.
  Dynablox: max_range 20 (paper). M-detector: coda_kitti.yaml (soglie KITTI, sensore CODa), 0,25x dichiarato.
  Moving only, pooled per sequenza (rec/prec/MOTA):
    seq:          0                 10                16               11                12
    SINS WNC      84,8/83,4/67,8    61,2/69,8/34,0    72,8/95,3/68,6   61,2/79,5/43,5    38,8/68,9/19,6
    Dynablox 20m  87,5/12,3/-542    70,1/29,7/-100    66,1/74,6/40,6   68,5/48,6/-8,6    59,1/18,7/-207
    (vecchio 25m  87,4/8,5/-852     76,1/28,5/-120    -                71,3/40,3/-39,3   60,1/15,7/-272)
    M-det KITTI   79,7/8,9/-811     71,4/40,5/-93     72,7/65,2/-37    69,0/37,2/-108    54,1/29,2/-121
    (vecchio nuScenes 0,25x seq 0: 79,9/17,5/-375; 1x: recall 10-30%, perde scansioni)
  Solo fotogrammi 'testing' (~15%): stesso quadro (Dynablox seq 11 MOTA 0,7, seq 12 -201; M-det -100 / -906 su seq 0).
  Lettura: con le condizioni del paper Dynablox migliora ovunque (meno falsi allarmi a 20 m). M-detector con la
  base KITTI (scelta prima dei risultati perché il sensore è il più vicino) ha il doppio dei falsi allarmi della base
  nuScenes su seq 0: si tiene KITTI come configurazione "fedele" e si riporta nuScenes come variante. In ogni caso
  entrambi i baseline trovano molti pedoni (recall 54-88%) ma con precisione 9-75%: SINS resta l'unico con MOTA
  positiva ovunque. Incidente: alle 00:17 il master ROS del container è morto e la catena è rimasta appesa 10 h;
  aggiunti attesa del master, rimozione container rimasto e watchdog di 30 min (d8fa9b1).
  Fase B: metrica M-detector riprodotta dalle uscite degli autori (0,754 vs 0,746); le etichette sono KITTI tracking
  (non SemanticKITTI), servono le scansioni tracking o le bag degli autori. DOALS in download dal portatile;
  pronto eval/baselines/dynablox/doals/ (immagine dynablox-noetic:doals + run_doals.sh, obiettivo 86,0 entro 20 m).
- 09/11 PC DEL LABORATORIO, A/B latenza fusione (profili coda_fz0/mdp_street_fz0: fusion_timeout_sec 0,001 +
  stamp_tolerance 0,15; coda_fz30/mdp_street_fz30: timeout 0,03; base WNC = 0,12). CODa 0/10/16 (9 finestre) x3,
  strada x3, controllo LWNCP = WNC rifatto oggi x1. Finestre 10_w1/10_w4/10_w5 rifatte (latenza falsata dal batch IoU
  in parallelo; vecchi json *_skewed). Moving only rec/prec/MOTA medi su 3 rip. [MOTA min-max]:
    profilo   seq 0                    seq 10                   seq 16                   PD trov./prec  lat. med/p90
    LWNCP x1  85,5/84,6/69,8           61,9/68,3/30,7           70,6/93,4/64,5           24,9/42,8      173/375 ms
    FZ30 x3   86,1/83,5/68,8 [68-70]   59,4/68,8/29,9 [29-31]   70,4/93,7/64,3 [62-68]   20,5/41,1      143/350 ms
    FZ0 x3    86,1/79,6/63,7 [62-66]   60,0/67,8/28,2 [27-29]   69,5/91,0/60,1 [59-61]   26,1/40,3      103/280 ms
  PD per ripetizione: FZ30 10,4 / 24,4 / 26,6% (una ripetizione crolla), FZ0 26,5 / 25,7 / 26,2%.
  Strada (tot rec/prec/MOTA; recall veicoli; recall persone; latenza med/p90):
    WNC  54,5/83,1/42,5 [40,6-44,0]; 32,0; 58,3; 96/109 ms
    FZ0  54,7/83,3/42,7 [41,2-44,0]; 32,2; 58,5;  6/19 ms
    FZ30 54,2/82,8/42,0 [41,7-42,2]; 30,3; 58,3; 37/50 ms
  Tempi per stadio (mediane da det.log, det_timing.py): pre / dbscan LiDAR / attesa camere / camere per tick:
  CODa LWNCP 23,4/19,6/70,0/2,7; FZ30 24,0/20,3/38,8/1,8; FZ0 23,9/20,2/4,9/3,1. Strada WNC attesa 94,9, FZ0 10,1,
  FZ30 39,6 (camere per tick ~0: sulla strada la fusione arriva quasi sempre dopo).
  DECISIONE (criterio utente: latenza giu', MOTA moving non peggio di ~1 punto su ogni seq e sulla strada, PD CODa non
  peggiore): NESSUN PROFILO VINCE, resta WNC (fusion_timeout_sec 0,12).
  - FZ0 dimezza la latenza mediana su CODa (173 -> 103 ms) ma perde 6,1 / 2,5 / 4,4 punti di MOTA su seq 0/10/16:
    con timeout ~0 la box camera del tick arriva al tick dopo e il tracker vede classi/box in ritardo.
  - FZ30 resta dentro ~1 punto (-1,0 / -0,8 / -0,2, strada -0,5) e toglie 30 ms di mediana, ma la PD peggiora
    (20,5 vs 24,9%, con una ripetizione a 10,4%) e la recall veicoli sulla strada scende 32,0 -> 30,3.
  Sulla strada FZ0 e' pari a WNC (42,7 vs 42,5) con latenza 6 ms: la strada non usa davvero le camere in tempo.
  SCOPERTA durante l'A/B: le run LWNC del 06/11 (riferimento SINS CODa 0/10/16 nella tabella fase D) hanno girato
  SENZA camere in fusione (camere per tick 0,0 in tutti i 27 log, pre 45 ms invece di 24, PD trovati 0-5%): quel
  giorno la macchina era piu' lenta e nessuna immagine arrivava entro 0,12 s. Anche le LWN del 05/11 (0,0). Le LWNCE
  (07/11), le held-out LHW/LHK e le LWNCP di oggi hanno le camere (2,6-2,8 per tick).
  Conseguenze: (1) i numeri SINS su 0/10/16 della fase D sono in pratica "SINS senza camere" (MOTA 67,8/34,0/68,6);
  con le camere (LWNCP x1) 69,8/30,7/64,5 e PD 24,9%. (2) Il confronto WNC vs WNCE (07/11) era sbilanciato (WNC
  senza camere, WNCE con): ricontrollato con LWNCP, WNC resta avanti su tutte e tre le seq (69,8/30,7/64,5 vs
  60,7/29,9/57,8), la decisione WNC regge. (3) Da rifare: LWNCP r2/r3 sulle 9 finestre (dopo la coda di oggi), poi la
  tabella fase D e l'IoU SINS su 0/10/16 vanno presi da LWNCP, non da LWNC.
- 09/11 sera, fase B Dynablox RIPRODOTTO su DOALS (8 sequenze, condizioni upstream, evaluate_data.py degli autori):
  IoU di oggetto entro 20 m 85,7 ± 9,1% vs 86,0% del paper (Station 86,7/87,3, Shopville 87,5/87,8, Hauptgebaeude
  85,7/86,0, Niederdorf 83,1/83,1). Dettagli in FIDELITY.md. Il primo lancio era fallito perché la launch upstream
  usa $(env USER) e il container non lo imposta (corretto in run_doals.sh).
  Nota: su CODa lo stesso Dynablox (20 m) ha precisione moving 12-75%: il divario non viene dalla build ma dal dominio
  (campus affollato, pedoni lenti e fermi, sensore 128 canali).
  Coda per la notte (PC libero 23:00-24:00 per altri utenti, guardia lab_gate.sh + controllo carico a mezzanotte):
  IoU CODa -> sins 5d0c8ea (camera_info frame fallback, cluster prima delle detections3d) LFX tuning x3, LHF held-out x3,
  IoU, strada x3 -> varianti LB1, LV, LJ, LA (x3 + strada) -> LiDAR-only LLF x3 + IoU.
- 09/11 sera, SINS rifatto sul codice corretto (sins 5d0c8ea: frame camera da camera_info senza calibrazione online;
  cluster pubblicati prima delle detections3d), profilo coda (= WNC), x3, PC del laboratorio. QUESTI sono i numeri SINS
  da usare per fase D e held-out (sostituiscono LWNC/LWNCP/LHW). Moving rec/prec/MOTA [min-max]; MOTA tutti; PD trovati/prec:
    LFX  seq 0  85,2/81,5/65,7 [64,9-66,6]; tutti 51,9    (LWNCP x1 vecchio: 85,5/84,6/69,8; tutti 51,8)
         seq 10 61,9/67,8/29,8 [29,2-30,4]; tutti 15,5    (30,7; 13,2)
         seq 16 72,2/92,5/64,8 [62,9-66,1]; tutti 50,3    (64,5; 47,7)
         PD 28,5% prec 46,8% (LWNCP 24,9/42,8)
    LHF  seq 11 62,5/79,1/43,9 [41,2-47,9]; tutti 30,8    (LHW vecchio x3: 43,5; 28,9)
         seq 12 40,5/70,2/21,8 [21,4-22,5]; tutti 17,9    (19,6; 12,7)
         PD 47,7% prec 36,0% (LHW 45,3/32,7)
    Strada mdp_street x3: 56,2/83,1/43,8 [43,4-44,1], veicoli recall 32,9, persone 60,1/76,2/40,2
         (WNC vecchio 42,5 [40,6-44,0], veicoli 32,0).
  Lettura: il fix migliora MOTA "tutti" ovunque (+2 / +2,6 / +1,9 / +5,2 su 10/16/11/12) e la PD; MOTA moving pari o
  meglio tranne seq 0 (65,7 vs 69,8 della sola ripetizione LWNCP: dentro il rumore di seq 0, ±4 sul portatile).
  Latenza tracce mediana/p90: LFX 179/329 ms; LHF 258/669 ms (pre 27,9 vs 24 ms: probabile carico di altri utenti sul PC
  in quel momento, da ricontrollare); strada 94/107 ms.
  IoU puntuale per etichetta (eval/results/coda_iou/, coda_point_iou.py; SINS = tutte le box pubblicate, media 3 rip.;
  baseline 1 rip.). moving / moving pedoni / moving veicoli / pedoni fermi / veicoli fermi / tutti, in %:
    seq 0   SINS 82,5/82,6/0,0/-/23,8/70,1      Dynablox 31,6/31,7/0,0/-/0,3/29,1    M-det 43,0/43,0/0,0/-/0,2/39,4
    seq 10  SINS 41,4/61,8/4,2/0,8/10,8/31,4    Dynablox 29,8/21,5/13,5/0,2/0,0/25,8 M-det 51,0/42,7/26,2/0,1/0,7/40,1
    seq 16  SINS 83,0/83,9/2,0/0,0/-/71,9       Dynablox 85,4/85,6/18,1/0,0/-/59,1   M-det 67,8/67,6/2,5/0,0/-/51,0
    seq 11  SINS 70,2/76,7/13,4/27,6/26,0/42,7  Dynablox 78,5/82,4/22,2/15,6/7,9/35,4 M-det 40,4/39,9/4,3/3,1/1,2/23,9
    seq 12  SINS 56,6/67,6/17,9/22,0/5,4/51,7   Dynablox 49,6/44,4/19,6/4,3/4,6/48,9 M-det 53,2/48,5/21,0/3,1/0,3/49,5
  Lettura: SINS ha l'IoU "tutti" più alta su tutte le seq e domina sui pedoni (in moto e fermi); sui VEICOLI IN MOTO è
  peggiore dei baseline (2-18% vs 13-26%), e su seq 10 (molti veicoli) M-detector lo supera anche su "moving": conferma
  la richiesta dell'utente di migliorare il tracking dei veicoli.
- 09/11 PORTATILE, STUDIO D/PD + DUE BUG (sins 5d0c8ea, root 7ef067c). Strumenti: eval/diagnostics/dpd_study/{funnel,pm_sep,cam_funnel}.py
  (profilo diagnostico coda_dbg / mdp_street_dbg: CSV per traccia per tick con i flag delle regole).
  BUG 1 tracker: senza calibrazione online (CODa, JRDB) il frame <camera>_refined non esiste -> nessuna camera usabile nel
  tracker (tutto "fuori FOV", niente decay in FOV, niente fallback pixel LiDAR-only). Ora frame del camera_info.
  BUG 2 dbscan: fused/clusters pubblicato DOPO le box -> voto punti v1 sui punti del tick precedente (44% "moto" a ogni velocita').
  Tutti i numeri SINS CODa/JRDB precedenti (tuning, held-out, fase D, LiDAR-only) sono col bug: rifatti dal lab (LFX/LHF/LLF).
  CODa 0/10/16 laptop 1 rip., moving MOTA prima->dopo 67,9->70,6 / 29,0->31,8 / 63,0->69,7; tutti 38,2->49,3 / 12,0->17,0 / 47,3->53,8.
  Strada MOTA 41,7->43,3. NB rumore 1 rip. alto (lab LFX 0_w2: 42,3/54,4/46,0).
  Imbuto dopo i fix (CODa): D ok 68,7% (non confermata 6,7, coasting con cluster 4,9, nessun cluster 4,5, disocclusione 3,0, C 1,9);
  PD ok 25,7% (traccia ferma senza classe 22,8, non confermata 19,2, coasting con cluster 13,6). Strada veicoli: non confermata 31%
  (45% fuori FOV), persone: disocclusione 9,4%.
  Ramo camera (cam_funnel): YOLO vede il 23% (ZED) / 61% (stereo) dei pedoni nell'immagine; 10_w2 YOLO vuoto nel 46-50% dei frame;
  primi ~2 s di ogni finestra senza YOLO (warm-up); foglia profondita' porta la classe al cluster solo 42% (ZED) / 9-24% (stereo),
  la classe della traccia arriva soprattutto dal fallback pixel (ora funzionante); PD giusta quando YOLO manca: 6-8%.
  Latenza YOLO frame->uscita: CODa 33-54 ms (p90 58-76), strada 190 ms (le maschere arrivano sempre dopo la fusione).
  Misura moto punto-superficie (>=0,5 s, odom): AUC 0,95 vs KF 0,92 vs voto v1 0,53. Varianti 1 rip. (moving MOTA 0/10/16 | PD 10/16 | strada MOTA, rec veicoli):
    DBF 70,6/31,8/69,7 | 13/18 | 43,3, 31,1   P1 (evidenza OR) 60,6/30,0/63,0: NO   KV veto 71,7/32,8/69,4 | 14/18 | 42,2, 29,9
    KB1 decay su frame YOLO nuovi 64,6/30,8/64,0 | 20/23 | -   KA gate fuori FOV 8 m/s 62,1/32,0/64,3 | 8/23 | 45,1, 34,4
    KJ tolleranza ingombro 61,1/29,9/71,5 | 18/18 | 46,4, 46,9 (persone prec 76,5->73,0).
  Lab stanotte (dalle 00:00, slot 23-24 libero): LB1, LV, LJ, LA x3 + strade x3, poi LLF (LiDAR-only col fix).
- 09/11 sera, DIAGNOSI VEICOLI IN MOTO su CODa (richiesta utente; LFX/LHF, codice corretto). Recall veicoli moving
  (GT solo veicoli, eval/results/coda_vehicle/): SINS ~6% in totale vs Dynablox/M-det ~40% (10_w4 16 vs 77/70; 16_w1 6 vs
  51/55; 12_w2 7 vs 41/41; 12_w3 0,2 vs 43/35; 10_w5 53 vs 9/41). I cluster ci sono (82-96% entro 1 m, ma 2-4 pezzi
  per veicolo). Dove si perde (eval/scripts/analysis/coda_vehicle_where.py, righe GT moving):
    10_w4 (camion 5,5 m, 2 m/s): traccia interna vicina ma stato "static", non pubblicata 51-59%; pubblicata ma
      centro 1-2,5 m fuori (box di un pezzo) 14-16%; nessuna traccia 9-13%.
    16_w1 (auto 5,7 m/s): nessuna traccia entro 1 m 59-81% (associazione/inseguimento a velocità alta).
    held-out 11/12: nessuna traccia 51-82%, interna "static" 17-40%.
  Due cause, entrambe da testare con fix fisici commutabili: (1) la conferma del moto non scatta per veicoli lunghi
  e frammentati (regola N: uscire dal punto di nascita di 0,5 ingombri = ~2,5 m per un camion; centro del pezzo che
  salta) -> varianti LJ/LA di stanotte ne coprono una parte; (2) i veicoli veloci non vengono agganciati (gate).

## 10/11 notte, lab PC: varianti x3 sul codice corretto (sins 5d0c8ea), CODa tuning 0/10/16 + strada

Riferimento LFX (profilo coda) / mdp_street. Moving MOTA 0/10/16 | MOTA tutti 0/10/16 | PD trovati/prec | strada MOTA/recall veicoli | recall veicoli in moto CODa tuning (GT solo veicoli, dynamic_only) | latenza tracce med/p90 ms.

| tag | moving MOTA | MOTA tutti | PD | strada | veic. CODa | latenza |
|---|---|---|---|---|---|---|
| LFX | 65.7/29.8/64.8 | 51.9/15.5/50.3 | 28.5/46.8 | 43.8/32.9 | 20.8 | 179/329 |
| LB1 (b1) | 65.6/31.0/64.0 | 44.3/12.6/49.9 | 28.3/42.0 | 43.4/31.2 | 18.3 | 174/300 |
| LV (v) | 74.3/33.3/68.2 | 50.5/16.6/52.0 | 27.3/44.2 | 44.3/31.5 | 18.3 | 153/242 |
| LJ (j) | 65.4/28.9/65.9 | 46.1/12.8/51.9 | 29.0/43.1 | 45.5/47.1 | 20.4 | 178/380 |
| LA (a) | 62.7/33.4/65.9 | 44.3/16.7/50.9 | 28.4/42.8 | 45.0/35.5 | 14.0 | 145/179 |

- LV: unica che migliora la MOTA moving su tutte e 3 le sequenze (+8.6/+3.5/+3.4), MOTA tutti alla pari, latenza più bassa.
- LJ: migliore in strada sui veicoli (recall +14), ma neutra o peggio su CODa e persone in strada −2.5.
- B1 e A abbassano la MOTA tutti su seq 0 (~−7.6) e la precisione PD. A ha la latenza migliore.
- Nessuna variante risolve i veicoli in moto su CODa (14-21%).
- Ripartenza 00:00 dopo lo slot 23-24: check CPU 5%, load 1.8, GPU libera (bags/lab_watchdog.txt).
- In coda: LLF (LiDAR-only, codice corretto), poi lavoro latenza LFS/LL3 + mdp_street_lt2 (root 429c390, sins 7e0ce44).

## 10/11 LLF: SINS LiDAR-only sul codice corretto (profilo coda, no camere) x3, 15 finestre, vs LFX/LHF (tutti i sensori)

| seq | LFX/LHF moving rec/prec/MOTA, tutti | LLF moving rec/prec/MOTA, tutti | IoU any moving / all: full → LiDAR-only |
|---|---|---|---|
| 0 | 85.2/81.5/65.7, 51.9 | 82.5/82.0/64.2, 34.8 | 82.5/70.1 → 82.1/64.4 |
| 10 | 61.9/67.8/29.8, 15.5 | 61.0/71.2/35.7, 31.4 | 41.4/31.4 → 47.0/31.2 |
| 16 | 72.2/92.5/64.8, 50.3 | 72.6/95.2/68.4, 47.0 | 83.0/71.9 → 87.1/58.3 |
| 11 | 62.5/79.1/43.9, 30.8 | 57.5/83.2/45.6, 34.1 | 70.2/42.7 → 67.8/23.9 |
| 12 | 40.5/70.2/21.8, 17.9 | 35.0/71.3/20.4, 19.2 | 56.6/51.7 → 49.9/43.7 |

- PD trovati: LiDAR-only 0 (tuning) / 2.4 (held-out) contro 28.5 / 47.7. Gli oggetti fermi (PD) vengono dalle camere.
- MOTA moving: alla pari o migliore senza camere (seq 10 +5.9, 16 +3.6, 11 +1.7). MOTA "tutti" su seq 10 31.4 vs 15.5: le camere su seq 10 aggiungono molti FP.
- IoU "all" più alta con le camere (fermi riconosciuti); IoU moving simile.
- Latenza tracce LLF 125/126 ms (med/p90) vs 179/329 (LFX) e 258/669 (LHF).

## 10/11 mattina: latenza (root 06a21ae, sins a0b33f8), x3, STEREO=1, adattatore cloud_stride 2

| tag | moving MOTA 0/10/16 | MOTA tutti 0/10/16 | PD trov/prec | veic. CODa | latenza tracce med/p90 | fusion wait |
|---|---|---|---|---|---|---|
| LFX (coda, harness vecchio) | 65.7/29.8/64.8 | 51.9/15.5/50.3 | 28.5/46.8 | 20.8 | 179/329 | 71.0 |
| LFS (coda, stride 2) | 65.7/32.9/64.4 | 44.6/15.0/49.7 | 31.3/44.2 | 15.5 | 74/146 | 42.3 |
| LL3 (coda_lt3, stride 2) | 64.6/32.9/64.1 | 46.5/16.7/49.9 | 32.1/46.6 | 14.5 | 68/104 | 29.8 |

Strada: mdp_street_lt2 MOTA 43.3 [42.4-44.6] (mdp_street 43.8), veicoli 31.2 (32.9), persone 40.2 (=); latenza tracce ~0/9 ms vs 94/107 (wait 0.2 ms, cam/tick 0: il YOLO della strada non arriva mai in tempo).
- LL3 vs LFS: stessa accuratezza (entro rumore), p90 146 → 104 ms. Lo stride 2 da solo porta la mediana 179 → 74 ms.

## 10/11 held-out 11/12, stesso harness (stride 2, STEREO=1, codice a0b33f8), x3: LHV (coda_v) vs LHS (coda)

| tag | s11 moving rec/prec/MOTA, tutti | s12 moving rec/prec/MOTA, tutti | PD trov/prec | veic. in moto (11/12) | latenza med/p90 |
|---|---|---|---|---|---|
| LHF (vecchio harness) | 62.5/79.1/43.9, 30.8 | 40.5/70.2/21.8, 17.9 | 47.7/36.0 | 3.5 | 258/669 |
| LHS (coda) | 62.8/80.6/46.7, 30.9 | 42.4/70.5/23.2, 18.0 | 50.8/34.1 | 3.7 | 96/169 |
| LHV (coda_v) | 62.0/82.2/47.2, 31.7 | 42.6/71.4/24.1, 18.9 | 51.9/34.7 | 3.9 | 96/170 |

- V su held-out: +0.5 / +0.9 di MOTA moving, entro il rumore (range LHS 45.2-48.3 / 22.5-24.2). Il +8.6 su seq 0 del tuning non si conferma: guadagno piccolo ma coerente, nessun peggioramento.

## 10/11 portatile: coerenza temporale camere + audit del workflow (sins c65e6c6 → fbb2445 → 232a110, root 3a1025b)

Verifiche:
- Stamp camere del robot corretti: imbardata da immagini (phase correlation) vs giroscopio su mdp_slopes, sfasamento
  front +5 ms / back +20 ms (corr 0.95), mentre l'arrivo è 164 / 214 ms dopo lo stamp → ritardo reale di
  compressione/trasporto (topic compressed/zstd/compressedDepth), non offset di orologio. Strada: robot fermo
  (odometria e IMU piatte), verifica non possibile lì.
- Stamp colore vs profondità: lab D435 identici (30 Hz, arrivo ~40 ms); strada identici 77% / 1 frame (10 Hz, ~150 ms);
  CODa front (ZED) mai uguali, ~42 ms (7.6 Hz); CODa stereo identici.
- Parametri: tutte le chiavi YAML (outdoor, coda_sy, mdp_street_sy, indoor) sono dichiarate dai nodi
  (eval/diagnostics/workflow_audit/param_audit.py).
- Topic (sonda su tutti i topic, eval/diagnostics/workflow_audit/topic_probe.py + probe_report.py).

Difetti trovati (tutti corretti con opzioni, default = comportamento precedente, accese in coda_sy / mdp_street_sy):
1. Foglie YOLO costruite all'arrivo della profondità con le box dell'immagine PRECEDENTE (CODa stereo 100 ms, front
   130 ms; stesso frame 0-1%). → yolo_leaf_sync: foglie dal frame YOLO più vicino in stamp, decise quando nessuno può
   essere più vicino (stereo 0 ms, front ~40 ms = minimo possibile). Maschera dello stesso frame della detection.
2. Profondità portata nel globale con la posa dell'ultimo scan, non del suo stamp → camera_pose_at_stamp
   (preprocessing) e pose_at_stamp (static_structures, anche per lo scan LiDAR).
3. Strada: la camera front usa il percorso immagine allineata, che NON costruiva foglie → su strada foglie mai
   esistite (classe solo dal fallback pixel). → yolo_leaf_aligned_path.
4. Strada: calibration_icp in ascolto su /…/aligned_depth_to_color/image_raw/zstd (nessun publisher, il bag è
   compressedDepth) → ICP mai eseguita, TF iniziale dopo 7 s. → depth_transport_from_preprocessing (launch).
5. Foglie arrivate dopo la fusione del loro scan scartate → fusion_late_leaves_max_age (entrano nello scan dopo, la
   fusione non le aspetta); una foglia tardiva dà solo la classe, nessun taglio geometrico (punti vecchi).
6. Fallback pixel del tracker e test "in FOV dell'immagine YOLO" (decadimento classe) con posa attuale e posizione
   attuale su box 2D vecchie (strada ~190 ms) → yolo2d_time_compensation (posa camera allo stamp dell'immagine via TF,
   traccia riportata indietro con la velocità KF).
7. Punti LiDAR oltre la profondità nelle foglie presi dall'ultimo scan → scan più vicino al frame.
8. Trigger profondità/scan: aspettava un frame successivo allo scan anche quando quello precedente era già il più
   vicino (front CODa p90 155 ms) → depth_trigger_early (29/75 ms).
Benigni: gseg3d_* disattivato da cfg (publisher senza messaggi); iscrizione a depth/image_rect_raw/compressedDepth
(percorso "fase 2" mai pubblicato); wall_markers ~25 Hz (una pubblicazione per sorgente).
Età (sonda, CODa 10_w2 coda_sy): LiDAR fused 79/123, tracce 96/140 ms; stereo SGBM+deproiezione harness 59 ms.
Strada coda_sy: tracce 7/10 ms (nessuna attesa), foglie 199/226 ms dallo stamp.
1 run strada (con sonda attiva, confondente): MOTA 40.3 vs lt2 44.6, veicoli 25.2 vs 34.2 → causa probabile il
taglio geometrico con foglie tardive (punto 5, corretto in 232a110). In verifica; x3 sul lab (sy_chain).

## 10/11 LVJ (coda_vj = V + J), x3 tuning + strada, harness stride 2 (sins a0b33f8)

| tag | moving MOTA 0/10/16 | MOTA tutti 0/10/16 | PD trov/prec | veic. CODa | latenza med/p90 |
|---|---|---|---|---|---|
| LFS (coda) | 65.7/32.9/64.4 | 44.6/15.0/49.7 | 31.3/44.2 | 15.5 | 74/146 |
| LVJ (coda_vj) | 71.7/34.5/67.2 | 46.9/18.5/52.0 | 33.1/44.2 | 19.0 | 79/154 |

Strada mdp_street_vj: MOTA 46.3 [45.7-46.7] (mdp_street 43.8), veicoli 44.5 prec 8.7 (32.9/6.9), persone 39.4 (40.2); latenza 93/107.
- VJ: migliora tutto su CODa tuning (+6.0/+1.6/+2.8 moving, MOTA tutti +2.3/+3.5/+2.3, PD +1.8, veicoli +3.5) e in strada (+2.5 MOTA, veicoli +11.6), persone in strada −0.8.

## 10/11 SY (coerenza temporale camere, root 31868e9 / sins 8599702), x3, stride 2, STEREO=1

| tag | moving MOTA | MOTA tutti | PD trov/prec | latenza med/p90 | fusion wait |
|---|---|---|---|---|---|
| LL3b (coda_lt3, stesso build) tuning 0/10/16 | 63.1/33.1/64.5 | 46.5/16.2/50.2 | 31.3/45.7 | 74/114 | 32.1 |
| LSY (coda_sy) tuning 0/10/16 | 66.3/33.8/71.2 | 46.5/15.4/56.4 | 33.0/46.2 | 53/103 | 21.2 |
| LHL3 held-out 11/12 | 47.2/24.7 | 31.8/20.8 | 51.2/35.3 | 71/117 | 33.1 |
| LHSY held-out 11/12 | 46.0/23.9 | 31.8/20.0 | 50.8/37.0 | 68/116 | 27.2 |

Strada mdp_street_sy: MOTA 42.7 [42.5-42.9] (lt2 43.3), veicoli 32.0 (31.2), persone 39.3 (40.2); latenza ~0/9 ms.
- Log SY: leaf sync gap front 24 ms / stereo 0. Fused scan-leaf mediana 64 ms su CODa tuning, 33 sulle held-out, 231 ms in strada. Late class-only per run: 113 tuning, 400 held-out, 1540 strada. Calibrazione ICP pubblicata in strada.
- SY: guadagno su tuning (seq 16 +6.7 moving), NON confermato su held-out (−1.2/−0.8, entro i range). In strada pari o peggio: le foglie arrivano quasi sempre tardi (YOLO ~230 ms).
- Disco 10/11: run obsolete (pre-fix) e diagnostiche già decise spostate su NAS derived/coda_runs_archive e mdp_runs_archive (KWN, LOCw4, PERFw4, LFZ0/30, LWNC/P, LHW, LHK, LLO, strada fz0/fz30/wn/wnc/wnce/wncjm).

## 10/11 VJSY / VJSYE x3, stride 2, STEREO=1 (build root 693feee / sins 758189b, M/MF spenti = identico al richiesto)

| tag | moving MOTA | MOTA tutti | PD trov/prec | veic. CODa (moving rec.) | latenza med/p90 |
|---|---|---|---|---|---|
| LVJ tuning 0/10/16 | 71.7/34.5/67.2 | 46.9/18.5/52.0 | 33.1/44.2 | 19.0 | 79/154 |
| LVS (coda_vjsy) tuning | 72.0/37.5/73.5 | 54.1/20.0/57.9 | 34.5/46.7 | 22.6 | 71/117 |
| LVE (coda_vjsye) tuning | 71.3/37.4/73.6 | 54.8/18.6/58.8 | 34.7/45.7 | 25.3 | 69/115 |
| LHS held-out 11/12 | 46.7/23.2 | 30.9/18.0 | 50.8/34.1 | 3.7 | 96/169 |
| LHVS held-out | 48.2/24.5 | 33.0/18.3 | 54.9/35.1 | 5.5 | 74/119 |
| LHVE held-out | 48.4/22.8 | 35.0/18.1 | 56.7/36.5 | 6.0 | 72/118 |

Strada: vj MOTA 46.3, veicoli 44.5, persone 39.4 | vjsy 47.2 [46.4-48.6], veicoli 44.2, persone 40.5 | vjsye 46.6 [45.9-47.6], veicoli 49.9 prec 9.2, persone 37.9.
- VJSY e VJSYE confermano su held-out (seq 11 +1.5/+1.7 moving, MOTA tutti +2.1/+4.1, PD +4/+6), seq 12 moving pari (VJSYE −0.4). Latenza −20 ms rispetto a LHS.
- VJSYE: più veicoli (CODa +2.7, strada +5.7), persone in strada −2.6.
