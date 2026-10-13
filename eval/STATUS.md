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
| **v2 (`889bae0`, passaggio di classe + box persona)** | **91,5%** | **79,2%** | **0,09 m** | 55 | **187** | 5 | **95,6%** | **87,4%** | **0,25 s** |

Escludendo i primi 25 s (`tables/own_mocap_after25s.csv`): v1 MOTA 49,5%, IDF1 54,8%; v2 (`889bae0`) MOTA 92,5%, IDF1 89,2%.
Sensibilità alla soglia GT (0,2–0,5 m/s): recall PD di v2 (`889bae0`) tra 72% e 89%, di v1 tra 26% e 32%.
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
