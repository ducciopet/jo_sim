# Piano per la pubblicazione di jo_detector_v2

Documento di lavoro. Si appoggia a `BENCHMARK_PLAN.md`, che contiene il protocollo sperimentale
nel dettaglio; qui ci sono il posizionamento, la venue, le scadenze e la struttura dell'articolo.
Data di stesura: 2026-10-13.

---

## 1. Venue e calendario

**Raccomandazione: RA-L con opzione IROS 2027.**

- La deadline di ICRA 2027 (metà settembre 2026) è già passata. IROS 2027 avrà la deadline
  presumibilmente a inizio marzo 2027, e RA-L con opzione IROS chiude di solito qualche giorno prima.
  Va verificato sul sito appena viene pubblicata la call.
- RA-L va bene per un sistema con esperimenti reali estesi, ha 8 pagine (6 + 2 a pagamento) e la
  revisione è a turni. LV-DOT e Xu et al. 2023, i baseline più vicini, sono usciti proprio lì.
- In alternativa, se a febbraio la matrice di benchmark esterni non è completa: un workshop IROS/ICRA
  o una conferenza più piccola (ECMR, ICAR) usando solo lab + mocap. È un ripiego, non il piano A.

Calendario a ritroso da una submission prevista il **2027-02-22** (circa 19 settimane):

| Settimane | Periodo | Obiettivo | Corrisponde a |
|---|---|---|---|
| 1–2 | 13/10 – 26/10 | Inventario, harness unico, test sintetici verdi | Fasi 0–1 di BENCHMARK_PLAN |
| 3–5 | 27/10 – 16/11 | Nuova campagna di registrazioni con mocap (§4), v1/v2 sulle bag Qualisys | — |
| 4–7 | 3/11 – 30/11 | Prima sequenza JRDB end-to-end (v2, poi v1) | Fase 2 |
| 6–10 | 17/11 – 21/12 | Container ROS1 dei baseline, sanity check Dynablox/DOALS | Fase 3 |
| 11–13 | 4/1 – 24/1 | Matrice completa, freeze del codice v2 (tag `paper-v1`) | Fase 4 |
| 12–14 | 11/1 – 31/1 | Ablation | Fase 5 |
| 13–17 | 18/1 – 21/2 | Scrittura, figure, video | Fase 6 |
| 17–18 | 8/2 – 21/2 | Revisione interna (relatore + un lettore esterno al gruppo) | — |

**Regola del freeze**: dopo il tag `paper-v1` si correggono solo bug, documentati, e ogni fix
richiede di rifare tutta la matrice. Sono vietati il tuning e l'aggiunta di feature.

---

## 2. Contributo: cosa è nuovo e cosa no

Il punto più delicato è che gran parte del `tracker_node` è un port, dichiarato nei commenti del codice,
di v1, che a sua volta deriva da LV-DOT (associazione multi-termine, KF ad accelerazione costante,
voting sui punti, sticky dynamic, decay semantico). Un revisore che conosce LV-DOT se ne accorge subito.
Il paper deve quindi delimitare onestamente la novità. Candidati, dal più forte al più debole:

1. **Ragionamento sulle occlusioni lungo il raggio LiDAR** (`isDisocclusionArtifact`,
   `isOcclusionExplained`). Sopprime i falsi "dinamici" dovuti allo sfondo che torna visibile e
   salva l'ID dei track occlusi da un altro oggetto dinamico. Non deriva da v1 né da LV-DOT ed è
   l'unico contributo algoritmico del tracker. Serve un'ablation dedicata e una figura che mostri il
   caso persona davanti al muro.
2. **Classificazione a tre stati dynamic / potentially-dynamic / static valutata contro motion
   capture**, con latenza di transizione (start/stop) misurata in secondi. Quasi nessuno in letteratura
   misura la latenza di classificazione con GT metrica: è un contributo di metodologia e di dati.
3. **Fusione N-camera RGB-D + LiDAR a livello di box orientati**, con gerarchia di nesting che
   separa i cluster sovra-fusi (`dbscan_detector_node`, `box_fusion.hpp`). LV-DOT supporta una sola
   camera e box allineati agli assi.
4. **Robustezza su terreno inclinato e su piattaforma cingolata**: piano del pavimento stimato per
   camera e filtrato con EMA, muri gestiti in modo asincrono, calibrazione LiDAR–camera raffinata
   online con GICP. È un contributo ingegneristico: va bene come "system", non come titolo.
5. **Uso a valle**: rimozione dei dinamici nella mappa GLIM (`glim`/`glim_ros2`, bbox rejection
   basata sui track). È opzionale, ma un esperimento "mappa con e senza rimozione" rende il
   paper molto più interessante per un pubblico SLAM.

**Titolo di lavoro**: *Occlusion-Aware Dynamic and Potentially-Dynamic Obstacle Tracking with
Multi-Camera RGB-D and LiDAR Fusion for Ground Robots*.

**Claim principale (da confermare con i numeri, non prima)**: a parità di detector, v2 riduce i
falsi dinamici e gli ID switch rispetto a LV-DOT e v1, soprattutto in scene affollate e con occlusioni,
a un costo computazionale compatibile con l'onboard.

Se i numeri non sostengono il punto 1, il paper diventa un system paper (punti 2–4) e il titolo va
cambiato di conseguenza. La decisione si prende a fine gennaio, guardando le ablation.

---

## 3. Esperimenti, in ordine di priorità

Il protocollo di dettaglio (split, manifest, formato `tracks.csv`, metriche) è in `BENCHMARK_PLAN.md`.

**E1. Lab con GT motion capture (Qualisys): esperimento principale, irrinunciabile.**
- Metriche: precision/recall/F1 per classe D/PD/S, MCC, latenza di onset e di stop, ID switch.
- Metodi: v2, v1, LV-DOT (ROS1, stesse bag convertite), e v2 senza YOLO.
- Le 6 sequenze attuali (`bags_benchmark/` + `qualysis/`, circa 6 minuti in tutto, una sola
  persona, solo camera frontale, nessuna odometria registrata) **non bastano**: vedi §4.
- **Stato (2026-10-14, campagna r8, 3 ripetizioni)**: v2 MOTA 96,5 ± 0,3%, IDF1 80,7 ± 6,7%, IDSW 3 contro
  v1 81,1 ± 2,3%, 60,4 ± 0,9%, 26. Numeri da riportare come media ± std (l'IDF1 cambia molto tra run). Dettaglio
  in `eval/STATUS.md`, tabelle `eval/tables/own_r8_*_reps.*`.
  Correzioni dei frammenti solo-camera (`df56a77`) e della classe (`4a41b17`), A/B col solo tracker sugli input r8:
  MOTA 97,2 ± 0,4%, IDF1 94,9 ± 3,1%, IDSW 2; vanno confermate nella prossima campagna a pipeline completa.

**E2. Dataset pubblico con box 3D: JRDB (prima scelta) o CODa.**
- Serve per i confronti esterni e la generalità. Metriche MOT: HOTA, MOTA, IDF1 per la detection
  e il tracking, più F1 sullo stato dinamico derivato dalla GT.
- Spazio su disco: oggi restano liberi **62 GB** su `/`, che non bastano per JRDB e tanto meno
  per CODa. Serve un disco esterno o storage del laboratorio prima della Fase 2.

**E3. LiDAR-only su DOALS** (point-level IoU), contro Dynablox e M-detector.
Serve solo come sanity check del harness e come riga "LiDAR-only" in tabella. Non è centrale.

**E4. Ablation di v2** (richiedono flag nel codice, vedi Fase 5):
senza disocclusione, senza budget di occlusione, dt nominale, senza class continuity,
senza YOLO, solo depth, camera singola invece di N.
Associazione `hungarian` contro `greedy_kuhn` (r8, solo tracker sugli stessi input): nessuna differenza
misurabile con una persona; va ripetuta sulle sequenze con più persone.

**E5. Runtime** su hardware onboard (il PC del robot, non il portatile con la RTX 4060):
latenza per nodo, CPU/GPU, frequenza effettiva, con un grafico a barre impilate.

**E6 (opzionale). Mappa GLIM con e senza rimozione dei dinamici**, con una metrica di "ghost
points" rispetto alla mappa statica.

---

## 4. Nuova campagna di registrazioni mocap (da pianificare subito)

È il collo di bottiglia reale, perché dipende dalla disponibilità del laboratorio Qualisys.
Scenari, ognuno ripetuto almeno 3 volte, con robot fermo e con robot in movimento:

| Scenario | Cosa testa |
|---|---|
| Persona cammina, si ferma 5 s, riparte (×3) | latenza onset/stop, transizione D→PD→D |
| Persona ferma davanti a un muro, poi si sposta lateralmente | **disocclusione** (contributo 1) |
| Due persone che si incrociano lungo il raggio del LiDAR | **occlusione D-davanti-a-D**, ID switch |
| Persona seduta / china / con carrello o scatola | PD con forma non standard, size gate |
| Persona fuori FOV camera (solo LiDAR) che entra nel FOV | fallback LiDAR-only, decay semantico |
| Oggetto non-persona spostato (sedia su ruote) | dinamico senza classe YOLO |

Requisiti:
- Un rigid body Qualisys **per ogni persona** e uno sul robot (JO), come già fatto.
- Registrare **entrambe le camere** e la depth allineata (`aligned_depth_to_color`). Le bag di maggio
  hanno solo la camera frontale con depth raw.
- Registrare `/odometry/filtered` oppure i dati per ricalcolarla. Le bag di maggio hanno `/tf` vuoto
  e nessuna odometria, quindi GLIM + EKF vanno rigirati offline.
- Una sincronizzazione temporale robusta tra il PC Qualisys e il robot (chrony/PTP sulla stessa rete),
  invece dell'allineamento temporale stimato a posteriori che fa oggi `detector_eval_pipeline.py`.
- Totale indicativo: circa 40 sequenze da 60 s, e una giornata di laboratorio.

---

## 5. Struttura dell'articolo (RA-L, 8 pagine)

1. **Introduction**: robot di terra in ambienti condivisi; la distinzione tra dinamico e
   potenzialmente dinamico conta per la navigazione e per lo SLAM; i limiti dei metodi esistenti
   (singola camera, occlusioni, terreno piatto). Contributi elencati in 3 punti al massimo.
2. **Related work**: (a) MOS e rimozione dinamici su LiDAR (Dynablox, M-detector, ERASOR, DUFOMap,
   4DMOS); (b) detection e tracking LiDAR-visivo per robot (LV-DOT, Xu 2023); (c) 3D MOT
   (AB3DMOT, SimpleTrack). Va scritto il prima possibile, perché fissa il posizionamento.
3. **System overview**: figura a pagina intera della pipeline (preprocessing → static structures →
   detection/fusione → tracker), con un riferimento esplicito a cosa eredita da LV-DOT.
4. **Method**: fusione N-camera e nesting; tracker con disocclusione e budget di occlusione (con
   l'algoritmo in pseudocodice); classificazione a tre stati.
5. **Experiments**: setup e dataset; E1; E2; E4 ablation; E5 runtime.
6. **Discussion / limitations**: dichiarare i limiti onestamente, cioè dipendenza dall'odometria,
   sola classe "person" per YOLO e calibrazione.
7. **Conclusion**.

Materiale aggiuntivo: un video di 2–3 minuti (molto utile per RA-L/IROS) e il link al codice.

---

## 6. Rilascio di codice e dati

- Il codice è oggi su `github.com/ducciopet/jo_detector_v2`. Prima della submission servono:
  licenza (BSD-3 o Apache-2, compatibile con le dipendenze; il segmentatore del terreno ha già
  `LICENSE-ground_segmentation.txt`, da verificare), un README "quick start" su bag di esempio
  e il tag della versione usata nel paper.
- Il dataset mocap è un contributo in sé se rilasciato: bag ROS2 + GT + script di valutazione.
  Va chiarito con il relatore se si può pubblicare (privacy delle persone riprese: servono un
  consenso, oppure solo LiDAR + depth senza RGB, oppure volti sfocati).
- La doppia cieca non si applica a RA-L, quindi il link al repo può stare nel paper.

---

## 7. Rischi principali

| Rischio | Impatto | Mitigazione |
|---|---|---|
| Disco insufficiente per JRDB/CODa | blocca E2 | disco esterno ≥ 2 TB entro novembre |
| Laboratorio mocap non disponibile | E1 debole | prenotarlo questa settimana; fallback: annotazione manuale con `GT_SCHEMA.md` |
| LV-DOT non riproducibile in ROS1 sulle nostre bag | manca il baseline più vicino | iniziare dal suo `corridor_demo.bag`; documentare le patch |
| v2 non batte v1/LV-DOT in modo netto | claim debole | spostare il claim su occlusioni e latenza (dove ha meccanismi dedicati) o farne un system paper |
| Tuning implicito su dati di test | rigetto o perdita di credibilità | split tune/eval fissato prima di guardare i numeri |
| Codice che cambia durante gli esperimenti | numeri non riproducibili | tag `paper-v1` + `manifest.json` per run |

---

## 8. Decisioni che spettano a te o al relatore

- Venue finale (RA-L+IROS consigliata) e se si vuole fare anche un workshop prima.
- Autori e ordine (chi ha scritto v1, GLIM modificato, chi gestisce il mocap).
- Se si rilascia il dataset mocap, e in che forma.
- Quanto tempo-calcolo e disco dedicare a JRDB/CODa.
