# eval/ — benchmark harness

Protocollo: `../BENCHMARK_PLAN.md`. Stato e numeri: `STATUS.md`.

## Struttura

| Percorso | Cosa fa |
|---|---|
| `harness/tracks.py` | formato unico `tracks.csv` (`t_ns,track_id,x,y,z,l,w,h,yaw,vx,vy,vz,state,class,score`) |
| `harness/metrics.py` | CLEAR MOT, IDF1, confusione D/PD/S, recall per stato, latenza di transizione, su una griglia temporale comune; tre livelli (`label_mode`): D/PD/S completo, solo moto (D contro il resto), D e PD unite |
| `harness/gt_mocap.py` | GT da motion capture (stato dalla velocità) e allineamento mocap ↔ odom basato solo sulle pose del robot |
| `harness/ros_io.py` | lettura bag (rosbags, nessuna installazione ROS necessaria) |
| `harness/tests/` | test sintetici: le metriche e l'allineamento devono recuperare valori iniettati noti |
| `run_own_eval.py` | valuta i run sulle sequenze di laboratorio con GT Qualisys |
| `report.py` | tabelle CSV + LaTeX dai risultati |
| `adapters/realsense_bag_adapter.py` | ricava gli stream RealSense mancanti (estrinseco, immagine compressa) da bag registrate con meno stream |
| `scripts/make_odom.sh` | genera l'odometria **una volta** per sequenza (GLIM + EKF) |
| `scripts/run_method.sh` | fa girare un metodo su bag sensori + bag odometria condivisa |
| `scripts/run_qualisys_batch.sh` | tutte le sequenze Qualisys, v1 e v2 |
| `scripts/run_campaign.sh` | campagna ripetuta e riprendibile: N ripetizioni di v2 e v1 a pipeline completa + ablation `greedy_kuhn` col solo tracker |
| `campaign_report.py` | valuta ogni ripetizione, scarta i run troncati (copertura < 97%), tabelle media ± std |
| `make_video.py` | video dall'alto GT contro track, metodi affiancati |
| `make_report_page.py` | pagina HTML della campagna (grafici + video) da `report_template.html` e `report_<C>_notes.html` |
| `detector_eval_pipeline.py` | valutatore precedente (solo classificazione D/PD/S), tenuto come riferimento |

## Uso

```bash
# test (sull'host; i plugin pytest di ROS vanno disattivati)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider eval/harness/tests

# run nel container (gli script vanno copiati in bags/replay/, che è montato)
docker exec <container> bash /home/ros/bags/replay/run_qualisys_batch.sh

# valutazione + tabelle (sull'host)
python3 eval/run_own_eval.py --runs bags/replay --mocap bags/qualysis --methods v1 v2 --dyn-speed 0.3 0.2 0.4 0.5
python3 eval/report.py --name own_mocap_full                                              # livello D/PD/S
python3 eval/report.py --summary eval/results/own/summary_dynamic_only.json --name own_mocap_dynamic_only
python3 eval/report.py --summary eval/results/own/summary_merged.json --name own_mocap_merged

# campagna ripetuta (numeri da paper): run nel container, poi valutazione, video e pagina sull'host
docker exec <container> bash -c "CAMPAIGN=r8 bash /home/ros/bags/replay/run_campaign.sh"   # rilanciabile: salta i job finiti
python3 eval/campaign_report.py --campaign r8 --reps a b c
python3 eval/make_video.py --results eval/results/own_r8a --seq 20260519_1947 --methods v1 v2 --out eval/videos/r8/r8a_1947.mp4
python3 eval/make_report_page.py --campaign r8                                            # -> eval/report_r8/index.html
```

## Scelte di protocollo

- **Griglia comune a 10 Hz**: ogni metodo viene campionato negli stessi istanti (ultimo output con età ≤ 0,25 s),
  così la frequenza di pubblicazione non avvantaggia né penalizza nessuno.
- **Oggetti d'interesse**: si valutano solo gli output con stato `dynamic` o `potentially_dynamic`; i track
  `static` di un metodo vengono ignorati.
- **Stato GT**: `dynamic` se la velocità orizzontale, stimata come differenza centrata su ±0,5 s, supera la soglia
  (0,3 m/s di default, con sweep di sensibilità); altrimenti `potentially_dynamic`.
- **Dropout del mocap**: i frame in cui il marker GT è perso dentro il suo intervallo temporale vengono saltati,
  per non contare come FP un rilevamento corretto.
- **ROI**: le predizioni fuori dal volume osservato dal mocap (+0,5 m) vengono ignorate.
- **Allineamento**: T_mocap_odom e l'offset di clock vengono stimati solo dalle pose del robot (odometria vs rigid
  body del robot), mai dall'output di un metodo. Nelle sequenze attuali il robot non ruota mai, quindi la
  traslazione del rigid body rispetto a `base_link` non è osservabile: è fissata a 0 (`--t-base-jo` per impostarla
  dopo averla misurata) e il bias corrispondente va dichiarato.
- **Odometria identica per tutti**: generata una sola volta per sequenza e riprodotta, invece di far girare GLIM
  a ogni run (GLIM non è deterministico).
