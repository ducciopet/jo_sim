# Baseline esterni: come ricostruirli (stato al 2026-10-15 sera)

Regola: codice originale, solo I/O adattato (righe `# JO` nei file di config); patch solo se documentate in `patches/`.
Protocollo comune: bag di laboratorio convertite in ROS1 al volo (`rosbags-convert`), odometria condivisa
`bags/replay/odom_<seq>`, detector avviato 12 s prima, play 1×. Valutazione: `python3 eval/baseline_eval.py --campaign r9`
(livelli "solo movimento" e "D e PD insieme"; adattatori `eval/adapters/marker_adapter.py`, `points_adapter.py`).

| Metodo | Cartella | Immagine Docker | Stato |
|---|---|---|---|
| LV-DOT (LiDAR+RGB-D) | `lvdot/` | `lvdot-noetic:upstream` | **fatto**: campagna 18 run, risultati in `eval/STATUS.md` |
| Dynablox (LiDAR) | `dynablox/` | `dynablox-noetic:upstream` | **fatto**: patch output-only per gli ID; campagna quasi completa (`bags/dynablox_out/campaign.log`) |
| M-detector (LiDAR) | `mdetector/` | `mdetector-noetic:upstream` | config + input (`make_input.py`) + script pronti; **build da verificare** (`build.log`), poi prova sulla 1950 |
| Xu et al. 2023 (RGB-D) | `xu_rgbd/` | `xu-rgbd-noetic:upstream` (FROM lvdot) | config + script pronti; **da costruire e provare**. Variante senza YOLO (la depth allineata manca nelle bag) |
| UniMT (LiDAR+camera) | `ros_packages_src/` | – | da fare: Python 3.10, torch 2.1.2, mmdet3d 1.4, spconv, flash-attn; pesi `cmdt_coda.pth` su Google Drive; nodo `perception_test/scripts/infer2.py` |
| JRMOT | `JRMOT_ROS_src/` | – | bloccato: serve un rilevatore 2D esterno, Gurobi (licenza), messaggi SPENCER |

Per ogni metodo pronto:
```
docker build -t <immagine> eval/baselines/<cartella>
bash eval/baselines/<cartella>/run_<metodo>.sh 20260519_1950 test_1950 --keep-input   # prova
setsid nohup bash eval/baselines/<cartella>/run_<metodo>_campaign.sh > bags/<metodo>_out/campaign.log 2>&1 < /dev/null &
python3 eval/baseline_eval.py --campaign r9 --reps a b c   # aggiungere il metodo a BASELINES in baseline_eval.py
```
Attenzione: mai `pkill -f` con un pattern che compare nel comando stesso (uccide la shell); spazio disco: `docker builder prune -f`.
