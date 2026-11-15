# Lab brief (15/11): the biggest SINS defects, worked on the lab PC

Reference profile = coda_vjsye2 / mdp_street_vjsye2 (kept after lab_r12p: R5, R12P rejected; STATUS 13/11).
Harness as lab_r12p.sh (ADAPTER_ARGS="-p cloud_stride:=2", lab_local.sh), x3 for any candidate, latency reported.

## Defects, from the reference numbers (lab x3) and the existing funnels (eval/diagnostics/dpd_study/funnel_VDB.txt)
1. Moving false positives on CODa: seq 10 moving recall ~72% but MOTA 37.7; held-out seq 12 moving precision 68.
   -> FP funnel first (eval/scripts/analysis/coda_fp.py on LVE2 runs, tuning windows only): what the false
   dynamic tracks are (static structure sliding with the viewpoint, fragments of real objects, ghost after occlusion).
2. Street vehicles: recall 48.1, precision 9.0 -> same FP analysis on the street run (street_why.py / street_funnel2.py).
3. Large vehicles seen front-on or fragmented: box covers only the visible face, centre 2-3 m off
   (eval/diagnostics/pd_fp_coda10/). Candidate: per-track extent high-water mark + centre shifted away from the
   sensor along the viewing ray for vehicle-class tracks (YOLO evidence), switchable param, default off.
4. PD recall: 27% of still-object rows lost as "track held static (no class)" (seq 10: 50%, mostly out of camera FOV)
   -> by design only camera-classified objects can be PD; check how many are in FOV and lose the class anyway.

## Rules
- Diagnose and tune ONLY on CODa tuning (0, 10, 16) and the street; held-out 11/12 only for the final x3 check.
- No tuning on the 6 Qualisys lab sequences. Outdoor changes only in cfg/outdoor. No per-cluster-type thresholds.
- SINS tracks a still object only if it is capable of motion (movable class); never add PD for static structure.
- Every fix: physical, switchable param (default off), A/B x3 vs reference, latency per scan reported.
- Do not recompile or edit run scripts while runs are in progress. Never touch other people's NAS folders.
- Report each result (numbers vs reference) to session jo-sim-sins-9a; STATUS.md is written by jo-sim-sins-9a.
