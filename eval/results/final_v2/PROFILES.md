# Final SINS profiles paper-v2 (FROZEN 28/11, user "ok" in the lab-PC session; md5s computed before any v2 final run)

sins paper-v2 = 220a266 (c7-assign-components, fast-forwarded into main and tagged paper-v2). It is paper-v1 (9ea5942)
plus three default-off/unchanged switches: track_reacquire_radius (off), tick_report_every (50 = unchanged),
track_assignment_components (off by default, ON in the _v2 profiles). paper-v1 and its results are untouched.
Each v2 profile = the frozen v1 profile + fusion_timeout_sec 0.05 -> 0.0 (NW, lab_queue 54-55) + track_assignment_components
true (C7, lab_queue 53). Overrides in jo_sim eval/configs/sins/<profile>/overrides.yaml (7c6c4941). The generated cfg
folders are not committed in sins (the tag holds only code); a copy is in eval/results/final_v2/cfg/.
md5 = md5 over the sorted relative paths + contents of sins/cfg/<profile>/ (as make_manifest.py). The v1 folders were
rechecked on 28/11 and still match eval/results/final/PROFILES.md.

| profile | used for | md5 of the generated cfg folder |
|---|---|---|
| coda_vjsye2i2pcar1lccbma_v2 | CODa tuning, CODa 11/12, CODa new 2/3/18 | 5738f9809ccd30145beb72e965e56e7a |
| kitti_tracking_v2 | KITTI tracking | 64a6c8caa05dd220358b5b8b283bd503 |
| doals_v2 | DOALS (LiDAR-only: fusion_timeout has no camera to wait for) | cdb8f1d2570e6280c5fd5f493dc9e829 |
| jrdb_v2 | JRDB | 930a12a000674f8eef01a91781d8a0f0 |
| lab_ucar1lccbma_v2 | Lab Qualisys | 287a207734a2d5cd631a20ad8b6f28f5 |

Harness, load gate, manifests, reps (CODa tuning 5, lab 6, others 3), requeue and QUIET_P90 values: the same as
paper-v1 (eval/scripts/final/run_final.sh unchanged). Run tag S2. Outputs: NAS derived/sins_final_v2/<ds>/, JSONs
eval/results/final_v2/<ds>/. Chain: eval/scripts/final/final_chain_v2.sh.
