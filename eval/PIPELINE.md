# SINS work pipeline to paper-v1 (agreed 17/11 between the laptop session and the lab PC session)

Rule for every stage: candidates are switchable parameters (default off, physical rationale in the comment, no
per-cluster-type thresholds); A/B x3 ALTERNATED in ONE chain on the lab PC; reference re-run in the same chain;
score e3 on CODa tuning (0, 10, 16; all + testing frames) + vehicles e3 + street; lab = regression check only.
Acceptance (user, 17/11): CODa vs the baselines first, native metric (MOTA all) on testing frames, then moving MOTA
and vehicles, then street; lab only "no clear regression". Latency per scan reported (med / p90) for every arm.
Held-out 11/12 never used before S6. Summaries appended to eval/diagnostics/lab_queue_17nov.md; STATUS by the laptop.

| Stage | Where | What | Arms (cumulative) | Decision |
|---|---|---|---|---|
| R0 | lab | reference = I2P + carrier + A (`*_vjsye2i2pca`) | - | adopted 17/11 (A helps CODa/street, carrier fixes lab) |
| S1 class | lab | M yolo id per camera; P3 keep class while the same 2D instance lives; P4 decay only when visible and not occluded | R0 / +M / +M+P3+P4 | keep the best arm if CODa MOTA all (testing) does not drop and vehicles/PD do not drop |
| S2 vehicles | lab | P2 mask-guided fragment merge (same instance mask only); P5 vehicle extent prior (nuScenes mean sizes) | S1 / +P2 / +P2+P5 | same gate; vehicle recall and MOTA all seq 10 are the targets |
| S3 assoc | lab | P1 one-to-one mask <-> cluster assignment, nearest cluster owns the mask (occlusion) | S2 / +P1 | only if S1-S2 leave the class/vehicle gap |
| S4 ground | lab | grid vs planes (floor_source lidar) vs planes (camera) | 3 arms | one model for all if within noise, else per-profile option, declared |
| S5 YOLO size | lab | inference 640 vs 1280 on CODa + street (lab already 640) | 2 arms | one value everywhere if within noise (scale param otherwise, declared) |
| B1 street baselines | laptop | CenterPoint+SimpleTrack, EagerMOT (ped, multi-class), DeepFusionMOT, Dynablox, M-detector on the Marina di Pisa street (data never seen by any CODa-trained model) | 1 run each | comparison on unseen data, as declared domain-transfer test |
| S6 freeze | lab | tag paper-v1 (sins + jo_sim); final chain: lab x3, CODa tuning + held-out x3, street x3, GLIM map; e3 all + testing; baselines re-scored | - | numbers for the paper |
| S7 paper | laptop | hand the paper session (sins-paper-4c): profile + commit, latencies, baseline list, e3 tables (native first) | - | - |
| later | lab | free-space evidence for dense/near objects (probe: AUC 0.83-0.90 near, none far) | after paper-v1 | - |

Fixed facts: data = lab, CODa, street (JRDB out; SiT/CrowdBot rejected); metric primary = MOTA all; e3; CODa split per
frame -> testing frames are 100 ms from training frames (declare); laptop harness fixed (run_method.sh +15 s).
Known measurement bug to fix before S6: street latency from lat_tracks reads -2 / 10 ms (lab, S1 setup).
