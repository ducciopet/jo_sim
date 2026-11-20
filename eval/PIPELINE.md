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
| S1+S2 (one chain, started 17/11 night, s12_chain.sh, sins branch yolo-assoc 43768e1; arms S0=R0, SA=I2P+A without carrier, SM, SMP, SP2 (P2 = existing yolo_refine_merge_fragments), SV; ETA 18/11 ~09:00) | | | | |
| S1 class | lab | M yolo id per camera; P3 keep class while the same 2D instance lives; P4 decay only when visible and not occluded | R0 / +M / +M+P3+P4 | keep the best arm if CODa MOTA all (testing) does not drop and vehicles/PD do not drop |
| S2 vehicles | lab | P2 mask-guided fragment merge (same instance mask only); P5 vehicle extent prior (nuScenes mean sizes) | S1 / +P2 / +P2+P5 | same gate; vehicle recall and MOTA all seq 10 are the targets |
| S2b vehicles (after p5b_chain, user 18/11) | lab | P5b vehicle extent only with reliable heading within 30 deg of the ray; P6 L-shape fit + class dims; P7 shape memory in the track frame (box and centre) | R0 / +best(P5,P5b) / +P6 / +P6+P7 | vehicle recall and MOTA all on testing frames |
| S3 assoc | lab | P1 one-to-one mask <-> cluster assignment, nearest cluster owns the mask (occlusion) | S2 / +P1 | only if S1-S2 leave the class/vehicle gap |
| S4 ground | lab | grid vs planes (floor_source lidar) vs planes (camera) | 3 arms | one model for all if within noise, else per-profile option, declared |
| S5 YOLO size | lab | inference 640 vs 1280 on CODa + street (lab already 640) | 2 arms | one value everywhere if within noise (scale param otherwise, declared) |
| B1 unseen data | lab | (a) lab sequences (mocap GT, never seen by any learned model): learned baselines with the 16-line weights (UniMT, EagerMOT) are already in campaign r9, re-run with all methods at S6; (b) street: Dynablox + M-detector (lab has the images; NAS). The street GT is a semi-automatic reference of MOVING objects (robot still, background model + travel >= 2 m), not hand-annotated: declare it; CODa-trained CenterPoint on the 16-line street LiDAR only as an optional domain-transfer note | 1 run each | comparison on unseen data |
| S6 freeze (after C2-C4) | lab | tag paper-v1 (sins + jo_sim); final chain: lab x3, CODa tuning + held-out x3, street x3, GLIM map; e3 all + testing; baselines re-scored | - | numbers for the paper |
| S7 paper | laptop | hand the paper session (sins-paper-4c): profile + commit, latencies, baseline list, e3 tables (native first) | - | - |
| LEAFC (running 19/11 13:17) | lab | Q0 R0 / QC +connected mask leaf / QB +size cap = bus / QS +speed 15 m/s; lab LQ0/LQC/LQS | cumulative | CODa testing MOTA all, vehicles, moving, street; people next to walls |
| B1 street | lab | Dynablox + M-detector on the street (after LEAFC) | - | baselines on unseen data |
| C2a assoc (user 19/11) | lab | still-object association on the occupied shape (box distance, gates only when the box left the shape) + coasting publication of still classified tracks; quick arm track_large_object_footprint | best LEAFC / +shape / +shape+PD coasting | re-births per object, PD out of FOV, IDSW |
| C2b structure (user 19/11) | lab | plateau splitter off; walls off; walls vetoed on movable masks / classified tracks | best / split off / walls off / walls veto | vehicle fragmentation, FP from facades, persons out of FOV |
| C3 PD memory | lab | still classified objects kept until free space is seen through their shape; PD->D from shape displacement; unclassified D by travel at any size up to a bus | best / +memory / +memory+motion | PD out of FOV, FP |
| C4 fragments | lab | only if still fragmented: no split inside a YOLO instance, merge pieces of one mask (P2), refine keep_rest | best / variants | fragmentation |
| later | lab | free-space evidence beyond C3 (far, sparse objects) | after paper-v1 | - |

Fixed facts: data = lab, CODa, street (JRDB out; SiT/CrowdBot rejected); metric primary = MOTA all; e3; CODa split per
frame -> testing frames are 100 ms from training frames (declare); laptop harness fixed (run_method.sh +15 s).
Known measurement bug to fix before S6: street latency from lat_tracks reads -2 / 10 ms (lab, S1 setup).


## Revision 19/11 evening (user: "procedi con tutto"; independent reviews by two other models, see STATUS)
Freeze date: Mon 24/11 (what is not ready stays out of paper-v1).
Decision rule (pre-registered): adopt an arm only if CODa testing MOTA-all improves on >= 2 of 3 tuning sequences by more
than the spread of its 3 repetitions, no sequence drops by more than 1 point, and the street is not worse; lab = no clear
regression; latency reported. Paired per-window differences are reported with the decision.
Order:
1. LEAFC (running) -> offline error decomposition on its outputs (vehicle FP: extra fragments in a GT box / same object
   with centre > 1 m / two-camera duplicates / ghosts; vehicles re-scored at a 2 m gate as a diagnostic; SINS vs EMMC per
   class on seq 10; share of vehicle points actually stripped by walls; car "too low" rejections) -> safety tag ->
   R1 = LEAFC winner + track_person_ray_offset 0 (rule compliance, in the reference, not an arm).
2. Chain B (reduced C2b): R1 / + splitter off / + roof 4.5 m outdoors (scale) / walls off only together with the size cap.
3. Chain A: YOLO instances as objects (C4-new: point ownership, mask erosion, instance-only objects need >= 2 frames,
   ground contact and min points; late leaves stay class-only; (camera, id) to the tracker) / + shape association /
   + existing switches (no second motion cue for classified tracks, travel proof instead of the person-size gate,
   class decay on fresh YOLO frames only).
4. Freeze 24/11: final chain as leave-one-out ablations from the final configuration; held-out 11/12 once.
Dropped: PD coasting as its own arm, walls veto, large-object exemption (size threshold), P-series revisits, late-leaf
motion compensation. After paper-v1: PD memory with free-space evidence (the native metric ignores out-of-FOV still
objects, coda_eval.py 103-109), late-leaf compensation.
Tooling: test a slowed / lockstep replay on the laptop to cut the run-to-run noise of logic A/B.

## Update 20/11
LEAFC and chain B: no arm passes the rule; R1 (= R0 + ray offset 0 + random_seed 1) is the reference. User requirement:
yolo_leaf_connected is mandatory (no distance window on mask points), so the base becomes R1 + connected (r1lc, on the lab).
Chain A on r1lc, cumulative: r1lcb (+ bus size cap) / r1lcbk (+ shape association) / r1lcbks (+ corroboration_all off,
decay on fresh YOLO frames). Code still to write: C4-new (YOLO instances as objects, absorbs fragments) and splitter off
only on vehicle-class clusters (CBN gained +6.5/+3.9 on seq 10/16 but -13 on seq 0); they join chain A when ready.
Evaluation: e3ib (e3 + --ignore-box) reported next to e3; the user decides which is primary.

## Decisions 20/11 afternoon (user confirmed)
1. Decision rule, replacing the 19/11 one from now on (old verdicts are reported in both forms): an arm is adopted if
   its paired GT-pooled MOTA-all over the 9 tuning windows (seq 0 w1-3, 10 w1-5, 16 w1) improves on BOTH testing frames
   and all frames by more than the run-to-run spread (x3) of the reference; veto if any sequence drops by more than the
   reference's own run range on all frames, or the street drops beyond its run range; lab = no clear regression; latency
   reported. Reason (independent review): seq 0 has 232 testing rows (1 pt = ~2 errors) while R1's own runs span ~7.
2. Metric: e3 stays primary; e3ib (--ignore-box) is a declared secondary column for all methods. Fixed before 11/12.
3. Order: chain A first (r1lc / r1lcb / r1lcbn / r1lcbk; contains the R1+connected reference, so the separate R1 vs
   r1lc chain is dropped), then r1lc_nf (lcn, lcf), then the tracker switches (ks) / walls off with the cap on the winner.
4. Freeze 24/11 is no longer binding: changes that take 4+ days are allowed if the expected gain is large (user).
   C4-new (YOLO instances as objects) is back in scope; out-of-FOV vehicle memory/registration stays after the paper
   (0 GT rows of "still in FOV, then moving out of FOV" on CODa 0/10/16 and the street; still out-of-FOV is ignored).

## Decisions 20/11 evening (user)
- Paper evaluation stays as universal as possible: primary table = e3 MOTA-all over all annotated movable classes (standard
  CLEAR-MOT, 1 m gate), moving-only and PD as secondary columns of the same table; e3ib as a declared extra column. No
  framing choice of a favourable subset; limits (far sparse LiDAR, no semantics out of the camera view by design) are
  stated as facts.
- No learning-based clustering/classification on the point cloud.
- Persons never seen by a camera: faster motion proof for person-sized unclassified clusters tested as a lab arm x3,
  decided by the pooled rule.
- e3ib testing frames of the CSV baselines: pending the NAS mount on the laptop (user).
