# Are CODa moving FP real but unannotated agents? (17/11, I2P = vjsye2i2p)

Question: how much of SINS's moving false positives on CODa (tuning windows 0/10/16, runs LVI2P x3, no replay) is a
hole in the ground truth. GT and metric are NOT changed here; this only measures.

## 1. Funnel (fp_funnel.py, coda_eval dynamic_only pipeline, 3 reps summed)
Moving FP 5377 (ref LVE2L 5423), moving FN 6781 (ref 7115). 85 % of the FP have no BUILT GT object near them
("no_object": no moving GT within 2.5 m, no still or ignored GT within 1.5 m).

## 2. Most of those are a metric artifact: annotations dropped at track edges (gt_edge.py)
`coda_eval.build_gt` -> `harness.gt_mocap.gt_tracks` drops every annotation whose speed is undefined (first/last
~0.5 s of each annotated track, half_window 0.5 s, and gaps > 0.3 s), but the frame stays on the grid. A prediction
on that object is therefore scored FP (and the object gives no FN). Rows dropped per window: 4-21 %
(0_w1 5, 0_w2 9, 0_w3 6, 10_w1 11, 10_w2 9, 10_w3 21, 10_w4 15, 10_w5 14, 16_w1 4 %).

Moving FP within 20 m (3 reps) with a RAW annotation (any class) within 1.5 m at the same frame:

| bucket | rows | on a dropped annotation |
|---|---|---|
| no_object | 2565 | 1763 (69 %) |
| dup_moving | 407 | 304 |
| off_moving | 259 | 140 |
| all moving FP | 3369 | 2345 (70 %) |

The no_object rows on dropped annotations are correct detections of annotated agents (mostly pedestrians just entering
or leaving the annotated span). This applies to every method evaluated with coda_eval (baselines too); the fix would
be to turn the dropped rows into ignore regions instead of deleting them. It is a metric change: NOT done here,
decision for the user.

## 3. The residual no_object FP (no raw annotation near): visual check (fp_cam.py, lidar_view.py)
36 tracks sampled (rep 1, range <= 20 m, weighted by FP frames, stratified by the published class: 26 none,
4 person, 4 vehicle, 2 bicycle). Only 6 are inside a camera image at their FP ticks (cameras look forward only);
the other 30 were checked in a LiDAR view (bird's-eye with scans at t-1 s / t / t+1 s + side view).

| verdict | tracks | FP frames |
|---|---|---|
| static structure (wall, pole, railing, door frame, hedge, stairs, glass reflection) | 25 | 98 |
| piece of an annotated object (box > 1.5 m off a vehicle centre, crowd fragment, annotation gap) | 7 | 17 |
| ghost with no LiDAR points (coasting / drifting track) | 2 | 3 |
| uncertain (could be a person) | 2 | 6 |
| unannotated pedestrian / cyclist (sure) | 0 | 0 |
| unannotated vehicle | 0 | 0 |

Per-track verdicts and notes: img/samples_LVI2P.csv; images img/*.jpg (camera: FP box red, other SINS tracks grey,
raw CODa GT green, 3x crop) and img/lidar_*.png.

## Conclusion
- Unannotated agents are at most ~5 % of the residual no_object FP (2 uncertain of 36, 0 sure): the CODa GT has
  no significant hole of unannotated people.
- ~70 % of the moving FP within 20 m sit on annotations that `build_gt` drops at track edges: an evaluation
  artifact common to all methods.
- What remains is genuinely SINS's: mostly static structure called moving (walls, poles, railings, door frames,
  often a fragment sliding along a wall), then boxes offset from large annotated vehicles.

Reproduce (lab venv): `fp_funnel.py LVI2P <9 windows>` -> `gt_edge.py fprows_LVI2P.csv` ->
`fp_cam.py fprows_LVI2P_edge.csv LVI2P 30 img` -> `lidar_view.py img/samples_LVI2P.csv fprows_LVI2P_edge.csv LVI2P img`.
