# Audit 19/11: which parameters/functions keep SINS from tracking D and PD (CODa, reference R0)
Laptop diagnostic runs (not timed results): R0 + per-track CSV (coda_vjsye2i2pcadbg, tag ADT, 9 tuning windows) and
R0 + lifecycle log + wall markers recorded (coda_vjsye2i2pcadbl, tag ADL, w1 10_w1 10_w4 16_w1).
Funnel (eval/diagnostics/dpd_study/funnel.py, rows_coda_ADT.csv), grouped by class with funnel_groups.py.

Share of annotated rows handled correctly (OK = output within 1 m with the right state):
moving: persons 82%, vehicles 39%, bikes/scooters 12%; still: vehicles 17%, bikes/scooters 23%, persons 2% (3 people).

Findings (impact order):
1. Leaf depth slice +-0.3 m (yolo_leaf_depth_tolerance): vehicles cut to ~0.6 m; leaf/cluster IoU < 0.12 -> "no candidate"
   (55% of car/truck leaves); "too low for its class" 26% of car leaves. Fix in test (leafc chain QC).
2. Cluster size cap 6 m / 40 m3 drops 9% of vehicle rows. In test (QB).
3. Walls: 33% of vehicle rows have a static-structures wall centred on the vehicle (pickup 35%, service vehicle 74%,
   delivery truck 22%, cars 0%: wall2d_min_vertical_extent 1.8 m); points within 0.15 m of a wall are stripped.
4. Fragmentation: 64% of vehicle rows reach the tracker as >= 2 detections, 48% with a piece < 0.7 m tall
   (plateau splitter split_* + refine keep_rest).
5. Still objects re-born: per window 13-33 new tracks per vehicle, 9-12 per bike, 2-5 per person. Refusal of the
   nearest track: another track took it 22-35%, velocity-direction gate 23-26%, top-height diff 15-19%, speed gate 11-15%
   (centre jumps of fragments read as motion). track_large_object_footprint (relaxed gates for large objects) is off (-1).
6. Out of camera after being seen: 73-82% of PD losses are a NEW track without class (not the same track losing it):
   still vehicles OK 16% out of FOV vs 39% in FOV; bikes 23% vs 34%.
7. Class cleared after 3 in-FOV ticks without YOLO (track_max_non_yolo_in_fov_frames) + growth trigger (5 ticks).
8. Unclassified movers need person size to become D (size gate: 6% of moving-vehicle rows); moving vehicles held PD 13%
   (corroboration). Bikes beyond 15 m: 53% not detected at all (thin, sensor limit).
