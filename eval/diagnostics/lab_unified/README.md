# lab_unified (17/11): unified profile lab_u vs indoor, x3 alternated, 6 Qualisys sequences

Pooled over the 6 sequences (eval/scripts/analysis/agg_lab.py eval/results/lab_unified/cls), GT person only (--annotated-classes person;
it ignored 0 predictions: every unmatched prediction was person-classified or unclassified):

| run | MOTA full | IDF1 | FP | FN | IDSW | MOTA dynamic_only |
|---|---|---|---|---|---|---|
| indoor a/b/c | 97.9 / 98.4 / 97.3 | 98.9 / 99.2 / 92.0 | 15 / 3 / 10 | 46 / 42 / 66 | 0 / 2 / 1 | 90.5 / 92.0 / 89.9 |
| lab_u a/b/c | 85.2 / 83.5 / 85.4 | 90.4 / 87.3 / 80.8 | 276 / 334 / 211 | 136 / 141 / 195 | 16 / 3 / 16 | 86.0 / 84.8 / 84.6 |

Per-scan latency (median of the 18 run medians / median p90): indoor 90.5 / 117.5 ms, lab_u 78 / 100 ms.
YOLO per frame in the pipeline: lab_u YOLO26s-seg 640 px 17.2 ms median / 20.2 p90; indoor box detector 6.4 / 7.6 ms.

Where lab_u loses: FP are person-classified POTENTIALLY_DYNAMIC tracks 2-7 m from the (only) person, concentrated in
1947 and 1950 (the person stops), e.g. LUNIb 1947 track 272: 258 ticks at ~6.5 m from the person, class person.
The camera (LUNIb_1947_trk272.jpg, LUNIa_1950_trk150.jpg) shows ONE person and one YOLO box only: the class reaches
a LiDAR cluster that is not the person (the glass doors/windows behind him give no depth, candidate mechanism:
yolo_leaf_lidar_beyond_depth / nearest_cluster picking the frame behind, then held by track_class_anchor_ttl 5 s).
Not tuned on these sequences; diagnosis only.
