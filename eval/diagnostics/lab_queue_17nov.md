# Lab PC queue, 17/11 (afternoon/evening) — summary for the laptop session

Code: sins branch `floor-carrier`, local on the lab PC (not pushed yet): 6cd67e5 floor_source + track_class_anchor_carrier,
cec8e5e track_point_shift_veto_unclassified_only. All default off.

## 1. A/B carrier + A (lab_carrier.sh), x3 alternated in one chain

### Lab (6 Qualisys seqs, GT person only, --annotated-classes person) — eval/results/lab_carrier

| | indoor LIN2 | lab_uc LUC (lab_u + carrier) | lab_uca LUCA (+ A) |
|---|---|---|---|
| MOTA full a/b/c | 97.8 / 97.9 / 98.0 | 93.9 / 94.0 / 93.1 | 94.9 / 88.2 / 96.4 |
| IDF1 | 98.6 / 99.0 / 99.0 | 87.1 / 88.3 / 88.0 | 87.7 / 86.1 / 98.2 |
| FP | 9 / 7 / 2 | 19 / 30 / 51 | 15 / 39 / 15 |
| FN | 55 / 53 / 55 | 154 / 139 / 147 | 130 / 297 / 87 |
| IDSW | 1 / 0 / 0 | 4 / 4 / 2 | 3 / 5 / 2 |
| MOTA dynamic_only | 90.1 / 91.1 / 90.7 | 86.4 / 86.8 / 86.4 | 88.4 / 86.1 / 88.0 |
| recall D | 94.7 / 95.5 / 94.5 | 91.7 / 92.8 / 92.6 | 93.6 / 92.6 / 93.0 |
| latency med / p90 (ms) | 93.5 / 122 | 78 / 105 | 79.5 / 105 |

Reading: carrier closes the lab_u FP (211-334 -> 19-51; lab_u bare from lab_unified: MOTA 83.5-85.4). A: dynamic_only +1
on average. Residual gap = FN on the STANDING person: 1950 ~70 FN in 5 of 6 lab_u runs (indoor 2), 1947 intermittent
(LUCAb 180), 1952 27-28 for all profiles. The extra lifecycle replay (lab_uca 1947) did not catch a class clear.
