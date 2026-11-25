# Final SINS profiles (DRAFT 25/11; final md5s recomputed at the paper-v1 tag on 27/11, before any final run)

sins 4d7d5c3 (c5-semantic-birth), jo_sim c9543605. md5 = md5 over the sorted relative paths + contents of sins/cfg/<profile>/ (as make_manifest.py).

| profile | used for | md5 of the generated cfg folder |
|---|---|---|
| coda_vjsye2i2pcar1lccbma | CODa tuning, CODa 11/12, CODa new 2/3/18 (method profile) | 7d3af8d35c16840eaf031c767d4092d8 |
| kitti_tracking | KITTI tracking (sensor overrides on the method profile) | 38073224e98c9b16e48cec2cd14d1ca3 |
| doals | DOALS (LiDAR-only, sensor overrides on the method profile) | 488aa7e3acd968e587220ea8d3151940 |
| jrdb | JRDB (sensor overrides + 15 Hz scan counts x1.5, Python round half-to-even, 19 keys) | 4b218a70ac7e5257184f5a51e7e49bbb |
| lab_ucar1lccbma | Lab Qualisys (ground_model planes: declared per-environment option, lab_queue 37) | f0b02e26c7abe8d25237b09aea98fc53 |

Not reported: street (user 25/11). Not adopted, default off in the code: track_extent_assoc_margin (chain G), fragment_merge_vehicle_anchor (chain H), ablation switches (FINAL_RUN_SPEC 6).
