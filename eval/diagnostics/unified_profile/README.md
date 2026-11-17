# Unified profile: indoor (lab) vs reference coda_vjsye2i2p, parameter by parameter (17/11)

`diff_indoor_vs_i2p.tsv`: every parameter whose value differs between sins/cfg/indoor and the generated
sins/cfg/coda_vjsye2i2p (97; `<absent>` = not in the file, code default). Proposed class per row:
- SCALA/SENSORE (47): keeps the indoor value (eps, voxel, range, box limits, ICP, wall/RANSAC, inference size, cameras/topics, depth sensor, camera frame stride).
- COMPORTAMENTO (41): takes the I2P value.
- OPZIONALE (suolo) (5): `preprocessing_node.ground_model` + ground_grid_*: lab planes, CODa/street grid (I2P unchanged).
- COMPORTAMENTO ? (4): per-class min heights (yolo_class_min_height_*, track_class_min_height_*): to confirm against "no per-type thresholds".
Launch level (not in the YAML): enable_calibration (lab true, CODa false) and STEREO / cloud_stride (CODa only) are sensor/extrinsics.
Classification to be confirmed by the laptop session before any lab run.
