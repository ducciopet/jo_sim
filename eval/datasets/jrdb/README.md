# JRDB: held-out benchmark for SINS (from 02/11/2026)

Data: `/mnt/kloster-reichenau/datasets_SINS/JRDB/train_dataset_with_activity` (JRDB 2022 train, 27 sequences).
This release has two VLP-16 (`.pcd`, binary_compressed), five rectified color cameras of the upper ring
(`images/image_0,2,4,6,8`, 752x480) and 3D person boxes (`labels/labels_3d`, ego frame, box centre). It has **no depth,
no odometry and no timestamps**.

## Choices (made before any SINS run on these sequences)
- **Frame rate 15 Hz**: the VLP-16 azimuth step in the .pcd is 0.3 deg = 15 rev/s; every frame has its own scan.
- **LiDAR = upper VLP-16 only**: the robot's own sensor; base profiles unchanged (2 deg rings, 15 m crop).
  The lower VLP-16 is used only for the odometry.
- **Odometry**: KISS-ICP once on upper+lower (`jrdb_to_bag.py`, voxel 0.3 m, no deskew), shared by every method.
  Check on clark-center_0: still people drift 3-5 cm/s in odom, walkers 1.3-1.8 m/s.
- **Depth**: the scan projected ONLINE into each camera (`eval/adapters/lidar_depth_node.py`, 14 ms/scan for 5
  cameras), so `depth_max_value` = 15 m (the LiDAR crop) in `eval/configs/sins/jrdb_*/overrides.yaml`.
- **Cameras**: all five (360 deg), one YOLO each (7.6 Hz each with the base stride 2, no drops on the RTX 4060).
- **Profile**: indoor sequences -> `jrdb_indoor` (base cfg/indoor), outdoor -> `jrdb_outdoor` (base cfg/outdoor),
  from the images.
- **Sequences** (`convert_jrdb.sh`): the six with the most still people within 15 m by JRDB's own action labels
  (sitting/standing vs walking), indoor: forbes-cafe_0, huang-basement_0, packard-poster-session_2, tressider-2019-04-26_2;
  outdoor: clark-center_0, clark-center_1. bytes-cafe_0 (used to check the calibration) is the debug sequence, never scored.
- **Evaluation**: `eval/coda_eval.py --range 15` (same rules as CODa: still people only inside a camera image, >= 1
  upper-LiDAR return, 1.2-15 m; JRDB `no_eval` boxes are ignored). Pool: `python3 eval/coda_pool.py --jrdb <TAG>`.

## Commands
    setsid nohup bash eval/datasets/jrdb/convert_jrdb.sh > bags/jrdb/convert.log 2>&1 < /dev/null &   # ~1 GB/sequence
    bash eval/scripts/jrdb_ab.sh <TAG> [forbes huangb packard2 tressider2 clark0 clark1]
(`run_jrdb.sh`, `run_method.sh`, `lidar_depth_node.py` must also be copied to `bags/replay/`.)
