# v1 (jo_detector) on `own` (lab sequences, bags/bags_benchmark, Qualisys GT)

Launch: `ros2 launch jo_detector run_detector.launch.py use_sim_time:=true rviz:=false
depth_transport:=raw color_transport:=raw calibration_camera_info_topic:=/front_camera/camera/depth/camera_info`

Only sensor-I/O settings differ from the robot's defaults (BENCHMARK_PLAN.md, rule 3):

| Setting | Robot default | Here | Why |
|---|---|---|---|
| `depth_transport` | `compressedDepth` | `raw` | the 2026-05-19 bags recorded `depth/image_rect_raw` uncompressed |
| `color_transport` | `compressed` | `raw` | same, for `color/image_raw` (YOLO node: `image_transport:=raw`) |
| `calibration_camera_info_topic` | color `camera_info` | depth `camera_info` | the bags have only the raw (unaligned) depth image; with the color K the ICP calibration converged 0.5–1.1 m off and differently per sequence |

Code changes needed to run at all (I/O only, no logic change; jo_detector `e09610a`, `ee170f3`):
transports made parameters (were hard-coded), `cv_bridge` removed from the YOLO node (fails to import
next to numpy>=2 in the container), launch args above.

Config file: unchanged `cfg/detector_param_jo_zotac_outdoor.yaml` (the one `run_detector.launch.py` loads).
