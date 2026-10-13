# v2 (jo_detector_v2) on `own` (lab sequences, bags/bags_benchmark, Qualisys GT)

Launch: `ros2 launch jo_detector_v2 run_detector.launch.py rviz:=false camera_names:=front_camera raw_depth_transport:=raw`
(env `indoor`, default), plus `eval/adapters/realsense_bag_adapter.py` running alongside.

Only sensor-I/O settings differ from the robot's defaults (BENCHMARK_PLAN.md, rule 3):

| Setting | Robot default | Here | Why |
|---|---|---|---|
| `camera_names` | front + back | front only | the 2026-05-19 bags have the front camera only |
| `raw_depth_transport` | `compressedDepth` | `raw` | raw depth recorded uncompressed |

Input adapter (`eval/adapters/realsense_bag_adapter.py`), because these bags lack three streams v2 reads:

| Stream v2 expects | Derived from | Exactness |
|---|---|---|
| `extrinsics/depth_to_color` | the bag's static TF | identical (1e-6) to the driver's message, checked on `20260910_124939_validation_lab` which has both |
| `color/image_raw/compressed` | `color/image_raw` | JPEG q=95 re-encoding |
| `depth/color/points` | `depth/image_rect_raw` + depth `camera_info` | plain deprojection in `depth_optical_frame` (what the RealSense pointcloud filter publishes, minus color) |

v1 reads the raw depth image directly and needs none of these; the adapter adds no information.

Code change needed (I/O only; jo_detector_v2 `ba0353a`): `camera_names` launch override.
