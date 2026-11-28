# Per run bag (read-only): share of fused detections by source mask (camera-only = no 'L', with camera = any non-L),
# share carrying a YOLO class, fused-detection latency (rec - stamp, median/p90), and per camera the YOLO 2D
# detection latency (rec - image stamp, median/p90). Output: one JSON line per run. usage: nw_fusion_stats.py run1 run2 ...
import sys, json, rosbag2_py, statistics as st
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def pct(v, q):
    v = sorted(v); return v[min(len(v) - 1, int(q * len(v)))] if v else None
for run in sys.argv[1:]:
    run = run.rstrip('/')
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    T = {t.name: t.type for t in r.get_all_topics_and_types()}
    det = '/sins/dbscan_detector/fused/detections3d'
    n = cam = camonly = ycls = 0; dl = []; yl = {}
    while r.has_next():
        tp, d, t = r.read_next()
        if tp == det or (tp.startswith('/yolo') and tp.endswith('/detections')):
            m = deserialize_message(d, get_message(T[tp])); s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if tp == det:
                dl.append((t - s) / 1e6)
                for x in m.detections:
                    if not x.results: continue
                    mk = x.results[0].hypothesis.class_id.split('+'); n += 1
                    cam += any(k != 'L' for k in mk); camonly += 'L' not in mk; ycls += len(x.results) > 1
            else:
                yl.setdefault(tp.split('/')[2], []).append((t - s) / 1e6)
    print(json.dumps(dict(run=run, dets=n, with_cam=cam / max(1, n), cam_only=camonly / max(1, n), yolo_class=ycls / max(1, n),
                          det_lat_med=st.median(dl) if dl else None, det_lat_p90=pct(dl, 0.9),
                          yolo={k: dict(n=len(v), med=st.median(v), p90=pct(v, 0.9)) for k, v in yl.items()})), flush=True)
