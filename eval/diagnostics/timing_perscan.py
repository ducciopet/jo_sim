# Per scan (by header stamp): latency (rec - stamp) of fused detections and of tracks, and their counts. Read-only.
import sys, json, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run, out = sys.argv[1].rstrip('/'), sys.argv[2]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
T = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {'/sins/dbscan_detector/fused/detections3d': 'det', '/sins/tracker/tracks': 'trk', '/perception/tracked_dynamic_obstacles': 'obs'}
D = {}
while r.has_next():
    n, d, t = r.read_next()
    if n not in want: continue
    m = deserialize_message(d, get_message(T[n])); h = m.header.stamp; s = h.sec * 10**9 + h.nanosec
    arr = next((getattr(m, a) for a in ('detections', 'tracks', 'obstacles') if hasattr(m, a)), [])
    e = D.setdefault(s, {}); e[want[n] + '_lat'] = (t - s) / 1e6; e[want[n] + '_n'] = len(arr)
json.dump(sorted([dict(stamp=k, **v) for k, v in D.items()], key=lambda x: x['stamp']), open(out, 'w'))
print(run, len(D))
