import sys, statistics as st, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
T={t.name:t.type for t in r.get_all_topics_and_types()}; L={}
while r.has_next():
    n,d,t=r.read_next()
    if n in('/tf','/tf_static'): continue
    try: h=deserialize_message(d,get_message(T[n])).header.stamp
    except Exception: continue
    L.setdefault(n,[]).append((t-(h.sec*1e9+h.nanosec))/1e6)
for n,v in sorted(L.items()): v.sort(); print(f"{n:55s} n={len(v):4d} med {st.median(v):6.0f}  p90 {v[int(.9*len(v))]:6.0f} ms")
