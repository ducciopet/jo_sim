import sys, statistics as st, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
T={t.name:t.type for t in r.get_all_topics_and_types()}; L=[]; S=[]
while r.has_next():
    n,d,t=r.read_next()
    if n=='/sins/tracker/tracks':
        m=deserialize_message(d,get_message(T[n])); h=m.header.stamp; s=h.sec*1e9+h.nanosec; L.append((t-s)/1e6); S.append(s)
L.sort(); print(f"tracce {len(L)}  latenza mediana {st.median(L):.0f} ms  p90 {L[int(.9*len(L))]:.0f} ms  stamp {(max(S)-min(S))/1e9:.1f}s")
