# Track latency of one run: recorded (sim-clock) time minus header stamp of /sins/tracker/tracks.
# 19/11 street fix: the sim clock of `ros2 bag play --clock` can run BEHIND the scan stamps (street: preprocessing logs
# "scan age at processing start" ~ -8.5 ms, CODa ~ -1 ms), which makes rec - stamp under-read the latency. A negative
# age is not physical (a scan cannot be processed before it was taken), so when <run>_logs/det.log reports a negative
# median age, it is added back: lat = (rec - stamp) - min(0, median age). The raw value is printed too.
import sys, re, os, statistics as st, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run = sys.argv[1].rstrip('/')
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
T={t.name:t.type for t in r.get_all_topics_and_types()}; L=[]; S=[]
while r.has_next():
    n,d,t=r.read_next()
    if n=='/sins/tracker/tracks':
        m=deserialize_message(d,get_message(T[n])); h=m.header.stamp; s=h.sec*1e9+h.nanosec; L.append((t-s)/1e6); S.append(s)
ages=[]
log=run+'_logs/det.log'
if os.path.exists(log):
    for line in open(log, errors='ignore'):
        g=re.search(r'scan age at processing start (-?[\d.]+) ms', line)
        if g: ages.append(float(g.group(1)))
off=-min(0.0, st.median(ages)) if ages else 0.0
L.sort(); C=[x+off for x in L]
print(f"tracce {len(L)}  latenza mediana {st.median(C):.0f} ms  p90 {C[int(.9*len(C))]:.0f} ms  stamp {(max(S)-min(S))/1e9:.1f}s"
      f"  (grezza {st.median(L):.0f}/{L[int(.9*len(L))]:.0f}, clock offset +{off:.1f} ms)")
