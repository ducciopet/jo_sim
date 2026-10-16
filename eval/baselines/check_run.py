#!/usr/bin/env python3
"""
Guard against a broken real-time replay: a run is valid only if its recorded bag holds (almost) all the shared
odometry and spans the whole sequence. Disk contention (e.g. converting a 5 GB input while another bag is replayed)
makes rosbag play fall behind: messages arrive in bursts and the method sees a shorter, compressed sequence.
usage: check_run.py <recorded.bag> <seq e.g. 20260519_1950>   (exit 1 if invalid)
"""
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader

R = Path(__file__).resolve().parents[2]
MIN_FRACTION = 0.97


def odom(bag):
    # the shared odometry; M-detector's runs record its per-cloud /odom_velodyne instead (one per scan, so only the
    # time span is compared). An empty connection list would make rosbags read every topic: checked explicitly.
    with AnyReader([Path(bag)]) as r:
        for topic in ('/odometry/filtered', '/odom_velodyne'):
            cs = [c for c in r.connections if c.topic == topic]
            if cs:
                ts = [t for _, t, _ in r.messages(cs)]
                return topic, len(ts), (ts[-1] - ts[0]) / 1e9
    return None, 0, 0.0


rec, seq = sys.argv[1], sys.argv[2]
_, n_ref, d_ref = odom(R / 'bags' / 'replay' / f'odom_{seq}')
topic, n, d = odom(rec)
ok = topic is not None and d >= MIN_FRACTION * d_ref and (topic != '/odometry/filtered' or n >= MIN_FRACTION * n_ref)
print(f'{"OK" if ok else "INVALID"} {rec}: {topic} {n}/{n_ref} msgs, {d:.1f}/{d_ref:.1f} s')
sys.exit(0 if ok else 1)
