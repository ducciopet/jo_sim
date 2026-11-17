#!/usr/bin/env python3
"""Per-frame YOLO time inside the pipeline, from a recorded run: std_msgs/Float64 [s] on every */inference_time topic
(yolo_seg_track: instance segmentation, the single YOLO with yolo_leaf_method mask; yolo_semantic_seg: the box/semantic
node). Prints one line per topic: n, median, p90 [ms].  usage: yolo_time.py <run bag dir>"""
import sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

with AnyReader([Path(sys.argv[1])]) as r:
    for c in [c for c in r.connections if c.topic.endswith(('inference_time', 'detection_time'))]:
        v = np.array([r.deserialize(raw, c.msgtype).data for _, _, raw in r.messages([c])]) * 1e3
        if len(v):
            print(f'{c.topic} n={len(v)} med {np.median(v):.1f} p90 {np.percentile(v, 90):.1f} ms')
