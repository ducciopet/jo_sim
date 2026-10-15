#!/usr/bin/env python3
"""
LV-DOT output (ROS1 bag, visualization_msgs/MarkerArray of LINE_LIST boxes in frame "map") -> unified tracks.

LV-DOT publishes /onboard_detector/dynamic_bboxes (boxes it classifies as dynamic) and /onboard_detector/tracked_bboxes
(every tracked box) as RViz markers: one LINE_LIST per box, centre in pose.position (x, y), the 8 corners in
points. The markers carry no header stamp and their id is only the index in that message, so:
  - time = the bag's receive time of the message (recorded with use_sim_time, i.e. the replayed sensor clock);
  - track_id = a new id for every box of every message: LV-DOT exposes no persistent identity, so identity metrics
    (IDF1, ID switches) are not applicable to it and only detection metrics (MODA, recall, false alarms) are compared.
LV-DOT has no "potentially dynamic" state: dynamic boxes are labelled 'dynamic'.
Its "map" frame is the odometry frame it was fed (the shared /odometry/filtered, frame odom).

Usage (library): load_lvdot(bag, topic='/onboard_detector/dynamic_bboxes') -> tracks DataFrame
"""

from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

from harness.tracks import EMPTY_FRAME_ID, make_tracks


def load_lvdot(bag, topic='/onboard_detector/dynamic_bboxes', state='dynamic'):
    rows, nid = [], 0
    with AnyReader([Path(bag)]) as r:
        cs = [c for c in r.connections if c.topic == topic]
        if not cs:
            raise ValueError(f'{bag}: no {topic}')
        for c, t, raw in r.messages(cs):
            m = r.deserialize(raw, c.msgtype)
            boxes = [k for k in m.markers if k.action == 0 and len(k.points) >= 8]
            if not boxes:
                rows.append(dict(t_ns=t, track_id=EMPTY_FRAME_ID))
                continue
            for k in boxes:
                pts = np.array([[p.x, p.y, p.z] for p in k.points])
                size = pts.max(axis=0) - pts.min(axis=0)
                rows.append(dict(t_ns=t, track_id=nid, x=k.pose.position.x, y=k.pose.position.y, z=k.pose.position.z,
                                 l=size[0], w=size[1], h=size[2], yaw=0.0, vx=0.0, vy=0.0, vz=0.0,
                                 state=state, score=1.0, **{'class': ''}))
                nid += 1
    return make_tracks(rows)
