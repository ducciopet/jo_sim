#!/usr/bin/env python3
"""
Baselines that only publish RViz boxes (visualization_msgs/MarkerArray, one LINE_LIST per box) -> unified tracks.

Used for LV-DOT (/onboard_detector/dynamic_bboxes) and Dynablox (/motion_detector/clusters). The box centre and size
come from the LINE_LIST points (axis-aligned boxes; LV-DOT also sets the centre in pose.position); other marker types
(e.g. Dynablox's text labels) are skipped.

- time: the header stamp when set (Dynablox), else the bag receive time (LV-DOT leaves the stamp empty; recorded with
  use_sim_time, so it is the replayed sensor clock).
- ids: persistent_ids=True keeps the marker id as the track id (Dynablox, with the output-only patch in
  eval/baselines/dynablox/patches/ that sets it to the tracked cluster id); False gives every box its own id
  (LV-DOT: its marker id is only the index in the message), so identity metrics are not applicable.
- state: every box is labelled `state` ('dynamic': these methods publish only what they flag as moving).
"""

from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

from harness.tracks import EMPTY_FRAME_ID, empty_tracks, make_tracks


def load_marker_boxes(bag, topic, persistent_ids=False, state='dynamic'):
    rows, nid = [], 0
    with AnyReader([Path(bag)]) as r:
        cs = [c for c in r.connections if c.topic == topic]
        if not cs:
            # Dynablox publishes only non-empty results: a recorded run with no message on the topic (other topics
            # present, e.g. the odometry) means it flagged nothing at all, not a failed run
            if any(c.msgcount for c in r.connections):
                return empty_tracks()
            raise ValueError(f'{bag}: no {topic} and no other message')
        for c, t, raw in r.messages(cs):
            m = r.deserialize(raw, c.msgtype)
            boxes = [k for k in m.markers if k.action == 0 and k.type == 5 and len(k.points) >= 8]  # 5 = LINE_LIST
            stamp = None
            if boxes:
                s = boxes[0].header.stamp
                stamp = s.sec * 1_000_000_000 + s.nanosec if (s.sec or s.nanosec) else None
            ts = stamp if stamp is not None else t
            if not boxes:
                rows.append(dict(t_ns=ts, track_id=EMPTY_FRAME_ID))
                continue
            for k in boxes:
                pts = np.array([[p.x, p.y, p.z] for p in k.points])
                lo, hi = pts.min(axis=0), pts.max(axis=0)
                centre = (lo + hi) / 2 + np.array([k.pose.position.x, k.pose.position.y, k.pose.position.z])
                tid = int(k.id) if persistent_ids else nid
                nid += 1
                rows.append(dict(t_ns=ts, track_id=tid, x=centre[0], y=centre[1], z=centre[2],
                                 l=hi[0] - lo[0], w=hi[1] - lo[1], h=hi[2] - lo[2], yaw=0.0, vx=0.0, vy=0.0, vz=0.0,
                                 state=state, score=1.0, **{'class': ''}))
    return make_tracks(rows)
