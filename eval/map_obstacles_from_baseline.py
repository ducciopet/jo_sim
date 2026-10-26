#!/usr/bin/env python3
"""
Map experiment with a literature baseline as the obstacle source: its recorded boxes (eval/adapters/marker_adapter.py,
odom frame, every box flagged as moving) rewritten as the jo_msgs/ObstacleArray stream GLIM's BBOX rejection reads
(/perception/tracked_dynamic_obstacles), one message per output frame, at the frame's own time, so it replays on the
benchmark bag's clock. Status DYNAMIC for every box (these methods publish only what they flag as moving), track id
= the method's own id when it has one, velocity zero (none is published: no velocity inflation of the box).

usage: map_obstacles_from_baseline.py <method: dynablox|lvdot> <seq e.g. 20260519_1950> [rep a]
writes bags/replay/obs_<method><rep>_<seq> (rosbag2, sqlite3)
"""

import shutil
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

sys.path.insert(0, str(Path(__file__).resolve().parent))
from adapters.marker_adapter import load_marker_boxes  # noqa: E402
from harness.tracks import EMPTY_FRAME_ID  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
SOURCES = {'dynablox': ('dynablox_out', 'dynablox', '/motion_detector/visualization/clusters', True),
           'lvdot': ('lvdot_out', 'lvdot', '/onboard_detector/dynamic_bboxes', False)}


def typestore():
    ts = get_typestore(Stores.ROS2_JAZZY)
    d = REPO / 'jo_msgs' / 'msg'
    types = {}
    for name in ('Obstacle', 'ObstacleArray'):
        types.update(get_types_from_msg((d / f'{name}.msg').read_text(), f'jo_msgs/msg/{name}'))
    ts.register(types)
    return ts


def main():
    method, seq = sys.argv[1], sys.argv[2]
    rep = sys.argv[3] if len(sys.argv) > 3 else 'a'
    d, prefix, topic, ids = SOURCES[method]
    boxes = load_marker_boxes(REPO / 'bags' / d / f'{prefix}{rep}_{seq}.bag', topic, persistent_ids=ids)
    out = REPO / 'bags' / 'replay' / f'obs_{method}{rep}_{seq}'
    shutil.rmtree(out, ignore_errors=True)
    ts = typestore()
    Obstacle, ObstacleArray = ts.types['jo_msgs/msg/Obstacle'], ts.types['jo_msgs/msg/ObstacleArray']
    T = ts.types
    Header, Time = T['std_msgs/msg/Header'], T['builtin_interfaces/msg/Time']
    Pose, Point, Quat = T['geometry_msgs/msg/Pose'], T['geometry_msgs/msg/Point'], T['geometry_msgs/msg/Quaternion']
    Vec3, Twist, Accel = T['geometry_msgs/msg/Vector3'], T['geometry_msgs/msg/Twist'], T['geometry_msgs/msg/Accel']
    zero = lambda: Vec3(x=0.0, y=0.0, z=0.0)
    with Writer(out, version=8) as w:
        conn = w.add_connection('/perception/tracked_dynamic_obstacles', 'jo_msgs/msg/ObstacleArray', typestore=ts)
        for t_ns, g in boxes.groupby('t_ns'):
            stamp = Time(sec=int(t_ns // 1_000_000_000), nanosec=int(t_ns % 1_000_000_000))
            obs = []
            for _, b in g.iterrows():
                if int(b['track_id']) == EMPTY_FRAME_ID or not np.isfinite(b['x']):
                    continue
                obs.append(Obstacle(
                    header=Header(stamp=stamp, frame_id='odom'), track_id=int(b['track_id']) % 2**32,
                    track_age=1000, status=Obstacle.STATUS_DYNAMIC,
                    pose=Pose(position=Point(x=float(b['x']), y=float(b['y']), z=float(b['z'])),
                              orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0)),
                    size=Vec3(x=float(b['l']), y=float(b['w']), z=float(b['h'])),
                    twist=Twist(linear=zero(), angular=zero()), accel=Accel(linear=zero(), angular=zero())))
            msg = ObstacleArray(header=Header(stamp=stamp, frame_id='odom'), obstacles=obs)
            w.write(conn, int(t_ns), ts.serialize_cdr(msg, 'jo_msgs/msg/ObstacleArray'))
    print('wrote', out, len(boxes), 'rows')


if __name__ == '__main__':
    main()
