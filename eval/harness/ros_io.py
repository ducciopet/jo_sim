"""Bag readers (rosbags, no ROS install needed): detector outputs, odometry, mocap.

All timestamps are message header stamps [ns] (sim/bag time), never record time.
"""

from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from .tracks import EMPTY_FRAME_ID, JO_STATUS, make_tracks

REPO = Path(__file__).resolve().parents[2]
JO_MSGS_DIR = REPO / 'jo_msgs' / 'msg'

_MOCAP_MSGS = {
    'mocap_msgs/msg/Marker': """int8 USE_NAME=0
int8 USE_INDEX=1
int8 USE_BOTH=2
int8 id_type
int32 marker_index
string marker_name
geometry_msgs/Point translation""",
    'mocap_msgs/msg/RigidBody': """string rigid_body_name
mocap_msgs/Marker[] markers
geometry_msgs/Pose pose""",
    'mocap_msgs/msg/RigidBodies': """std_msgs/Header header
uint32 frame_number
mocap_msgs/RigidBody[] rigidbodies""",
}


def typestore():
    ts = get_typestore(Stores.ROS2_JAZZY)
    add = {}
    for f in JO_MSGS_DIR.glob('*.msg'):
        add.update(get_types_from_msg(f.read_text(), f'jo_msgs/msg/{f.stem}'))
    for name, text in _MOCAP_MSGS.items():
        add.update(get_types_from_msg(text, name))
    ts.register(add)
    return ts


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def yaw_from_quat(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _connections(reader, topic=None, msgtype=None, suffix=None):
    cs = [c for c in reader.connections
          if (topic is None or c.topic == topic)
          and (msgtype is None or c.msgtype == msgtype)
          and (suffix is None or c.topic.endswith(suffix))]
    return cs


def load_obstacles(bag, topic=None):
    """jo_msgs/ObstacleArray stream -> unified tracks (frame: the messages' header frame).

    topic=None picks the single ObstacleArray topic ending in 'tracked_dynamic_obstacles'
    (v1: /jo_detector/..., v2: /perception/...). Returns (tracks, frame_id, topic).
    """
    rows, frame_ids = [], set()
    with AnyReader([Path(bag)], default_typestore=typestore()) as r:
        cs = _connections(r, topic=topic, msgtype='jo_msgs/msg/ObstacleArray',
                          suffix=None if topic else 'tracked_dynamic_obstacles')
        if len(cs) != 1:
            raise ValueError(f'{bag}: expected one ObstacleArray topic, found {[c.topic for c in cs]}')
        for c, _, raw in r.messages(cs):
            m = r.deserialize(raw, c.msgtype)
            t = stamp_ns(m.header)
            frame_ids.add(m.header.frame_id)
            if not m.obstacles:
                rows.append(dict(t_ns=t, track_id=EMPTY_FRAME_ID))
                continue
            for o in m.obstacles:
                rows.append(dict(
                    t_ns=t, track_id=int(o.track_id),
                    x=o.pose.position.x, y=o.pose.position.y, z=o.pose.position.z,
                    l=o.size.x, w=o.size.y, h=o.size.z, yaw=yaw_from_quat(o.pose.orientation),
                    vx=o.twist.linear.x, vy=o.twist.linear.y, vz=o.twist.linear.z,
                    state=JO_STATUS.get(int(o.status), 'static'), score=float(o.track_age),
                    **{'class': ''}))
        found_topic = cs[0].topic
    if len(frame_ids) > 1:
        raise ValueError(f'{bag}: obstacles published in several frames {frame_ids}')
    return make_tracks(rows), (frame_ids.pop() if frame_ids else ''), found_topic


def load_odometry(bag, topic='/odometry/filtered'):
    """nav_msgs/Odometry -> array [t_s, x, y, yaw] (child frame in header frame)."""
    out = []
    with AnyReader([Path(bag)], default_typestore=typestore()) as r:
        for c, _, raw in r.messages(_connections(r, topic=topic)):
            m = r.deserialize(raw, c.msgtype)
            p = m.pose.pose
            out.append((stamp_ns(m.header) / 1e9, p.position.x, p.position.y, yaw_from_quat(p.orientation)))
    a = np.array(out)
    return a[np.argsort(a[:, 0])] if len(a) else a.reshape(0, 4)


def load_mocap(bag, topic='/rigid_bodies'):
    """mocap4r2 RigidBodies -> {body name: array [t_s, x, y, z, yaw]}, invalid (NaN) samples dropped."""
    bodies = {}
    with AnyReader([Path(bag)], default_typestore=typestore()) as r:
        for c, _, raw in r.messages(_connections(r, topic=topic)):
            m = r.deserialize(raw, c.msgtype)
            t = stamp_ns(m.header) / 1e9
            for rb in m.rigidbodies:
                p, q = rb.pose.position, rb.pose.orientation
                if not np.isfinite([p.x, p.y, p.z, q.w]).all():
                    continue
                bodies.setdefault(rb.rigid_body_name, []).append((t, p.x, p.y, p.z, yaw_from_quat(q)))
    return {k: np.array(sorted(v)) for k, v in bodies.items()}
