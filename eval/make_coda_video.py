#!/usr/bin/env python3
"""
Replay video of one CODa window (eval/datasets/coda_to_bag.py): what SINS sees and what it outputs, next to a
LiDAR-only baseline.

Top: front camera with YOLO's 2D boxes (thin, grey) and SINS's tracks projected as 3D boxes, coloured by state
(red = moving D, amber = still but able to move PD), with the track id. Bottom: bird's-eye view around the robot
(odom frame, robot-centred, heading up): the current LiDAR scan, the annotated agents (green outlines, CODa GT),
SINS tracks (left) and the baseline's detections (right, Dynablox by default).

usage: make_coda_video.py --window w1 --run bags/coda/runs/B1_v2_coda0_w1 \
          [--baseline bags/dynablox_out/dynabloxa_coda0_w1.bag --baseline-method dynablox] --out eval/videos/coda_w1.mp4
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont
from rosbags.highlevel import AnyReader

E = Path(__file__).resolve().parent
sys.path.insert(0, str(E))
sys.path.insert(0, str(E / 'adapters'))
from coda_eval import BASELINES                                     # noqa: E402
from harness.ros_io import load_obstacles                           # noqa: E402
from harness.tracks import objects                                  # noqa: E402
from marker_adapter import load_marker_boxes                        # noqa: E402
from points_adapter import load_dynamic_points                      # noqa: E402

COL = {'dynamic': (230, 57, 70), 'potentially_dynamic': (244, 162, 97), 'static': (140, 140, 140)}
GT_COL, BASE_COL, BG = (60, 200, 110), (80, 150, 255), (18, 20, 24)


def quat_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_mat(tr):
    T = np.eye(4)
    T[:3, :3] = quat_rot(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def stamp(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def read_data(bag):
    imgs, clouds, statics, K = [], [], {}, None
    with AnyReader([Path(bag)]) as r:
        want = {'/front_camera/camera/color/image_raw', '/velodyne_points', '/tf_static',
                '/front_camera/camera/color/camera_info'}
        for c, _, raw in r.messages([c for c in r.connections if c.topic in want]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/tf_static':
                for tr in m.transforms:
                    statics[(tr.header.frame_id, tr.child_frame_id)] = tf_mat(tr.transform)
            elif c.topic.endswith('camera_info'):
                K = np.array(m.k).reshape(3, 3)
            elif c.topic.endswith('image_raw'):
                imgs.append((stamp(m.header), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)))
            else:
                n = m.width * m.height
                xyz = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)[:, :12].copy().view(np.float32)
                clouds.append((stamp(m.header), xyz[::3]))
    return imgs, clouds, statics, K


def read_odom(bag):
    t, T = [], []
    with AnyReader([Path(bag)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            M = np.eye(4)
            M[:3, :3], M[:3, 3] = quat_rot(q.x, q.y, q.z, q.w), [p.x, p.y, p.z]
            t.append(stamp(m.header))
            T.append(M)
    return np.array(t), np.array(T)


def read_yolo(bag):
    out = {}
    with AnyReader([Path(bag)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic.endswith('/detections')
                                     and c.msgtype == 'vision_msgs/msg/Detection2DArray']):
            m = r.deserialize(raw, c.msgtype)
            out[stamp(m.header)] = [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y)
                                    for d in m.detections]
    return out


def nearest(ts, t):
    k = int(np.clip(np.searchsorted(ts, t), 1, len(ts) - 1))
    return k - 1 if abs(ts[k - 1] - t) < abs(ts[k] - t) else k


def corners(x, y, z, l, w, h, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    dx = np.array([1, 1, -1, -1, 1, 1, -1, -1]) * l / 2
    dy = np.array([1, -1, -1, 1, 1, -1, -1, 1]) * w / 2
    dz = np.array([-1, -1, -1, -1, 1, 1, 1, 1]) * h / 2
    return np.stack([x + c * dx - s * dy, y + s * dx + c * dy, z + dz], 1)


EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def frame_rows(df, t, tol=0.15e9):
    """rows of the stream's frame nearest to t (within tol)."""
    if df.empty:
        return df
    ts = np.unique(df['t_ns'].to_numpy())
    k = nearest(ts, t) if len(ts) > 1 else 0
    return df[df['t_ns'] == ts[k]] if abs(ts[k] - t) < tol else df.iloc[:0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--window', required=True)
    ap.add_argument('--run', required=True)
    ap.add_argument('--baseline')
    ap.add_argument('--baseline-method', default='dynablox')
    ap.add_argument('--baseline-name', default='Dynablox')
    ap.add_argument('--bev-range', type=float, default=20.0)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--t0', type=float, default=0.0, help='start [s] into the window')
    ap.add_argument('--dur', type=float, default=1e9)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    R = E.parent / 'bags/coda'
    imgs, clouds, st, K = read_data(R / f'coda0_{a.window}')
    ot, oT = read_odom(R / f'coda0_{a.window}_odom')
    T_base_vel = st[('base_link', 'velodyne')]
    T_vel_cam = st[('velodyne', 'front_camera_depth_optical_frame')] @ st[('front_camera_depth_optical_frame',
                                                                          'front_camera_color_optical_frame')]
    tracks, _, _ = load_obstacles(a.run)
    tracks = objects(tracks)
    yolo = read_yolo(a.run)
    yts = np.array(sorted(yolo))
    gt = pd.read_csv(R / f'coda0_{a.window}_gt.csv', keep_default_na=False)
    gt = gt[gt['track_id'].astype(int) >= 0]
    for k in ('x', 'y', 'z', 'l', 'w', 'h', 'yaw'):
        gt[k] = gt[k].astype(float)
    base = None
    if a.baseline:
        topic, ids = BASELINES[a.baseline_method]
        base = objects(load_dynamic_points(a.baseline, topic, min_points=20) if ids is None
                       else load_marker_boxes(a.baseline, topic, persistent_ids=ids))
    cts = np.array([c[0] for c in clouds])
    t_start = imgs[0][0] + a.t0 * 1e9
    imgs = [im for im in imgs if t_start <= im[0] <= t_start + a.dur * 1e9]

    W = a.width
    H0 = imgs[0][1].shape[0]
    sc = W / imgs[0][1].shape[1]
    Hc = int(round(H0 * sc))
    nb = 2 if base is not None else 1
    bw = W // nb
    bh = bw if nb == 2 else W // 2
    Hh = (Hc + bh) // 2 * 2
    try:
        font = ImageFont.truetype('DejaVuSans-Bold.ttf', 18)
        small = ImageFont.truetype('DejaVuSans.ttf', 14)
    except OSError:
        font = small = ImageFont.load_default()
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{Hh}',
                           '-r', str(a.fps), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '28',
                           '-preset', 'slow', '-movflags', '+faststart', a.out], stdin=subprocess.PIPE)
    for n, (t, img) in enumerate(imgs):
        T_odom_base = oT[nearest(ot, t)]
        T_odom_cam = T_odom_base @ T_base_vel @ T_vel_cam
        T_cam_odom = np.linalg.inv(T_odom_cam)
        frame = Image.new('RGB', (W, Hh), BG)
        cam = Image.fromarray(img).resize((W, Hc))
        d = ImageDraw.Draw(cam)
        if len(yts):
            k = nearest(yts, t)
            if abs(yts[k] - t) < 0.2e9:
                for cx, cy, sx, sy in yolo[yts[k]]:
                    d.rectangle([(cx - sx / 2) * sc, (cy - sy / 2) * sc, (cx + sx / 2) * sc, (cy + sy / 2) * sc],
                                outline=(210, 210, 210), width=1)
        tr = frame_rows(tracks, t)
        for _, o in tr.iterrows():
            P = corners(o.x, o.y, o.z, max(o.l, .3), max(o.w, .3), max(o.h, .3), o.yaw)
            Pc = (T_cam_odom[:3, :3] @ P.T).T + T_cam_odom[:3, 3]
            if (Pc[:, 2] < 0.5).any():
                continue
            uv = (K @ Pc.T).T
            uv = uv[:, :2] / uv[:, 2:] * sc
            col = COL.get(o.state, COL['static'])
            for i, j in EDGES:
                d.line([tuple(uv[i]), tuple(uv[j])], fill=col, width=3)
            d.text((uv[:, 0].min(), uv[:, 1].min() - 20), f'#{int(o.track_id)}', fill=col, font=small)
        frame.paste(cam, (0, 0))
        # bird's-eye views, robot-centred, heading up
        ci = nearest(cts, t)
        pts = (T_odom_base @ T_base_vel @ np.c_[clouds[ci][1], np.ones(len(clouds[ci][1]))].T).T[:, :3]
        Rinv = np.linalg.inv(T_odom_base)

        def to_px(xy, x0):
            q = (Rinv[:2, :2] @ (xy - T_odom_base[:2, 3]).T).T          # base frame: x forward, y left
            s = bh / (2 * a.bev_range)
            return np.c_[x0 + bw / 2 - q[:, 1] * s, Hc + bh / 2 - q[:, 0] * s]

        g = frame_rows(gt, t, tol=0.6e9)
        panels = [('SINS', tr, None)] + ([(a.baseline_name, frame_rows(base, t), BASE_COL)] if base is not None else [])
        for p, (name, rows, fixed) in enumerate(panels):
            x0 = p * bw
            bev = ImageDraw.Draw(frame)
            keep = (np.abs(pts[:, 2] - T_odom_base[2, 3]) < 2.5)
            uv = to_px(pts[keep, :2], x0)
            ok = (uv[:, 0] >= x0) & (uv[:, 0] < x0 + bw) & (uv[:, 1] >= Hc) & (uv[:, 1] < Hh)
            for u, v in uv[ok]:
                bev.point((u, v), fill=(105, 110, 120))
            for _, o in g.iterrows():
                c4 = to_px(corners(o.x, o.y, o.z, o.l, o.w, o.h, o.yaw)[:4, :2], x0)
                bev.polygon([tuple(q) for q in c4], outline=GT_COL, width=2)
            for _, o in rows.iterrows():
                c4 = to_px(corners(o.x, o.y, o.z, max(o.l, .3), max(o.w, .3), 1, o.yaw)[:4, :2], x0)
                col = fixed or COL.get(o.state, COL['static'])
                bev.polygon([tuple(q) for q in c4], fill=col, outline=col)
            rob = to_px(T_odom_base[:2, 3][None], x0)[0]
            bev.polygon([(rob[0], rob[1] - 10), (rob[0] - 7, rob[1] + 7), (rob[0] + 7, rob[1] + 7)], fill=(255, 255, 255))
            bev.text((x0 + 12, Hc + 10), name, fill=(255, 255, 255), font=font)
            if p:
                bev.line([(x0, Hc), (x0, Hh)], fill=(60, 64, 72), width=2)
        lg = ImageDraw.Draw(frame)
        lx, ly = 12, Hh - 28
        for lab, col in (('in movimento (D)', COL['dynamic']), ('fermo, può muoversi (PD)', COL['potentially_dynamic']),
                         ('annotato in CODa', GT_COL)) + ((('rilevato da ' + a.baseline_name, BASE_COL),) if base is not None else ()):
            lg.rectangle([lx, ly + 3, lx + 14, ly + 17], fill=col)
            lg.text((lx + 20, ly), lab, fill=(235, 235, 235), font=small)
            lx += 30 + int(lg.textlength(lab, font=small))
        lg.text((W - 90, Hc + 10), f'{(t - imgs[0][0]) / 1e9:5.1f} s', fill=(235, 235, 235), font=small)
        ff.stdin.write(np.asarray(frame).tobytes())
        if n % 100 == 0:
            print(f'{n}/{len(imgs)}', flush=True)
    ff.stdin.close()
    ff.wait()
    print(a.out)


if __name__ == '__main__':
    main()
