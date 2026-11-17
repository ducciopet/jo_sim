#!/usr/bin/env python3
"""
SiT scene zip (Bae et al., NeurIPS 2023: Clearpath Husky, 2x VLP-16, 5 color cameras 1920x1200, 10 Hz, 3D boxes with
track ids in a world frame) -> ROS2 bags in the shape SINS reads, plus the GT, as jrdb_to_bag.py does for JRDB.
Run with eval/datasets/.venv/bin/python (python-lzf, opencv).

Conventions checked on Crossroad_1 / Three_way_Intersection_3 / Hallway_1 before any SINS run (17/11):
  - ego_trajectory/<f>.txt = T_world_top (row-major 4x4): the world-frame boxes contain the top-LiDAR points mapped
    with it (box z = centre: 31k vs 17k points inside with z = bottom);
  - label_3d: class, "Class:id", h, l, w, x, y, z, yaw; the box heading in the world frame is -yaw (median angle to the
    motion direction of moving tracks 1-8 deg with -yaw, 17-63 deg with +yaw; points inside 34k vs 18k);
  - calib P<k>_intrinsic / P<k>_extrinsic = K and [R|t] top-LiDAR -> camera k+1 (cam_img/<k+1>, label_2d camera
    k+1: 56-98% of 2D boxes contain the projected 3D centre with the matching k, 0% with any other). The images are
    already rectified: the scan projects onto edges with K alone (P<k>_distortion is not applied; with it the 2D
    boxes match worse), so CameraInfo carries zero distortion.

Choices (made before any SINS run, as for JRDB):
  - LiDAR given to the methods: the TOP VLP-16 only (one VLP-16, as the lab robot and JRDB); 10 Hz, every frame its own
    scan, bag time = 1 s + frame / 10 Hz (SiT has no timestamps in the release).
  - odometry: SiT's own ego_trajectory (RTK/IMU + LiDAR), shared by every method; base_link = ground plane under the
    top LiDAR from the best RANSAC fit over the scene (coda_to_bag.ground_frame).
  - cameras: all five (360 deg), PNG re-encoded as JPEG q90 on /cam<k>/camera/color/image_raw/compressed; no depth in
    the bag: eval/adapters/lidar_depth_node.py projects the scan online, as for JRDB.
  - GT: every annotated class (Pedestrian, Pedestrain_sitting -> Pedestrian_sitting, Cyclist, Motorcyclist, Car, Bus,
    Truck), npts = top-LiDAR returns in the box, in_fov = box centre inside one of the five images; state left empty
    (derived at evaluation time from speed, coda_eval.py).

Writes <out>/ (sensor bag), <out>_odom/, <out>_gt.csv, <out>_gt.json.
usage: sit_to_bag.py <scene zip> <out> [--jpeg-quality 90]
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
from sit_io import Scene                                             # noqa: E402
from coda_to_bag import (LATCHED, M, TS, ground_frame, header,       # noqa: E402
                         quat_from_rot, transform_msg, cloud_msg)
from rosbags.rosbag2 import StoragePlugin, Writer                    # noqa: E402

RATE = 10.0
CAMS = [1, 2, 3, 4, 5]
W, H = 1920, 1200
CLASS_FIX = {'Pedestrain_sitting': 'Pedestrian_sitting'}


def info_msg(K, ns, frame):
    P = np.zeros(12)
    P[[0, 1, 2, 4, 5, 6, 10]] = K.ravel()[[0, 1, 2, 3, 4, 5, 8]]
    return M['sensor_msgs/msg/CameraInfo'](
        header=header(ns, frame), height=H, width=W, distortion_model='plumb_bob', d=np.zeros(5),
        k=K.ravel().astype(float), r=np.eye(3).ravel(), p=P, binning_x=0, binning_y=0,
        roi=M['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def overhead_fraction(scan):
    r = np.linalg.norm(scan[:, :3], axis=1)
    up = scan[(scan[:, 2] > 0) & (r > 1)]
    return float(np.mean(up[:, 2] > 2.0)) if len(up) else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('zip')
    ap.add_argument('out')
    ap.add_argument('--jpeg-quality', type=int, default=90)
    a = ap.parse_args()
    out = Path(a.out)
    for p in (out, Path(f'{out}_odom')):
        if p.exists():
            raise SystemExit(f'{p} exists')
    s = Scene(a.zip)
    if not s.has_labels:
        raise SystemExit(f'{a.zip}: no label_3d')
    frames = s.frames
    t_ns = {f: int(1_000_000_000 + j * 1e9 / RATE) for j, f in enumerate(frames)}
    calib = s.calib(frames[0])                       # identical in every frame (checked: calib/<f>.txt per frame)
    T_w_top = {f: s.pose(f) for f in frames}
    scans = {f: s.cloud(f) for f in frames}

    fits = [ground_frame(scans[f]) for f in frames[::10]]
    T_top_base, h, tilt, n_inl = max(fits, key=lambda g: g[3])
    T_base_top = np.linalg.inv(T_top_base)
    T_odom_w = T_base_top @ np.linalg.inv(T_w_top[frames[0]])

    def T_odom_base(f):
        return T_odom_w @ T_w_top[f] @ T_top_base

    # ── sensor bag ──────────────────────────────────────────────────────────
    t0 = t_ns[frames[0]]
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), a.jpeg_quality]
    with Writer(out, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_cloud = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=TS)
        c_tfs = w.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=TS, offered_qos_profiles=LATCHED)
        statics = [transform_msg(T_base_top, 'base_link', 'velodyne', t0),
                   transform_msg(np.eye(4), 'base_link', 'base_footprint', t0)]
        conns = {}
        for k in CAMS:
            n = f'cam{k}'
            K, E = calib[k - 1]
            T_cam_top = np.eye(4)
            T_cam_top[:3] = E
            conns[k] = (w.add_connection(f'/{n}/camera/color/image_raw/compressed', 'sensor_msgs/msg/CompressedImage',
                                         typestore=TS),
                        w.add_connection(f'/{n}/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=TS))
            statics += [transform_msg(np.linalg.inv(T_cam_top), 'velodyne', f'{n}_depth_optical_frame', t0),
                        transform_msg(np.eye(4), f'{n}_depth_optical_frame', f'{n}_color_optical_frame', t0)]
        w.write(c_tfs, t0 - 1, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](transforms=statics), 'tf2_msgs/msg/TFMessage'))
        for j, f in enumerate(frames):
            t = t_ns[f]
            w.write(c_cloud, t, TS.serialize_cdr(cloud_msg(scans[f], t), 'sensor_msgs/msg/PointCloud2'))
            for k in CAMS:
                n = f'cam{k}'
                img = cv2.imdecode(np.frombuffer(s.image_bytes(f, k), np.uint8), cv2.IMREAD_COLOR)
                ok, jpg = cv2.imencode('.jpg', img, enc)
                msg = M['sensor_msgs/msg/CompressedImage'](header=header(t, f'{n}_color_optical_frame'),
                                                           format='bgr8; jpeg compressed bgr8', data=jpg.ravel())
                w.write(conns[k][0], t, TS.serialize_cdr(msg, 'sensor_msgs/msg/CompressedImage'))
                w.write(conns[k][1], t, TS.serialize_cdr(info_msg(calib[k - 1][0], t, f'{n}_color_optical_frame'),
                                                         'sensor_msgs/msg/CameraInfo'))
            if j % 50 == 0:
                print(f'bag {j}/{len(frames)}', flush=True)

    # ── odometry bag ────────────────────────────────────────────────────────
    with Writer(Path(f'{out}_odom'), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        prev = None
        V3 = M['geometry_msgs/msg/Vector3']
        for f in frames:
            t, T = t_ns[f], T_odom_base(f)
            q = quat_from_rot(T[:3, :3])
            v = np.zeros(3)
            if prev is not None:
                v = T[:3, :3].T @ (T[:3, 3] - prev[1][:3, 3]) / ((t - prev[0]) * 1e-9)
            prev = (t, T)
            odom = M['nav_msgs/msg/Odometry'](
                header=header(t, 'odom'), child_frame_id='base_link',
                pose=M['geometry_msgs/msg/PoseWithCovariance'](pose=M['geometry_msgs/msg/Pose'](
                    position=M['geometry_msgs/msg/Point'](x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
                    orientation=M['geometry_msgs/msg/Quaternion'](x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))),
                    covariance=np.zeros(36)),
                twist=M['geometry_msgs/msg/TwistWithCovariance'](twist=M['geometry_msgs/msg/Twist'](
                    linear=V3(x=float(v[0]), y=float(v[1]), z=float(v[2])), angular=V3(x=0.0, y=0.0, z=0.0)),
                    covariance=np.zeros(36)))
            w.write(c_odom, t, TS.serialize_cdr(odom, 'nav_msgs/msg/Odometry'))
            w.write(c_tf, t, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](
                transforms=[transform_msg(T, 'odom', 'base_link', t)]), 'tf2_msgs/msg/TFMessage'))

    # ── GT ──────────────────────────────────────────────────────────────────
    import pandas as pd

    def in_fov(p_top):
        for K, E in calib.values():
            pc = E[:, :3] @ p_top + E[:, 3]
            if pc[2] > 0.5:
                u, v = (K @ pc)[:2] / pc[2]
                if 0 <= u < W and 0 <= v < H:
                    return True
        return False

    rows, ids, classes = [], {}, set()
    for f in frames:
        t = t_ns[f]
        T_top_w = np.linalg.inv(T_w_top[f])
        T_o = T_odom_base(f) @ T_base_top @ T_top_w           # odom <- world at this frame (world is static)
        objs = s.labels_3d(f)
        if not objs:
            rows.append(dict(t_ns=t, track_id=-1))
        P_w = scans[f][:, :3] @ T_w_top[f][:3, :3].T + T_w_top[f][:3, 3]
        for cls, sid, bh, bl, bw, x, y, z, yaw in objs:
            cls = CLASS_FIX.get(cls, cls)
            classes.add(cls)
            tid = ids.setdefault(sid, len(ids))
            hd = -yaw                                          # world-frame heading (see the docstring)
            cs, sn = np.cos(hd), np.sin(hd)
            d = P_w - [x, y, z]
            npts = int(np.sum((np.abs(d[:, 0] * cs + d[:, 1] * sn) <= bl / 2) &
                              (np.abs(-d[:, 0] * sn + d[:, 1] * cs) <= bw / 2) & (np.abs(d[:, 2]) <= bh / 2)))
            p = T_o[:3, :3] @ [x, y, z] + T_o[:3, 3]
            hx = T_o[:3, :3] @ [cs, sn, 0.0]
            p_top = T_top_w[:3, :3] @ [x, y, z] + T_top_w[:3, 3]
            rows.append(dict(t_ns=t, track_id=tid, x=p[0], y=p[1], z=p[2], l=bl, w=bw, h=bh,
                             yaw=float(np.arctan2(hx[1], hx[0])), state='', score=1.0, **{'class': cls},
                             occluded='', npts=npts, range=float(np.hypot(p_top[0], p_top[1])), in_fov=in_fov(p_top)))
    pd.DataFrame(rows).to_csv(f'{out}_gt.csv', index=False, float_format='%.4f')
    path = np.array([T_odom_base(f)[:3, 3] for f in frames])
    ovh = float(np.median([overhead_fraction(scans[f]) for f in frames[::10]]))
    json.dump(dict(seq=Path(a.zip).stem, rate_hz=RATE, t0_ns=t0, t1_ns=t_ns[frames[-1]], annotated_frames=frames,
                   annotated_t_ns=[t_ns[f] for f in frames], classes=sorted(classes), instance_ids=ids, frame='odom',
                   lidar_height_m=h, lidar_tilt_deg=tilt, ground_inliers=n_inl, T_base_velodyne=T_base_top.tolist(),
                   overhead_fraction=ovh, path_length_m=float(np.sum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))),
                   end_drift_z_m=float(path[-1, 2] - path[0, 2]), cameras=[f'cam{k}' for k in CAMS]),
              open(f'{out}_gt.json', 'w'), indent=1)
    print(f'done: {len(frames)} scans, {len(ids)} objects {sorted(classes)}; LiDAR {h:.2f} m above ground, tilt '
          f'{tilt:.1f} deg; overhead {ovh:.2f}; path {np.round(path[-1] - path[0], 1)}')


if __name__ == '__main__':
    main()
