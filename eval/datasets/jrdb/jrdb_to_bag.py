#!/usr/bin/env python3
"""
JRDB sequence (train_dataset_with_activity) -> ROS2 bags in the shape SINS reads, plus the GT, as coda_to_bag.py does
for CODa. Run with eval/datasets/.venv/bin/python (kiss-icp, python-lzf).

What JRDB gives and what it lacks (this release): two VLP-16 (upper, lower) as .pcd, five rectified color cameras of
the upper ring (image_0/2/4/6/8, 752x480, already undistorted: `undistorted_img_K` + `upper2cam` project the LiDAR
onto the edges, checked on bytes-cafe), 3D person boxes in the ego frame. NO depth, NO odometry, NO timestamps.
  - LiDAR given to the methods: the UPPER VLP-16 only = the robot's own sensor (one VLP-16, the base profiles' 2 deg
    ring spacing and 15 m crop). The lower one is used only for the odometry below.
  - timestamps: frames at 15 Hz (--rate), the JRDB annotation rate; every frame has its own scan.
  - odometry: KISS-ICP once on upper + lower clouds (no deskew: no per-point times), shared by every method, as
    BENCHMARK_PLAN.md prescribes for datasets without usable odometry.
  - depth: none in the bag; eval/adapters/lidar_depth_node.py projects the scan onto each camera ONLINE.

Writes:
  <out>/          /velodyne_points (upper, frame velodyne), per camera cam<k>: /cam<k>/camera/color/image_raw/compressed
                  (the dataset's own JPEG bytes, not re-encoded) + /cam<k>/camera/color/camera_info, /tf_static
                  (base_link->velodyne from a ground-plane fit, velodyne->cam<k>_depth_optical_frame = inv(upper2cam),
                  depth optical == color optical: the projected depth is already aligned to the color image).
  <out>_odom/     /odometry/filtered + /tf odom->base_link.
  <out>_gt.csv    GT rows (coda_eval.py format): class Pedestrian, npts = upper-LiDAR returns in the box, in_fov = box
                  centre inside one of the five images; JRDB's no_eval boxes get npts 0 (= ignored, not missed).
  <out>_gt.json   sidecar (frames, ids, indoor/outdoor guess, odometry summary).

usage: jrdb_to_bag.py <jrdb_root> <sequence> <out> [--first N --last N] [--rate 15]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
from jrdb_io import calib, labels_3d, read_pcd                       # noqa: E402
from coda_to_bag import (LATCHED, M, TS, ground_frame, header,       # noqa: E402
                         quat_from_rot, transform_msg, cloud_msg)
from rosbags.rosbag2 import StoragePlugin, Writer                    # noqa: E402

CAMS = [0, 2, 4, 6, 8]
W, H = 752, 480


def tf(T, p):
    return p[:, :3] @ T[:3, :3].T + T[:3, 3]


def rz(a):
    T = np.eye(4)
    T[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return T


def info_msg(K, ns, frame):
    P = np.zeros(12)
    P[[0, 1, 2, 4, 5, 6, 10]] = K.ravel()[[0, 1, 2, 3, 4, 5, 8]]
    return M['sensor_msgs/msg/CameraInfo'](
        header=header(ns, frame), height=H, width=W, distortion_model='plumb_bob', d=np.zeros(5),
        k=K.ravel().astype(float), r=np.eye(3).ravel(), p=P, binning_x=0, binning_y=0,
        roi=M['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def odometry(clouds_up, clouds_lo, L2U):
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP
    cfg = KISSConfig()
    cfg.data.max_range, cfg.data.min_range, cfg.data.deskew = 30.0, 1.0, False
    cfg.mapping.voxel_size = 0.3
    icp = KissICP(cfg)
    poses = []
    for up, lo in zip(clouds_up, clouds_lo):
        pts = np.vstack([up[:, :3], tf(L2U, lo)]).astype(np.float64)
        icp.register_frame(pts, np.zeros(len(pts)))
        poses.append(icp.last_pose.copy())          # T_world_upper
    return poses


def overhead_fraction(scan):
    """Share of near-vertical returns (within 25 deg of the zenith, 2-15 m) that hit something above 2 m over the
    sensor: a ceiling indoors, ~0 under the open sky (trees and canopies give a little)."""
    r = np.linalg.norm(scan[:, :3], axis=1)
    up = scan[(scan[:, 2] > 0) & (r > 1)]
    return float(np.mean(up[:, 2] > 2.0)) if len(up) else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('root')
    ap.add_argument('seq')
    ap.add_argument('out')
    ap.add_argument('--first', type=int, default=0)
    ap.add_argument('--last', type=int, default=-1)
    ap.add_argument('--rate', type=float, default=15.0)
    a = ap.parse_args()
    root, seq, out = a.root, a.seq, Path(a.out)
    for p in (out, Path(f'{out}_odom')):
        if p.exists():
            raise SystemExit(f'{p} exists')

    c = calib(root)
    U2E, L2U = c['lidar']['upper2ego'], c['lidar']['lower2upper']
    E2U = np.linalg.inv(U2E)
    labels = labels_3d(root, seq)
    n_all = len(labels)
    last = n_all - 1 if a.last < 0 else a.last
    frames = list(range(a.first, last + 1))
    t_ns = {i: int(1_000_000_000 + (i - a.first) * 1e9 / a.rate) for i in frames}   # bag starts at t = 1 s

    print(f'{seq}: reading {len(frames)} scans', flush=True)
    up = [read_pcd(f'{root}/pointclouds/upper_velodyne/{seq}/{i:06d}.pcd') for i in frames]
    lo = [read_pcd(f'{root}/pointclouds/lower_velodyne/{seq}/{i:06d}.pcd') for i in frames]
    print('odometry (KISS-ICP)', flush=True)
    T_w_up = odometry(up, lo, L2U)

    T_up_base, h, tilt, n_inl = ground_frame(up[0])
    T_base_up = np.linalg.inv(T_up_base)
    T_odom_w = T_base_up @ np.linalg.inv(T_w_up[0])

    def T_odom_base(k):
        return T_odom_w @ T_w_up[k] @ T_up_base

    # ── sensor bag ──────────────────────────────────────────────────────────
    t0 = t_ns[frames[0]]
    with Writer(out, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_cloud = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=TS)
        c_tfs = w.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=TS, offered_qos_profiles=LATCHED)
        statics = [transform_msg(T_base_up, 'base_link', 'velodyne', t0),
                   transform_msg(np.eye(4), 'base_link', 'base_footprint', t0)]
        conns = {}
        for k in CAMS:
            n = f'cam{k}'
            conns[k] = (w.add_connection(f'/{n}/camera/color/image_raw/compressed', 'sensor_msgs/msg/CompressedImage',
                                         typestore=TS),
                        w.add_connection(f'/{n}/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=TS))
            statics += [transform_msg(np.linalg.inv(c[f'sensor_{k}']['upper2cam']), 'velodyne', f'{n}_depth_optical_frame', t0),
                        transform_msg(np.eye(4), f'{n}_depth_optical_frame', f'{n}_color_optical_frame', t0)]
        w.write(c_tfs, t0 - 1, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](transforms=statics), 'tf2_msgs/msg/TFMessage'))
        for j, i in enumerate(frames):
            t = t_ns[i]
            w.write(c_cloud, t, TS.serialize_cdr(cloud_msg(up[j], t), 'sensor_msgs/msg/PointCloud2'))
            for k in CAMS:
                n = f'cam{k}'
                data = np.frombuffer(open(f'{root}/images/image_{k}/{seq}/{i:06d}.jpg', 'rb').read(), np.uint8)
                img = M['sensor_msgs/msg/CompressedImage'](header=header(t, f'{n}_color_optical_frame'),
                                                           format='bgr8; jpeg compressed bgr8', data=data)
                w.write(conns[k][0], t, TS.serialize_cdr(img, 'sensor_msgs/msg/CompressedImage'))
                w.write(conns[k][1], t, TS.serialize_cdr(info_msg(c[f'sensor_{k}']['undistorted_img_K'], t,
                                                                  f'{n}_color_optical_frame'), 'sensor_msgs/msg/CameraInfo'))
            if j % 300 == 0:
                print(f'bag {j}/{len(frames)}', flush=True)

    # ── odometry bag ────────────────────────────────────────────────────────
    with Writer(Path(f'{out}_odom'), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        prev = None
        for j, i in enumerate(frames):
            t, T = t_ns[i], T_odom_base(j)
            q = quat_from_rot(T[:3, :3])
            v = np.zeros(3)
            if prev is not None:
                v = T[:3, :3].T @ (T[:3, 3] - prev[1][:3, 3]) / ((t - prev[0]) * 1e-9)
            prev = (t, T)
            V3 = M['geometry_msgs/msg/Vector3']
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
    Ks = {k: (c[f'sensor_{k}']['upper2cam'], c[f'sensor_{k}']['undistorted_img_K']) for k in CAMS}

    def in_fov(p_up):
        for T, K in Ks.values():
            pc = T[:3, :3] @ p_up + T[:3, 3]
            if pc[2] > 0.5:
                u, v = (K @ pc)[:2] / pc[2]
                if 0 <= u < W and 0 <= v < H:
                    return True
        return False

    rows, ids = [], {}
    for j, i in enumerate(frames):
        t = t_ns[i]
        T = T_odom_base(j) @ T_base_up @ E2U            # odom <- ego at this frame
        objs = labels.get(f'{i:06d}.pcd', [])
        if not objs:
            rows.append(dict(t_ns=t, track_id=-1))
        P_ego = tf(U2E, up[j])
        for o in objs:
            b = o['box']
            tid = ids.setdefault(o['label_id'], len(ids))
            cs, sn = np.cos(b['rot_z']), np.sin(b['rot_z'])
            d = P_ego - [b['cx'], b['cy'], b['cz']]
            npts = int(np.sum((np.abs(d[:, 0] * cs + d[:, 1] * sn) <= b['l'] / 2) &
                              (np.abs(-d[:, 0] * sn + d[:, 1] * cs) <= b['w'] / 2) & (np.abs(d[:, 2]) <= b['h'] / 2)))
            if o['attributes'].get('no_eval'):
                npts = 0
            ce = np.array([b['cx'], b['cy'], b['cz']])
            p = T[:3, :3] @ ce + T[:3, 3]
            Rb = T[:3, :3] @ rz(b['rot_z'])[:3, :3]
            rows.append(dict(t_ns=t, track_id=tid, x=p[0], y=p[1], z=p[2], l=b['l'], w=b['w'], h=b['h'],
                             yaw=float(np.arctan2(Rb[1, 0], Rb[0, 0])), state='', score=1.0, **{'class': 'Pedestrian'},
                             occluded='', npts=npts, range=float(np.hypot(b['cx'], b['cy'])),
                             in_fov=in_fov(E2U[:3, :3] @ ce + E2U[:3, 3])))
    pd.DataFrame(rows).to_csv(f'{out}_gt.csv', index=False, float_format='%.4f')
    path = np.array([T_odom_base(j)[:3, 3] for j in range(len(frames))])
    ovh = float(np.median([overhead_fraction(up[j]) for j in range(0, len(frames), max(1, len(frames) // 20))]))
    json.dump(dict(seq=seq, first=frames[0], last=frames[-1], rate_hz=a.rate, t0_ns=t0, t1_ns=t_ns[frames[-1]],
                   annotated_frames=frames, annotated_t_ns=[t_ns[i] for i in frames], classes=['Pedestrian'],
                   instance_ids=ids, frame='odom', lidar_height_m=h, lidar_tilt_deg=tilt, ground_inliers=n_inl,
                   T_base_velodyne=T_base_up.tolist(), overhead_fraction=ovh,
                   path_length_m=float(np.sum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))),
                   end_drift_z_m=float(path[-1, 2] - path[0, 2]), cameras=[f'cam{k}' for k in CAMS]),
              open(f'{out}_gt.json', 'w'), indent=1)
    print(f'done: {len(frames)} scans, {len(ids)} people; LiDAR {h:.2f} m above ground, tilt {tilt:.1f} deg; '
          f'overhead {ovh:.2f}; path {path[-1] - path[0]}')


if __name__ == '__main__':
    main()
