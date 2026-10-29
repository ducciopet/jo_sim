#!/usr/bin/env python3
"""
CODa sequence (or a frame window of it) -> ROS2 bags in the shape SINS reads from our lab bags, plus the GT.

Writes three things next to each other:
  <out>/            sensor bag: /velodyne_points (OS1-128, frame velodyne), /front_camera/camera/color/image_raw +
                    camera_info (ZED left, cam3), /front_camera/camera/depth/image_rect_raw + camera_info (ZED depth,
                    already aligned to the left image), /tf_static. The remaining RealSense-style streams (extrinsics,
                    compressed color, depth cloud) come from eval/adapters/realsense_bag_adapter.py, as for lab bags.
  <out>_odom/       /odometry/filtered + /tf odom->base_link from CODa's dense poses (one per scan), the shared
                    odometry every method replays (same role as bags/replay/odom_<seq> for lab bags).
  <out>_gt.csv      GT tracks (eval/harness/tracks.py format) in the odom frame: the annotated 3D boxes of the chosen
                    classes, instance ids made numeric, state left empty (derived at evaluation time from speed).
  <out>_gt.json     sidecar: frames, classes, instance-id map, base_link definition.

Frames: velodyne = CODa's os1 frame. base_link = gravity-aligned frame on the ground under the LiDAR, from a plane
fit on the first scan of the window (CODa's own os1_to_base is a different, z-down convention). odom = base_link at
the first pose of the sequence. Camera: TF from calib_os1_to_cam3 (factory calibration), so SINS runs with
enable_calibration:=false. Images are the ZED's rectified output: distortion written as zero.

usage: coda_to_bag.py <coda_root> <seq> <first_frame> <last_frame> <out> [--classes Pedestrian ...]
"""
import argparse
import glob
import json
import os
import re
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from rosbags.interfaces import Qos, QosDurability, QosHistory, QosLiveliness, QosReliability, QosTime
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_JAZZY)
M = TS.types
INF = QosTime(9223372036, 854775807)
LATCHED = [Qos(QosHistory.KEEP_LAST, 1, QosReliability.RELIABLE, QosDurability.TRANSIENT_LOCAL,
               INF, INF, QosLiveliness.AUTOMATIC, INF, False)]
CAM = 'front_camera'


def stamp(ns):
    return M['builtin_interfaces/msg/Time'](sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def header(ns, frame):
    return M['std_msgs/msg/Header'](stamp=stamp(ns), frame_id=frame)


def quat_from_rot(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def rot_from_quat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform_msg(T, parent, child, ns):
    q = quat_from_rot(T[:3, :3])
    return M['geometry_msgs/msg/TransformStamped'](
        header=header(ns, parent), child_frame_id=child,
        transform=M['geometry_msgs/msg/Transform'](
            translation=M['geometry_msgs/msg/Vector3'](x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
            rotation=M['geometry_msgs/msg/Quaternion'](x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))))


def read_matrix(path):
    d = yaml.safe_load(open(path))
    m = d['extrinsic_matrix']
    return np.array(m['data'], dtype=float).reshape(m['rows'], m['cols'])


def load_scan(root, seq, i):
    a = np.fromfile(f'{root}/3d_comp/os1/{seq}/3d_comp_os1_{seq}_{i}.bin', dtype=np.float32).reshape(-1, 4)
    return a[np.abs(a[:, :3]).sum(1) > 0]


def points_in_box(scan, b):
    """LiDAR returns inside an annotated box (sensor frame), the visibility criterion of KITTI/nuScenes."""
    p = (scan[:, :3] - [b['cX'], b['cY'], b['cZ']]) @ rot_rpy(b['r'], b['p'], b['y'])
    return int(np.sum((np.abs(p[:, 0]) <= b['l'] / 2) & (np.abs(p[:, 1]) <= b['w'] / 2) & (np.abs(p[:, 2]) <= b['h'] / 2)))


def in_camera_fov(b, T_cam_os1, K, intr):
    """Box centre projects inside the camera image (in front of it): the object can be seen by the camera."""
    p = T_cam_os1 @ [b['cX'], b['cY'], b['cZ'], 1.0]
    if p[2] <= 0.5:
        return False
    u, v = (K @ p[:3])[:2] / p[2]
    return bool(0 <= u < intr['image_width'] and 0 <= v < intr['image_height'])


def ground_frame(scan, rng=np.random.default_rng(0)):
    """base_link in the os1 frame: z along the ground normal, origin on the ground under the LiDAR, x along os1 x.
    RANSAC plane on the near points below the sensor."""
    r = np.linalg.norm(scan[:, :2], axis=1)
    cand = scan[(r > 2) & (r < 15) & (scan[:, 2] < -0.3), :3].astype(float)
    best, best_n = None, 0
    for _ in range(300):
        p = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        if n[2] < 0:
            n = -n
        if n[2] < 0.95:          # not near-horizontal
            continue
        inl = np.abs((cand - p[0]) @ n) < 0.05
        if inl.sum() > best_n:
            best, best_n = (n, p[0]), inl.sum()
    n, p0 = best
    inl = cand[np.abs((cand - p0) @ n) < 0.05]
    c = inl.mean(0)                                   # least-squares refit on the inliers
    n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
    n = n if n[2] > 0 else -n
    h = float(n @ (np.zeros(3) - c))                  # height of the LiDAR above the plane
    x = np.array([1.0, 0, 0]) - n[0] * n
    x /= np.linalg.norm(x)
    R = np.stack([x, np.cross(n, x), n], axis=1)      # base axes expressed in os1
    T_os1_base = np.eye(4)
    T_os1_base[:3, :3] = R
    T_os1_base[:3, 3] = -h * n                        # ground point under the sensor
    return T_os1_base, h, float(np.degrees(np.arccos(n[2]))), len(inl)


def cloud_msg(pts, ns):
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    F = M['sensor_msgs/msg/PointField']
    fields = [F(name=n, offset=4 * k, datatype=7, count=1) for k, n in enumerate(['x', 'y', 'z', 'intensity'])]
    return M['sensor_msgs/msg/PointCloud2'](
        header=header(ns, 'velodyne'), height=1, width=len(pts), fields=fields, is_bigendian=False,
        point_step=16, row_step=16 * len(pts), data=pts.view(np.uint8).ravel(), is_dense=True)


def image_msg(img, ns, frame, encoding):
    step = img.shape[1] * (3 if img.ndim == 3 else 2)
    return M['sensor_msgs/msg/Image'](header=header(ns, frame), height=img.shape[0], width=img.shape[1],
                                      encoding=encoding, is_bigendian=0, step=step,
                                      data=np.ascontiguousarray(img).view(np.uint8).ravel())


def info_msg(intr, ns, frame):
    K = np.array(intr['camera_matrix']['data'], dtype=float)
    P = np.zeros(12)
    P[[0, 1, 2, 4, 5, 6, 10]] = K[[0, 1, 2, 3, 4, 5, 8]]
    return M['sensor_msgs/msg/CameraInfo'](
        header=header(ns, frame), height=intr['image_height'], width=intr['image_width'],
        distortion_model='plumb_bob', d=np.zeros(5), k=K, r=np.eye(3).ravel(), p=P,
        binning_x=0, binning_y=0,
        roi=M['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def frames_by_stamp(pattern):
    out = []
    for f in glob.glob(pattern):
        out.append((int(re.findall(r'_(\d+)\.png$', f)[0]), f))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('root')
    ap.add_argument('seq')
    ap.add_argument('first', type=int)
    ap.add_argument('last', type=int)
    ap.add_argument('out')
    ap.add_argument('--classes', nargs='+', default=['Pedestrian'])
    ap.add_argument('--no-camera', action='store_true')
    ap.add_argument('--gt-only', action='store_true', help='rewrite <out>_gt.csv/.json only (bags already written)')
    a = ap.parse_args()
    root, seq, out = a.root, a.seq, Path(a.out)

    t_scan = (np.loadtxt(f'{root}/timestamps/{seq}.txt') * 1e9).astype(np.int64)
    poses = np.loadtxt(f'{root}/poses/dense/{seq}.txt')       # t x y z qw qx qy qz, one per scan
    assert len(poses) == len(t_scan), (len(poses), len(t_scan))
    frames = range(a.first, a.last + 1)
    t0, t1 = t_scan[a.first], t_scan[a.last]

    T_os1_base, h, tilt, n_inl = ground_frame(load_scan(root, seq, a.first))
    T_base_os1 = np.linalg.inv(T_os1_base)

    def T_world_os1(i):
        x, y, z, qw, qx, qy, qz = poses[i, 1:]
        T = np.eye(4)
        T[:3, :3] = rot_from_quat(qx, qy, qz, qw)
        T[:3, 3] = [x, y, z]
        return T

    T_odom_world = T_base_os1 @ np.linalg.inv(T_world_os1(0))   # odom = base_link at the sequence's first pose

    def T_odom_base(i):
        return T_odom_world @ T_world_os1(i) @ T_os1_base

    calib = f'{root}/calibrations/{seq}'
    T_cam_os1 = read_matrix(f'{calib}/calib_os1_to_cam3.yaml')    # p_cam = E p_os1
    intr = yaml.safe_load(open(f'{calib}/calib_cam3_intrinsics.yaml'))
    K = np.array(intr['camera_matrix']['data']).reshape(3, 3)

    for p in (out, Path(f'{out}_odom')):
        if p.exists() and not a.gt_only:
            raise SystemExit(f'{p} exists')

    # ── sensor bag ──────────────────────────────────────────────────────────
    color = depth = []
    if not a.no_camera and not a.gt_only:
        color = [x for x in frames_by_stamp(f'{root}/2d_raw/cam3/{seq}/*.png') if t0 - 50_000_000 <= x[0] <= t1]
        depth = [x for x in frames_by_stamp(f'{root}/3d_raw/cam3/{seq}/*.png') if t0 - 50_000_000 <= x[0] <= t1]
    with (open(os.devnull) if a.gt_only else Writer(out, version=9, storage_plugin=StoragePlugin.MCAP)) as w:
      if not a.gt_only:
        c_cloud = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=TS)
        c_tfs = w.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=TS, offered_qos_profiles=LATCHED)
        statics = [transform_msg(T_base_os1, 'base_link', 'velodyne', t0),
                   transform_msg(np.eye(4), 'base_link', 'base_footprint', t0)]
        if not a.no_camera:
            c_img = w.add_connection(f'/{CAM}/camera/color/image_raw', 'sensor_msgs/msg/Image', typestore=TS)
            c_info = w.add_connection(f'/{CAM}/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=TS)
            c_dep = w.add_connection(f'/{CAM}/camera/depth/image_rect_raw', 'sensor_msgs/msg/Image', typestore=TS)
            c_dinfo = w.add_connection(f'/{CAM}/camera/depth/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=TS)
            # RealSense driver convention (depth optical = parent of color optical): SINS republishes depth->color from
            # the extrinsics in this direction, the opposite one would make a loop in the TF tree
            statics += [transform_msg(np.linalg.inv(T_cam_os1), 'velodyne', f'{CAM}_depth_optical_frame', t0),
                        transform_msg(np.eye(4), f'{CAM}_depth_optical_frame', f'{CAM}_color_optical_frame', t0)]
        w.write(c_tfs, int(t0) - 1, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](transforms=statics),
                                                     'tf2_msgs/msg/TFMessage'))
        events = [(int(t_scan[i]), 'cloud', i) for i in frames]
        events += [(t, 'color', f) for t, f in color] + [(t, 'depth', f) for t, f in depth]
        for k, (t, kind, x) in enumerate(sorted(events, key=lambda e: e[0])):
            if kind == 'cloud':
                msg, conn = cloud_msg(load_scan(root, seq, x), t), c_cloud
                w.write(conn, t, TS.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2'))
            elif kind == 'color':
                img = np.array(Image.open(x).convert('RGB'))
                w.write(c_img, t, TS.serialize_cdr(image_msg(img, t, f'{CAM}_color_optical_frame', 'rgb8'),
                                                   'sensor_msgs/msg/Image'))
                w.write(c_info, t, TS.serialize_cdr(info_msg(intr, t, f'{CAM}_color_optical_frame'),
                                                    'sensor_msgs/msg/CameraInfo'))
            else:
                img = np.array(Image.open(x)).astype(np.uint16)
                w.write(c_dep, t, TS.serialize_cdr(image_msg(img, t, f'{CAM}_depth_optical_frame', '16UC1'),
                                                   'sensor_msgs/msg/Image'))
                w.write(c_dinfo, t, TS.serialize_cdr(info_msg(intr, t, f'{CAM}_depth_optical_frame'),
                                                     'sensor_msgs/msg/CameraInfo'))
            if k % 500 == 0:
                print(f'{k}/{len(events)}', flush=True)

    # ── odometry bag ────────────────────────────────────────────────────────
    with (open(os.devnull) if a.gt_only else Writer(Path(f'{out}_odom'), version=9, storage_plugin=StoragePlugin.MCAP)) as w:
      if not a.gt_only:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        prev = None
        for i in frames:
            t, T = int(t_scan[i]), T_odom_base(i)
            q = quat_from_rot(T[:3, :3])
            v = np.zeros(3)
            if prev is not None:                       # body-frame linear velocity from the previous pose
                v = T[:3, :3].T @ (T[:3, 3] - prev[1][:3, 3]) / ((t - prev[0]) * 1e-9)
            prev = (t, T)
            pose = M['geometry_msgs/msg/Pose'](
                position=M['geometry_msgs/msg/Point'](x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
                orientation=M['geometry_msgs/msg/Quaternion'](x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])))
            twist = M['geometry_msgs/msg/Twist'](linear=M['geometry_msgs/msg/Vector3'](x=float(v[0]), y=float(v[1]), z=float(v[2])),
                                                 angular=M['geometry_msgs/msg/Vector3'](x=0.0, y=0.0, z=0.0))
            odom = M['nav_msgs/msg/Odometry'](
                header=header(t, 'odom'), child_frame_id='base_link',
                pose=M['geometry_msgs/msg/PoseWithCovariance'](pose=pose, covariance=np.zeros(36)),
                twist=M['geometry_msgs/msg/TwistWithCovariance'](twist=twist, covariance=np.zeros(36)))
            w.write(c_odom, t, TS.serialize_cdr(odom, 'nav_msgs/msg/Odometry'))
            w.write(c_tf, t, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](
                transforms=[transform_msg(T, 'odom', 'base_link', t)]), 'tf2_msgs/msg/TFMessage'))

    # ── GT ──────────────────────────────────────────────────────────────────
    rows, ids, annotated = [], {}, []
    for i in frames:
        f = f'{root}/3d_bbox/os1/{seq}/3d_bbox_os1_{seq}_{i}.json'
        if not os.path.exists(f):
            continue
        annotated.append(i)
        t = int(t_scan[i])
        T = T_odom_base(i) @ T_base_os1                 # odom <- os1 at this scan
        boxes = [b for b in json.load(open(f))['3dbbox'] if b['classId'] in a.classes]
        scan = load_scan(root, seq, i) if boxes else None
        if not boxes:
            rows.append(dict(t_ns=t, track_id=-1))
        for b in boxes:
            tid = ids.setdefault(b['instanceId'], len(ids))
            c = T[:3, :3] @ [b['cX'], b['cY'], b['cZ']] + T[:3, 3]
            Rb = T[:3, :3] @ rot_rpy(b['r'], b['p'], b['y'])
            rows.append(dict(t_ns=t, track_id=tid, x=c[0], y=c[1], z=c[2], l=b['l'], w=b['w'], h=b['h'],
                             yaw=float(np.arctan2(Rb[1, 0], Rb[0, 0])), state='', score=1.0,
                             **{'class': b['classId']}, occluded=b['labelAttributes'].get('isOccluded', ''),
                             npts=points_in_box(scan, b), range=float(np.hypot(b['cX'], b['cY'])),
                             in_fov=in_camera_fov(b, T_cam_os1, K, intr)))
    import pandas as pd
    pd.DataFrame(rows).to_csv(f'{out}_gt.csv', index=False, float_format='%.4f')
    json.dump(dict(seq=seq, first=a.first, last=a.last, t0_ns=int(t0), t1_ns=int(t1), annotated_frames=annotated,
                   annotated_t_ns=[int(t_scan[i]) for i in annotated], classes=a.classes, instance_ids=ids,
                   frame='odom', lidar_height_m=h, lidar_tilt_deg=tilt, ground_inliers=n_inl,
                   T_base_velodyne=T_base_os1.tolist(), n_color=len(color), n_depth=len(depth)),
              open(f'{out}_gt.json', 'w'), indent=1)
    print(f'done: {len(frames)} scans, {len(color)} color, {len(depth)} depth, {len(annotated)} annotated frames, '
          f'{len(ids)} instances; LiDAR {h:.2f} m above ground, tilt {tilt:.1f} deg')


if __name__ == '__main__':
    main()
