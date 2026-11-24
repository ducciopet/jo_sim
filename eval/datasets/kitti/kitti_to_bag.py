#!/usr/bin/env python3
"""
KITTI tracking sequence (training) -> ROS2 bags in the shape SINS reads, plus the GT, as jrdb_to_bag.py / coda_to_bag.py.
Run with eval/datasets/.venv/bin/python (rosbags, numpy, pandas).

KITTI tracking gives: HDL-64E scans (velodyne/<seq>/<frame>.bin, float32 x y z r), left color camera (image_02, rectified
PNG, ~1242x375), calib/<seq>.txt (P0-P3, R_rect, Tr_velo_cam, Tr_imu_velo), oxts/<seq>.txt (GPS/IMU per frame, 10 Hz),
label_02/<seq>.txt (frame, track_id, type, truncated, occluded, alpha, 2D box, h w l, x y z (bottom centre, rectified
camera-0 frame), rotation_y). No depth: as for JRDB, eval/adapters/lidar_depth_node.py projects the scan onto the
camera ONLINE.
  - timestamps: frame index at 10 Hz (the dataset's rate; its own timestamps files are not needed).
  - odometry: the OXTS poses (KITTI devkit Mercator projection, first frame = origin), shared by every method.
  - base_link: the vehicle frame under the LiDAR, LiDAR 1.73 m above the ground (KITTI setup), level.
  - GT: every labelled object except DontCare/Misc, class names mapped to the CODa-style names coda_eval.py uses
    (Car/Van/Truck/Tram -> their own names, Pedestrian, Person_sitting -> Pedestrian, Cyclist). KITTI labels only
    objects visible in image_02: in_fov is True for every row, and coda_eval.py is run with --camera-only
    --cam-json <out>_cam.json (predictions outside the camera image ignored, the KITTI condition). DontCare boxes are written as npts 0 rows
    (ignored regions) only if they have a 3D box (they usually do not; then they are skipped).

Writes:
  <out>/        /velodyne_points (frame velodyne), /cam2/camera/color/image_raw/compressed (the PNG bytes, not
                re-encoded), /cam2/camera/color/camera_info, /tf_static (base_link->velodyne,
                velodyne->cam2_depth_optical_frame (rectified camera 2), depth optical == color optical).
  <out>_odom/   /odometry/filtered + /tf odom->base_link.
  <out>_gt.csv  GT rows (coda_eval.py format).
  <out>_gt.json sidecar.
  <out>_cam.json per-frame T_cam_odom of camera 2 + K + image size (coda_eval.py --cam-json).

usage: kitti_to_bag.py <kitti_tracking_root> <seq (e.g. 0000)> <out> [--split training] [--rate 10]
NOTE (24/11): written on the laptop, NOT run there (no tests on the laptop); first run = the lab smoke test.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from coda_to_bag import LATCHED, M, TS, header, quat_from_rot, transform_msg, cloud_msg   # noqa: E402
from rosbags.rosbag2 import StoragePlugin, Writer                                        # noqa: E402

LIDAR_HEIGHT = 1.73
CLASS_MAP = {'Car': 'Car', 'Van': 'Van', 'Truck': 'Truck', 'Tram': 'Tram', 'Pedestrian': 'Pedestrian',
             'Person_sitting': 'Pedestrian', 'Cyclist': 'Cyclist'}


def read_calib(path):
    c = {}
    for line in open(path):
        if ':' not in line and ' ' not in line:
            continue
        k, *v = line.replace(':', ' ').split()
        c[k] = np.array([float(x) for x in v])
    P2 = c['P2'].reshape(3, 4)
    R = np.eye(4)
    R[:3, :3] = c['R_rect'].reshape(3, 3)
    Tvc = np.eye(4)
    Tvc[:3, :] = c['Tr_velo_cam'].reshape(3, 4)
    K = P2[:, :3]
    t2 = np.linalg.solve(K, P2[:, 3])          # rectified cam0 -> rectified cam2 offset
    T02 = np.eye(4)
    T02[:3, 3] = t2
    T_cam2_velo = T02 @ R @ Tvc                # velodyne point -> rectified camera-2 frame
    T_cam0_velo = R @ Tvc                      # velodyne point -> rectified camera-0 frame (label frame)
    return K, T_cam2_velo, T_cam0_velo


def oxts_poses(path):
    """KITTI devkit convertOxtsToPose: Mercator xy, altitude z, roll/pitch/yaw -> T_world_imu, first pose = identity."""
    rows = [np.array([float(x) for x in l.split()]) for l in open(path) if l.strip()]
    er = 6378137.0
    scale = np.cos(rows[0][0] * np.pi / 180.0)
    T0, out = None, []
    for r in rows:
        lat, lon, alt, roll, pitch, yaw = r[:6]
        tx = scale * lon * np.pi * er / 180.0
        ty = scale * er * np.log(np.tan((90.0 + lat) * np.pi / 360.0))
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        T = np.eye(4)
        T[:3, :3] = Rz @ Ry @ Rx
        T[:3, 3] = [tx, ty, alt]
        if T0 is None:
            T0 = np.linalg.inv(T)
        out.append(T0 @ T)
    return out


def read_labels(path):
    lab = {}
    for l in open(path):
        f = l.split()
        if len(f) < 17:
            continue
        lab.setdefault(int(f[0]), []).append(dict(tid=int(f[1]), type=f[2], trunc=float(f[3]), occ=int(f[4]),
                                                 h=float(f[10]), w=float(f[11]), l=float(f[12]),
                                                 x=float(f[13]), y=float(f[14]), z=float(f[15]), ry=float(f[16])))
    return lab


def info_msg(K, W, H, ns, frame):
    P = np.zeros(12)
    P[[0, 1, 2, 4, 5, 6, 10]] = K.ravel()[[0, 1, 2, 3, 4, 5, 8]]
    return M['sensor_msgs/msg/CameraInfo'](
        header=header(ns, frame), height=H, width=W, distortion_model='plumb_bob', d=np.zeros(5),
        k=K.ravel().astype(float), r=np.eye(3).ravel(), p=P, binning_x=0, binning_y=0,
        roi=M['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def png_size(data):
    return int.from_bytes(data[16:20], 'big'), int.from_bytes(data[20:24], 'big')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('root')
    ap.add_argument('seq')
    ap.add_argument('out')
    ap.add_argument('--split', default='training')
    ap.add_argument('--rate', type=float, default=10.0)
    a = ap.parse_args()
    R, out = Path(a.root) / a.split, Path(a.out)
    for p in (out, Path(f'{out}_odom')):
        if p.exists():
            raise SystemExit(f'{p} exists')
    K, T_c2_v, T_c0_v = read_calib(R / 'calib' / f'{a.seq}.txt')
    calib_lines = {l.split()[0].rstrip(':'): np.array([float(x) for x in l.split()[1:]])
                   for l in open(R / 'calib' / f'{a.seq}.txt') if l.strip()}
    T_imu_v = np.eye(4)                                                  # velodyne -> imu
    T_v_imu = np.eye(4)
    T_v_imu[:3, :] = calib_lines['Tr_imu_velo'].reshape(3, 4)           # imu point -> velodyne frame
    T_imu_v = np.linalg.inv(T_v_imu)
    poses = oxts_poses(R / 'oxts' / f'{a.seq}.txt')                     # T_world_imu
    scans = sorted((R / 'velodyne' / a.seq).glob('*.bin'))
    frames = [int(s.stem) for s in scans]
    labels = read_labels(R / 'label_02' / f'{a.seq}.txt')
    t_ns = {i: int(1_000_000_000 + i * 1e9 / a.rate) for i in frames}

    T_base_v = np.eye(4)
    T_base_v[2, 3] = LIDAR_HEIGHT                                        # level LiDAR, base_link on the ground below it
    T_v_base = np.linalg.inv(T_base_v)

    def T_world_base(i):
        return poses[i] @ T_imu_v @ T_v_base

    T_odom_world = np.linalg.inv(T_world_base(frames[0]))

    def T_odom_base(i):
        return T_odom_world @ T_world_base(i)

    first_png = open(R / 'image_02' / a.seq / f'{frames[0]:06d}.png', 'rb').read()
    W, H = png_size(first_png)
    t0 = t_ns[frames[0]]
    with Writer(out, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_cloud = w.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=TS)
        c_tfs = w.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=TS, offered_qos_profiles=LATCHED)
        c_img = w.add_connection('/cam2/camera/color/image_raw/compressed', 'sensor_msgs/msg/CompressedImage', typestore=TS)
        c_inf = w.add_connection('/cam2/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=TS)
        statics = [transform_msg(T_base_v, 'base_link', 'velodyne', t0),
                   transform_msg(np.eye(4), 'base_link', 'base_footprint', t0),
                   transform_msg(np.linalg.inv(T_c2_v), 'velodyne', 'cam2_depth_optical_frame', t0),
                   transform_msg(np.eye(4), 'cam2_depth_optical_frame', 'cam2_color_optical_frame', t0)]
        w.write(c_tfs, t0 - 1, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](transforms=statics), 'tf2_msgs/msg/TFMessage'))
        for j, (s, i) in enumerate(zip(scans, frames)):
            t = t_ns[i]
            pts = np.fromfile(s, dtype=np.float32).reshape(-1, 4)
            w.write(c_cloud, t, TS.serialize_cdr(cloud_msg(pts, t), 'sensor_msgs/msg/PointCloud2'))
            png = R / 'image_02' / a.seq / f'{i:06d}.png'
            if png.exists():
                data = np.frombuffer(png.read_bytes(), np.uint8)
                img = M['sensor_msgs/msg/CompressedImage'](header=header(t, 'cam2_color_optical_frame'),
                                                           format='bgr8; png compressed bgr8', data=data)
                w.write(c_img, t, TS.serialize_cdr(img, 'sensor_msgs/msg/CompressedImage'))
                w.write(c_inf, t, TS.serialize_cdr(info_msg(K, W, H, t, 'cam2_color_optical_frame'),
                                                   'sensor_msgs/msg/CameraInfo'))
            if j % 200 == 0:
                print(f'bag {j}/{len(frames)}', flush=True)

    with Writer(Path(f'{out}_odom'), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection('/odometry/filtered', 'nav_msgs/msg/Odometry', typestore=TS)
        c_tf = w.add_connection('/tf', 'tf2_msgs/msg/TFMessage', typestore=TS)
        prev = None
        V3 = M['geometry_msgs/msg/Vector3']
        for i in frames:
            t, T = t_ns[i], T_odom_base(i)
            q = quat_from_rot(T[:3, :3])
            v = np.zeros(3) if prev is None else T[:3, :3].T @ (T[:3, 3] - prev[1][:3, 3]) / ((t - prev[0]) * 1e-9)
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

    import pandas as pd
    T_v_c0 = np.linalg.inv(T_c0_v)
    rows, n_obj = [], 0
    for j, (s, i) in enumerate(zip(scans, frames)):
        t = t_ns[i]
        objs = [o for o in labels.get(i, []) if o['type'] in CLASS_MAP]
        if not objs:
            rows.append(dict(t_ns=t, track_id=-1))
            continue
        pts = np.fromfile(s, dtype=np.float32).reshape(-1, 4)[:, :3]
        T = T_odom_base(i) @ T_base_v                                     # odom <- velodyne
        for o in objs:
            c_cam = np.array([o['x'], o['y'] - o['h'] / 2, o['z'], 1.0])  # box centre, camera-0 rect (y down)
            c_v = (T_v_c0 @ c_cam)[:3]
            yaw_v = -o['ry'] - np.pi / 2                                  # camera ry (about -y) -> velodyne yaw (about z)
            cs, sn = np.cos(yaw_v), np.sin(yaw_v)
            d = pts - c_v
            npts = int(np.sum((np.abs(d[:, 0] * cs + d[:, 1] * sn) <= o['l'] / 2) &
                              (np.abs(-d[:, 0] * sn + d[:, 1] * cs) <= o['w'] / 2) & (np.abs(d[:, 2]) <= o['h'] / 2)))
            p = T[:3, :3] @ c_v + T[:3, 3]
            Rz = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]])
            Rb = T[:3, :3] @ Rz
            rows.append(dict(t_ns=t, track_id=o['tid'], x=p[0], y=p[1], z=p[2], l=o['l'], w=o['w'], h=o['h'],
                             yaw=float(np.arctan2(Rb[1, 0], Rb[0, 0])), state='', score=1.0,
                             **{'class': CLASS_MAP[o['type']]}, occluded=o['occ'], npts=npts,
                             range=float(np.hypot(c_v[0], c_v[1])), in_fov=True))
            n_obj += 1
    pd.DataFrame(rows).to_csv(f'{out}_gt.csv', index=False, float_format='%.4f')
    json.dump(dict(seq=a.seq, split=a.split, rate_hz=a.rate, t0_ns=t0, t1_ns=t_ns[frames[-1]], frames=len(frames),
                   annotated_t_ns=[t_ns[i] for i in frames], classes=sorted(set(CLASS_MAP.values())), frame='odom',
                   lidar_height_m=LIDAR_HEIGHT, T_base_velodyne=T_base_v.tolist(), image_size=[W, H],
                   cameras=['cam2'], gt_fov='image_02 only (coda_eval --camera-only --cam-json <out>_cam.json)'),
              open(f'{out}_gt.json', 'w'), indent=1)
    T_c2_base = T_c2_v @ T_v_base
    json.dump(dict(K=K.tolist(), img_shape=[H, W], frames=[dict(t=t_ns[i] / 1e9, T_cam_odom=(T_c2_base @ np.linalg.inv(T_odom_base(i))).tolist())
                                                          for i in frames]), open(f'{out}_cam.json', 'w'))
    print(f'done: {len(frames)} scans, {n_obj} GT rows, image {W}x{H}')


if __name__ == '__main__':
    main()
