#!/usr/bin/env python3
"""
CODa stereo pair of one converted window -> an extra ROS2 bag played next to the window's sensor bag.

CODa's stereo cameras (cam0 left / cam1 right, rectified, 1224x1024) have one frame per LiDAR scan (same frame index,
8213 = 8213 in seq 0): unlike the ZED (cam3, ~15% of its frames missing, not synchronised) every scan has its image.
Written as a second camera, 'stereo_camera', in the same RealSense-style layout as front_camera:
  /stereo_camera/camera/color/image_raw + camera_info   cam0 rectified (rgb8), K = P0[:3,:3]
  /stereo_camera/right/image_raw + camera_info          cam1 rectified (mono8), P with Tx = -fx*B (CODa stores it /1000)
  /tf_static  velodyne -> stereo_camera_depth_optical_frame (rectified left frame: R0 * E_os1->cam0), and identity
              depth -> color optical: the depth image is computed online on the left image by
              eval/adapters/stereo_depth_node.py, as a stereo camera's driver would.
Stamps = the scan stamps of the same frame index. No depth in this bag.

usage: coda_stereo_bag.py <coda root> <window gt json, e.g. bags/coda/coda0_w1_gt.json> <out bag>
"""
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from rosbags.rosbag2 import StoragePlugin, Writer

sys.path.insert(0, str(Path(__file__).resolve().parent))
from coda_to_bag import LATCHED, M, TS, header, image_msg, read_matrix, transform_msg  # noqa: E402

CAM = 'stereo_camera'


def mat(d, key):
    m = d[key]
    return np.array(m['data'], float).reshape(m['rows'], m['cols'])


def mono8_msg(img, ns, frame):
    return M['sensor_msgs/msg/Image'](header=header(ns, frame), height=img.shape[0], width=img.shape[1],
                                      encoding='mono8', is_bigendian=0, step=img.shape[1],
                                      data=np.ascontiguousarray(img).ravel())


def info(P, w, h, ns, frame):
    K = P[:3, :3]
    return M['sensor_msgs/msg/CameraInfo'](
        header=header(ns, frame), height=h, width=w, distortion_model='plumb_bob', d=np.zeros(5), k=K.ravel(),
        r=np.eye(3).ravel(), p=P[:3].ravel(), binning_x=0, binning_y=0,
        roi=M['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def main():
    root, gt_json, out = sys.argv[1:4]
    w = json.load(open(gt_json))
    seq, first, last = w['seq'], w['first'], w['last']
    t_scan = (np.loadtxt(f'{root}/timestamps/{seq}.txt') * 1e9).astype(np.int64)
    calib = f'{root}/calibrations/{seq}'
    E0 = read_matrix(f'{calib}/calib_os1_to_cam0.yaml')
    i0 = yaml.safe_load(open(f'{calib}/calib_cam0_intrinsics.yaml'))
    i1 = yaml.safe_load(open(f'{calib}/calib_cam1_intrinsics.yaml'))
    R0 = np.eye(4)
    R0[:3, :3] = mat(i0, 'rectification_matrix')
    P0, P1 = mat(i0, 'projection_matrix'), mat(i1, 'projection_matrix')
    if abs(P1[0, 3]) < 1.0:
        P1[0, 3] *= 1000.0
    W, H = i0['image_width'], i0['image_height']
    t0 = int(t_scan[first])
    if Path(out).exists():
        raise SystemExit(f'{out} exists')
    with Writer(out, version=9, storage_plugin=StoragePlugin.MCAP) as wr:
        c_tfs = wr.add_connection('/tf_static', 'tf2_msgs/msg/TFMessage', typestore=TS, offered_qos_profiles=LATCHED)
        c_l = wr.add_connection(f'/{CAM}/camera/color/image_raw', 'sensor_msgs/msg/Image', typestore=TS)
        c_li = wr.add_connection(f'/{CAM}/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=TS)
        c_r = wr.add_connection(f'/{CAM}/right/image_raw', 'sensor_msgs/msg/Image', typestore=TS)
        c_ri = wr.add_connection(f'/{CAM}/right/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=TS)
        statics = [transform_msg(np.linalg.inv(R0 @ E0), 'velodyne', f'{CAM}_depth_optical_frame', t0),
                   transform_msg(np.eye(4), f'{CAM}_depth_optical_frame', f'{CAM}_color_optical_frame', t0)]
        wr.write(c_tfs, t0 - 1, TS.serialize_cdr(M['tf2_msgs/msg/TFMessage'](transforms=statics), 'tf2_msgs/msg/TFMessage'))
        for k, i in enumerate(range(first, last + 1)):
            t = int(t_scan[i])
            left = np.array(Image.open(f'{root}/2d_rect/cam0/{seq}/2d_rect_cam0_{seq}_{i}.png').convert('RGB'))
            right = np.array(Image.open(f'{root}/2d_rect/cam1/{seq}/2d_rect_cam1_{seq}_{i}.png').convert('L'))
            wr.write(c_l, t, TS.serialize_cdr(image_msg(left, t, f'{CAM}_color_optical_frame', 'rgb8'), 'sensor_msgs/msg/Image'))
            wr.write(c_li, t, TS.serialize_cdr(info(np.vstack([P0, [0, 0, 0, 1]]), W, H, t, f'{CAM}_color_optical_frame'),
                                               'sensor_msgs/msg/CameraInfo'))
            wr.write(c_r, t, TS.serialize_cdr(mono8_msg(right, t, f'{CAM}_right_optical_frame'), 'sensor_msgs/msg/Image'))
            wr.write(c_ri, t, TS.serialize_cdr(info(np.vstack([P1, [0, 0, 0, 1]]), W, H, t, f'{CAM}_right_optical_frame'),
                                               'sensor_msgs/msg/CameraInfo'))
            if k % 100 == 0:
                print(f'{k}/{last - first + 1}', flush=True)
    print(out)


if __name__ == '__main__':
    main()
