#!/usr/bin/env python3
"""
Input bag for Xu et al. 2023 WITH the learning (YOLO) module (ROS1): depth, color, odometry + an aligned depth.

The YOLO branch reads the color image and a depth image aligned to it (RealSense /aligned_depth_to_color, which our
bags do not record), and the C++ node walks both with one image size (image_cols x image_rows, the depth's). So the
color stream is resized to the depth size, 848x480 (intrinsics scaled accordingly, see detector_param_jo_yolo.yaml),
and every depth frame is re-projected into that resized color camera, as librealsense's align does: back-project
with the depth intrinsics, shift by the URDF depth->color offset (optical frames have the same orientation; the color
optical origin is +0.059 m along optical x), project with the color intrinsics, keep the nearest depth per pixel.
Input adapter only; nothing of the method changes.

usage: make_input_yolo.py <seq e.g. 20260519_1950> <out.bag>
"""

import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

R = Path(__file__).resolve().parents[3]
DEPTH_K = (436.965, 436.965, 431.921, 240.134)        # 848x480, bag camera_info
COLOR_K = (644.308, 643.637, 650.24, 374.757)         # 1280x720, bag camera_info
W, H = 848, 480                                        # target size = depth size
SX, SY = W / 1280.0, H / 720.0
COLOR_K_RS = (COLOR_K[0] * SX, COLOR_K[1] * SY, COLOR_K[2] * SX, COLOR_K[3] * SY)
DEPTH_TO_COLOR = np.array([-0.059, 0.0, 0.0])          # point in depth optical -> color optical (URDF)
DEPTH, COLOR = '/front_camera/camera/depth/image_rect_raw', '/front_camera/camera/color/image_raw'


def align(depth):
    fx, fy, cx, cy = DEPTH_K
    v, u = np.nonzero(depth)
    z = depth[v, u].astype(np.float64) / 1000.0
    p = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1) + DEPTH_TO_COLOR
    fx, fy, cx, cy = COLOR_K_RS
    uc = np.round(p[:, 0] * fx / p[:, 2] + cx).astype(int)
    vc = np.round(p[:, 1] * fy / p[:, 2] + cy).astype(int)
    ok = (p[:, 2] > 0) & (uc >= 0) & (uc < W) & (vc >= 0) & (vc < H)
    uc, vc, zz = uc[ok], vc[ok], np.round(p[ok, 2] * 1000.0).astype(np.uint16)
    out = np.zeros((H, W), np.uint16)
    order = np.argsort(-zz.astype(np.int64))  # far first, near last: the nearest depth wins
    out[vc[order], uc[order]] = zz[order]
    return out


def main():
    seq, out = sys.argv[1], Path(sys.argv[2])
    ts1 = get_typestore(Stores.ROS1_NOETIC)
    tmp = out.with_suffix('.raw.bag')
    for f in (out, tmp):
        f.unlink(missing_ok=True)
    subprocess.run(['rosbags-convert', '--src', str(R / 'bags' / 'bags_benchmark' / f'duccio_test_{seq}_0'),
                    str(R / 'bags' / 'replay' / f'odom_{seq}'), '--dst', str(tmp), '--dst-typestore', 'ros1_noetic',
                    '--include-topic', DEPTH, COLOR, '/odometry/filtered'], check=True)
    Img = 'sensor_msgs/msg/Image'
    with Reader(tmp) as r, Writer(out) as w:
        cons = {c.topic: w.add_connection(c.topic, c.msgtype, typestore=ts1) for c in r.connections}
        c_al = w.add_connection('/camera/aligned_depth_to_color/image_raw', Img, typestore=ts1)
        for c, t, raw in r.messages():
            if c.topic == COLOR:
                m = ts1.deserialize_ros1(raw, Img)
                img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
                img = np.asarray(PILImage.fromarray(img).resize((W, H), PILImage.BOX))  # area average
                m.height, m.width, m.step, m.data = H, W, W * img.shape[2], np.ascontiguousarray(img).reshape(-1)
                w.write(cons[c.topic], t, ts1.serialize_ros1(m, Img))
            elif c.topic == DEPTH:
                w.write(cons[c.topic], t, raw)
                m = ts1.deserialize_ros1(raw, Img)
                d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
                m.data = align(d).view(np.uint8).reshape(-1)
                m.height, m.width, m.step = H, W, W * 2
                m.header.frame_id = 'front_camera_color_optical_frame'
                w.write(c_al, t, ts1.serialize_ros1(m, Img))
            else:
                w.write(cons[c.topic], t, raw)
    tmp.unlink()
    print('wrote', out)


if __name__ == '__main__':
    main()
