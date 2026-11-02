#!/usr/bin/env python3
"""
lidar_depth_node.py
─────────────────────────────────────────────────────────────────────────────
Online depth for color cameras that have none (JRDB: five color cameras, no depth sensor), from the LiDAR scan that
SINS already receives: each scan is projected into every camera and published in the RealSense-style layout SINS
reads, as stereo_depth_node.py does for a stereo pair.

  in   /velodyne_points, /<cam>/camera/color/camera_info, static TF velodyne -> <cam>_depth_optical_frame
  out  /<cam>/camera/depth/image_rect_raw  16UC1 [mm], aligned to the color image (depth optical == color optical),
       stamp = the scan's; /<cam>/camera/depth/camera_info = the color intrinsics

Each LiDAR return lands on one pixel (nearest return wins where several fall on the same pixel); every other pixel is
0 = invalid, as a depth driver writes holes. No densification: the depth carries exactly the LiDAR's information, and
realsense_bag_adapter.py turns it into the depth cloud like any other camera. Stateless and causal (one scan in, one
image per camera out), a few ms per scan.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener


def quat_to_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class LidarDepth(Node):
    def __init__(self):
        super().__init__('lidar_depth_node')
        self.cams = list(self.declare_parameter('camera_names', ['cam0']).value)
        self.min_depth = float(self.declare_parameter('min_depth', 0.3).value)
        self.max_depth = float(self.declare_parameter('max_depth', 30.0).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.info, self.T, self.pub, self.pub_info = {}, {}, {}, {}
        for c in self.cams:
            self.create_subscription(CameraInfo, f'/{c}/camera/color/camera_info',
                                     lambda m, c=c: self.info.__setitem__(c, m), 10)
            self.pub[c] = self.create_publisher(Image, f'/{c}/camera/depth/image_rect_raw', 10)
            self.pub_info[c] = self.create_publisher(CameraInfo, f'/{c}/camera/depth/camera_info', 10)
        self.create_subscription(PointCloud2, '/velodyne_points', self.on_cloud, 10)
        self.times = []

    def transform(self, cam, src):
        if cam not in self.T:
            try:
                t = self.tf_buffer.lookup_transform(f'{cam}_depth_optical_frame', src, rclpy.time.Time())
            except Exception:
                return None
            q, p = t.transform.rotation, t.transform.translation
            T = np.eye(4)
            T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
            T[:3, 3] = [p.x, p.y, p.z]
            self.T[cam] = T
        return self.T[cam]

    def on_cloud(self, m):
        t0 = time.time()
        off = {f.name: f.offset for f in m.fields}
        raw = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
        P = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1).astype(np.float64)
        P = P[np.isfinite(P).all(1)]
        for c in self.cams:
            info = self.info.get(c)
            T = self.transform(c, m.header.frame_id)
            if info is None or T is None:
                continue
            q = P @ T[:3, :3].T + T[:3, 3]
            q = q[(q[:, 2] > self.min_depth) & (q[:, 2] < self.max_depth)]
            K = np.array(info.k).reshape(3, 3)
            u = np.round(K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(int)
            v = np.round(K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(int)
            ok = (u >= 0) & (u < info.width) & (v >= 0) & (v < info.height)
            u, v, z = u[ok], v[ok], (q[ok, 2] * 1000.0).astype(np.uint16)
            order = np.argsort(-z)                      # far first, so the nearest return is written last and wins
            depth = np.zeros((info.height, info.width), np.uint16)
            depth[v[order], u[order]] = z[order]
            out = Image()
            out.header.stamp = m.header.stamp
            out.header.frame_id = f'{c}_depth_optical_frame'
            out.height, out.width = info.height, info.width
            out.encoding, out.is_bigendian, out.step = '16UC1', 0, info.width * 2
            out.data = depth.tobytes()
            di = CameraInfo()
            di.header = out.header
            di.height, di.width, di.distortion_model = info.height, info.width, info.distortion_model
            di.d, di.k, di.r, di.p = info.d, info.k, info.r, info.p
            self.pub[c].publish(out)
            self.pub_info[c].publish(di)
        self.times.append(time.time() - t0)
        if len(self.times) == 100:
            self.get_logger().info(f'lidar depth: {1e3 * np.mean(self.times):.1f} ms/scan for {len(self.cams)} cameras')
            self.times = []


def main():
    rclpy.init()
    rclpy.spin(LidarDepth())


if __name__ == '__main__':
    main()
