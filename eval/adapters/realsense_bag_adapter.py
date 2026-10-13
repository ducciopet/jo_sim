#!/usr/bin/env python3
"""
realsense_bag_adapter.py
─────────────────────────────────────────────────────────────────────────────
Input adapter for bags recorded with fewer RealSense streams than
jo_detector_v2 expects (e.g. bags/bags_benchmark, 2026-05-19: raw color,
raw unaligned depth, no extrinsics topic). Publishes, per camera, the two
streams that can be derived exactly from what the bag already contains:

  /<cam>/camera/extrinsics/depth_to_color      realsense2_camera_msgs/Extrinsics
      from the bag's own static TF (color_optical <- depth_optical), latched.
      Verified identical (1e-6) to the driver's message on
      bags/20260910_124939_validation_lab, which has both.
  /<cam>/camera/color/image_raw/compressed     sensor_msgs/CompressedImage
      JPEG re-encoding of the raw color stream (v2's YOLO nodes only read
      the compressed transport).

  /<cam>/camera/depth/color/points               sensor_msgs/PointCloud2
      deprojection of the raw depth image with the depth camera's own
      intrinsics, in <cam>_depth_optical_frame, invalid (0) pixels dropped:
      the same thing the RealSense "pointcloud" filter publishes, minus color.
      (depth_image_proc's point_cloud_xyz_node would do the same, but its lazy
      subscription never triggers under rmw_cyclonedds in this container.)

This is an I/O adapter only: no data is invented, every output is a lossless
(extrinsics) or near-lossless (JPEG q=95) re-encoding of bag content.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from realsense2_camera_msgs.msg import Extrinsics
from tf2_ros import Buffer, TransformListener


def quat_to_rot(x, y, z, w):
    # scipy is unusable in the container (system scipy built for numpy 1.x)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class RealsenseBagAdapter(Node):
    def __init__(self):
        super().__init__('realsense_bag_adapter')
        self.cameras = list(self.declare_parameter('camera_names', ['front_camera']).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 95).value)
        self.depth_scale = float(self.declare_parameter('depth_scale', 1000.0).value)  # raw units per meter (RealSense: mm)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.extr_pubs = {}
        self.extr_done = set()
        self.img_pubs = {}
        self.cloud_pubs = {}
        self.depth_K = {}
        for cam in self.cameras:
            self.extr_pubs[cam] = self.create_publisher(
                Extrinsics, f'/{cam}/camera/extrinsics/depth_to_color', latched)
            self.img_pubs[cam] = self.create_publisher(
                CompressedImage, f'/{cam}/camera/color/image_raw/compressed', 10)
            self.create_subscription(
                Image, f'/{cam}/camera/color/image_raw',
                lambda msg, cam=cam: self.on_color(cam, msg), 10)
            self.cloud_pubs[cam] = self.create_publisher(
                PointCloud2, f'/{cam}/camera/depth/color/points', 10)
            self.create_subscription(
                CameraInfo, f'/{cam}/camera/depth/camera_info',
                lambda msg, cam=cam: self.depth_K.__setitem__(cam, np.array(msg.k).reshape(3, 3)), 10)
            self.create_subscription(
                Image, f'/{cam}/camera/depth/image_rect_raw',
                lambda msg, cam=cam: self.on_depth(cam, msg), 10)
        self.create_timer(0.5, self.publish_extrinsics)

    def publish_extrinsics(self):
        for cam in self.cameras:
            if cam in self.extr_done:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    f'{cam}_color_optical_frame', f'{cam}_depth_optical_frame', rclpy.time.Time())
            except Exception:
                continue
            q = tf.transform.rotation
            t = tf.transform.translation
            R = quat_to_rot(q.x, q.y, q.z, q.w)
            msg = Extrinsics()
            msg.rotation = R.flatten(order='F').tolist()  # rs2_extrinsics: column-major
            msg.translation = [t.x, t.y, t.z]
            self.extr_pubs[cam].publish(msg)
            self.extr_done.add(cam)
            self.get_logger().info(f'[{cam}] depth_to_color extrinsics published from TF: t={np.round(msg.translation, 4)}')

    def on_color(self, cam, msg):
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding != 'bgr8':
            self.get_logger().error(f'unsupported color encoding {msg.encoding}', throttle_duration_sec=5.0)
            return
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'bgr8; jpeg compressed bgr8'
        out.data = buf.tobytes()
        self.img_pubs[cam].publish(out)


    def on_depth(self, cam, msg):
        K = self.depth_K.get(cam)
        if K is None:
            return
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().error(f'unsupported depth encoding {msg.encoding}', throttle_duration_sec=5.0)
            return
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        v, u = np.nonzero(d)
        z = d[v, u].astype(np.float32) / self.depth_scale
        x = (u.astype(np.float32) - K[0, 2]) * z / K[0, 0]
        y = (v.astype(np.float32) - K[1, 2]) * z / K[1, 1]
        pts = np.stack([x, y, z], axis=1).astype(np.float32)
        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * len(pts)
        out.is_dense = True
        out.data = pts.tobytes()
        self.cloud_pubs[cam].publish(out)


def main():
    rclpy.init()
    rclpy.spin(RealsenseBagAdapter())


if __name__ == '__main__':
    main()
