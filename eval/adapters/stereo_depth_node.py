#!/usr/bin/env python3
"""
stereo_depth_node.py
─────────────────────────────────────────────────────────────────────────────
Online depth for a rectified stereo pair, in the RealSense-style layout SINS reads: what a stereo camera's driver
(ZED SDK, RealSense) does on the robot, here for CODa's cam0/cam1 (eval/datasets/coda_stereo_bag.py).

  in   /<cam>/camera/color/image_raw (rectified left, rgb8) + /<cam>/right/image_raw (rectified right, mono8) and
       their camera_info, paired by identical stamp (the pair is triggered together)
  out  /<cam>/camera/depth/image_rect_raw  16UC1 [mm], aligned to the left image, frame <cam>_depth_optical_frame
       /<cam>/camera/depth/camera_info     the left camera's intrinsics

OpenCV semi-global matching (SGBM 3-way) on the pair downscaled by `scale` (0.5: ~25 ms per pair on this CPU for
1224x1024, inside the 100 ms of a 10 Hz scan), disparity scaled back to full resolution, depth = fx * B / d with
B = -Tx / fx from the right camera's P. Pixels without a valid match, or farther than max_depth, are 0 (invalid,
as the RealSense driver writes them). Stateless and causal: each pair is processed when it arrives.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class StereoDepth(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')
        self.cam = self.declare_parameter('camera', 'stereo_camera').value
        self.scale = float(self.declare_parameter('scale', 0.5).value)
        self.max_depth = float(self.declare_parameter('max_depth', 20.0).value)
        self.num_disp = int(self.declare_parameter('num_disparities', 96).value)   # at the downscaled size
        self.block = int(self.declare_parameter('block_size', 5).value)
        c = self.cam
        self.left, self.right = {}, {}
        self.K = self.B = None
        self.create_subscription(Image, f'/{c}/camera/color/image_raw', lambda m: self.on_img(m, self.left), 10)
        self.create_subscription(Image, f'/{c}/right/image_raw', lambda m: self.on_img(m, self.right), 10)
        self.create_subscription(CameraInfo, f'/{c}/camera/color/camera_info', self.on_left_info, 10)
        self.create_subscription(CameraInfo, f'/{c}/right/camera_info', self.on_right_info, 10)
        self.pub = self.create_publisher(Image, f'/{c}/camera/depth/image_rect_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, f'/{c}/camera/depth/camera_info', 10)
        self.left_info = None
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=self.num_disp, blockSize=self.block,
            P1=8 * self.block ** 2, P2=32 * self.block ** 2, uniquenessRatio=10, speckleWindowSize=100,
            speckleRange=2, disp12MaxDiff=1, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.times = []

    def on_left_info(self, m):
        self.left_info = m
        self.K = np.array(m.k).reshape(3, 3)

    def on_right_info(self, m):
        fx, tx = m.p[0], m.p[3]
        if fx > 0 and tx != 0:
            self.B = -tx / fx

    @staticmethod
    def key(m):
        return (m.header.stamp.sec, m.header.stamp.nanosec)

    def on_img(self, m, store):
        store[self.key(m)] = m
        for d in (self.left, self.right):          # keep the buffers short
            while len(d) > 10:
                d.pop(next(iter(d)))
        k = self.key(m)
        if k in self.left and k in self.right and self.K is not None and self.B and self.left_info is not None:
            self.process(self.left.pop(k), self.right.pop(k))

    def process(self, lm, rm):
        t0 = time.time()
        L = np.frombuffer(lm.data, np.uint8).reshape(lm.height, lm.width, -1)
        L = cv2.cvtColor(L, cv2.COLOR_RGB2GRAY) if L.shape[2] == 3 else L[:, :, 0]
        R = np.frombuffer(rm.data, np.uint8).reshape(rm.height, rm.width)
        Ls = cv2.resize(L, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        Rs = cv2.resize(R, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        disp = self.sgbm.compute(Ls, Rs).astype(np.float32) / 16.0          # fixed point, 4 fractional bits
        disp = cv2.resize(disp, (lm.width, lm.height), interpolation=cv2.INTER_NEAREST) / self.scale
        depth = np.zeros_like(disp)
        ok = disp > 0.5
        depth[ok] = self.K[0, 0] * self.B / disp[ok]
        depth[(depth > self.max_depth) | ~np.isfinite(depth)] = 0.0
        out = Image()
        out.header.stamp = lm.header.stamp
        out.header.frame_id = f'{self.cam}_depth_optical_frame'
        out.height, out.width = lm.height, lm.width
        out.encoding, out.is_bigendian, out.step = '16UC1', 0, lm.width * 2
        out.data = (depth * 1000.0).astype(np.uint16).tobytes()
        info = self.left_info
        info.header = out.header
        self.pub.publish(out)
        self.pub_info.publish(info)
        self.times.append(time.time() - t0)
        if len(self.times) == 50:
            self.get_logger().info(f'stereo depth: {1e3 * np.mean(self.times):.1f} ms/pair (max {1e3 * np.max(self.times):.1f})')
            self.times = []


def main():
    rclpy.init()
    rclpy.spin(StereoDepth())


if __name__ == '__main__':
    main()
