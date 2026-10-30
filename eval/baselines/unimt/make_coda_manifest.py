#!/usr/bin/env python3
"""
Input manifest for UniMT on one CODa window (offline, same frames as eval/datasets/coda_to_bag.py): per LiDAR scan the
OS1 file, the two rectified stereo images (cam0, cam1: CMDT's CODa model is trained on both), and ego2global = pose of
the OS1 in the window's odom frame (the frame of the GT and of every other method's output). Calibration as for a
ROS stereo pair: points go to the rectified left frame (R0 · E_os1->cam0), cam0 projects with P0, cam1 with P1 whose
Tx = -fx·B, here moved into cam1's extrinsic (CODa stores it divided by 1000: -0.151 for B = 0.196 m, fx = 769).

usage: make_coda_manifest.py <coda root> <window gt json, e.g. bags/coda/coda0_w1_gt.json> <out .npz>
"""
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'datasets'))
from coda_to_bag import ground_frame, load_scan, read_matrix, rot_from_quat  # noqa: E402


def mat(d, key):
    m = d[key]
    return np.array(m['data'], float).reshape(m['rows'], m['cols'])


def main():
    root, gt_json, out = sys.argv[1:4]
    w = json.load(open(gt_json))
    seq, first, last = w['seq'], w['first'], w['last']
    t_scan = (np.loadtxt(f'{root}/timestamps/{seq}.txt') * 1e9).astype(np.int64)
    poses = np.loadtxt(f'{root}/poses/dense/{seq}.txt')
    T_os1_base = ground_frame(load_scan(root, seq, first))[0]     # same odom as coda_to_bag (deterministic RANSAC seed)
    T_base_os1 = np.linalg.inv(T_os1_base)

    def T_world_os1(i):
        x, y, z, qw, qx, qy, qz = poses[i, 1:]
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rot_from_quat(qx, qy, qz, qw), [x, y, z]
        return T

    T_odom_world = T_base_os1 @ np.linalg.inv(T_world_os1(0))
    calib = f'{root}/calibrations/{seq}'
    E0 = read_matrix(f'{calib}/calib_os1_to_cam0.yaml')
    i0 = yaml.safe_load(open(f'{calib}/calib_cam0_intrinsics.yaml'))
    i1 = yaml.safe_load(open(f'{calib}/calib_cam1_intrinsics.yaml'))
    R0 = np.eye(4)
    R0[:3, :3] = mat(i0, 'rectification_matrix')
    lidar2cam = R0 @ E0
    P0, P1 = np.eye(4), np.eye(4)
    P0[:3] = mat(i0, 'projection_matrix')
    P1[:3] = mat(i1, 'projection_matrix')
    if abs(P1[0, 3]) < 1.0:
        P1[0, 3] *= 1000.0
    # CMDT's loader (LoadMultiViewImageFromFilesNus) keeps only the 3x3 of cam2img: the baseline goes into cam1's
    # extrinsic instead (rectified right frame = rectified left frame shifted by Tx/fx along x)
    shift = np.eye(4)
    shift[0, 3] = P1[0, 3] / P1[0, 0]
    frames = np.arange(first, last + 1)
    np.savez(out, t_ns=t_scan[frames], frame=frames,
             ego2global=np.stack([T_odom_world @ T_world_os1(i) for i in frames]),
             lidar=np.array([f'{root}/3d_comp/os1/{seq}/3d_comp_os1_{seq}_{i}.bin' for i in frames]),
             cam0=np.array([f'{root}/2d_rect/cam0/{seq}/2d_rect_cam0_{seq}_{i}.png' for i in frames]),
             cam1=np.array([f'{root}/2d_rect/cam1/{seq}/2d_rect_cam1_{seq}_{i}.png' for i in frames]),
             cam2img=np.stack([P0[:3, :3], P1[:3, :3]]), lidar2cam=np.stack([lidar2cam, shift @ lidar2cam]))
    print(out, len(frames), 'frames; P1 Tx', P1[0, 3])


if __name__ == '__main__':
    main()
