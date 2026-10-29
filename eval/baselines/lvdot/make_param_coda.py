#!/usr/bin/env python3
"""
LV-DOT parameter file for one CODa window: detector_param_jo.yaml with only the sensor I/O replaced (lines marked
CODa): ZED left intrinsics (depth registered to it, 960x540), base_link -> camera / LiDAR from the window's ground fit
(<window>_gt.json) and CODa's os1 -> cam3 calibration, floor/roof at LV-DOT's own defaults (odom origin = base_link on
the ground). Every detection threshold keeps the value of the lab runs (= upstream).
usage: make_param_coda.py <window dir e.g. bags/coda/coda0_w1> <CODa root> <out.yaml>
"""
import json
import re
import sys

import numpy as np
import yaml

win, root, out = sys.argv[1:4]
src = open(__file__.replace('make_param_coda.py', 'detector_param_jo.yaml')).read()
T_bv = np.array(json.load(open(f'{win}_gt.json'))['T_base_velodyne'])
E = np.array(yaml.safe_load(open(f'{root}/calibrations/0/calib_os1_to_cam3.yaml'))['extrinsic_matrix']['data']).reshape(4, 4)
K = yaml.safe_load(open(f'{root}/calibrations/0/calib_cam3_intrinsics.yaml'))['camera_matrix']['data']
T_bc = T_bv @ np.linalg.inv(E)


def mat(name, T, note):
    rows = [', '.join(f'{v:.6f}' for v in r) for r in T]
    pad = ' ' * (len(name) + 3)
    return f'{name}: [{rows[0]},   # CODa: {note}\n' + ',\n'.join(pad + r for r in rows[1:]) + ']'


def sub(pattern, repl):
    global src
    src, n = re.subn(pattern, repl, src, flags=re.S | re.M)
    assert n == 1, pattern


intr = f'[{K[0]}, {K[4]}, {K[2]}, {K[5]}]'
sub(r'^depth_intrinsics:.*?$', f'depth_intrinsics: {intr} # CODa: ZED left (cam3), depth registered to it (960x540)')
sub(r'^color_intrinsics:.*?$', f'color_intrinsics: {intr} # CODa: ZED left (cam3)')
sub(r'^image_cols:.*?$', 'image_cols: 960 # CODa')
sub(r'^image_rows:.*?$', 'image_rows: 540 # CODa')
sub(r'^body_to_camera_depth:.*?\]', mat('body_to_camera_depth', T_bc, 'base_link -> cam3 (ground fit + calib_os1_to_cam3)'))
sub(r'^body_to_camera_color:.*?\]', mat('body_to_camera_color', T_bc, 'same frame as the depth'))
sub(r'^body_to_lidar:.*?\]', mat('body_to_lidar', T_bv, 'base_link -> os1 (ground fit)'))
sub(r'^ground_height:.*?$', 'ground_height: 0.2 # CODa: LV-DOT default (odom origin = base_link on the ground)')
sub(r'^roof_height:.*?$', 'roof_height: 2.0 # CODa: LV-DOT default')
sub(r'^# LV-DOT detector_param.yaml.*?$', '# LV-DOT detector_param.yaml with only the sensor I/O adapted (JO lab robot, then CODa: lines marked CODa).')
open(out, 'w').write(src)
