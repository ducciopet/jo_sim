"""SiT (Bae et al., NeurIPS 2023) scene zip readers: pcd (PCL binary_compressed), ego pose, labels, calibration."""
import struct
import zipfile

import lzf
import numpy as np


class Scene:
    def __init__(self, zip_path):
        self.z = zipfile.ZipFile(zip_path)
        names = self.z.namelist()
        self.pre = [n for n in names if 'velo/top/data/' in n][0].split('velo/')[0]
        self.frames = sorted(int(n.rsplit('/', 1)[1][:-4]) for n in names if '/velo/top/data/' in '/' + n and n.endswith('.pcd'))
        self.has_labels = any('label_3d/' in n for n in names)

    def read(self, rel):
        return self.z.read(self.pre + rel)

    def cloud(self, f, which='top'):
        """x y z intensity (sensor frame of `which`; concat is in the top-sensor frame)."""
        data = self.read(f'velo/{which}/data/{f}.pcd')
        pos, hdr = 0, {}
        while True:
            e = data.index(b'\n', pos)
            k, *v = data[pos:e].decode().split()
            hdr[k] = v
            pos = e + 1
            if k == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        names, sizes, types = hdr['FIELDS'], [int(s) for s in hdr['SIZE']], hdr['TYPE']
        assert hdr['DATA'][0] == 'binary_compressed'
        csize, usize = struct.unpack('<II', data[pos:pos + 8])
        raw = lzf.decompress(data[pos + 8:pos + 8 + csize], usize)
        cols, off = {}, 0
        for nm, sz, ty in zip(names, sizes, types):
            cols[nm] = np.frombuffer(raw, np.dtype({'F': 'f', 'U': 'u', 'I': 'i'}[ty] + str(sz)), n, off)
            off += n * sz
        p = np.stack([cols['x'], cols['y'], cols['z'], cols['intensity']], 1).astype(np.float32)
        return p[np.isfinite(p).all(1) & (np.linalg.norm(p[:, :3], axis=1) > 0.3)]

    def pose(self, f):
        """T_world_top (4x4)."""
        return np.array(self.read(f'ego_trajectory/{f}.txt').decode().split(','), float).reshape(4, 4)

    def labels_3d(self, f):
        """[(class, track id str, a, b, c, x, y, z, yaw)] raw fields, world frame."""
        try:
            txt = self.read(f'label_3d/{f}.txt').decode()
        except KeyError:
            return []
        out = []
        for ln in txt.splitlines():
            q = ln.split()
            if len(q) >= 9:
                out.append((q[0], q[1], *map(float, q[2:9])))
        return out

    def labels_2d(self, f):
        """[(class, track id str, cam 1..5, x, y, w, h)]."""
        try:
            txt = self.read(f'label_2d/{f}.txt').decode()
        except KeyError:
            return []
        out = []
        for ln in txt.splitlines():
            q = ln.split()
            if len(q) >= 7:
                out.append((q[0], q[1], int(q[2]), *map(float, q[3:7])))
        return out

    def calib(self, f=0):
        """{k: (K 3x3, E 3x4)} for P0..P4 (camera k+1)."""
        d = {}
        for ln in self.read(f'calib/{f}.txt').decode().splitlines():
            if ':' in ln:
                k, v = ln.split(':', 1)
                d[k.strip()] = np.array(v.split(), float)
        return {i: (d[f'P{i}_intrinsic'].reshape(3, 3), d[f'P{i}_extrinsic'].reshape(3, 4)) for i in range(5)
                if f'P{i}_intrinsic' in d}

    def image_bytes(self, f, cam):
        return self.read(f'cam_img/{cam}/data_rgb/{f}.png')
