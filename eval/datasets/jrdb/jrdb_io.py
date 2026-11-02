"""JRDB (train_dataset_with_activity) readers: PCD (binary_compressed), calibration, 3D labels."""
import json
import struct

import lzf
import numpy as np
import yaml


def read_pcd(path):
    """x y z intensity ring of a JRDB .pcd (PCL binary_compressed: lzf of the column-major field arrays)."""
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            line = f.readline().decode().strip()
            k, *v = line.split()
            hdr[k] = v
            if k == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        names, sizes, types = hdr['FIELDS'], [int(s) for s in hdr['SIZE']], hdr['TYPE']
        if hdr['DATA'][0] == 'binary_compressed':
            csize, usize = struct.unpack('<II', f.read(8))
            raw = lzf.decompress(f.read(csize), usize)
            cols, off = {}, 0
            for nm, sz, ty in zip(names, sizes, types):
                dt = np.dtype({'F': 'f', 'U': 'u', 'I': 'i'}[ty] + str(sz))
                cols[nm] = np.frombuffer(raw, dt, n, off)
                off += n * sz
        else:
            dt = np.dtype([(nm, {'F': 'f', 'U': 'u', 'I': 'i'}[ty] + str(sz)) for nm, sz, ty in zip(names, sizes, types)])
            a = np.frombuffer(f.read(n * dt.itemsize), dt, n)
            cols = {nm: a[nm] for nm in names}
    return np.stack([cols['x'], cols['y'], cols['z'], cols['intensity']], 1).astype(np.float32)


def calib(root):
    lid = yaml.safe_load(open(f'{root}/calibration/lidars.yaml'))
    return {k: (np.array(v) if isinstance(v, list) else {kk: np.array(vv) for kk, vv in v.items()})
            for k, v in lid.items()}


def labels_3d(root, seq):
    return json.load(open(f'{root}/labels/labels_3d/{seq}.json'))['labels']
