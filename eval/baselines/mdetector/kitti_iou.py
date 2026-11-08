#!/usr/bin/env python3
"""Point-wise moving-event IoU on the M-detector KITTI release, same rules as upstream src/cal_recall_multi.cpp.

GT label (low 16 bits): 1000..65534 = moving object point, <251 = static, 251..999 and >=65535 = ignored.
Prediction: 251..65534 = moving. IoU = TP / (TP + FN + FP), pooled over all frames of a sequence ("Average iou" in
upstream). Reads the Figshare archive directly (KITTI.zip from 10.6084/m9.figshare.24481966).

usage: kitti_iou.py KITTI.zip [--pred predictions] [--pred-dir DIR]   (DIR = our own predictions, same layout)
"""
import argparse, os, re, sys, zipfile
import numpy as np


def counts(gt, pr):
    gt = gt & 0xFFFF
    pr = pr & 0xFFFF
    g_mov = (gt >= 1000) & (gt < 65535)
    g_sta = gt < 251
    p_mov = (pr >= 251) & (pr < 65535)
    return int((g_mov & p_mov).sum()), int((g_mov & ~p_mov).sum()), int((g_sta & p_mov).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('zip')
    ap.add_argument('--pred', nargs='+', default=None, help='prediction folder names in the zip (default: all)')
    ap.add_argument('--pred-dir', help='root of our own predictions: <dir>/<seq>/*.label')
    a = ap.parse_args()
    z = zipfile.ZipFile(a.zip)
    names = z.namelist()
    seqs = sorted({m[1] for n in names if (m := re.match(r'sequences/(\d+)/labels/', n))})
    preds = a.pred or sorted({m[1] for n in names if (m := re.match(r'sequences/\d+/(predictions[^/]*)/', n))})
    if a.pred_dir:
        preds = ['OURS']
    tot = {p: [0, 0, 0] for p in preds}
    per = {p: [] for p in preds}
    for s in seqs:
        labels = sorted(n for n in names if n.startswith(f'sequences/{s}/labels/') and n.endswith('.label'))
        for p in preds:
            t = [0, 0, 0]
            for ln in labels:
                gt = np.frombuffer(z.read(ln), dtype=np.uint32)
                f = os.path.basename(ln)
                if p == 'OURS':
                    path = os.path.join(a.pred_dir, s, f)
                    if not os.path.exists(path):
                        continue
                    pr = np.fromfile(path, dtype=np.uint32)
                else:
                    pn = f'sequences/{s}/{p}/{f}'
                    if pn not in names:
                        continue
                    pr = np.frombuffer(z.read(pn), dtype=np.uint32)
                if len(pr) != len(gt):
                    print(f'size mismatch {s} {p} {f}: {len(pr)} vs {len(gt)}', file=sys.stderr)
                    continue
                c = counts(gt, pr)
                t = [x + y for x, y in zip(t, c)]
            iou = t[0] / max(1, sum(t))
            per[p].append(iou)
            tot[p] = [x + y for x, y in zip(tot[p], t)]
            print(f'{s} {p:22s} tp {t[0]:8d} fn {t[1]:8d} fp {t[2]:8d} IoU {iou:.3f}', flush=True)
    print('\nsummary (pooled over all points | mean of per-sequence IoU)')
    for p in preds:
        t = tot[p]
        print(f'{p:22s} pooled {t[0] / max(1, sum(t)):.3f} | seq-mean {np.mean(per[p]):.3f}  ({len(per[p])} seqs)')


if __name__ == '__main__':
    main()
