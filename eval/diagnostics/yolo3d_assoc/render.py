# Figures for the YOLO mask -> 3D page, from assoc_demo.py npz files. Also computes the "proposal": all LiDAR points in
# the mask, then only an outlier filter = the largest 3D connected component (points within R of each other).
import sys, io, json, glob, numpy as np, pandas as pd, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image
from scipy.spatial import cKDTree
from scipy.sparse.csgraph import connected_components
D = sys.argv[1]; OUT = sys.argv[2]; R = 0.5
C = dict(bg='#b9b4ab', drop='#e8892b', keep='#1f9e5a', gt='#222222', cut='#d63b3b', obj='#3a6fd8', prop='#1f9e5a')
plt.rcParams.update({'font.size': 11, 'font.family': 'DejaVu Sans', 'axes.spines.top': False, 'axes.spines.right': False})

def corners(c, yaw, s):
    R2 = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    q = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * s[:2] / 2
    return q @ R2.T + c[:2]

def obb(p):
    m = p.mean(0); d = p - m; c11, c12, c22 = (d[:, 0]**2).sum(), (d[:, 0]*d[:, 1]).sum(), (d[:, 1]**2).sum()
    yaw = 0.5 * np.arctan2(2 * c12, c11 - c22); R2 = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    q = d[:, :2] @ R2; lo, hi = q.min(0), q.max(0); cxy = m[:2] + R2 @ ((lo + hi) / 2)
    return np.array([cxy[0], cxy[1], 0]), yaw, np.array([*(hi - lo), 0])

def proposal(P, inm, dep=None, G=1.0):
    """all LiDAR points in the mask; outliers = points separated by an empty depth gap > G m from the main group"""
    idx = np.where(inm)[0]; d = dep[idx]; o = np.argsort(d); seg = np.split(np.arange(len(d)), np.where(np.diff(d[o]) > G)[0] + 1)
    out = np.zeros(len(P), bool); out[idx[o[max(seg, key=len)]]] = True; return out

def proposal_cc(P, inm):
    idx = np.where(inm)[0]; p = P[idx]
    pairs = cKDTree(p).query_pairs(R, output_type='ndarray')
    from scipy.sparse import coo_matrix
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(p), len(p)))
    n, lab = connected_components(g, directed=False); big = np.bincount(lab).argmax()
    out = np.zeros(len(P), bool); out[idx[lab == big]] = True; return out

def figure(f, tag):
    z = np.load(f, allow_pickle=True); meta = json.loads(str(z['meta']))
    P, uv, inm, leaf, cut, obj, dep = z['P'], z['uv'], z['inm'], z['leaf'], z['cut'], z['obj'], z['depth']
    img = np.array(Image.open(io.BytesIO(z['img'].tobytes())))[:, :, ::-1]; mask = z['mask']; x1, y1, x2, y2 = z['mbox']
    prop = proposal_cc(P, inm & (P[:, 2] > z['gt'][2] - z['gt'][5] / 2 + 0.15)); pc, py, ps = obb(P[prop])
    # 1) image: mask + projected LiDAR points
    pad = 60; X1, Y1, X2, Y2 = int(max(0, x1 - pad)), int(max(0, y1 - pad)), int(min(img.shape[1], x2 + pad)), int(min(img.shape[0], y2 + pad))
    ov = img.astype(float).copy(); ov[mask] = 0.6 * ov[mask] + 0.4 * np.array([58, 111, 216]); ov = ov.astype(np.uint8)
    fig, ax = plt.subplots(figsize=(7.2, 7.2 * (Y2 - Y1) / (X2 - X1)), dpi=130)
    ax.imshow(ov[Y1:Y2, X1:X2], extent=(X1, X2, Y2, Y1))
    for sel, col, s in [(inm & ~leaf, C['drop'], 7), (leaf, C['keep'], 9)]:
        ax.scatter(uv[sel, 0], uv[sel, 1], s=s, c=col, edgecolors='none')
    ax.set_xlim(X1, X2); ax.set_ylim(Y2, Y1); ax.axis('off'); fig.tight_layout(pad=0)
    fig.savefig(f'{OUT}/{tag}_img.jpg', dpi=130); plt.close(fig)
    # 2) depth histogram of the mask points
    d = dep[inm]; fig, ax = plt.subplots(figsize=(7.2, 2.6), dpi=130)
    bins = np.arange(d.min(), min(d.max(), d.min() + 14) + 0.3, 0.3)
    ax.hist(d, bins=bins, color=C['drop'], edgecolor='white')
    ax.hist(dep[leaf], bins=bins, color=C['keep'], edgecolor='white')
    zl, zh = meta['win']; ax.axvspan(zl, zh, color=C['keep'], alpha=0.12)
    ax.set_xlim(d.min() - 0.5, min(d.max(), d.min() + 14)); ax.set_xlabel('distanza dalla camera lungo l\'asse ottico [m]'); ax.set_ylabel('punti LiDAR\nnella maschera')
    fig.tight_layout(); fig.savefig(f'{OUT}/{tag}_hist.svg'); fig.savefig(f'{OUT}/{tag}_hist.png', dpi=110); plt.close(fig)
    # 3) bird's eye view
    gt = z['gt']; cb = z['cutbox']; ob = z['objbox']
    near = np.hypot(P[:, 0] - gt[0], P[:, 1] - gt[1]) < 6 + gt[3] / 2
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.8), dpi=130, sharex=True, sharey=True)
    for ax, title, sel, box, bc in [(axs[0], 'Oggi: fetta di ±0,3 m', leaf, (cb[:3], cb[3], cb[4:]), C['cut']),
                                    (axs[1], 'Proposta: maschera, senza suolo, punti collegati', prop, (pc, py, ps), C['prop'])]:
        ax.scatter(P[near & ~inm, 0], P[near & ~inm, 1], s=2, c=C['bg'], edgecolors='none')
        ax.scatter(P[near & inm & ~sel, 0], P[near & inm & ~sel, 1], s=4, c=C['drop'], edgecolors='none')
        ax.scatter(P[sel, 0], P[sel, 1], s=5, c=C['keep'], edgecolors='none')
        ax.add_patch(Polygon(corners(gt[:2], gt[6], gt[3:5]), fill=False, ec=C['gt'], ls='--', lw=1.4))
        ax.add_patch(Polygon(corners(*box), fill=False, ec=bc, lw=2.4))
        ax.set_title(title, fontsize=12, loc='left'); ax.set_aspect('equal')
        ax.set_xlim(gt[0] - 5, gt[0] + 5); ax.set_ylim(gt[1] - 5, gt[1] + 5)
    axs[0].set_xlabel('x [m] (LiDAR)'); axs[1].set_xlabel('x [m] (LiDAR)'); axs[0].set_ylabel('y [m]')
    fig.tight_layout(); fig.savefig(f'{OUT}/{tag}_bev.svg'); fig.savefig(f'{OUT}/{tag}_bev.png', dpi=110); plt.close(fig)
    meta['prop_len'] = float(ps[:2].max()); meta['prop_pts'] = int(prop.sum())
    return meta

if __name__ == '__main__':
    picks = sys.argv[3].split(',')
    out = {t: figure(f'{D}/{t}.npz', t) for t in picks}
    json.dump(out, open(f'{OUT}/picks.json', 'w'), indent=1); print(json.dumps(out, indent=1))
    # proposal on every frame for the summary
    rows = []
    for f in sorted(glob.glob(f'{D}/*.npz')):
        z = np.load(f, allow_pickle=True); m = json.loads(str(z['meta'])); pr = proposal_cc(z['P'], z['inm'] & (z['P'][:, 2] > z['gt'][2] - z['gt'][5] / 2 + 0.15))
        m['prop_len'] = float(obb(z['P'][pr])[2][:2].max()); rows.append(m)
    df = pd.DataFrame(rows); df.to_csv(f'{OUT}/summary_prop.csv', index=False)
    print(df.groupby('track')[['gt_l', 'obj_len', 'cut_len', 'nocap_len', 'prop_len']].median().round(2))

    # per-frame lengths
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.7), dpi=130, sharey=True)
    nm = {2: 'Utility vehicle (3,96 m)', 7: 'Pickup di traverso (5,85 m)', 8: 'Pickup da dietro (6,57 m)'}
    for ax, (tr, g) in zip(axs, df.groupby('track')):
        g = g.sort_values('t'); x = (g.t - df.t.min()) / 1e9
        ax.plot(x, g.obj_len, color=C['obj'], lw=1.6, label='lunghezza vista dal LiDAR')
        ax.plot(x, g.prop_len, color=C['prop'], lw=1.6, label='proposta')
        ax.plot(x, g.cut_len, color=C['cut'], lw=1.6, label='oggi')
        ax.axhline(g.gt_l.iloc[0], color=C['gt'], ls='--', lw=1, label='vera (GT)')
        ax.set_title(nm.get(tr, str(tr)), fontsize=11, loc='left'); ax.set_xlabel('tempo [s]')
    axs[0].set_ylabel('lunghezza box [m]'); h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, frameon=False, fontsize=10, loc='lower center', ncol=4); fig.tight_layout(rect=(0, 0.1, 1, 1)); fig.savefig(f'{OUT}/lengths.svg'); fig.savefig(f'{OUT}/lengths.png', dpi=110)
    for tr, g in df.groupby('track'):
        print(tr, 'frames', len(g), 'prop>=0.8*lidar %.0f%%' % (100 * (g.prop_len >= 0.8 * g.obj_len).mean()), 'cut>=0.8*lidar %.0f%%' % (100 * (g.cut_len >= 0.8 * g.obj_len).mean()))
