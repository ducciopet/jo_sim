"""Camera branch of SINS on CODa: for every annotated object row whose box projects into a camera image, does
(1) YOLO detect it (a 2D box of a target class with IoU >= 0.3 with the projected GT box, nearest YOLO frame within
0.1 s), (2) the fused LiDAR/camera detection near it carry a YOLO class (depth-leaf refinement, dbscan_detector),
(3) the track near it carry a class (tracker: depth leaf, LiDAR-only pixel fallback, anchor), (4) the output give
the right state (rows_coda_<tag>.csv of funnel.py)?
usage: cam_funnel.py <tag> <win> [...]     (bags/coda/runs/<tag>_v2_coda<win>, <tag>_trk_<win>.csv)
"""
import sys
import collections
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
sys.path.insert(0, '/home/duccio/Desktop/jo_sim_sins/eval')
sys.path.insert(0, str(Path(__file__).parent))
from rosbags.highlevel import AnyReader          # noqa: E402
from harness.ros_io import typestore              # noqa: E402
from funnel import gt_speed, REPO                 # noqa: E402

# intrinsics logged by tracker_node (camera_info of the adapter)
CAMS = {'front_camera': (569.83, 565.48, 439.27, 360.58, 960, 540),
        'stereo_camera': (769.33, 769.33, 626.12, 525.46, 1224, 1024)}
TARGET = {'person', 'bicycle', 'car', 'motorbike', 'motorcycle', 'bus', 'truck', 'dog', 'cat', 'horse'}


def mat(tr):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]).as_matrix()
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def read(run):
    st, odom, yolo, fused = {}, [], collections.defaultdict(dict), {}
    with AnyReader([Path(run)], default_typestore=typestore()) as r:
        for c, t, raw in r.messages([c for c in r.connections if c.topic in
                                     ('/tf_static', '/tf', '/yolo_seg_track/front_camera/detections',
                                      '/yolo_seg_track/stereo_camera/detections', '/sins/dbscan_detector/fused/detections3d')]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic in ('/tf_static', '/tf'):
                for tt in m.transforms:
                    if c.topic == '/tf_static':
                        st[(tt.header.frame_id, tt.child_frame_id)] = mat(tt.transform)
                    elif tt.child_frame_id == 'base_link':
                        odom.append((tt.header.stamp.sec + tt.header.stamp.nanosec * 1e-9, mat(tt.transform)))
                continue
            s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if 'yolo' in c.topic:
                cam = c.topic.split('/')[2]
                yolo[cam][s] = [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y,
                                 d.results[0].hypothesis.class_id if d.results else '') for d in m.detections]
            else:
                fused[s] = [(d.bbox.center.position.x, d.bbox.center.position.y,
                             d.results[1].hypothesis.class_id if len(d.results) > 1 else '') for d in m.detections]
    odom.sort(key=lambda x: x[0])
    return st, odom, yolo, fused


def chain(st, child, parent='base_link'):
    """T_parent_child through the static tree (child up to parent)."""
    T = np.eye(4); f = child
    while f != parent:
        k = [k for k in st if k[1] == f]
        if not k:
            return None
        T = st[k[0]] @ T; f = k[0][0]
    return T


def iou(a, b):
    ax0, ay0, ax1, ay1 = a; bx0, by0, bx1, by1 = b
    iw = max(0, min(ax1, bx1) - max(ax0, bx0)); ih = max(0, min(ay1, by1) - max(ay0, by0))
    i = iw * ih
    return i / max(1e-9, (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - i)


def nearest(d, t, tol):
    if not d:
        return None
    ks = np.fromiter(d.keys(), dtype=np.int64)
    k = ks[np.argmin(np.abs(ks - t))]
    return d[k] if abs(k - t) <= tol else None


def window(tag, c):
    st, odom, yolo, fused = read(REPO / f'bags/coda/runs/{tag}_v2_coda{c}')
    T_base_cam = {cam: chain(st, f'{cam}_color_optical_frame') for cam in CAMS}
    ot = np.array([o[0] for o in odom])
    K = pd.read_csv(REPO / f'bags/coda/runs/{tag}_trk_{c}.csv'); K['tn'] = (K.t * 1e9).round().astype('int64')
    Kt = np.unique(K.tn); KG = {t: g for t, g in K.groupby('tn')}
    F = pd.read_csv(Path(__file__).parent / f'rows_coda_{tag}.csv'); F = F[F.win == c].set_index(['t_ns', 'gid'])
    g = pd.read_csv(REPO / f'bags/coda/coda{c}_gt.csv', keep_default_na=False)
    g = g[g.track_id.astype(int) >= 0].copy()
    for col in ('x', 'y', 'z', 'l', 'w', 'h', 'yaw'):
        g[col] = g[col].astype(float)
    g['track_id'] = g.track_id.astype(int); g['t_ns'] = g.t_ns.astype('int64'); g['npts'] = g.npts.astype(int)
    g = g.sort_values('t_ns'); g['spd'] = gt_speed(g); g = g[g.npts >= 1]
    rows = []
    for q in g.itertuples():
        if (q.t_ns, q.track_id) not in F.index:
            continue                       # out of range for the funnel
        f = F.loc[(q.t_ns, q.track_id)]
        i = np.argmin(np.abs(ot - q.t_ns / 1e9))
        T_odom_base = odom[i][1]
        cy, sy = np.cos(q.yaw), np.sin(q.yaw)
        corners = np.array([[q.x + a * q.l / 2 * cy - b * q.w / 2 * sy, q.y + a * q.l / 2 * sy + b * q.w / 2 * cy, q.z + e * q.h / 2, 1]
                            for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)])
        for cam, (fx, fy, cx, cyy, W, H) in CAMS.items():
            if T_base_cam[cam] is None or not yolo.get(cam):
                continue
            P = (np.linalg.inv(T_odom_base @ T_base_cam[cam]) @ corners.T).T
            if (P[:, 2] <= 0.3).any():
                continue
            u = fx * P[:, 0] / P[:, 2] + cx; v = fy * P[:, 1] / P[:, 2] + cyy
            box = (max(0, u.min()), max(0, v.min()), min(W, u.max()), min(H, v.max()))
            if box[2] - box[0] < 2 or box[3] - box[1] < 2:
                continue
            full = (box[2] - box[0]) * (box[3] - box[1]) / max(1e-9, (u.max() - u.min()) * (v.max() - v.min()))
            if full < 0.5:                 # less than half of the box inside the image
                continue
            Y = nearest(yolo[cam], q.t_ns, 0.1e9)
            if Y is None:
                yolo_state = 'no YOLO frame'
            else:
                best = max([iou(box, (x - w / 2, y - h / 2, x + w / 2, y + h / 2)) for x, y, w, h, k in Y if k in TARGET] or [0])
                yolo_state = 'YOLO hit' if best >= 0.3 else ('YOLO partial' if best > 0.1 else 'YOLO miss')
            Fu = nearest(fused, q.t_ns, 0.06e9) or []
            leaf = any(np.hypot(x - q.x, y - q.y) < 1.0 and k for x, y, k in Fu)
            kt = Kt[np.argmin(np.abs(Kt - q.t_ns))]; k = KG[kt]
            dk = np.minimum(np.hypot(k.x - q.x, k.y - q.y), np.hypot(k.ox - q.x, k.oy - q.y))
            kk = k[dk < 1.0]
            track_cls = bool(len(kk)) and (kk.cls != '-').any()
            rows.append(dict(win=c, cam=cam, cls=q._12 if False else g.loc[q.Index, 'class'], D=q.spd > 0.3,
                             rng=float(np.linalg.norm(P.mean(0)[:3])), px_h=box[3] - box[1], yolo=yolo_state, leaf=leaf,
                             track=len(kk) > 0, track_cls=track_cls, ok=f.cause == 'OK', cause=f.cause))
    return pd.DataFrame(rows)


if __name__ == '__main__':
    tag = sys.argv[1]
    R = pd.concat([window(tag, w if '_' in w else f'0_{w}') for w in sys.argv[2:]])
    R.to_csv(Path(__file__).parent / f'cam_rows_{tag}.csv', index=False)
    for cam, C in R.groupby('cam'):
        print(f'\n===== {cam}: {len(C)} object rows in the image =====')
        C = C.assign(band=pd.cut(C.rng, [0, 5, 10, 15, 25]))
        print('YOLO by range band:\n', pd.crosstab(C.band, C.yolo, normalize='index').mul(100).round(0).to_string())
        print('pixel height median by band:', C.groupby('band', observed=True).px_h.median().round(0).to_dict())
        H = C[C.yolo == 'YOLO hit']
        print(f'\nof the YOLO hits ({len(H)}): fused detection carries the class {H.leaf.mean():.0%}, '
              f'a track is there {H.track.mean():.0%}, the track has a class {H.track_cls.mean():.0%}, output right {H.ok.mean():.0%}')
        print('  by band, track has class:', H.groupby('band', observed=True).track_cls.mean().round(2).to_dict())
        print('  by band, cluster leaf class:', H.groupby('band', observed=True).leaf.mean().round(2).to_dict())
        for lab, S in (('D', C[C.D]), ('PD', C[~C.D])):
            print(f'  {lab}: rows {len(S)}, YOLO hit {np.mean(S.yolo == "YOLO hit"):.0%}, track class {S.track_cls.mean():.0%}, '
                  f'output right {S.ok.mean():.0%}; when YOLO hit: track class {S[S.yolo == "YOLO hit"].track_cls.mean():.0%}, '
                  f'right {S[S.yolo == "YOLO hit"].ok.mean():.0%}; when YOLO missed: right {S[S.yolo == "YOLO miss"].ok.mean():.0%}')
        print('  by GT class (YOLO hit / track class):', C.groupby('cls').agg(n=('ok', 'size'), yolo=('yolo', lambda s: (s == 'YOLO hit').mean()),
                                                                            tcls=('track_cls', 'mean')).round(2).to_dict('index'))
