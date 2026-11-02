#!/usr/bin/env python3
"""
Where the potentially-dynamic (PD) state fails, on CODa windows: a diagnostic run (coda_ab.sh with MORE_TOPICS listing
the intermediate topics, see DIAG_TOPICS) is followed stage by stage.

A. Still objects MISSED (annotated, not moving, in a camera view, not published as PD within 1 m): the first stage that
   loses it, in pipeline order
     1 YOLO        no 2D detection of a dynamic class lying mostly inside its projected box in any camera
     2 preproc     fewer than 5 LiDAR points of it survive the floor/roof/wall crop (/sins/lidar/processed)
     3 leaf        no semantic leaf of a dynamic class on it (3a none within 2 m, 3b beside it, 3c 1-2 points)
     4 cluster     no cluster within 1 m (merged into something bigger, or split)
     5 YOLO->cluster   cluster(s) there but none carries a class this tick
     6 tracker     no track within 1 m
     7 tracker     track(s) there but none with a class
     8 state       a classified track there, published with another state (dynamic / static)
     0 OK          published PD within 1 m
B. FALSE PD (published PD, no annotated object kept by the evaluation within 1 m): what they are
     class of the track (tracker results[1]), observed height, distance to the nearest annotated box of ANY class/state
     (< 1.5 m: a duplicate or a mis-placed box of a real object; beyond: unannotated or a structure), whether a YOLO
     leaf of a dynamic class lies on it this tick (class from the camera) or not (memory / LiDAR-only fallback),
     and the persistent tracks behind most of them (position, size, class) to look at in the images.
usage: pd_funnel.py <TAG> [w1 w2 w3]          (CODa: bags/coda/runs/<TAG>_v2_coda0_<w>)
       pd_funnel.py --jrdb <TAG> <seq> [...]   (JRDB: bags/jrdb/runs/<TAG>_v2_jrdb_<seq>, five cameras, 15 m)
"""
import collections
import sys
import types
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

EVAL = Path(__file__).resolve().parents[1]
REPO = EVAL.parent
sys.path.insert(0, str(EVAL))
import coda_eval as ce                                             # noqa: E402
from harness.ros_io import load_obstacles, load_odometry           # noqa: E402
from harness.tracks import objects                                 # noqa: E402

DIAG_TOPICS = ('/sins/lidar/processed /sins/front_camera/semantic_leaves /sins/stereo_camera/semantic_leaves '
               '/sins/dbscan_detector/lidar/detections3d')
DYN = {'person', 'bicycle', 'car', 'motorbike', 'bus', 'truck', 'dog', 'cat', 'horse'}
NAMES = [l.strip() for l in open(REPO / 'sins/cfg/coco.names')]
DYN_IDX = [NAMES.index(c) for c in DYN if c in NAMES]
A = types.SimpleNamespace(min_pts=1, pd_everywhere=False, range=25., min_range=1.2, dyn_speed=0.3, ignore_dist=1.,
                          max_height=2.5)


def quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_of(tr):
    T = np.eye(4)
    T[:3, :3] = quat(tr.rotation)
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def stamp(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


def statics_K(bag, cam):
    st, K, WH = {}, None, None
    with AnyReader([Path(bag)]) as r:
        for c, _, b in r.messages([c for c in r.connections if c.topic in ('/tf_static', f'/{cam}/camera/color/camera_info')]):
            m = r.deserialize(b, c.msgtype)
            if c.topic == '/tf_static':
                for t in m.transforms:
                    st[(t.header.frame_id, t.child_frame_id)] = T_of(t.transform)
            elif K is None:
                K, WH = np.array(m.k).reshape(3, 3), (m.width, m.height)
    return st, K, WH


def full_odom(bag):
    t, T = [], []
    with AnyReader([Path(bag)]) as r:
        for c, _, b in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(b, c.msgtype)
            M = np.eye(4)
            M[:3, :3] = quat(m.pose.pose.orientation)
            p = m.pose.pose.position
            M[:3, 3] = [p.x, p.y, p.z]
            t.append(stamp(m))
            T.append(M)
    return np.array(t), np.array(T)


def read(bag):
    D = collections.defaultdict(list)
    with AnyReader([Path(bag)]) as r:
        for c, _, b in r.messages([c for c in r.connections]):
            t = c.topic
            if t == '/sins/lidar/processed':
                m = r.deserialize(b, c.msgtype)
                a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
                D[t].append((stamp(m), a[:, :12].copy().view(np.float32)))
            elif t.endswith('semantic_leaves'):
                m = r.deserialize(b, c.msgtype)
                a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step) if m.width else np.zeros((0, 20), np.uint8)
                off = {f.name: f.offset for f in m.fields}
                ci = off.get('class_idx', 16)
                cls = a[:, ci:ci + 4].copy().view(np.float32).ravel().astype(int) if len(a) else np.zeros(0, int)
                D['leaves'].append((stamp(m), a[:, :12].copy().view(np.float32) if len(a) else np.zeros((0, 3), np.float32), cls))
            elif t in ('/sins/dbscan_detector/lidar/detections3d', '/sins/dbscan_detector/fused/detections3d', '/sins/tracker/tracks'):
                m = r.deserialize(b, c.msgtype)
                D[t].append((stamp(m), [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z,
                                         d.bbox.size.x, d.bbox.size.y, d.bbox.size.z,
                                         [h.hypothesis.class_id for h in d.results], d.id) for d in m.detections]))
            elif (t.startswith('/yolo_seg_track/') or t.startswith('/yolo_semantic_seg/')) and t.endswith('/detections'):
                m = r.deserialize(b, c.msgtype)
                D['yolo_' + t.split('/')[2]].append((stamp(m), [
                    (d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y, d.results[0].hypothesis.class_id)
                    for d in m.detections if d.results]))
    return D


def near(L, t, tol=1.5e8):
    if not L:
        return None
    ts = np.array([x[0] for x in L])
    k = np.argmin(abs(ts - t))
    return L[k] if abs(ts[k] - t) <= tol else None


def inbox(P, g, m=0.2):
    c, s = np.cos(g.yaw), np.sin(g.yaw)
    q = P - [g.x, g.y, g.z]
    lx, ly = q[:, 0] * c + q[:, 1] * s, -q[:, 0] * s + q[:, 1] * c
    return (abs(lx) <= g.l / 2 + m) & (abs(ly) <= g.w / 2 + m) & (abs(q[:, 2]) <= g.h / 2 + m)


def main():
    jrdb = '--jrdb' in sys.argv
    args = [x for x in sys.argv[1:] if x != '--jrdb']
    tag = args[0]
    wins = args[1:] or ['w1', 'w2', 'w3']
    if jrdb:
        A.range = 15.0
    stage_pts = collections.defaultdict(collections.Counter)
    stage, stage_cls = collections.Counter(), collections.defaultdict(collections.Counter)
    fp_rows = []
    stage5 = []
    for w in wins:
        base = REPO / (f'bags/jrdb/{w}' if jrdb else f'bags/coda/coda0_{w}')
        run = REPO / (f'bags/jrdb/runs/{tag}_v2_jrdb_{w}' if jrdb else f'bags/coda/runs/{tag}_v2_coda0_{w}')
        odom = load_odometry(f'{base}_odom')
        ot, oT = full_odom(f'{base}_odom')
        gt, ign, frames, _ = ce.build_gt(f'{base}_gt.csv', odom, A)
        raw = pd.read_csv(f'{base}_gt.csv')
        raw = raw[raw.track_id >= 0]
        Gall = objects(gt)
        G = Gall[Gall.state != 'dynamic']
        cams = {}
        if jrdb:
            for cam in ('cam0', 'cam2', 'cam4', 'cam6', 'cam8'):
                stc, Kc, WHc = statics_K(base, cam)
                Tbv = stc[('base_link', 'velodyne')]
                cams[cam] = (Tbv @ stc[('velodyne', f'{cam}_depth_optical_frame')] @
                             stc[(f'{cam}_depth_optical_frame', f'{cam}_color_optical_frame')], Kc, WHc)
        else:
            stf, Kf, WHf = statics_K(base, 'front_camera')
            Tbv = stf[('base_link', 'velodyne')]
            cams['front_camera'] = (Tbv @ stf[('velodyne', 'front_camera_depth_optical_frame')] @
                                    stf[('front_camera_depth_optical_frame', 'front_camera_color_optical_frame')], Kf, WHf)
        if not jrdb and Path(f'{base}_stereo').exists():
            sts, Ks, WHs = statics_K(f'{base}_stereo', 'stereo_camera')
            if ('velodyne', 'stereo_camera_depth_optical_frame') in sts:
                cams['stereo_camera'] = (Tbv @ sts[('velodyne', 'stereo_camera_depth_optical_frame')] @
                                         sts[('stereo_camera_depth_optical_frame', 'stereo_camera_color_optical_frame')], Ks, WHs)
        D = read(run)
        pub = objects(load_obstacles(str(run))[0])
        pt = np.unique(pub.t_ns)
        # ── A: missed still objects ──────────────────────────────────────────
        for _, g in G.iterrows():
            gr = raw[(raw.t_ns == g.t_ns) & (raw.track_id == g.track_id)]
            if not len(gr):
                continue
            gr = gr.iloc[0]
            cls = gr['class']
            Tob = oT[np.argmin(abs(ot - g.t_ns))]
            seen = infov = False
            for cam, (Tbc, K, WH) in cams.items():
                Tco = np.linalg.inv(Tob @ Tbc)
                c, s = np.cos(gr.yaw), np.sin(gr.yaw)
                cr = np.array([(Tco @ [gr.x + c * dx * gr.l - s * dy * gr.w, gr.y + s * dx * gr.l + c * dy * gr.w,
                                       gr.z + dz * gr.h, 1])[:3] for dx in (-.5, .5) for dy in (-.5, .5) for dz in (-.5, .5)])
                if (cr[:, 2] < 0.5).any():
                    continue
                uv = (K @ cr.T).T
                uv = uv[:, :2] / uv[:, 2:]
                x0, y0 = uv.min(0)
                x1, y1 = uv.max(0)
                if not (0 <= (x0 + x1) / 2 < WH[0] and 0 <= (y0 + y1) / 2 < WH[1]):
                    continue
                infov = True
                Y = near(D['yolo_' + cam], g.t_ns)
                if Y is None:
                    continue
                for (cx, cy, sx, sy, cl) in Y[1]:
                    if cl not in DYN:
                        continue
                    ix = max(0, min(x1, cx + sx / 2) - max(x0, cx - sx / 2))
                    iy = max(0, min(y1, cy + sy / 2) - max(y0, cy - sy / 2))
                    # seen: the YOLO box lies mostly inside the projected annotation (centre inside, >= half of its
                    # area). Not IoU: a person sitting behind a table is detected head and torso only, while the
                    # annotated box reaches the floor and the chair (IoU < 0.3 for a correct detection, JRDB)
                    if x0 <= cx <= x1 and y0 <= cy <= y1 and sx * sy > 0 and ix * iy / (sx * sy) >= 0.5:
                        seen = True
            if not infov:
                continue
            q = pub[pub.t_ns == pt[np.argmin(abs(pt - g.t_ns))]]
            d = np.hypot(q.x - g.x, q.y - g.y)
            if len(q) and d.min() < 1 and q.loc[d.idxmin(), 'state'] == 'potentially_dynamic':
                st = '0 PD ok'
            else:
                P = near(D['/sins/lidar/processed'], g.t_ns)
                npp = int(inbox(P[1], gr).sum()) if P else 0
                if not seen:
                    st = '1 YOLO does not detect it'
                elif npp < 5:
                    st = '2 preprocessing: <5 LiDAR points left on it'
                else:
                    ts = np.array([x[0] for x in D['leaves']])
                    ks = np.where(abs(ts - g.t_ns) < 1.5e8)[0] if len(ts) else []
                    leaf = any((inbox(D['leaves'][k][1], gr, 0.3) & np.isin(D['leaves'][k][2], DYN_IDX)).sum() >= 3 for k in ks)
                    Lc = near(D['/sins/dbscan_detector/lidar/detections3d'], g.t_ns)
                    own = [x for x in Lc[1] if np.hypot(x[0] - g.x, x[1] - g.y) < 1.0] if Lc else []
                    Fc = near(D['/sins/dbscan_detector/fused/detections3d'], g.t_ns)
                    fcl = [x for x in Fc[1] if np.hypot(x[0] - g.x, x[1] - g.y) < 1.0] if Fc else []
                    Tc = near(D['/sins/tracker/tracks'], g.t_ns)
                    tr = [x for x in Tc[1] if np.hypot(x[0] - g.x, x[1] - g.y) < 1.0] if Tc else []
                    if not leaf:
                        best, nmax = 99, 0
                        for k in ks:
                            Lp, Lcl = D['leaves'][k][1], D['leaves'][k][2]
                            sel = np.isin(Lcl, DYN_IDX)
                            if sel.any():
                                best = min(best, np.hypot(Lp[sel, 0] - g.x, Lp[sel, 1] - g.y).min())
                                nmax = max(nmax, int(inbox(Lp[sel], gr, 0.3).sum()))
                        st = ('3a leaf: none of a dynamic class within 2 m' if best > 2 else
                              ('3b leaf: nearby but beside it (0.3-2 m)' if nmax == 0 else '3c leaf: 1-2 points on it only'))
                    elif not own and not fcl:
                        st = '4 clustering: no cluster centred on it (merged/split)'
                    elif not any(len(x[6]) > 1 for x in fcl):
                        st = '5 YOLO->cluster: cluster without class this tick'
                        lcls = collections.Counter()
                        for k in ks:
                            sel = inbox(D['leaves'][k][1], gr, 0.3) & np.isin(D['leaves'][k][2], DYN_IDX)
                            lcls.update(NAMES[i] for i in D['leaves'][k][2][sel])
                        cl_h = max([x[5] for x in fcl + own] or [0])
                        cl_l = max([max(x[3], x[4]) for x in fcl + own] or [0])
                        stage5.append(dict(cls=cls, leaf=lcls.most_common(1)[0][0] if lcls else '', cl_h=cl_h, cl_l=cl_l,
                                           n_leaf=sum(lcls.values()), d=min([np.hypot(x[0] - g.x, x[1] - g.y) for x in fcl + own] or [9])))
                    elif not tr:
                        st = '6 tracker: no track on it'
                    elif not any(len(x[6]) > 1 and x[6][1] for x in tr):
                        st = '7 tracker: track without class'
                    else:
                        st = '8 tracker: classified track, state ' + (tr[0][6][0] if tr[0][6] else '?')
            stage[st] += 1
            stage_cls[cls][st] += 1
            stage_pts[pd.cut([gr.npts], [-1, 4, 9, 19, 49, 1e9], labels=['1-4', '5-9', '10-19', '20-49', '50+'])[0]][st] += 1
        # ── B: false PD ──────────────────────────────────────────────────────
        pdp = pub[pub.state == 'potentially_dynamic']
        kept = Gall
        for t, q in pdp.groupby('t_ns'):
            ft = frames[np.argmin(abs(frames - t))]
            if abs(ft - t) > 1.5e8:
                continue
            g = kept[kept.t_ns == ft]
            ig = ign[ign.t_ns == ft]
            rr = raw[raw.t_ns == ft]
            Tc = near(D['/sins/tracker/tracks'], t)
            ts = np.array([x[0] for x in D['leaves']])
            ks = np.where(abs(ts - t) < 1.5e8)[0] if len(ts) else []
            for _, o in q.iterrows():
                dk = np.hypot(g.x - o.x, g.y - o.y).min() if len(g) else 99
                if dk < 1.0:
                    continue                                  # matched (or matchable) to a kept GT: not a false PD
                if len(ig) and np.hypot(ig.x - o.x, ig.y - o.y).min() < A.ignore_dist:
                    continue                                  # dropped by the evaluation
                dr = np.hypot(rr.x - o.x, rr.y - o.y)
                da = dr.min() if len(rr) else 99
                ca = rr['class'].values[np.argmin(dr)] if len(rr) else ''
                tcls = ''
                if Tc:
                    tr = [x for x in Tc[1] if np.hypot(x[0] - o.x, x[1] - o.y) < 0.5]
                    if tr and len(tr[0][6]) > 1:
                        tcls = tr[0][6][1]
                leafed = any(((np.hypot(D['leaves'][k][1][:, 0] - o.x, D['leaves'][k][1][:, 1] - o.y) < max(o.l, o.w) / 2 + 0.2)
                              & np.isin(D['leaves'][k][2], DYN_IDX)).sum() >= 3 for k in ks)
                fp_rows.append(dict(w=w, t=t, track=o.track_id, x=o.x, y=o.y, l=o.l, w_=o.w, h=o.h, cls=tcls or '?',
                                    d_ann=da, ann_cls=ca, leaf=leafed))
    n = sum(stage.values())
    print(f'A. still objects in a camera view: {n} rows (window-frames)')
    for k in sorted(stage):
        print(f'   {k:58s} {100 * stage[k] / n:5.1f}%')
    for cl, cnt in stage_cls.items():
        m = sum(cnt.values())
        print(f'   {cl:16s} n={m:5d} ' + ' '.join(f'{k[:2].strip()}:{100 * v / m:.0f}%' for k, v in sorted(cnt.items())))
    if stage5:
        S5 = pd.DataFrame(stage5)
        print('\n   stage 5 detail (leaf on it, cluster there, no class):')
        print('   annotated class x leaf class:', S5.groupby(['cls', 'leaf']).size().to_dict())
        print('   cluster height:', pd.cut(S5.cl_h, [0, 0.4, 0.6, 0.8, 1.1, 9]).value_counts().sort_index().to_dict())
        print('   cluster longest side:', pd.cut(S5.cl_l, [0, 0.5, 1, 2, 3, 6, 99]).value_counts().sort_index().to_dict())
        print('   leaf points on it: median %.0f; cluster centre to object: median %.2f m' % (S5.n_leaf.median(), S5.d.median()))
    print('\n   by LiDAR returns on the object (annotation npts):')
    for b in ['1-4', '5-9', '10-19', '20-49', '50+']:
        cnt = stage_pts.get(b)
        if cnt:
            m = sum(cnt.values())
            print(f'   {b:6s} n={m:5d} ' + ' '.join(f'{k[:2].strip()}:{100 * v / m:.0f}%' for k, v in sorted(cnt.items())))
    F = pd.DataFrame(fp_rows)
    if F.empty:
        print('B. no false PD')
        return
    print(f'\nB. false PD rows: {len(F)} ({F.track.nunique()} tracks)')
    print('   by window:', F.groupby('w').size().to_dict())
    print('   by track class:', F.cls.value_counts().to_dict())
    print('   class from a YOLO leaf this tick: %.0f%%' % (100 * F.leaf.mean()))
    bins = pd.cut(F.d_ann, [0, 1.5, 3, 6, 99], labels=['<1.5 m (duplicate/offset of an annotated object)', '1.5-3 m', '3-6 m', '>6 m'])
    print('   distance to the nearest annotated object:', bins.value_counts().sort_index().to_dict())
    print('   nearest annotated class when < 1.5 m:', F[F.d_ann < 1.5].ann_cls.value_counts().to_dict())
    hb = pd.cut(F.h, [0, 0.5, 0.8, 1.2, 1.6, 9])
    print('   observed height:', hb.value_counts().sort_index().to_dict())
    top = F.groupby(['w', 'track']).agg(n=('t', 'size'), x=('x', 'median'), y=('y', 'median'), l=('l', 'median'),
                                        wd=('w_', 'median'), h=('h', 'median'), cls=('cls', lambda s: s.mode().iat[0]),
                                        d_ann=('d_ann', 'median'), leaf=('leaf', 'mean')).sort_values('n', ascending=False)
    print('   tracks behind most false PD rows:')
    print(top.head(15).round(2).to_string())


if __name__ == '__main__':
    main()
