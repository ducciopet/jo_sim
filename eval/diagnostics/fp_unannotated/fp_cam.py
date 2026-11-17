#!/usr/bin/env python3
"""Are SINS's moving false positives with no annotated object nearby real but unannotated agents?
Input: fprows_<TAG>.csv from fp_funnel.py (one row per unmatched DYNAMIC prediction per 10 Hz grid tick, with its bucket).
Sample: bucket no_object, rep 1, range <= 20 m (the coda_eval range); tracks weighted by their FP frames, stratified by
the class SINS publishes for the track (/sins/tracker/tracks results: person / car / truck / none), spread over windows.
For every sampled track, the FP tick whose 3D box projects fully and largest into the front (960x540) or stereo
(1224x1024) camera is drawn: FP box red with id + class, other SINS tracks grey, CODa GT boxes green (all of them,
ignored ones included), plus a 3x crop around the box. No visible tick -> out_of_fov (counted, no image).
usage: fp_cam.py fprows_LVI2P[_edge].csv TAG N_SAMPLES OUT_DIR  (windows read from the NAS coda_windows, read only)
"""
import sys
from pathlib import Path
import numpy as np, pandas as pd
from PIL import Image, ImageDraw, ImageFont
from rosbags.highlevel import AnyReader

R = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(R / 'eval'))
from harness.ros_io import typestore, load_obstacles      # noqa: E402
from harness.tracks import objects                       # noqa: E402
from make_coda_video import tf_mat, read_odom, nearest, corners, EDGES   # noqa: E402

WIN = Path('/mnt/nas-imperatore/datasets_SINS/derived/coda_windows')
CAMS = {'front': ('', '/front_camera/camera/color', 'front_camera_depth_optical_frame', 'front_camera_color_optical_frame'),
        'stereo': ('_stereo', '/stereo_camera/camera/color', 'stereo_camera_depth_optical_frame',
                   'stereo_camera_color_optical_frame')}
rows_csv, tag, N, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), Path(sys.argv[4])
out.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(0)
F = pd.read_csv(rows_csv, keep_default_na=False)
F = F[(F.b == 'no_object') & (F.rng.astype(float) <= 20) & (F.rep == '')].copy()
if 'raw_gt' in F:     # gt_edge.py output: keep only FP with no raw annotation within 1.5 m either
    F = F[F.raw_gt.astype(str) != 'True']
F['t'] = F.t.astype(float)


def run_dir(w):
    return R / f'bags/coda/runs/{tag}_v2_coda{w}'


def track_classes(run):
    """track id -> majority object class published by the tracker over the track's life ('none' if never one)."""
    cl = {}
    with AnyReader([run], default_typestore=typestore()) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/sins/tracker/tracks']):
            for d in r.deserialize(raw, c.msgtype).detections:
                k = [h.hypothesis.class_id for h in d.results
                     if h.hypothesis.class_id not in ('static', 'dynamic', 'potentially_dynamic', 'tracked')]
                cl.setdefault(int(d.id.split('_')[-1]), []).append(k[0] if k else 'none')
    return {i: pd.Series(v).value_counts().index[0] if (pd.Series(v) != 'none').sum() == 0
            else pd.Series([x for x in v if x != 'none']).value_counts().index[0] for i, v in cl.items()}


# tracks, published class, FP frames
T = F.groupby(['w', 'id']).agg(n=('t', 'size'), fp=('fp', 'median'), h=('h', 'median'), rng=('rng', 'median'),
                               travel=('travel', 'first')).reset_index()
cls = {w: track_classes(run_dir(w)) for w in T.w.unique()}
T['cls'] = [cls[w].get(int(i), 'none') for w, i in zip(T.w, T.id)]
T['vcls'] = T.cls.replace({'car': 'vehicle', 'truck': 'vehicle', 'bus': 'vehicle'})
print('candidate tracks / FP frames by published class:')
print(T.groupby('vcls').agg(tracks=('id', 'size'), fp_frames=('n', 'sum')).to_string())
# allocation: proportional to FP frames, at least 4 per class present
share = T.groupby('vcls').n.sum() / T.n.sum()
alloc = (share * N).round().clip(lower=4).astype(int)
alloc = np.minimum(alloc, T.groupby('vcls').size())
pick = []
for c, k in alloc.items():
    g = T[T.vcls == c]
    pick.append(g.loc[rng.choice(g.index, size=k, replace=False, p=g.n / g.n.sum())])
S = pd.concat(pick).sort_values(['w', 'id']).reset_index(drop=True)
print('sampled per class:', S.vcls.value_counts().to_dict(), '| per window:', S.w.value_counts().to_dict())

try:
    font = ImageFont.truetype('DejaVuSans-Bold.ttf', 20)
except OSError:
    font = ImageFont.load_default()
res = []
for w, g in S.groupby('w'):
    run = run_dir(w)
    st = {}
    with AnyReader([run], default_typestore=typestore()) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
            for tr in r.deserialize(raw, c.msgtype).transforms:
                st[(tr.header.frame_id, tr.child_frame_id)] = tf_mat(tr.transform)
    ot, oT = read_odom(R / f'bags/coda/coda{w}_odom')
    pred = objects(load_obstacles(run)[0])
    gt = pd.read_csv(R / f'bags/coda/coda{w}_gt.csv', keep_default_na=False)
    gt = gt[gt.track_id.astype(int) >= 0]
    for k in ('x', 'y', 'z', 'l', 'w', 'h', 'yaw'):
        gt[k] = gt[k].astype(float)
    gts = np.unique(gt.t_ns.to_numpy())
    cams = {}
    for name, (suf, top, depth, color) in CAMS.items():
        with AnyReader([WIN / f'coda{w}{suf}']) as r:
            if not any(c.topic == top + '/camera_info' for c in r.connections):
                continue                      # seq 16 windows have no front camera
            info = next(r.deserialize(raw, c.msgtype) for c, _, raw in
                        r.messages([c for c in r.connections if c.topic == top + '/camera_info']))
            K = np.array(info.k).reshape(3, 3)
            ts = np.array(sorted(c_t for c, c_t, _ in r.messages([c for c in r.connections
                                                                     if c.topic == top + '/image_raw'])))
        cams[name] = dict(K=K, wh=(info.width, info.height), ts=ts, T_vel_cam=st[('velodyne', depth)] @ st[(depth, color)],
                          suf=suf, top=top)
    T_base_vel = st[('base_link', 'velodyne')]

    def project(cam, t, P):
        Tc = np.linalg.inv(oT[nearest(ot, t)] @ T_base_vel @ cam['T_vel_cam'])
        Pc = (Tc[:3, :3] @ P.T).T + Tc[:3, 3]
        if (Pc[:, 2] < 0.5).any():
            return None
        uv = (cam['K'] @ Pc.T).T
        return uv[:, :2] / uv[:, 2:]

    for _, s in g.iterrows():
        tr = pred[pred.track_id == int(s.id)]
        fpt = F[(F.w == w) & (F.id == s.id)].t.to_numpy()
        best = None
        for t in fpt:
            o = tr.iloc[int(np.argmin(np.abs(tr.t_ns.to_numpy() / 1e9 - t)))]
            P = corners(o.x, o.y, o.z, max(o.l, .3), max(o.w, .3), max(o.h, .3), o.yaw)
            for name, cam in cams.items():
                k = nearest(cam['ts'], t * 1e9)
                if abs(cam['ts'][k] - t * 1e9) > 0.1e9:
                    continue
                uv = project(cam, cam['ts'][k], P)
                if uv is None:
                    continue
                W_, H_ = cam['wh']
                inside = (uv[:, 0].min() >= 0) & (uv[:, 0].max() < W_) & (uv[:, 1].min() >= 0) & (uv[:, 1].max() < H_)
                area = np.ptp(uv[:, 0]) * np.ptp(uv[:, 1])
                if inside and (best is None or area > best[0]):
                    best = (area, name, int(cam['ts'][k]), o)
        rec = dict(w=w, id=int(s.id), cls=s.cls, fp_frames=int(s.n), footprint=round(s.fp, 2), h=round(s.h, 2),
                   range=round(float(s.rng), 1), travel=round(float(s.travel), 1))
        if best is None:
            res.append(dict(rec, cam='', image='', verdict='out_of_fov'))
            continue
        _, name, ti, o = best
        cam = cams[name]
        with AnyReader([WIN / f'coda{w}{cam["suf"]}']) as r:
            m = next(r.deserialize(raw, c.msgtype) for c, c_t, raw in
                     r.messages([c for c in r.connections if c.topic == cam['top'] + '/image_raw'], start=ti, stop=ti + 1))
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        if m.encoding.lower().startswith('bgr'):
            img = img[:, :, ::-1]
        im = Image.fromarray(np.ascontiguousarray(img)); d = ImageDraw.Draw(im)
        tg = gts[nearest(gts, ti)] if len(gts) else None
        if tg is not None and abs(tg - ti) < 0.15e9:
            for _, q in gt[gt.t_ns == tg].iterrows():
                uv = project(cam, ti, corners(q.x, q.y, q.z, q.l, q.w, q.h, q.yaw))
                if uv is not None:
                    for i, j in EDGES:
                        d.line([tuple(uv[i]), tuple(uv[j])], fill=(60, 200, 110), width=2)
        tp = pred.t_ns.to_numpy()
        fr = pred[pred.t_ns == tp[int(np.argmin(np.abs(tp - ti)))]]
        for _, q in fr.iterrows():
            if q.track_id == o.track_id:
                continue
            uv = project(cam, ti, corners(q.x, q.y, q.z, max(q.l, .3), max(q.w, .3), max(q.h, .3), q.yaw))
            if uv is not None:
                for i, j in EDGES:
                    d.line([tuple(uv[i]), tuple(uv[j])], fill=(170, 170, 170), width=1)
        uv = project(cam, ti, corners(o.x, o.y, o.z, max(o.l, .3), max(o.w, .3), max(o.h, .3), o.yaw))
        for i, j in EDGES:
            d.line([tuple(uv[i]), tuple(uv[j])], fill=(235, 40, 40), width=3)
        x0, y0, x1, y1 = uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max()
        d.text((x0, max(0, y0 - 24)), f'{int(s.id)} {s.cls}', fill=(235, 40, 40), font=font)
        cx, cy, half = (x0 + x1) / 2, (y0 + y1) / 2, max(x1 - x0, y1 - y0, 60) * 1.3
        crop = im.crop((int(cx - half), int(cy - half), int(cx + half), int(cy + half))).resize((m.height // 2 * 2,) * 2)
        sc = 720 / m.height
        full = im.resize((int(m.width * sc), 720))
        canvas = Image.new('RGB', (full.width + 720, 720), (18, 20, 24))
        canvas.paste(full, (0, 0)); canvas.paste(crop.resize((720, 720)), (full.width, 0))
        ImageDraw.Draw(canvas).text((8, 8), f'{w} {name} t={ti/1e9:.1f} id {int(s.id)} cls {s.cls}  range {s.rng:.1f} m  '
                                    f'fp {s.fp:.2f} h {s.h:.2f}  travel {s.travel:.1f} m  FP frames {int(s.n)}',
                                    fill=(255, 255, 0), font=font)
        fn = f'{w}_id{int(s.id)}_{s.cls}.jpg'
        canvas.save(out / fn, quality=82)
        res.append(dict(rec, cam=name, image=fn, verdict=''))
pd.DataFrame(res).to_csv(out / f'samples_{tag}.csv', index=False)
print(pd.DataFrame(res).to_string(index=False))
