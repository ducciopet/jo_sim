#!/usr/bin/env python3
"""LiDAR view of the sampled FP tracks the cameras do not see (fp_cam.py verdict out_of_fov).
Left: bird's-eye 10x10 m around the FP box at its middle FP tick, odom frame; scans at t-1 s (blue), t (white), t+1 s
(orange), points 0.2-2.5 m above the local ground; FP box red, its whole trajectory red line; other SINS tracks grey;
raw CODa annotations green (dropped edge rows included). A moving agent leaves displaced blobs; structure overlaps.
Right: side view of the points within 0.8 m of the box centre at t (horizontal = distance along the robot->object
line, vertical = z above the local ground). usage: lidar_view.py samples_TAG.csv fprows_TAG[_edge].csv TAG IMG_DIR
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
from make_coda_video import tf_mat, read_odom, nearest, corners   # noqa: E402

WIN = Path('/mnt/nas-imperatore/datasets_SINS/derived/coda_windows')
S = pd.read_csv(sys.argv[1], keep_default_na=False)
F = pd.read_csv(sys.argv[2], keep_default_na=False)
F = F[F.rep == '']
tag, out = sys.argv[3], Path(sys.argv[4])
try:
    font = ImageFont.truetype('DejaVuSans-Bold.ttf', 16)
except OSError:
    font = ImageFont.load_default()
PX, HALF = 60, 5.0          # px per metre, half size of the BEV crop [m]
for w, g in S[S.verdict == 'out_of_fov'].groupby('w'):
    run = R / f'bags/coda/runs/{tag}_v2_coda{w}'
    st = {}
    with AnyReader([run], default_typestore=typestore()) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
            for tr in r.deserialize(raw, c.msgtype).transforms:
                st[(tr.header.frame_id, tr.child_frame_id)] = tf_mat(tr.transform)
    ot, oT = read_odom(R / f'bags/coda/coda{w}_odom')
    Tbv = st[('base_link', 'velodyne')]
    pred = objects(load_obstacles(run)[0])
    gt = pd.read_csv(R / f'bags/coda/coda{w}_gt.csv', keep_default_na=False)
    gt = gt[gt.track_id.astype(int) >= 0]
    for k in ('x', 'y', 'z', 'l', 'w', 'h', 'yaw'):
        gt[k] = gt[k].astype(float)
    gts = np.unique(gt.t_ns.astype('int64').to_numpy())
    with AnyReader([WIN / f'coda{w}']) as r:
        con = [c for c in r.connections if c.topic == '/velodyne_points']
        cts = np.array(sorted(c_t for _, c_t, _ in r.messages(con)))

        def scan(t_ns):
            k = nearest(cts, t_ns)
            m = next(r.deserialize(raw, c.msgtype) for c, _, raw in r.messages(con, start=cts[k], stop=cts[k] + 1))
            n = m.width * m.height
            xyz = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)[:, :12].copy().view(np.float32).astype(float)
            T = oT[nearest(ot, cts[k])] @ Tbv
            return xyz @ T[:3, :3].T + T[:3, 3]

        for _, s in g.iterrows():
            tr = pred[pred.track_id == int(s.id)].sort_values('t_ns')
            fpt = np.sort(F[(F.w == w) & (F.id.astype(int) == int(s.id))].t.astype(float).to_numpy())
            t = fpt[len(fpt) // 2]
            o = tr.iloc[int(np.argmin(np.abs(tr.t_ns.to_numpy() / 1e9 - t)))]
            gz = o.z - o.h / 2                                         # local ground ~ bottom of the box
            W = int(2 * HALF * PX)
            im = Image.new('RGB', (W + 420, W), (18, 20, 24)); d = ImageDraw.Draw(im)

            def px(x, y):
                return ((x - o.x + HALF) * PX, (HALF - (y - o.y)) * PX)
            for dt, col in ((-1.0, (70, 130, 255)), (1.0, (255, 150, 40)), (0.0, (235, 235, 235))):
                P = scan(int((t + dt) * 1e9))
                P = P[(np.abs(P[:, 0] - o.x) < HALF) & (np.abs(P[:, 1] - o.y) < HALF) &
                      (P[:, 2] - gz > 0.2) & (P[:, 2] - gz < 2.5)]
                for x, y, _ in P:
                    u, v = px(x, y); d.rectangle([u - 1, v - 1, u + 1, v + 1], fill=col)
                if dt == 0.0:
                    P0 = P
            k = gts[nearest(gts, int(t * 1e9))]
            for _, q in gt[gt.t_ns.astype('int64') == k].iterrows():
                c4 = corners(q.x, q.y, q.z, q.l, q.w, q.h, q.yaw)[:4]
                d.polygon([px(*c[:2]) for c in c4], outline=(60, 200, 110), width=2)
            tp = pred.t_ns.to_numpy()
            for _, q in pred[pred.t_ns == tp[int(np.argmin(np.abs(tp - t * 1e9)))]].iterrows():
                if q.track_id != o.track_id:
                    d.polygon([px(*c[:2]) for c in corners(q.x, q.y, q.z, q.l, q.w, q.h, q.yaw)[:4]], outline=(150, 150, 150))
            d.line([px(x, y) for x, y in zip(tr.x, tr.y)], fill=(235, 40, 40), width=2)
            d.polygon([px(*c[:2]) for c in corners(o.x, o.y, o.z, o.l, o.w, o.h, o.yaw)[:4]], outline=(235, 40, 40), width=3)
            # robot direction arrow
            rp = oT[nearest(ot, int(t * 1e9))][:3, 3]
            dv = np.array([rp[0] - o.x, rp[1] - o.y]); dv /= max(np.linalg.norm(dv), 1e-6)
            d.line([px(o.x, o.y), px(o.x + 2 * dv[0], o.y + 2 * dv[1])], fill=(255, 255, 0), width=1)
            # side view
            near = P0[np.hypot(P0[:, 0] - o.x, P0[:, 1] - o.y) < 0.8]
            ax = np.array([-dv[1], dv[0]])
            for x, y, z in near:
                u = W + 210 + ((x - o.x) * ax[0] + (y - o.y) * ax[1]) * 120
                v = W - 40 - (z - gz) * 120
                d.rectangle([u - 2, v - 2, u + 2, v + 2], fill=(235, 235, 235))
            for hh in (0.5, 1.0, 1.5, 2.0, 2.5):
                v = W - 40 - hh * 120; d.line([(W + 20, v), (W + 400, v)], fill=(60, 60, 60)); d.text((W + 22, v - 16), f'{hh} m', fill=(120, 120, 120), font=font)
            d.text((8, 8), f'{w} t={t:.1f} id {int(s.id)} cls {s.cls}  range {s.range} m  fp {s.footprint} h {s.h}  travel {s.travel} m',
                   fill=(255, 255, 0), font=font)
            d.text((8, 30), 'blue t-1s  white t  orange t+1s  | red FP box+trajectory  green CODa GT  grey other tracks  yellow -> robot',
                   fill=(200, 200, 200), font=font)
            d.text((W + 20, 8), f'side view, {len(near)} pts within 0.8 m', fill=(200, 200, 200), font=font)
            fn = f'lidar_{w}_id{int(s.id)}_{s.cls}.png'
            im.save(out / fn)
            print(fn)
