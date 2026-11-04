"""Project static-sensor pseudo-GT boxes onto the front camera of the Marina di Pisa street bag, for visual checking.
usage: gt_overlay.py <out_dir> [n_frames]"""
import sys
from pathlib import Path
import numpy as np, pandas as pd, cv2
from rosbags.highlevel import AnyReader

BAG, ODOM, GT = Path('bags/mdp/mdp_street'), Path('bags/mdp/mdp_street_odom'), 'bags/mdp/mdp_street_gt.csv'
out = Path(sys.argv[1]); out.mkdir(exist_ok=True); N = int(sys.argv[2]) if len(sys.argv) > 2 else 12


def qR(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T(tr):
    M = np.eye(4); q, p = tr.rotation, tr.translation
    M[:3, :3] = qR(q.x, q.y, q.z, q.w); M[:3, 3] = [p.x, p.y, p.z]; return M


with AnyReader([ODOM]) as r:
    c = [c for c in r.connections if c.topic == '/odometry/filtered']
    m = r.deserialize(next(r.messages(c))[2], c[0].msgtype)
    P, Q = m.pose.pose.position, m.pose.pose.orientation
    T_ob = np.eye(4); T_ob[:3, :3] = qR(Q.x, Q.y, Q.z, Q.w); T_ob[:3, 3] = [P.x, P.y, P.z]

g = pd.read_csv(GT); g = g[g.track_id >= 0]
# pick frames with in-FOV objects, half with vehicles, spread over time
fr = g[g.in_fov].groupby('t_ns')['class'].agg(lambda s: 'vehicle' in set(s))
rng = np.random.default_rng(0)
veh = fr[fr].index.values; per = fr[~fr].index.values
pick = set(rng.choice(veh, min(N // 2, len(veh)), replace=False)) | set(rng.choice(per, N - min(N // 2, len(veh)), replace=False))
pick = np.array(sorted(pick))

with AnyReader([BAG]) as r:
    tf = {}
    for cc, _, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
        for t in r.deserialize(raw, cc.msgtype).transforms:
            tf[t.child_frame_id] = (t.header.frame_id, T(t.transform))
    M = np.eye(4); f = 'front_camera_color_optical_frame'
    while f != 'base_link':
        par, Tp = tf[f]; M = Tp @ M; f = par
    T_oc = T_ob @ M; T_co = np.linalg.inv(T_oc)
    ci = [c for c in r.connections if c.topic == '/front_camera/camera/color/camera_info']
    K = np.array(r.deserialize(next(r.messages(ci))[2], ci[0].msgtype).k).reshape(3, 3)
    ic = [c for c in r.connections if c.topic == '/front_camera/camera/color/image_raw/compressed']
    k = 0; tiles = []
    for cc, t, raw in r.messages(ic):
        if k >= len(pick): break
        if t < pick[k] - 50_000_000: continue
        tgt = pick[k]; k += 1
        im = cv2.imdecode(np.frombuffer(r.deserialize(raw, cc.msgtype).data, np.uint8), cv2.IMREAD_COLOR)
        for _, o in g[g.t_ns == tgt].iterrows():
            c, s = np.cos(o.yaw), np.sin(o.yaw)
            dx, dy = np.meshgrid([-o.l / 2, o.l / 2], [-o.w / 2, o.w / 2]); dx, dy = dx.ravel(), dy.ravel()
            pts = np.array([[o.x + c * a - s * b, o.y + s * a + c * b, o.z + hz] for a, b in zip(dx, dy) for hz in (-o.h / 2, o.h / 2)])
            pc = (T_co[:3, :3] @ pts.T).T + T_co[:3, 3]
            if (pc[:, 2] < 0.3).any(): continue
            uv = (K @ pc.T).T; uv = uv[:, :2] / uv[:, 2:]
            x0, y0 = uv.min(0).astype(int); x1, y1 = uv.max(0).astype(int)
            col = (0, 0, 255) if o['class'] == 'vehicle' else (0, 255, 0)
            cv2.rectangle(im, (x0, y0), (x1, y1), col, 3)
            cv2.putText(im, f"#{o.track_id} {o['class'][0]} {o.range:.0f}m", (max(0, x0), max(25, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
        cv2.putText(im, f"{(tgt - g.t_ns.min()) / 1e9:.0f}s", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 3)
        tiles.append(cv2.resize(im, (640, 360)))
for i in range(0, len(tiles), 4):
    rows = [np.hstack(tiles[j:j + 2] + [np.zeros_like(tiles[0])] * (2 - len(tiles[j:j + 2]))) for j in range(i, min(i + 4, len(tiles)), 2)]
    cv2.imwrite(str(out / f'sheet{i // 4}.jpg'), np.vstack(rows))
print(len(tiles), 'frames')
