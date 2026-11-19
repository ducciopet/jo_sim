# YOLO mask -> 3D leaf -> cut of the LiDAR object, re-done offline on real CODa frames with the reference profile's
# numbers (preprocessing.yaml coda_vjsye2: bin 0.3, valley 0.08, depth tolerance 0.3, min points 10, nearest cluster
# for LiDAR leaves beyond depth_max 6 m with min box coverage 0.5; dbscan_detector.yaml: refine margin 0.10; PCA yaw OBB
# as sins::computeObb). The LiDAR object is approximated by the scan points inside the GT box (+0.15 m), not the
# DBSCAN cluster. Writes per-frame npz for the figures and prints leaf/object/GT lengths.
import sys, os, json, numpy as np, pandas as pd, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
def qmat(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
from ultralytics import YOLO

SEQ = sys.argv[1] if len(sys.argv) > 1 else 'coda10_w1'
B = f'/home/ros/bags/coda/{SEQ}'
OUT = f'/home/ros/bags/audit/yolo3d/{SEQ}'; os.makedirs(OUT, exist_ok=True)
BIN, VALLEY, TOL, MINP, COV, DMAX, MARGIN = 0.3, 0.08, 0.3, 10, 0.5, 6.0, 0.10
VEH = ['Car', 'Pickup Truck', 'Utility Vehicle']

def T(tr, q):
    M = np.eye(4); M[:3, :3] = qmat(q.x, q.y, q.z, q.w); M[:3, 3] = [tr.x, tr.y, tr.z]; return M

def read(path, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    tt = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        t, d, ts = r.read_next()
        if t in topics: yield t, deserialize_message(d, get_message(tt[t])), ts

def window(depths, us, bw):
    """depthModeValleyWindow, nearest-cluster variant (LiDAR leaf beyond depth_max) with the profile numbers."""
    dmin, dmax = depths.min(), depths.max(); nb = max(1, int(np.ceil((dmax - dmin) / BIN)) + 1)
    b = np.clip(((depths - dmin) / BIN).astype(int), 0, nb - 1); h = np.bincount(b, minlength=nb)
    best = int(np.argmax(h)); floor = max(1, int(np.ceil(h[best] * VALLEY))); lo = hi = best; found = False
    if 2 * (depths > DMAX).sum() > len(depths):
        s = 0
        while s < nb:
            if h[s] < floor: s += 1; continue
            e, n, pk = s, 0, s
            while e < nb and h[e] >= floor:
                n += h[e]; pk = e if h[e] > h[pk] else pk; e += 1
            zl, zh = dmin + s * BIN, dmin + e * BIN; sel = (depths >= zl) & (depths < zh)
            wide = sel.any() and (us[sel].max() - us[sel].min()) >= COV * bw
            if n >= MINP and wide: best, lo, hi, found = pk, s, e - 1, True; break
            s = e
    if not found:
        while lo > 0 and h[lo - 1] >= floor: lo -= 1
        while hi < nb - 1 and h[hi + 1] >= floor: hi += 1
    mode = dmin + (best + 0.5) * BIN
    nocap = (dmin + lo * BIN, dmin + (hi + 1) * BIN)
    return (max(nocap[0], mode - TOL), min(nocap[1], mode + TOL)), nocap

def obb(p):
    """sins::computeObb yaw-only: PCA yaw in xy, extents = min/max along the axes. Returns centre, yaw, size."""
    m = p.mean(0); d = p - m; c11, c12, c22 = (d[:, 0]**2).sum(), (d[:, 0]*d[:, 1]).sum(), (d[:, 1]**2).sum()
    yaw = 0.5 * np.arctan2(2 * c12, c11 - c22); R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    q = d[:, :2] @ R; lo, hi = q.min(0), q.max(0); cxy = m[:2] + R @ ((lo + hi) / 2)
    return np.array([cxy[0], cxy[1], (p[:, 2].min() + p[:, 2].max()) / 2]), yaw, np.array([*(hi - lo), np.ptp(p[:, 2])])

def inbox(p, c, yaw, s, e):
    R = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]); q = (p[:, :2] - c[:2]) @ R.T
    return (np.abs(q[:, 0]) <= s[0] / 2 + e) & (np.abs(q[:, 1]) <= s[1] / 2 + e) & (np.abs(p[:, 2] - c[2]) <= s[2] / 2 + e)

# --- calibration and poses
Tbv = Tvc = None; K = None
for t, m, ts in read(f'{B}/{SEQ}.mcap', ['/tf_static', '/front_camera/camera/color/camera_info']):
    if t == '/tf_static':
        for x in m.transforms:
            if x.child_frame_id == 'velodyne': Tbv = T(x.transform.translation, x.transform.rotation)
            if x.child_frame_id == 'front_camera_depth_optical_frame': Tvc = T(x.transform.translation, x.transform.rotation)
    else: K = np.array(m.k).reshape(3, 3); W, H = m.width, m.height
    if Tbv is not None and Tvc is not None and K is not None: break
odo = [(ts, T(m.pose.pose.position, m.pose.pose.orientation)) for t, m, ts in read(f'{B}_odom/{SEQ}_odom.mcap', ['/odometry/filtered'])]
ots = np.array([o[0] for o in odo])
gt = pd.read_csv(f'/home/ros/bags/coda/{SEQ}_gt.csv'); gt = gt[gt['class'].isin(VEH)]
gts = np.sort(gt.t_ns.unique())

# --- scans and images paired to the GT frames (10 Hz GT on scan stamps)
want = set(gts.tolist()); scans = {}; imgs = []
for t, m, ts in read(f'{B}/{SEQ}.mcap', ['/velodyne_points', '/front_camera/camera/color/image_raw']):
    st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
    if t == '/velodyne_points':
        k = gts[np.argmin(np.abs(gts - st))]
        if abs(k - st) < 20e6: scans[k] = (st, pc2.read_points_numpy(m, field_names=('x', 'y', 'z')))
    else: imgs.append((st, m))
model = YOLO('/home/ros/src/sins/weights/yolo26s-seg.pt'); names = model.names
Tcv = np.linalg.inv(Tvc); rows = []
for k, (st, P) in sorted(scans.items()):
    P = P[np.isfinite(P).all(1)]
    i = int(np.argmin([abs(a - st) for a, _ in imgs])); its, im = imgs[i]
    if abs(its - st) > 60e6: continue
    Tob = odo[int(np.argmin(np.abs(ots - st)))][1]; Tov = Tob @ Tbv; Tvo = np.linalg.inv(Tov)
    pc = (Tcv[:3, :3] @ P.T).T + Tcv[:3, 3]; front = pc[:, 2] > 0.5
    uv = np.full((len(P), 2), -1.0); uv[front] = (K @ pc[front].T).T[:, :2] / pc[front][:, 2:3]
    inimg = front & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    a = np.frombuffer(im.data, np.uint8).reshape(im.height, im.width, -1)[:, :, :3]; rgb = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if im.encoding == 'rgb8' else a.copy()
    r = model.predict(rgb, imgsz=1280, conf=0.3, verbose=False)[0]
    if r.masks is None: continue
    masks = [(names[int(b.cls)], float(b.conf), b.xyxy[0].cpu().numpy(), cv2.resize(mk, (W, H)) > 0.5)
             for b, mk in zip(r.boxes, r.masks.data.cpu().numpy()) if names[int(b.cls)] in ('car', 'truck', 'bus') and float(b.conf) >= 0.5]
    for _, g in gt[gt.t_ns == k].iterrows():
        c = (Tvo @ np.array([g.x, g.y, g.z, 1]))[:3]; Ry = np.arctan2(Tvo[1, 0], Tvo[0, 0]); yaw = g.yaw + Ry
        obj = inbox(P, c, yaw, np.array([g.l, g.w, g.h]), 0.15) & (P[:, 2] > c[2] - g.h / 2 + 0.15)
        if obj.sum() < 30 or not inimg[obj].mean() > 0.8: continue
        u = np.clip(uv[obj & inimg].astype(int), 0, [W - 1, H - 1])
        best = max(masks, key=lambda mm: mm[3][u[:, 1], u[:, 0]].mean(), default=None)
        if best is None or best[3][u[:, 1], u[:, 0]].mean() < 0.5: continue
        ui = np.clip(uv.astype(int), 0, [W - 1, H - 1]); inm = inimg & best[3][ui[:, 1], ui[:, 0]]
        if inm.sum() < MINP: continue
        dep = pc[inm, 2]; (zl, zh), (nl, nh) = window(dep, uv[inm, 0], best[2][2] - best[2][0])
        leaf = np.zeros(len(P), bool); leaf[np.where(inm)[0][(dep >= zl) & (dep < zh)]] = True
        leaf_nc = np.zeros(len(P), bool); leaf_nc[np.where(inm)[0][(dep >= nl) & (dep < nh)]] = True
        if leaf.sum() < MINP: continue
        lc, ly, ls = obb(P[leaf]); cut = obj & inbox(P, lc, ly, ls, MARGIN)
        oc, oy, os_ = obb(P[obj]); cc, cy, cs = obb(P[cut]) if cut.sum() >= 3 else (lc, ly, ls * 0)
        nc_c, nc_y, nc_s = obb(P[leaf_nc])
        ray = np.arctan2(c[1], c[0]); rel = np.degrees(abs((yaw - ray + np.pi / 2) % np.pi - np.pi / 2))
        rec = dict(t=int(k), track=int(g.track_id), cls=g['class'], range=float(np.hypot(c[0], c[1])), rel_deg=float(rel),
                   gt_l=float(g.l), gt_w=float(g.w), obj_pts=int(obj.sum()), mask_pts=int(inm.sum()), leaf_pts=int(leaf.sum()),
                   obj_len=float(os_[:2].max()), leaf_len=float(ls[:2].max()), cut_len=float(cs[:2].max()), cut_pts=int(cut.sum()),
                   nocap_len=float(nc_s[:2].max()), win=[float(zl), float(zh)], win_nocap=[float(nl), float(nh)])
        rows.append(rec); print(json.dumps(rec), flush=True)
        np.savez_compressed(f'{OUT}/{k}_{int(g.track_id)}.npz', P=P, uv=uv, inimg=inimg, inm=inm, leaf=leaf, leaf_nc=leaf_nc, obj=obj,
                            cut=cut, depth=pc[:, 2], gt=np.array([c[0], c[1], c[2], g.l, g.w, g.h, yaw]), leafbox=np.r_[lc, ly, ls],
                            cutbox=np.r_[cc, cy, cs], objbox=np.r_[oc, oy, os_], ncbox=np.r_[nc_c, nc_y, nc_s], mbox=best[2],
                            mask=best[3], img=cv2.imencode('.jpg', rgb, [cv2.IMWRITE_JPEG_QUALITY, 90])[1], meta=json.dumps(rec))
pd.DataFrame(rows).to_csv(f'{OUT}/summary.csv', index=False)
