# YOLO26s-seg, same weights/classes/thresholds as the SINS reference profile (9 classes, bicycle 0.3, others 0.5, 640 px)
import glob, os, time
import cv2, numpy as np
from ultralytics import YOLO
m = YOLO('/home/ros/src/sins/weights/yolo26s-seg.pt')
names = m.names
keep = {'person', 'bicycle', 'car', 'motorcycle', 'motorbike', 'bus', 'truck', 'dog', 'cat', 'horse'}
col = {'person': (60, 60, 230), 'bicycle': (0, 170, 255), 'car': (230, 140, 30), 'truck': (200, 90, 160), 'bus': (200, 90, 160), 'motorcycle': (0, 200, 200)}
D = '/home/ros/bags/audit/yolo_demo'
for f in sorted(glob.glob(f'{D}/*_[0-9]*s.jpg')):
    im = cv2.imread(f)
    t = time.time(); r = m.predict(im, imgsz=640, conf=0.3, verbose=False)[0]; dt = (time.time() - t) * 1000
    over = im.copy(); n = 0
    if r.masks is not None:
        for b, msk in zip(r.boxes, r.masks.data.cpu().numpy()):
            cls = names[int(b.cls)]; sc = float(b.conf)
            if cls not in keep or (cls != 'bicycle' and sc < 0.5): continue
            c = col.get(cls, (120, 200, 60)); n += 1
            mm = cv2.resize(msk, (im.shape[1], im.shape[0])) > 0.5
            over[mm] = (0.45 * np.array(c) + 0.55 * over[mm]).astype(np.uint8)
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            cv2.rectangle(over, (x1, y1), (x2, y2), c, 2)
            lab = f'{cls} {sc:.2f}'
            (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(over, (x1, max(0, y1 - th - 8)), (x1 + tw + 6, y1), c, -1)
            cv2.putText(over, lab, (x1 + 3, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    h = 720; w = int(im.shape[1] * h / im.shape[0])
    cv2.imwrite(f.replace('.jpg', '_yolo.jpg'), cv2.resize(over, (w, h)), [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(os.path.basename(f), n, 'objects', f'{dt:.0f} ms')
