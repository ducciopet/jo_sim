import json, re, sys, numpy as np
N='/mnt/nas-imperatore/datasets_SINS/derived/sins_final/'; D='eval/results/final/timing_diag/'
runs={'k0018':'kitti/0018_r1/SF_v2_kitti_0018','k0006':'kitti/0006_r1/SF_v2_kitti_0006','c2_w2':'coda_new/2_w2_r1/SF_v2_coda2_w2','c3_w1':'coda_new/3_w1_r1/SF_v2_coda3_w1','l1945':'lab/20260519_1945_r1/SF_20260519_1945'}
def md(x): return f'{np.median(x):.0f}' if len(x) else '-'
def mx(x): return f'{np.max(x):.0f}' if len(x) else '-'
for k,p in runs.items():
    S=json.load(open(D+k+'.json')); t=[s for s in S if 'trk_lat' in s]
    lat=np.array([s['trk_lat'] for s in t]); thr=np.percentile(lat,90); slow=[s for s in t if s['trk_lat']>=thr]
    g=lambda A,key:[s[key] for s in A if key in s]
    print(f'\n== {k}: stamps {len(S)}, with tracks {len(t)} ({100*(1-len(t)/len(S)):.0f}% of det stamps lost before tracks); with det {len(g(S,"det_lat"))}')
    for name,A in [('median scans',t),('slowest 10%',slow)]:
        print(f'  {name:12s} trk_lat {md(g(A,"trk_lat"))} det_lat {md(g(A,"det_lat"))} | det_n {md(g(A,"det_n"))} trk_n {md(g(A,"trk_n"))} obs_n {md(g(A,"obs_n"))}')
    log=open(N+p+'_logs/det.log',errors='ignore').read()
    f=lambda rx:[float(x) for x in re.findall(rx,log)]
    pre=f(r'\| scan ([\d.]+) ms'); raw=f(r'raw=(\d+) \| after'); vox=f(r'budget \d+\)=(\d+)')
    lms=f(r'lidar: pts=[\d.]+ clusters=[\d.]+ boxes=[\d.]+ \(.*?\) ([\d.]+)ms'); lcl=f(r'lidar: pts=[\d.]+ clusters=([\d.]+)'); lpt=f(r'lidar: pts=([\d.]+)')
    obj=f(r'boxes -> ([\d.]+) objects'); wt=f(r'wait ([\d.]+)ms'); age=f(r'detections age at callback start ([\d.]+) ms')
    tt=re.findall(r'tick timing \(median/max of 50, ms\): prep ([\d.]+)/[\d.]+ assoc ([\d.]+)/([\d.]+) fallback ([\d.]+)/[\d.]+ matched ([\d.]+)/([\d.]+) spawn ([\d.]+)/[\d.]+ publish ([\d.]+)/[\d.]+ \| tracks (\d+)',log)
    tick=[sum(float(x) for x in (a[0],a[1],a[3],a[4],a[6],a[7])) for a in tt]; ntr=[float(a[8]) for a in tt]; assoc=[float(a[1]) for a in tt]; matched=[float(a[4]) for a in tt]
    try: ld=[float(x) for x in re.findall(r'lidar depth: ([\d.]+) ms/scan',open(N+p+'_logs/lidar_depth.log',errors='ignore').read())]
    except FileNotFoundError: ld=[]
    print(f'  log windows (median / max): preprocessing scan ms {md(pre)}/{mx(pre)} (n {len(pre)}), raw pts {md(raw)}, after voxel {md(vox)}')
    print(f'    dbscan lidar stage ms {md(lms)}/{mx(lms)}, lidar pts {md(lpt)}, clusters {md(lcl)}/{mx(lcl)}, fused objects {md(obj)}/{mx(obj)}, fusion wait ms {md(wt)}/{mx(wt)}')
    print(f'    tracker tick ms (sum of medians) {md(tick)}/{mx(tick)} [assoc {md(assoc)}/{mx(assoc)}, matched {md(matched)}/{mx(matched)}], tracks {md(ntr)}/{mx(ntr)}, det age at tracker {md(age)}/{mx(age)} | lidar_depth ms {md(ld)}/{mx(ld)}')
