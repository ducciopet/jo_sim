"""Does an independent geometric detector (Dynablox, M-detector) see a mover where SINS's moving FP / TP outputs are?
For each SINS moving output (rep 1) from fp_funnel.py: hit = a baseline output within 1.5 m and 0.15 s. Controls:
TP(control) = up to 400 SINS true positives per window; NULL = each FP shifted 5 m in a random direction (the
baseline's own false-alarm rate). A no_object FP bucket far above NULL and close to TP = real movers the GT lacks.
usage (repo root, after fp_funnel.py LVE2 ...): python3 eval/diagnostics/lab_fp/baseline_corrob.py [TAG=LVE2]
baselines: NAS derived/baselines_out/{dynablox,mdetector}F_coda<win>.bag (read only). ~15 min for the 9 tuning windows.
"""
import sys
from pathlib import Path
E = Path(__file__).resolve().parents[2]; sys.path.insert(0, str(E)); sys.path.insert(0, str(E / 'adapters'))
import numpy as np, pandas as pd
from marker_adapter import load_marker_boxes
from points_adapter import load_dynamic_points
from harness.tracks import objects
S=str(E.parent)+'/bags/diag/'; TAG=sys.argv[1] if len(sys.argv)>1 else 'LVE2'
B='/mnt/nas-imperatore/datasets_SINS/derived/baselines_out/'
F=pd.read_csv(S+f'fprows_{TAG}.csv'); T=pd.read_csv(S+f'tprows_{TAG}.csv')
F=F[F.rep.isna()]; T=T[T.rep.isna()]
out=[]
for w in sorted(F.w.unique()):
    for m in ('dynablox','mdetector'):
        f=f'{B}{m}F_coda{w}.bag'
        try:
            P=objects(load_marker_boxes(f,'/motion_detector/visualization/clusters',persistent_ids=True) if m=='dynablox' else load_dynamic_points(f,'/m_detector/point_out',min_points=20))
        except Exception as e:
            print('skip',w,m,e); continue
        pt=P.t_ns.to_numpy()/1e9
        def hit(rows):
            h=[]
            for _,r in rows.iterrows():
                q=P[np.abs(pt-r.t)<0.15]
                h.append(len(q)>0 and np.min(np.hypot(q.x-r.x,q.y-r.y))<1.5)
            return np.array(h,bool)
        fw=F[F.w==w]; tw=T[T.w==w]
        hf=hit(fw)
        for b in fw.b.unique():
            out.append(dict(w=w,m=m,b=b,n=int((fw.b==b).sum()),hit=int(hf[(fw.b==b).to_numpy()].sum())))
        rng_=np.random.default_rng(0); a_=rng_.uniform(0,2*np.pi,len(fw)); nw=fw.assign(x=fw.x+5*np.cos(a_),y=fw.y+5*np.sin(a_))
        hn=hit(nw); out.append(dict(w=w,m=m,b='NULL(FP shifted 5m)',n=len(hn),hit=int(hn.sum())))
        ht=hit(tw.sample(min(len(tw),400),random_state=0)) if len(tw) else np.array([])
        out.append(dict(w=w,m=m,b='TP(control)',n=len(ht),hit=int(ht.sum())))
O=pd.DataFrame(out); O['seq']=O.w.str.split('_').str[0]
G=O.groupby(['m','seq','b'])[['n','hit']].sum(); G['pct']=(100*G.hit/G.n).round(0)
print(G.to_string())
