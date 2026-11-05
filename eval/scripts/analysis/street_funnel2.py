import sys, numpy as np, pandas as pd
from rosbags.highlevel import AnyReader
from pathlib import Path
run=sys.argv[1]
g=pd.read_csv('bags/mdp/mdp_street_gt.csv'); g=g[g.track_id>=0]
g=g[(g.range<=15)&(g.range>=1.2)]
# speed of GT
g=g.sort_values(['track_id','t_ns'])
sp=[]
for tid,d in g.groupby('track_id', sort=False):
    t=d.t_ns.values/1e9; x=d.x.values; y=d.y.values
    for i in range(len(d)):
        j0=np.searchsorted(t,t[i]-1); j1=np.searchsorted(t,t[i]+1,'right')-1
        sp.append(np.hypot(x[j1]-x[j0],y[j1]-y[j0])/max(t[j1]-t[j0],0.1) if t[j1]-t[j0]>0.5 else np.nan)
g['speed']=sp
def det_rows(topic, kind):
    out={}
    with AnyReader([Path(run)]) as r:
        c=[c for c in r.connections if c.topic==topic]
        for cc,ts,raw in r.messages(c):
            m=r.deserialize(raw,cc.msgtype); t=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            L=[]
            if kind=='obs':
                for o in m.obstacles: L.append((o.pose.position.x,o.pose.position.y,o.size.x,o.size.y,o.status,''))
            else:
                for d in m.detections:
                    b=d.bbox; lab=d.results[0].hypothesis.class_id if d.results else ''
                    L.append((b.center.position.x,b.center.position.y,b.size.x,b.size.y,-1,lab))
            out[t]=np.array([l[:5] for l in L],float).reshape(-1,5), [l[5] for l in L]
    return out
D=det_rows('/sins/dbscan_detector/fused/detections3d','det')
T=det_rows('/sins/tracker/tracks','det')
O=det_rows('/perception/tracked_dynamic_obstacles','obs')
res=[]
for _,row in g.iterrows():
    t=row.t_ns
    def near(S, R=1.5):
        if t not in S or not len(S[t][0]): return None
        a,labs=S[t]; d=np.hypot(a[:,0]-row.x,a[:,1]-row.y); k=np.argmin(d)
        return (d[k], a[k], labs[k]) if d[k]<R else None
    R = 2.5 if row['class']=='vehicle' else 1.0
    nd, nt, no = near(D,R), near(T,R), near(O,R)
    res.append(dict(speed=row.speed, fov=bool(row.in_fov), rng=row.range, cls=row['class'], det=nd is not None, det_size=max(nd[1][2],nd[1][3]) if nd else np.nan,
        trk=nt is not None, trk_state=nt[2] if nt else '', out=no is not None, out_status=int(no[1][4]) if no else -1))
R=pd.DataFrame(res)
R['dyn']=R.out_status.isin([1,3]); R['rb']=pd.cut(R.rng,[0,5,10,15])
def f(d): return pd.Series({'n':len(d),'cluster':d.det.mean(),'track':d.trk.mean(),'out_any':d.out.mean(),'out_dynamic':d.dyn.mean(),'out_static':(d.out_status==0).mean(),'out_pot':(d.out_status==2).mean()})
print(R.groupby(['cls','fov']).apply(f).round(2).to_string())
R['sb']=pd.cut(R.speed,[-.01,.5,1,3,30])
print(R.groupby(['cls','sb'],observed=True).apply(f).round(2).to_string())
print(R.groupby(['cls','rb'],observed=True).apply(f).round(2).to_string())
