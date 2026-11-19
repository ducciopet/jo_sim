import sys, numpy as np, pandas as pd
sys.path.insert(0,'/home/duccio/Desktop/jo_sim_sins/eval'); sys.path.insert(0,'/home/duccio/Desktop/jo_sim_sins/eval/diagnostics/coda_walkers')
from ped_funnel_lib import det3d, nearest
from harness.ros_io import load_odometry
B='/home/duccio/Desktop/jo_sim_sins/bags/coda/'; tag=sys.argv[1]
VEH=['Car','Pickup Truck','Utility Vehicle','Service Vehicle','Delivery Truck','Bus']; out=[]
for w in sys.argv[2:]:
    c=w if '_' in w else '0_'+w
    g=pd.read_csv(B+f'coda{c}_gt.csv'); g=g[g['class'].isin(VEH+['Bike','Scooter'])]
    D=det3d(B+f'runs/{tag}_v2_coda{c}','/sins/dbscan_detector/fused/detections3d'); Dt=np.array(sorted(D))
    od=load_odometry(B+f'coda{c}_odom')
    for _,q in g.iterrows():
        rx,ry=np.interp(q.t_ns/1e9,od[:,0],od[:,1]),np.interp(q.t_ns/1e9,od[:,0],od[:,2]); r=np.hypot(q.x-rx,q.y-ry)
        if not 1.2<=r<=25: continue
        dets=nearest(D,Dt,q.t_ns); cy,sy=np.cos(q.yaw),np.sin(q.yaw); ins=[]
        for d in dets:
            dx,dy=d[0]-q.x,d[1]-q.y; u=dx*cy+dy*sy; v=-dx*sy+dy*cy
            if abs(u)<=q.l/2+0.3 and abs(v)<=q.w/2+0.3: ins.append(d)
        out.append(dict(win=c,cls='veicolo' if q['class'] in VEH else 'bici',gl=q.l,gh=q.h,rng=r,npts=q.npts,n=len(ins),
             maxlen=max([d[2] for d in ins],default=0),maxh=max([d[3] for d in ins],default=0),
             low=sum(d[3]<0.7 for d in ins),cls_any=any(d[4] not in ('','unknown') for d in ins)))
O=pd.DataFrame(out); O.to_csv(f'/tmp/claude-1000/-home-duccio-Desktop-jo-sim-sins/6b5f28fe-4bd1-41ab-95fb-b1ee18ac2c62/scratchpad/frag_{tag}.csv',index=False)
for k,X in O.groupby('cls'):
    print(k,len(X),'rows | 0 det %.0f%%, 1 det %.0f%%, >=2 det %.0f%% | largest piece / GT length median %.2f | rows with a piece < 0.7 m tall %.0f%%'%(
        100*(X.n==0).mean(),100*(X.n==1).mean(),100*(X.n>=2).mean(),(X.maxlen/X.gl)[X.n>0].median(),100*(X.low>0).mean()))
    X=X.assign(b=pd.cut(X.rng,[0,10,15,25])); print(X.groupby('b',observed=True).apply(lambda y: f"0det {100*(y.n==0).mean():.0f}%  frag {100*(y.n>=2).mean():.0f}%  len {(y.maxlen/y.gl)[y.n>0].median():.2f}",include_groups=False).to_dict())
