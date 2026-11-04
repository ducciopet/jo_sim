import sys; sys.path.insert(0,'/home/duccio/Desktop/jo_sim_sins/eval')
import numpy as np, pandas as pd, collections
from harness.ros_io import load_obstacles, load_odometry
from harness.tracks import objects
C=collections.Counter()
for run,pre in zip(sys.argv[1::2],sys.argv[2::2]):
    od=load_odometry(pre+'_odom'); g=pd.read_csv(pre+'_gt.csv'); g=g[(g.track_id>=0)&(g['class']=='Pedestrian')&(g.npts>=20)]
    O=objects(load_obstacles(run)[0]); Ot=np.unique(O.t_ns)
    for _,q in g.iterrows():
        rx,ry=np.interp(q.t_ns/1e9,od[:,0],od[:,1]),np.interp(q.t_ns/1e9,od[:,0],od[:,2])
        r=np.hypot(q.x-rx,q.y-ry)
        if not 1.2<r<25: continue
        o=O[O.t_ns==Ot[np.argmin(np.abs(Ot-q.t_ns))]]
        ok=len(o) and np.min(np.hypot(o.x-q.x,o.y-q.y))<1.0
        k='FOV' if q.in_fov else 'fuori'
        C[(k,'n')]+=1; C[(k,'ok')]+=bool(ok)
for k in ['FOV','fuori']: print(k, C[(k,'n')], f"{C[(k,'ok')]/C[(k,'n')]:.0%}")
