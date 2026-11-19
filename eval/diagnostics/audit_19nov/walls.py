import sys, numpy as np, pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
sys.path.insert(0,'/home/duccio/Desktop/jo_sim_sins/eval')
from harness.ros_io import load_odometry
B='/home/duccio/Desktop/jo_sim_sins/bags/coda/'; tag=sys.argv[1]; ts=get_typestore(Stores.ROS2_JAZZY)
G={'Pedestrian':'persone'}
for c in ['Car','Pickup Truck','Utility Vehicle','Service Vehicle','Delivery Truck','Bus']: G[c]='veicoli'
for c in ['Bike','Scooter','Motorcycle']: G[c]='bici'
def quat_yaw(q): return np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def corners(x,y,l,w,yaw):
    c,s=np.cos(yaw),np.sin(yaw); return np.array([[x+c*a-s*b,y+s*a+c*b] for a,b in [(l/2,w/2),(l/2,-w/2),(-l/2,-w/2),(-l/2,w/2)]])
def inside(p,x,y,l,w,yaw,m=0.0):
    c,s=np.cos(yaw),np.sin(yaw); dx,dy=p[:,0]-x,p[:,1]-y; u=dx*c+dy*s; v=-dx*s+dy*c
    return (np.abs(u)<=l/2+m)&(np.abs(v)<=w/2+m)
rows=[]
for w in sys.argv[2:]:
    c=w if '_' in w else '0_'+w
    W={}
    with AnyReader([Path(B+f'runs/{tag}_v2_coda{c}')],default_typestore=ts) as r:
        con=[x for x in r.connections if x.topic=='/sins/static_structures/wall_markers']
        for cn,t,raw in r.messages(connections=con):
            m=r.deserialize(raw,cn.msgtype); bx=[]
            for mk in m.markers:
                if mk.ns!='wall_bbox' or mk.action!=0: continue
                p=mk.pose.position; bx.append((p.x,p.y,mk.scale.x,mk.scale.y,quat_yaw(mk.pose.orientation),mk.header.frame_id))
            W[t]=bx
    if not W: print(c,'no wall markers recorded'); continue
    Wt=np.array(sorted(W)); fr={b[5] for v in W.values() for b in v}
    g=pd.read_csv(B+f'coda{c}_gt.csv'); g['grp']=g['class'].map(G).fillna('altro')
    od=load_odometry(B+f'coda{c}_odom')
    for _,q in g.iterrows():
        rx,ry=np.interp(q.t_ns/1e9,od[:,0],od[:,1]),np.interp(q.t_ns/1e9,od[:,0],od[:,2])
        if not 1.2<=np.hypot(q.x-rx,q.y-ry)<=25: continue
        # wall markers stamped by bag receive time: take the latest set at or before t
        k=Wt[np.searchsorted(Wt,q.t_ns,side='right')-1] if q.t_ns>=Wt[0] else None
        if k is None: continue
        P=corners(q.x,q.y,q.l,q.w,q.yaw); hit=0.0
        # share of a 7x7 grid over the GT footprint that lies within 0.15 m of a wall box
        u=np.linspace(-q.l/2,q.l/2,7); v=np.linspace(-q.w/2,q.w/2,7); U,V=np.meshgrid(u,v); cc,ss=np.cos(q.yaw),np.sin(q.yaw)
        pts=np.c_[q.x+cc*U.ravel()-ss*V.ravel(), q.y+ss*U.ravel()+cc*V.ravel()]
        m=np.zeros(len(pts),bool)
        for b in W[k]: m|=inside(pts,b[0],b[1],b[2],b[3],b[4],0.15)
        cen=np.array([[b[0],b[1]] for b in W[k]]) if W[k] else np.zeros((0,2))
        own=bool(len(cen)) and bool(inside(cen,q.x,q.y,q.l,q.w,q.yaw,0.3).any())
        rows.append(dict(win=c,grp=q.grp,cls=q['class'],h=q.h,frac=m.mean(),own=own))
    print(c,'wall frames',len(W),'frames',fr)
O=pd.DataFrame(rows)
print(O.groupby('grp').apply(lambda x: pd.Series({'rows':len(x),'touched by a wall %':100*(x.frac>0).mean(),'wall centred on the object %':100*x.own.mean()}),include_groups=False).round(1))
print(O[O.grp=='veicoli'].groupby('cls').apply(lambda x: f"{100*(x.frac>0).mean():.0f}% touched, {100*x.own.mean():.0f}% wall centred on it",include_groups=False).to_dict())
