import sys, numpy as np, pandas as pd, collections
sys.path.insert(0,'eval'); sys.path.insert(0,'/tmp/claude-1000/-home-duccio-Desktop-jo-sim-sins/d4ee6b82-7d39-4af3-ab17-94c4c64f01d3/scratchpad')
from rosbags.highlevel import AnyReader; from pathlib import Path
from harness.ros_io import load_obstacles, load_odometry
from harness.tracks import objects
import coda_eval as ce, types
exec(open('eval/diagnostics/coda_yolo_seen.py').read().split("res=collections.Counter()")[0])
names=[l.strip() for l in open('sins/cfg/coco.names')]
def stamp(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def cloud(m):
    a=np.frombuffer(m.data,np.uint8).reshape(-1,m.point_step); return a
def read(bag):
    D=collections.defaultdict(list)
    with AnyReader([Path(bag)]) as r:
        for c,_,b in r.messages([c for c in r.connections]):
            t=c.topic
            if t=='/sins/lidar/processed':
                m=r.deserialize(b,c.msgtype); D[t].append((stamp(m),cloud(m)[:,:12].copy().view(np.float32)))
            elif t.endswith('semantic_leaves'):
                m=r.deserialize(b,c.msgtype); a=cloud(m)
                if len(a): D['leaves'].append((stamp(m),a[:,:12].copy().view(np.float32),a[:,16:20].copy().view(np.float32).ravel().astype(int)))
                else: D['leaves'].append((stamp(m),np.zeros((0,3),np.float32),np.zeros(0,int)))
            elif t in ('/sins/dbscan_detector/lidar/detections3d','/sins/dbscan_detector/fused/detections3d','/sins/tracker/tracks'):
                m=r.deserialize(b,c.msgtype)
                D[t].append((stamp(m),[(d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.center.position.z,d.bbox.size.x,d.bbox.size.y,d.bbox.size.z,[h.hypothesis.class_id for h in d.results],d.id) for d in m.detections]))
            elif t.startswith('/yolo_semantic_seg/') and t.endswith('/detections'):
                m=r.deserialize(b,c.msgtype); D['yolo_'+t.split('/')[2]].append((stamp(m),[(d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size_x,d.bbox.size_y,d.results[0].hypothesis.class_id) for d in m.detections if d.results]))
    return D
def near(L,t,tol=1.5e8):
    ts=np.array([x[0] for x in L]); k=np.argmin(abs(ts-t)); return L[k] if abs(ts[k]-t)<=tol else None
def inbox(P,g,m=0.2):
    c,s=np.cos(g.yaw),np.sin(g.yaw); q=P-[g.x,g.y,g.z]; lx=q[:,0]*c+q[:,1]*s; ly=-q[:,0]*s+q[:,1]*c
    return (abs(lx)<=g.l/2+m)&(abs(ly)<=g.w/2+m)&(abs(q[:,2])<=g.h/2+m)
stage=collections.Counter(); stage_cls=collections.defaultdict(collections.Counter); extra=collections.defaultdict(list)
for w in ['w1','w2','w3']:
    odom=load_odometry(f'bags/coda/coda0_{w}_odom'); ot,oT=full_odom(f'bags/coda/coda0_{w}_odom')
    gt,ign,frames,_=ce.build_gt(f'bags/coda/coda0_{w}_gt.csv',odom,a)
    raw=pd.read_csv(f'bags/coda/coda0_{w}_gt.csv'); raw=raw[raw.track_id>=0]
    G=objects(gt); G=G[G.state!='dynamic']
    stf,Kf,WHf=statics_K(f'bags/coda/coda0_{w}','front_camera'); sts,Ks,WHs=statics_K(f'bags/coda/coda0_{w}_stereo','stereo_camera')
    Tbv=stf[('base_link','velodyne')]
    cams={'front_camera':(Tbv@stf[('velodyne','front_camera_depth_optical_frame')]@stf[('front_camera_depth_optical_frame','front_camera_color_optical_frame')],Kf,WHf),
          'stereo_camera':(Tbv@sts[('velodyne','stereo_camera_depth_optical_frame')]@sts[('stereo_camera_depth_optical_frame','stereo_camera_color_optical_frame')],Ks,WHs)}
    D=read(f'bags/coda/runs/K1_v2_coda0_{w}')
    pub=objects(load_obstacles(f'bags/coda/runs/K1_v2_coda0_{w}')[0]); pt=np.unique(pub.t_ns)
    for _,g in G.iterrows():
        gr=raw[(raw.t_ns==g.t_ns)&(raw.track_id==g.track_id)]
        if not len(gr): continue
        gr=gr.iloc[0]; cls=gr['class']
        if cls=='Horse': continue
        Todom_base=oT[np.argmin(abs(ot-g.t_ns))]; seen=False; infov=False
        for cam,(Tbc,K,WH) in cams.items():
            Tco=np.linalg.inv(Todom_base@Tbc); c,s=np.cos(gr.yaw),np.sin(gr.yaw); cr=[]
            for dx in (-.5,.5):
                for dy in (-.5,.5):
                    for dz in (-.5,.5):
                        cr.append((Tco@[gr.x+c*dx*gr.l-s*dy*gr.w, gr.y+s*dx*gr.l+c*dy*gr.w, gr.z+dz*gr.h,1])[:3])
            cr=np.array(cr)
            if (cr[:,2]<0.5).any(): continue
            uv=(K@cr.T).T; uv=uv[:,:2]/uv[:,2:]; x0,y0=uv.min(0); x1,y1=uv.max(0)
            if not (0<=(x0+x1)/2<WH[0] and 0<=(y0+y1)/2<WH[1]): continue
            infov=True; Y=near(D['yolo_'+cam],g.t_ns)
            if Y is None: continue
            for (cx,cy,sx,sy,cl) in Y[1]:
                if cl not in DYN: continue
                ix=max(0,min(x1,cx+sx/2)-max(x0,cx-sx/2)); iy=max(0,min(y1,cy+sy/2)-max(y0,cy-sy/2)); inter=ix*iy; uni=(x1-x0)*(y1-y0)+sx*sy-inter
                if uni>0 and inter/uni>=0.3: seen=True
        if not infov: continue
        q=pub[pub.t_ns==pt[np.argmin(abs(pt-g.t_ns))]]; d=np.hypot(q.x-g.x,q.y-g.y)
        if len(q) and d.min()<1 and q.loc[d.idxmin(),'state']=='potentially_dynamic': st='0 PD ok'
        else:
            P=near(D['/sins/lidar/processed'],g.t_ns); npp=int(inbox(P[1],gr).sum()) if P else 0
            if not seen: st='1 YOLO does not detect it'
            elif npp<5: st='2 preprocessing: <5 LiDAR points left on it'
            else:
                # leaves within 0.15 s, any camera, dynamic class, >=3 pts in box
                ts=np.array([x[0] for x in D['leaves']]); ks=np.where(abs(ts-g.t_ns)<1.5e8)[0]
                leaf=any((inbox(D['leaves'][k][1],gr,0.3)&np.isin(D['leaves'][k][2],[names.index(c) for c in DYN])).sum()>=3 for k in ks)
                Lc=near(D['/sins/dbscan_detector/lidar/detections3d'],g.t_ns)
                own=[x for x in Lc[1] if np.hypot(x[0]-g.x,x[1]-g.y)<1.0] if Lc else []
                Fc=near(D['/sins/dbscan_detector/fused/detections3d'],g.t_ns)
                fcl=[x for x in Fc[1] if np.hypot(x[0]-g.x,x[1]-g.y)<1.0] if Fc else []
                Tc=near(D['/sins/tracker/tracks'],g.t_ns)
                tr=[x for x in Tc[1] if np.hypot(x[0]-g.x,x[1]-g.y)<1.0] if Tc else []
                if not leaf:
                    best=99; nmax=0
                    for k in ks:
                        Lp,Lc_=D['leaves'][k][1],D['leaves'][k][2]
                        sel=np.isin(Lc_,[names.index(c) for c in DYN])
                        if sel.any():
                            dd=np.hypot(Lp[sel,0]-g.x,Lp[sel,1]-g.y); best=min(best,dd.min())
                            nmax=max(nmax,int((inbox(Lp[sel],gr,0.3)).sum()))
                    st='3a leaf: none of a dynamic class within 2 m' if best>2 else ('3b leaf: nearby but beside it (0.3-2 m)' if nmax==0 else '3c leaf: 1-2 points on it only')
                elif not own and not fcl: st='4 clustering: no cluster centred on it (merged/split)'
                elif not any(len(x[6])>1 for x in fcl): st='5 YOLO->cluster: cluster without class this tick'
                elif not tr: st='6 tracker: no track on it'
                elif not any(len(x[6])>1 and x[6][1] for x in tr): st='7 tracker: track without class'
                else: st='8 tracker: track with class but state '+ (tr[0][6][0] if tr[0][6] else '?')
                if st.startswith('4'): extra['merged sizes'].append(max([max(x[3],x[4]) for x in (Lc[1] if Lc else []) if inbox(np.array([[x[0],x[1],x[2]]]),gr,1.5).any()] or [0]))
            st=st
        stage[st]+=1; stage_cls[cls][st]+=1
n=sum(stage.values())
for k in sorted(stage): print(f'{k:60s} {100*stage[k]/n:5.1f}%')
for cl,cnt in stage_cls.items():
    m=sum(cnt.values()); print(cl, {k[:2]:'%.0f%%'%(100*v/m) for k,v in sorted(cnt.items())})
if extra['merged sizes']: print('stage 4: size of the cluster around it, median %.1f m'%np.median(extra['merged sizes']))
