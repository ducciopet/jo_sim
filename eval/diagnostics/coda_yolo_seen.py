import sys, numpy as np, pandas as pd, collections
sys.path.insert(0,'eval')
from rosbags.highlevel import AnyReader; from pathlib import Path
from harness.ros_io import load_obstacles, load_odometry
from harness.tracks import objects
import coda_eval as ce, types
a=types.SimpleNamespace(min_pts=1,pd_everywhere=False,range=25.,min_range=1.2,dyn_speed=0.3,ignore_dist=1.,max_height=2.5)
DYN={'person','bicycle','car','motorbike','bus','truck','dog','cat','horse'}
def quat(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def T_of(tr):
    T=np.eye(4); T[:3,:3]=quat(tr.rotation); T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]; return T
def statics_K(bag,cam):
    st={}; K=None; WH=None
    with AnyReader([Path(bag)]) as r:
        for c,_,b in r.messages([c for c in r.connections if c.topic in ('/tf_static',f'/{cam}/camera/color/camera_info')]):
            m=r.deserialize(b,c.msgtype)
            if c.topic=='/tf_static':
                for t in m.transforms: st[(t.header.frame_id,t.child_frame_id)]=T_of(t.transform)
            elif K is None: K=np.array(m.k).reshape(3,3); WH=(m.width,m.height)
    return st,K,WH
def full_odom(bag):
    t=[];T=[]
    with AnyReader([Path(bag)]) as r:
        for c,_,b in r.messages([c for c in r.connections if c.topic=='/odometry/filtered']):
            m=r.deserialize(b,c.msgtype); p=m.pose.pose
            M=np.eye(4); M[:3,:3]=quat(p.orientation); M[:3,3]=[p.position.x,p.position.y,p.position.z]
            t.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec); T.append(M)
    return np.array(t),np.array(T)
res=collections.Counter(); bycls=collections.defaultdict(collections.Counter)
for w in ['w1','w2','w3']:
    odom=load_odometry(f'bags/coda/coda0_{w}_odom'); ot,oT=full_odom(f'bags/coda/coda0_{w}_odom')
    gt,ign,frames,_=ce.build_gt(f'bags/coda/coda0_{w}_gt.csv',odom,a)
    raw=pd.read_csv(f'bags/coda/coda0_{w}_gt.csv'); raw=raw[raw.track_id>=0]
    G=objects(gt); G=G[G.state!='dynamic']
    stf,Kf,WHf=statics_K(f'bags/coda/coda0_{w}','front_camera'); sts,Ks,WHs=statics_K(f'bags/coda/coda0_{w}_stereo','stereo_camera')
    Tbv=stf[('base_link','velodyne')]
    cams={'front_camera':(Tbv@stf[('velodyne','front_camera_depth_optical_frame')]@stf[('front_camera_depth_optical_frame','front_camera_color_optical_frame')],Kf,WHf),
          'stereo_camera':(Tbv@sts[('velodyne','stereo_camera_depth_optical_frame')]@sts[('stereo_camera_depth_optical_frame','stereo_camera_color_optical_frame')],Ks,WHs)}
    for tag in ['G1','G1r2','G1r3']:
        Y={}
        with AnyReader([Path(f'bags/coda/runs/{tag}_v2_coda0_{w}')]) as r:
            for c,_,b in r.messages([c for c in r.connections if c.topic.startswith('/yolo_semantic_seg/') and c.topic.endswith('/detections')]):
                m=r.deserialize(b,c.msgtype); cam=c.topic.split('/')[2]
                Y.setdefault(cam,[]).append((m.header.stamp.sec*10**9+m.header.stamp.nanosec,[(d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size_x,d.bbox.size_y,d.results[0].hypothesis.class_id) for d in m.detections if d.results]))
        for cam in Y: Y[cam].sort(key=lambda x:x[0])
        pub=objects(load_obstacles(f'bags/coda/runs/{tag}_v2_coda0_{w}')[0]); pt=np.unique(pub.t_ns)
        for _,g in G.iterrows():
            gr=raw[(raw.t_ns==g.t_ns)&(raw.track_id==g.track_id)]
            if not len(gr): continue
            gr=gr.iloc[0]; cls=gr['class']
            Todom_base=oT[np.argmin(abs(ot-g.t_ns))]
            seen=False; infov=False
            for cam,(Tbc,K,WH) in cams.items():
                Tco=np.linalg.inv(Todom_base@Tbc)
                # box corners -> image rect
                c,s=np.cos(gr.yaw),np.sin(gr.yaw)
                cr=[]
                for dx in (-.5,.5):
                    for dy in (-.5,.5):
                        for dz in (-.5,.5):
                            P=[gr.x+c*dx*gr.l-s*dy*gr.w, gr.y+s*dx*gr.l+c*dy*gr.w, gr.z+dz*gr.h,1]
                            q=Tco@P; cr.append(q[:3])
                cr=np.array(cr)
                if (cr[:,2]<0.5).any(): continue
                uv=(K@cr.T).T; uv=uv[:,:2]/uv[:,2:]
                x0,y0=uv.min(0); x1,y1=uv.max(0)
                if x1<0 or y1<0 or x0>WH[0] or y0>WH[1] or not (0<= (x0+x1)/2 <WH[0] and 0<=(y0+y1)/2<WH[1]): continue
                infov=True
                L=Y.get(cam,[])
                if not L: continue
                ts=np.array([l[0] for l in L]); k=np.argmin(abs(ts-g.t_ns))
                if abs(ts[k]-g.t_ns)>1.5e8: continue
                for (cx,cy,sx,sy,cl) in L[k][1]:
                    if cl not in DYN: continue
                    ix=max(0,min(x1,cx+sx/2)-max(x0,cx-sx/2)); iy=max(0,min(y1,cy+sy/2)-max(y0,cy-sy/2))
                    inter=ix*iy; uni=(x1-x0)*(y1-y0)+sx*sy-inter
                    if uni>0 and inter/uni>=0.3: seen=True
            if not infov: continue
            q=pub[pub.t_ns==pt[np.argmin(abs(pt-g.t_ns))]]; d=np.hypot(q.x-g.x,q.y-g.y)
            pdok=len(q)>0 and d.min()<1 and q.loc[d.idxmin(),'state']=='potentially_dynamic'
            key=('YOLO sees it' if seen else 'YOLO misses it', 'PD' if pdok else 'not PD')
            res[key]+=1; bycls[cls][key]+=1
n=sum(res.values())
for k in sorted(res): print(k, '%.0f%%'%(100*res[k]/n))
for cl,cnt in bycls.items():
    m=sum(cnt.values()); print(cl, {f'{a[:9]}/{b}':'%.0f%%'%(100*v/m) for (a,b),v in sorted(cnt.items())})
