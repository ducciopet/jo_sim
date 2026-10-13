import sys, collections
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
ts=get_typestore(Stores.ROS2_JAZZY)
add={}
for f in Path('/home/duccio/Desktop/jo_sim_lv-dot/jo_msgs/msg').glob('*.msg'):
    add.update(get_types_from_msg(f.read_text(), f'jo_msgs/msg/{f.stem}'))
ts.register(add)
for b in sys.argv[1:]:
    st=collections.Counter(); n=0; ne=0; ids=set(); t=[]
    with AnyReader([Path(b)], default_typestore=ts) as r:
        conns=[x for x in r.connections if x.topic.endswith('tracked_dynamic_obstacles') and 'Obstacle' in x.msgtype]
        for c,tt,raw in (r.messages(conns) if conns else []):
            m=r.deserialize(raw,c.msgtype); n+=1; t.append(tt)
            if m.obstacles: ne+=1
            for o in m.obstacles: st[o.status]+=1; ids.add(o.track_id)
    hz = (len(t)-1)/((t[-1]-t[0])/1e9) if len(t)>1 else 0
    print(b, 'msgs',n,f'({hz:.1f} Hz)','non-empty',ne,'unique ids',len(ids),'status',{ {1:'D',2:'PD',0:'S',3:'T'}[k]:v for k,v in st.items()})
