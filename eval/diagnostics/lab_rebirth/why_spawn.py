"""For each handover A->B on a moving GT object (rebirth.py on the LC_trk CSVs), the lifecycle spawn line of B:
why did A (the object's previous track) not take B's first detection?  usage: why_spawn.py handovers.csv"""
import sys, re, glob, numpy as np, pandas as pd
R = '/home/punk-opc/Desktop/jo_sim_sins/'
H = pd.read_csv(sys.argv[1])
H = H[H.kind.isin(['overlap', 'gap'])].copy()
rx = re.compile(r'spawn #(\d+) t=([\d.]+) det_class=\'([^\']*)\' size=([\d.]+)x([\d.]+) at \(([-\d.]+),([-\d.]+)\)(.*)')
ent = re.compile(r'#(\d+) hits=(\d+) conf=(\d) spd=([-\d.na]+) d=([\d.]+) cls=\'([^\']*)\' size=([\d.]+)x([\d.]+) base=\S+ why=(\w+)')
spawn = {}
for w in H.name.unique():
    log = f'{R}bags/coda/runs/LC_v2_coda{w}_logs/det.log' if w != 'street' else f'{R}bags/mdp/runs/mdp_street_vjsye2lc_r1_logs/det.log'
    S = {}
    for line in open(log, errors='ignore'):
        m = rx.search(line)
        if not m: continue
        S[int(m.group(1))] = (m.group(3), float(m.group(4)), float(m.group(5)),
                              {int(e[0]): e for e in ent.findall(m.group(8))})
    spawn[w] = S
out = []
for _, h in H.iterrows():
    S = spawn[h['name']]; b = int(h.b); a = int(h.a)
    if b not in S: out.append(dict(kind=h.kind, cls=h.cls, why='no_spawn_line')); continue
    dc, sx, sy, near = S[b]
    if a in near:
        e = near[a]; out.append(dict(kind=h.kind, cls=h.cls, a_missed=h.a_missed, a_spd=h.a_spd, v=h.v, why=e[8], d=float(e[4]), a_hits=int(e[1]), a_conf=int(e[2]),
                                     a_cls=e[5], a_size=max(float(e[6]), float(e[7])), b_size=max(sx, sy), b_cls=dc))
    else:
        out.append(dict(kind=h.kind, cls=h.cls, why='A not within 4 m / dead', b_size=max(sx, sy), b_cls=dc))
O = pd.DataFrame(out); O['cls2'] = np.where(O.cls.str.lower().isin(['pedestrian', 'person']), 'ped', 'veh/other')
print(pd.crosstab([O.kind, O.why], O.cls2, margins=True).to_string())
for why in ['matched_other', 'size', 'score', 'veldir', 'height', 'speed', 'dist', 'candidate_unassigned']:
    s = O[O.why == why]
    if len(s): print(why, len(s), 'd med %.2f | A size med %.2f B size med %.2f | A conf %.0f%% | B cls %s' % (
        s.d.median(), s.a_size.median(), s.b_size.median(), 100 * s.a_conf.mean(), s.b_cls.value_counts().head(3).to_dict()))
for why in ['speed','veldir','height','yolo_base_growth','matched_other']:
    s=O[O.why==why]
    if len(s): print('##',why,len(s),'A missed: 0 %.0f%% 1-2 %.0f%% >=3 %.0f%% | A KF speed med %.2f | GT v med %.2f'%(100*(s.a_missed==0).mean(),100*s.a_missed.between(1,2).mean(),100*(s.a_missed>=3).mean(),s.a_spd.median(),s.v.median()))
