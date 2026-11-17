"""e3 table (copy of agg_e2.py on the e3 dirs): CODa (coda_e2) moving MOTA per seq [range over reps], MOTA all, PD found/prec, moving-vehicle recall
(coda_vehicle_e2, tuning only), IDSW; street (mdp_e2) MOTA tot, vehicle/person recall, person precision; latency.
usage: agg_e2.py <tuning tags,> <heldout tags,> <street profiles,>"""
import json, statistics as st, re, os, sys
def J(f): return json.load(open(f))['metrics']
def pooled(fs, key='dynamic_only'):
    t={k:sum(J(f)[key][k] for f in fs) for k in ('tp','fp','fn','idsw','num_gt')}
    return 100*t['tp']/max(1,t['num_gt']), 100*t['tp']/max(1,t['tp']+t['fp']), 100*(1-(t['fn']+t['fp']+t['idsw'])/max(1,t['num_gt'])), t['idsw']
def pdm(fs):
    ok=found=pred=0
    for f in fs:
        m=J(f)['full']; c=m['confusion']
        ok+=c['potentially_dynamic']['potentially_dynamic']; found+=sum(c['potentially_dynamic'].values())
        pred+=sum(c[g]['potentially_dynamic'] for g in c)+m['fp_by_state'].get('potentially_dynamic',0)
    return 100*ok/max(1,found), 100*ok/max(1,pred)
T={'0':['w1','w2','w3'],'10':['10_w%d'%i for i in range(1,6)],'16':['16_w1']}
H={'11':['11_w1','11_w2','11_w3'],'12':['12_w1','12_w2','12_w3']}
VW=['0_w1','10_w1','10_w3','10_w4','10_w5','16_w1']
F=lambda p,x,w: f'eval/results/coda_e3/{p}{x}_{w}.json'
def row(p,S,veh):
    reps=[x for x in ['','r2','r3'] if all(os.path.exists(F(p,x,w)) for w in sum(S.values(),[]))]
    if not reps: return
    mv,al=[],[]
    for s,W in S.items():
        r=[pooled([F(p,x,w) for w in W]) for x in reps]; a=[pooled([F(p,x,w) for w in W],'full')[2] for x in reps]
        mv.append(f"{st.mean(v[2] for v in r):.1f}[{min(v[2] for v in r):.1f}-{max(v[2] for v in r):.1f}]"); al.append(f"{st.mean(a):.1f}")
    q=[pdm([F(p,x,w) for w in sum(S.values(),[])]) for x in reps]
    ids=st.mean(pooled([F(p,x,w) for w in sum(S.values(),[])],'full')[3] for x in reps)
    vr=''
    if veh:
        v=[]
        for x in reps:
            fs=[f'eval/results/coda_vehicle_e3/{p}{x}_{w}.json' for w in VW]
            if all(os.path.exists(f) for f in fs): v.append(pooled(fs)[0])
        vr=f" | veic {st.mean(v):.1f}[{min(v):.1f}-{max(v):.1f}]" if v else ''
    print(f"{p:7s} n={len(reps)} | moving {'/'.join(mv)} | tutti {'/'.join(al)} | PD {st.mean(v[0] for v in q):.1f}/{st.mean(v[1] for v in q):.1f}{vr} | IDSW {ids:.0f}")
a1=sys.argv[1].split(',') if sys.argv[1] else []; a2=sys.argv[2].split(',') if len(sys.argv)>2 and sys.argv[2] else []; a3=sys.argv[3].split(',') if len(sys.argv)>3 and sys.argv[3] else []
for p in a1: row(p,T,True)
for p in a2: row(p,H,False)
for p in a3:
    G=lambda r,s='': f'eval/results/mdp_e3/{p}_{r}{s}.json'
    rr=[r for r in ['r1','r2','r3'] if os.path.exists(G(r))]
    if not rr: continue
    a=[pooled([G(r)]) for r in rr]; v=[pooled([G(r,'_vehicle')]) for r in rr]; pe=[pooled([G(r,'_person')]) for r in rr]
    m=lambda L,i: st.mean(x[i] for x in L)
    print(f"street {p:20s} n={len(rr)} MOTA {m(a,2):.1f}[{min(x[2] for x in a):.1f}-{max(x[2] for x in a):.1f}] | veic rec {m(v,0):.1f}[{min(x[0] for x in v):.1f}-{max(x[0] for x in v):.1f}] prec {m(v,1):.1f} | pers rec {m(pe,0):.1f} prec {m(pe,1):.1f} MOTA {m(pe,2):.1f}")
L={}
for line in open('eval/results/lab_latency.txt'):
    mm=re.match(r'(\S+) (\S+) tracce (\d+)\s+latenza mediana (-?\d+) ms\s+p90 (\d+)',line)
    if not mm: continue
    key=re.sub(r'r[23]$','',mm[1]) if not mm[1].startswith('mdp') else mm[1]
    L.setdefault(key,[]).append((int(mm[4]),int(mm[5])))
for k in a1+a2+a3:
    v=L.get(k,[])
    if v: print(f"lat {k:20s} med {st.median(x[0] for x in v):.0f} p90 {st.median(x[1] for x in v):.0f} ms (n={len(v)})")
