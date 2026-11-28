import json, sys, numpy as np
from pathlib import Path
from math import comb
import os
N = Path(os.environ.get('NO', '/mnt/nas-imperatore/datasets_SINS/derived/sins_final/dev/rq')); R = Path(os.environ.get('RES', 'eval/results/dev/rq'))
WS = ['w1','w2','w3','10_w1','10_w2','10_w3','10_w4','10_w5','16_w1']; SEQ = {'w1':'0','w2':'0','w3':'0'}
ARMS = sys.argv[1:] or ['MF','RQ']
def sel(arm, w, k):
    d = N/arm/f'{w}_r{k}'; s = d/'selected.json'
    use = json.load(open(s)).get('use','orig') if s.exists() else 'orig'
    return use == 'rerun', (N/arm/f'{w}_r{k}_rerun' if use=='rerun' else d)
def tag(arm,k): return arm if k==1 else f'{arm}r{k}'
def js(arm, sub, w, k):
    rr,_ = sel(arm,w,k); return json.load(open(R/arm/sub/f'{tag(arm,k)}_{w}{"_rerun" if rr else ""}.json'))['metrics']['full']
def err(m): return m['fp']+m['fn']+m['idsw']
ref, arm = ARMS
for sub in ['e3','e3/test','e3ib','e3ib/test','pdprev','pdprev/test','state']:
    try:
        win=loss=tie=0; per=[]
        for w in WS:
            s=''
            for k in (1,2,3):
                d = err(js(arm,sub,w,k))-err(js(ref,sub,w,k)); win+=d<0; loss+=d>0; tie+=d==0; s+=f'{d:+d} '
            per.append(f'{w}:{s.strip()}')
        n=win+loss+tie; p=sum(comb(n,i) for i in range(win,n+1))/2**n
        # pooled MOTA per arm (sum over windows, mean over reps) and per seq
        def pooled(a, ws):
            out=[]
            for k in (1,2,3):
                M=[js(a,sub,w,k) for w in ws]; g=sum(m['num_gt'] for m in M)
                out.append(100*(1-sum(err(m) for m in M)/g) if g else float('nan'))
            return out
        pr, pa = pooled(ref,WS), pooled(arm,WS)
        line=f'{sub:12s} wins {win}/{n} (L {loss}, T {tie}) p {p:.3f} {"ADOPT" if win>=19 else "no"} | pooled MOTA {ref} {np.mean(pr):.1f} {arm} {np.mean(pa):.1f} ({np.mean(pa)-np.mean(pr):+.1f})'
        vs=[]
        for sq, ws in [('0',['w1','w2','w3']),('10',[w for w in WS if w.startswith('10_')]),('16',['16_w1'])]:
            a, b = pooled(ref,ws), pooled(arm,ws)
            veto = np.mean(b) < min(a)
            vs.append(f'seq{sq} {np.mean(a):.1f}->{np.mean(b):.1f}{" VETO" if veto else ""}')
        print(line+' | '+', '.join(vs))
        if sub in ('e3','e3/test'): print('   delta err per window (r1 r2 r3): '+' | '.join(per))
    except FileNotFoundError as e: print(sub, 'missing', e)
# latency/drops from manifests
for a in ARMS:
    L=[];P=[];D=[];rr=0
    for w in WS:
        for k in (1,2,3):
            r,d = sel(a,w,k); rr+=r; m=json.load(open(d/'manifest.json'))
            L.append(m); 
    print(a, 'reruns used', rr, 'manifest keys', list(L[0])[:30])
