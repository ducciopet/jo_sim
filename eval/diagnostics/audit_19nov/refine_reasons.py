import re,glob,sys,collections
pat=re.compile(r"yolo_refine: (ACCEPTED|no candidate at all|skipped ambiguous match|skipped distant match|skipped too low for its class|[a-z ]+?)[ (].*?class=(\w+)")
C=collections.Counter(); other=collections.Counter()
for f in glob.glob(sys.argv[1]):
    for l in open(f,errors='ignore'):
        if 'yolo_refine:' not in l: continue
        m=pat.search(l)
        if m: C[(m[2] if m[2] in ('person','car','truck','bus','bicycle','motorbike') else 'other', m[1].strip())]+=1
        else: other[l.split('yolo_refine:')[1][:60]]+=1
for cls in ['person','car','truck','bus','bicycle','motorbike','other']:
    tot=sum(v for (c,_),v in C.items() if c==cls)
    if tot: print(cls, tot, {r:f'{100*v/tot:.0f}%' for (c,r),v in sorted(C.items(),key=lambda x:-x[1]) if c==cls})
print('unparsed',sum(other.values()), other.most_common(5))
