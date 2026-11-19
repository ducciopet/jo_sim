import pandas as pd, sys
R=pd.read_csv(sys.argv[1]).sort_values(['win','gid','t_ns'])
G={'Pedestrian':'persone'}
for c in ['Car','Pickup Truck','Utility Vehicle','Service Vehicle','Delivery Truck','Bus']: G[c]='veicoli'
for c in ['Bike','Scooter','Motorcycle']: G[c]='bici/monopattini'
R['grp']=R.cls.map(G).fillna('altro')
R['seen']=R.groupby(['win','gid']).fov.cummax()
R['where']=R.fov.map({True:'in camera',False:'fuori'}); R.loc[~R.fov&R.seen,'where']='fuori, visto prima'; R.loc[~R.fov&~R.seen,'where']='fuori, mai visto'
pd.set_option('display.width',200)
for lab,S in (('D (in moto)',R[R.D]),('PD (fermi)',R[~R.D])):
    print(f'\n##### {lab}: {len(S)} righe')
    print('OK % per gruppo e posizione:'); print(S.assign(ok=S.cause=='OK').pivot_table(index='grp',columns='where',values='ok',aggfunc='mean').mul(100).round(0))
    print(S.pivot_table(index='grp',columns='where',values='ok' if False else 'cause',aggfunc='size'))
    for g,X in S.groupby('grp'):
        t=X.cause.value_counts(normalize=True).mul(100).round(1).head(7)
        print(f'  {g} ({len(X)}):', '; '.join(f'{k} {v}%' for k,v in t.items()))
