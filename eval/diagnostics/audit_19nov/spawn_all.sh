#!/bin/bash
# usage: spawn_all.sh TAG win...  -> spawn reasons for vehicles and bikes (all windows pooled)
S=/tmp/claude-1000/-home-duccio-Desktop-jo-sim-sins/6b5f28fe-4bd1-41ab-95fb-b1ee18ac2c62/scratchpad; B=/home/duccio/Desktop/jo_sim_sins/bags/coda
TAG=$1; shift
for grp in veicoli bici persone; do
  > $S/spawn_$grp.log; python3 - "$grp" "$@" <<'P'
import sys,pandas as pd
grp=sys.argv[1]; B='/home/duccio/Desktop/jo_sim_sins/bags/coda/'
cls={'veicoli':['Car','Pickup Truck','Utility Vehicle','Service Vehicle','Delivery Truck','Bus'],'bici':['Bike','Scooter','Motorcycle'],'persone':['Pedestrian']}[grp]
for w in sys.argv[2:]:
    c=w if '_' in w else '0_'+w
    g=pd.read_csv(B+f'coda{c}_gt.csv'); g=g[g['class'].isin(cls)]; g.to_csv(f'/tmp/claude-1000/-home-duccio-Desktop-jo-sim-sins/6b5f28fe-4bd1-41ab-95fb-b1ee18ac2c62/scratchpad/gt_{grp}_{c}.csv',index=False)
P
done
