"""CODa moving / merged MOTA per sequence through the coda_eval.py pipeline, twice: as scored (base) and without the
GT boxes ignored for being closer than --min-range (the operator walking ~0.9 m behind the robot) in the ignore set.
Shows how much the 1 m ignore radius around the operator removes predictions of people walking next to the robot.
Diagnostic only: coda_eval.py itself is unchanged. usage (repo root): python3 eval/diagnostics/lab_fp/eval_noopign.py LVE2 0_w1 ... 16_w1
"""
import sys, argparse
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np, coda_eval as ce
from pathlib import Path
from harness.ros_io import load_odometry, load_obstacles
from harness.metrics import EvalConfig, evaluate
R=str(Path(__file__).resolve().parents[3])+'/'
a=argparse.Namespace(range=25.0,min_range=1.2,ignore_dist=1.0,max_height=2.5,min_pts=1,pd_everywhere=False,camera_only=False,cam_json=None,dyn_speed=0.3,extra_ignore=None)
tag=sys.argv[1]; wins=sys.argv[2:]
acc={}
for w in wins:
  for rep in ('','r2','r3'):
    run=f'{R}bags/coda/runs/{tag}{rep}_v2_coda{w}'
    if not Path(run,'metadata.yaml').exists(): continue
    odom=load_odometry(f'{R}bags/coda/coda{w}_odom'); oz=ce.odom_ground_z(f'{R}bags/coda/coda{w}_odom')
    gt,ign,frames,_=ce.build_gt(f'{R}bags/coda/coda{w}_gt.csv',odom,a)
    raw,_,_=load_obstacles(run,None)
    for mode,ig in (('base',ign),('no_op_ign',ign[ce.robot_dist(ign,odom)>=1.2])):
      pred,_,_=ce.filter_pred(raw,ig,frames,odom,oz,a)
      for lm in ('dynamic_only','merged'):
        r=evaluate(gt,pred,EvalConfig(match_dist=1.0,label_mode=lm,skip_missing_gt=False))
        k=(w.split('_')[0],mode,lm); s=acc.setdefault(k,[0,0,0,0,0])
        s[0]+=r.tp; s[1]+=r.fp; s[2]+=r.fn; s[3]+=r.idsw; s[4]+=r.num_gt
for k in sorted(acc):
    tp,fp,fn,ids,ng=acc[k]; print(k, 'MOTA %.1f  rec %.1f  prec %.1f  FN %d FP %d'%(100*(1-(fp+fn+ids)/max(1,ng)),100*tp/max(1,tp+fn),100*tp/max(1,tp+fp),fn,fp))
