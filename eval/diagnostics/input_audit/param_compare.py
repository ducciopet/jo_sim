#!/usr/bin/env python3
"""Input audit (17/11): effective parameters (param_dump.sh output) vs the profile YAML (sins/cfg/<env>).
For every node of the profile's YAML files, the dumped node(s) whose name starts with that node's name: every YAML
parameter whose effective value differs is printed (launch overrides show up here), plus the key settings.
usage: param_compare.py <dump dir> <env>"""
import sys, yaml
from pathlib import Path

D, env = Path(sys.argv[1]), sys.argv[2]
C = Path('sins/cfg') / env

def flat(d, p=''):
    out = {}
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            out.update(flat(v, f'{p}{k}.'))
        else:
            out[p + k] = v
    return out

dumps = {}
for f in D.glob('*.yaml'):
    y = yaml.safe_load(open(f)) or {}
    for node, body in y.items():
        dumps[node.strip('/')] = flat((body or {}).get('ros__parameters', {}))
print(f'{env}: {len(dumps)} nodes dumped:', ' '.join(sorted(dumps)))
KEY = ('weights_path', 'target_classes', 'inference_size', 'detection_inference_size', 'camera_names', 'class_score_thresholds',
       'conf_threshold', 'confidence_threshold', 'ground_model', 'floor_source', 'yolo_leaf_method', 'enable_detection',
       'track_class_anchor_carrier', 'track_point_shift_veto_unclassified_only', 'track_class_anchor_ttl', 'lidar_range_x',
       'track_classified_corroboration_all', 'track_inherit_motion_probation')
nd = 0
for f in sorted(C.glob('*.yaml')):
    y = yaml.safe_load(open(f)) or {}
    for node, body in y.items():
        exp = flat((body or {}).get('ros__parameters', {}))
        got = [n for n in dumps if n == node or n.startswith(node + '_') or n.startswith(node)]
        if not got:
            print(f'  {f.name}:{node}: NOT RUNNING (no dumped node)')
            continue
        for n in got:
            act = dumps[n]
            for k, v in exp.items():
                a = act.get(k, '<not declared>')
                same = a == v or (isinstance(v, (int, float)) and isinstance(a, (int, float)) and abs(a - v) < 1e-9) \
                    or (isinstance(v, list) and isinstance(a, list) and [str(x) for x in a] == [str(x) for x in v])
                if not same and a != '<not declared>':
                    nd += 1; print(f'  DIFF {n}.{k}: yaml {v!r} effective {a!r}')
            keys = {k: act[k] for k in KEY if k in act}
            if keys:
                print(f'  {n}: ' + ', '.join(f'{k}={v}' for k, v in keys.items()))
print(f'{env}: {nd} effective values differ from the YAML')
