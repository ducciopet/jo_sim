#!/usr/bin/env python3
"""Per-stage timing from SINS det.log files: preprocessing 'scan X ms', dbscan 'lidar: ... X ms', fusion
'cameras/tick C, wait W ms' (dbscan_detector periodic summary). Prints the medians per group of runs.
usage: det_timing.py <glob of run _logs dirs>... e.g. 'bags/coda/runs/LFZ0*_logs' 'bags/mdp/runs/mdp_street_fz0_*_logs'
"""
import glob, re, statistics as st, sys

for pat in sys.argv[1:]:
    pre, lid, cams, wait = [], [], [], []
    dirs = sorted(glob.glob(pat))
    for d in dirs:
        try:
            txt = open(d + '/det.log', errors='replace').read()
        except OSError:
            continue
        pre += [float(x) for x in re.findall(r'preprocessing_node\]: lidar point counts:.*\| scan ([0-9.]+) ms', txt)]
        lid += [float(x) for x in re.findall(r'\| lidar: pts=[^|]*?([0-9.]+)ms \|', txt)]
        for c, w in re.findall(r'cameras/tick ([0-9.]+), wait ([0-9.]+)ms', txt):
            cams.append(float(c)); wait.append(float(w))
    med = lambda v: f'{st.median(v):.1f}' if v else '-'
    print(f'{pat}: runs {len(dirs)} | pre {med(pre)} ms | dbscan lidar {med(lid)} ms | wait {med(wait)} ms | '
          f'cameras/tick {med(cams)}  (n pre {len(pre)}, dbscan {len(lid)}, fusion {len(wait)})')
