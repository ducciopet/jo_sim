#!/usr/bin/env python3
"""Per-run manifest.json for the SINS final runs (FINAL_RUN_SPEC 5). SKELETON, prepared 25/11, used from the freeze (05/12).
Fields: sins + jo_sim commits, profile name + md5 of its generated cfg folder, YOLO weights md5, ultralytics/torch/CUDA versions in the container, host, load average (1 min) at
start and end, GPU, input scans vs outputs per node (dropped %), replay rate, date, rerun flag.
usage: make_manifest.py --run <run dir> --input <input bag> --profile <name> --load-start <x> [--rate 1.0] [--rerun-of <dir>]"""
import argparse, datetime, hashlib, json, os, socket, subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
ap = argparse.ArgumentParser()
ap.add_argument('--run', required=True); ap.add_argument('--input', required=True); ap.add_argument('--profile', required=True)
ap.add_argument('--load-start', type=float, required=True); ap.add_argument('--rate', type=float, default=1.0)
ap.add_argument('--rerun-of', default=None)
ap.add_argument('--gate', default='quiet', help='load gate outcome: quiet | loaded (deadline passed)')
ap.add_argument('--load-log', default=None, help='file of "<epoch> <load1> <load5>" samples taken every 30 s during the run')
ap.add_argument('--quiet-p90', type=float, default=0.0, help='dataset quiet latency p90 [ms]; 0 = not set')
a = ap.parse_args()
git = lambda d: subprocess.run(['git', '-C', str(d), 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True).stdout.strip()
def md5_tree(d):
    h = hashlib.md5()
    for f in sorted(Path(d).rglob('*')):
        if f.is_file(): h.update(f.relative_to(d).as_posix().encode()); h.update(f.read_bytes())
    return h.hexdigest()
md5 = lambda f: hashlib.md5(Path(f).read_bytes()).hexdigest()
def msgcounts(bag):
    import yaml
    info = yaml.safe_load(open(Path(bag) / 'metadata.yaml'))['rosbag2_bagfile_information']
    return {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
inp, out = msgcounts(a.input), msgcounts(a.run)
scans = max(v for k, v in inp.items() if 'points' in k or 'velodyne' in k or 'lidar' in k)   # input LiDAR scans
nodes = {k: out.get(k, 0) for k in ('/sins/dbscan_detector/fused/detections3d', '/sins/tracker/tracks', '/perception/tracked_dynamic_obstacles')}
def container_versions():   # YOLO runtime inside the SINS container (peer request 25/11)
    r = subprocess.run(['docker', 'exec', 'jo_sim_sins', 'python3', '-c',
                        'import ultralytics, torch; print(ultralytics.__version__, torch.__version__, torch.version.cuda)'],
                       capture_output=True, text=True).stdout.split()
    return dict(zip(['ultralytics', 'torch', 'cuda'], r)) if len(r) == 3 else {}
gpu = subprocess.run(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'], capture_output=True, text=True).stdout.strip()
m = dict(date=datetime.datetime.now().isoformat(timespec='seconds'), host=socket.gethostname(), gpu=gpu,
         sins_commit=git(ROOT / 'sins'), jo_sim_commit=git(ROOT), profile=a.profile,
         profile_md5=md5_tree(ROOT / 'sins/cfg' / a.profile), yolo_weights_md5={f.name: md5(f) for f in sorted((ROOT / 'sins/weights').glob('*.pt'))},
         load1_start=a.load_start, load1_end=os.getloadavg()[0], replay_rate=a.rate, input_scans=scans,
         outputs=nodes, dropped_pct={k: round(100.0 * (1 - v / scans), 2) if scans else None for k, v in nodes.items()},
         rerun_of=a.rerun_of, container=container_versions())
import statistics
L = [list(map(float, l.split()[1:3])) for l in open(a.load_log)] if a.load_log and Path(a.load_log).exists() else []
m['load_gate'] = a.gate
m['load_during'] = dict(n=len(L), load1_median=statistics.median(x[0] for x in L) if L else None, load1_max=max((x[0] for x in L), default=None),
                        load5_median=statistics.median(x[1] for x in L) if L else None, load5_max=max((x[1] for x in L), default=None))
lat = subprocess.run(['docker', 'exec', 'jo_sim_sins', 'bash', '-lc', 'source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash; '
                      f'python3 /home/ros/bags/replay/lat_tracks.py /home/ros/{Path(a.run).resolve().relative_to(ROOT)}'],
                     capture_output=True, text=True).stdout.strip().splitlines()
import re
mm = re.search(r'mediana (\d+) ms\s+p90 (\d+) ms', lat[-1]) if lat else None
m['latency_ms'] = dict(median=int(mm.group(1)), p90=int(mm.group(2))) if mm else None
m['flags'] = [f for f, on in [('dropped>2%', any(v is not None and v > 2.0 for v in m['dropped_pct'].values())),
                              ('loaded', a.gate == 'loaded'),
                              ('p90>1.5x quiet', bool(mm and a.quiet_p90 > 0 and int(mm.group(2)) > 1.5 * a.quiet_p90))] if on]
m['rerun_needed'] = bool(m['flags'])
json.dump(m, open(Path(a.run) / 'manifest.json', 'w'), indent=1)
print(json.dumps({k: m[k] for k in ('profile', 'input_scans', 'dropped_pct', 'rerun_needed')}))
