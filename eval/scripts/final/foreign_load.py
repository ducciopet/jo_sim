#!/usr/bin/env python3
"""CPU and GPU used OUTSIDE the jo_sim_sins container (FINAL_RUN_SPEC 5, gate of 26/11): CPU% from /proc utime+stime deltas over
--dt seconds, summed over every process whose cgroup is not the container's scope (100 = one core); GPU SM% from nvidia-smi pmon
(one sample), summed over processes outside the container. Prints "<foreign_cpu_pct> <foreign_gpu_pct>".
usage: foreign_load.py [--dt 5] [--container jo_sim_sins]"""
import argparse, os, subprocess, time
ap = argparse.ArgumentParser(); ap.add_argument('--dt', type=float, default=5.0); ap.add_argument('--container', default='jo_sim_sins')
a = ap.parse_args()
cid = subprocess.run(['docker', 'inspect', '-f', '{{.Id}}', a.container], capture_output=True, text=True).stdout.strip()
HZ = os.sysconf('SC_CLK_TCK')
def inside(pid):
    try: return bool(cid) and cid in open(f'/proc/{pid}/cgroup').read()
    except OSError: return True     # gone: ignore
def ticks():
    out = {}
    for p in os.listdir('/proc'):
        if not p.isdigit() or inside(p): continue
        try:
            f = open(f'/proc/{p}/stat').read().rsplit(')', 1)[1].split()
            out[p] = int(f[11]) + int(f[12])          # utime + stime
        except (OSError, IndexError, ValueError): pass
    return out
t0, c0 = time.time(), ticks(); time.sleep(a.dt); t1, c1 = time.time(), ticks()
cpu = 100.0 * sum(max(0, c1[p] - c0[p]) for p in c1 if p in c0) / HZ / (t1 - t0)
gpu = 0.0
try:
    for l in subprocess.run(['nvidia-smi', 'pmon', '-c', '1', '-s', 'u'], capture_output=True, text=True, timeout=15).stdout.splitlines():
        f = l.split()
        if l.startswith('#') or len(f) < 4 or not f[1].isdigit() or inside(f[1]): continue
        gpu += float(f[3]) if f[3] not in ('-',) else 0.0
except (OSError, subprocess.TimeoutExpired): gpu = -1.0
print(f'{cpu:.0f} {gpu:.0f}')
