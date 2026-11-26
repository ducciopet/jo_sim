#!/usr/bin/env python3
"""Choose between a flagged final run and its rerun (FINAL_RUN_SPEC 5, 26/11): the rerun replaces the flagged run unless it is
flagged too; then the better-gated one is kept (gate quiet over loaded, then the lower foreign-CPU median during the run) and marked
loaded in the tables. Writes <orig>/selected.json {use: orig|rerun, loaded: bool, why}. usage: select_run.py <orig dir> <rerun dir>"""
import json, sys
from pathlib import Path
o, r = Path(sys.argv[1]), Path(sys.argv[2])
mo, mr = (json.load(open(d / 'manifest.json')) for d in (o, r))
if not mr['rerun_needed']:
    sel = dict(use='rerun', loaded=False, why='rerun unflagged')
else:
    key = lambda m: (m.get('load_gate') != 'quiet', (m.get('load_during') or {}).get('foreign_cpu_median') or 9999.0)
    use = 'rerun' if key(mr) <= key(mo) else 'orig'
    sel = dict(use=use, loaded=True, why=f"both flagged (orig {mo['flags']}, rerun {mr['flags']}): better-gated kept")
json.dump(sel, open(o / 'selected.json', 'w'), indent=1); print(o.name, sel)
