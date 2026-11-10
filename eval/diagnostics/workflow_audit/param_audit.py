"""Parameters in the YAML profiles vs parameters the nodes declare: a key the code never declares is silently ignored
(rclcpp/rclpy only see declared names unless undeclared overrides are allowed). Also: declared but set nowhere (always
the code default), and per-camera sub-keys. usage: param_audit.py <cfg dir> [...]"""
import re, sys, yaml
from pathlib import Path
SRC = Path('/home/duccio/Desktop/jo_sim_sins/sins/src')
SCR = Path('/home/duccio/Desktop/jo_sim_sins/sins/scripts')
LAUNCH = Path('/home/duccio/Desktop/jo_sim_sins/sins/launch')


def declared(text):
    names = set(re.findall(r'declare_parameter[^(\n]*\(\s*"([^"]+)"', text))
    names |= set(re.findall(r"declare_parameter\(\s*'([^']+)'", text))
    names |= set(re.findall(r'declare_parameters?\(\s*["\']([^"\']*)["\']', text))
    return names


def camera_keys(text):
    # per-camera keys read through a helper p("key", default) / get(cam + ".key")
    k = set(re.findall(r'\bp\(\s*"([a-z0-9_]+)"', text))
    k |= set(re.findall(r'name \+ "\.([a-z0-9_]+)"', text))
    return k


code = {}
for f in list(SRC.glob('*.cpp')) + list(SRC.glob('*.py')) + list(SCR.glob('*.py')):
    code[f.name] = f.read_text()
node_src = {}
for fn, t in code.items():
    for n in re.findall(r'(?:Node\(|super\(\)\.__init__\()\s*"?\'?([a-z_]+)', t):
        node_src.setdefault(n, set()).add(fn)
for d in sys.argv[1:]:
    print(f'\n######## {d}')
    for y in sorted(Path(d).glob('*.yaml')):
        try:
            Y = yaml.safe_load(y.read_text()) or {}
        except Exception as e:
            print(y.name, 'YAML ERROR', e); continue
        for node, body in Y.items():
            if not isinstance(body, dict) or 'ros__parameters' not in body:
                continue
            P = body['ros__parameters'] or {}
            srcs = node_src.get(node.strip('/'), set())
            if not srcs:
                print(f'  {y.name}: node "{node}" -> no source declares a node with this name ({len(P)} params ignored?)'); continue
            txt = '\n'.join(code[s] for s in srcs)
            dec = declared(txt); cam = camera_keys(txt)
            unknown = []
            for k, v in P.items():
                if k in dec:
                    continue
                if isinstance(v, dict):
                    bad = [kk for kk in v if kk not in cam and kk not in dec]
                    if bad:
                        unknown.append(f'{k}.{{{",".join(bad)}}}')
                    continue
                unknown.append(k)
            unset = sorted(dec - set(P))
            print(f'  {y.name} [{node} <- {",".join(sorted(srcs))}]: {len(P)} keys, NOT DECLARED: {unknown or "-"}')
            if unset and d.endswith('outdoor'):
                print(f'      declared, not in the profile (code default): {len(unset)}: {unset}')
