#!/usr/bin/env python3
"""
Parallel HTTP range download (the CODa server gives ~1 MB/s per connection; 16 connections give several times that).
Resumable: finished blocks are listed in <out>.parts; rerun with the same arguments to continue.
usage: pget.py <url> <out> [connections 16] [block MB 64]
"""
import os
import sys
import threading
import urllib.request

url, out = sys.argv[1], sys.argv[2]
n = int(sys.argv[3]) if len(sys.argv) > 3 else 16
block = (int(sys.argv[4]) if len(sys.argv) > 4 else 64) << 20
size = int(urllib.request.urlopen(urllib.request.Request(url, method='HEAD'), timeout=60).headers['Content-Length'])
parts_file = out + '.parts'
done = set(int(x) for x in open(parts_file).read().split()) if os.path.exists(parts_file) else set()
if not os.path.exists(out):
    with open(out, 'wb') as f:
        f.truncate(size)
todo = [i for i in range((size + block - 1) // block) if i not in done]
lock = threading.Lock()
failed = []


def worker():
    while True:
        with lock:
            if not todo:
                return
            i = todo.pop(0)
        a, b = i * block, min(size, (i + 1) * block) - 1
        for attempt in range(8):
            try:
                req = urllib.request.Request(url, headers={'Range': f'bytes={a}-{b}'})
                data = urllib.request.urlopen(req, timeout=120).read()
                if len(data) != b - a + 1:
                    raise IOError(f'short read {len(data)}')
                with lock:
                    with open(out, 'r+b') as f:
                        f.seek(a)
                        f.write(data)
                    with open(parts_file, 'a') as pf:
                        pf.write(f'{i}\n')
                    done.add(i)
                break
            except Exception as e:  # retry the same block
                if attempt == 7:
                    with lock:
                        failed.append(i)


ts = [threading.Thread(target=worker) for _ in range(n)]
for t in ts:
    t.start()
for t in ts:
    t.join()
if failed:
    sys.exit(f'{len(failed)} blocks failed, rerun to resume')
os.remove(parts_file)
print(f'{out}: {size / 1e9:.1f} GB')
