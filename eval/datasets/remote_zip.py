#!/usr/bin/env python3
"""
Extract selected members of a remote zip over HTTP range requests, without downloading the whole archive
(e.g. only image_2/image_3 of the labelled SemanticKITTI sequences from KITTI's 69 GB data_odometry_color.zip).
Resumable: a member already on disk with the right size is skipped.

usage: remote_zip.py <url> <dest> [--match REGEX] [--list] [--workers 16]
"""
import argparse
import io
import os
import re
import sys
import threading
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import requests


class HttpFile(io.RawIOBase):
    def __init__(self, url, session=None):
        self.url, self.pos, self.s = url, 0, session or requests.Session()
        self.size = int(self.s.head(url, allow_redirects=True).headers['Content-Length'])

    def seekable(self):
        return True

    def readable(self):
        return True

    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else self.pos + off if whence == 1 else self.size + off
        return self.pos

    def tell(self):
        return self.pos

    def readinto(self, b):
        end = min(self.size, self.pos + len(b))
        if end <= self.pos:
            return 0
        for t in range(5):
            try:
                r = self.s.get(self.url, headers={'Range': f'bytes={self.pos}-{end - 1}'}, timeout=60)
                r.raise_for_status()
                break
            except requests.RequestException:
                if t == 4:
                    raise
        n = len(r.content)
        b[:n] = r.content
        self.pos += n
        return n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('url')
    ap.add_argument('dest')
    ap.add_argument('--match', default='.')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--dump', help='with --list: write "name<TAB>compressed size" of the matching members to this file')
    ap.add_argument('--names', help='file with exact member names (one per line) to extract instead of --match')
    ap.add_argument('--workers', type=int, default=16)
    a = ap.parse_args()
    z = zipfile.ZipFile(io.BufferedReader(HttpFile(a.url), 1 << 20))
    want = set(open(a.names).read().split('\n')) - {''} if a.names else None
    members = [m for m in z.infolist() if not m.is_dir() and (m.filename in want if want is not None else re.search(a.match, m.filename))]
    print(f'{len(members)} members, {sum(m.compress_size for m in members) / 1e9:.1f} GB', flush=True)
    if a.list:
        if a.dump:
            with open(a.dump, 'w') as f:
                for m in members:
                    f.write(f'{m.filename}\t{m.compress_size}\n')
        return
    dest = Path(a.dest)
    local = threading.local()

    def get(m):
        out = dest / m.filename
        if out.exists() and out.stat().st_size == m.file_size:
            return 0
        out.parent.mkdir(parents=True, exist_ok=True)
        if not hasattr(local, 'zf'):         # one connection per worker, central directory read once
            local.zf = zipfile.ZipFile(io.BufferedReader(HttpFile(a.url), 1 << 20))
        zf = local.zf
        tmp = out.with_name(out.name + '.part')
        with zf.open(m) as src, open(tmp, 'wb') as dst:
            dst.write(src.read())
        os.replace(tmp, out)
        return 1

    done = 0
    with ThreadPoolExecutor(a.workers) as ex:
        for i, n in enumerate(ex.map(get, members)):
            done += n
            if i % 2000 == 0:
                print(f'{i}/{len(members)}', flush=True)
    print(f'done: {done} new, {len(members)} total')


if __name__ == '__main__':
    sys.exit(main())
