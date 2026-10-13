"""Unified per-method / ground-truth track format (BENCHMARK_PLAN.md, "Formato unico").

One row per (frame, object):

    t_ns,track_id,x,y,z,l,w,h,yaw,vx,vy,vz,state,class,score

- t_ns: frame timestamp [ns]. All rows of one output message share it; a frame
  with no objects is written as a single row with track_id = -1 so that empty
  outputs are distinguishable from missing ones.
- x..vz: box centre [m], size [m], yaw [rad], velocity [m/s], in the frame named
  by the file's sidecar (results are always written in the GT frame).
- state: one of STATES; class: free text ('' if unknown); score: method-specific.
"""

import numpy as np
import pandas as pd

COLUMNS = ['t_ns', 'track_id', 'x', 'y', 'z', 'l', 'w', 'h', 'yaw',
           'vx', 'vy', 'vz', 'state', 'class', 'score']

DYNAMIC = 'dynamic'
POTENTIALLY_DYNAMIC = 'potentially_dynamic'
STATIC = 'static'
STATES = (DYNAMIC, POTENTIALLY_DYNAMIC, STATIC)

EMPTY_FRAME_ID = -1

# jo_msgs/Obstacle status constants -> unified state. STATUS_TRACKED is not a
# classifier output (v1 publishes it for generic tracked objects): treated as static.
JO_STATUS = {0: STATIC, 1: DYNAMIC, 2: POTENTIALLY_DYNAMIC, 3: STATIC}


def empty_tracks():
    return pd.DataFrame({c: pd.Series(dtype='int64' if c in ('t_ns', 'track_id') else
                                      ('object' if c in ('state', 'class') else 'float64'))
                         for c in COLUMNS})


def make_tracks(rows):
    """rows: iterable of dicts with (a subset of) COLUMNS; missing numeric fields -> NaN."""
    df = pd.DataFrame(list(rows))
    if df.empty:
        return empty_tracks()
    for c in COLUMNS:
        if c not in df:
            df[c] = '' if c in ('state', 'class') else np.nan
    df = df[COLUMNS]
    df['t_ns'] = df['t_ns'].astype('int64')
    df['track_id'] = df['track_id'].astype('int64')
    df['class'] = df['class'].fillna('').astype(str)
    df['state'] = df['state'].fillna('').astype(str)
    return df.sort_values(['t_ns', 'track_id'], kind='stable').reset_index(drop=True)


def write_tracks(df, path):
    df[COLUMNS].to_csv(path, index=False, float_format='%.4f')


def read_tracks(path):
    df = pd.read_csv(path, dtype={'state': str, 'class': str}, keep_default_na=False)
    for c in COLUMNS:
        if c in ('state', 'class'):
            continue
        df[c] = pd.to_numeric(df[c], errors='coerce')
    df['t_ns'] = df['t_ns'].astype('int64')
    df['track_id'] = df['track_id'].astype('int64')
    return df[COLUMNS]


def frames(df):
    """Sorted unique frame timestamps (including empty frames)."""
    return np.unique(df['t_ns'].to_numpy())


def objects(df):
    """Rows that describe real objects (drops empty-frame markers)."""
    return df[df['track_id'] != EMPTY_FRAME_ID]
