#!/usr/bin/env python3
"""
Copy a subset of topics from an MCAP bag into a new MCAP, reading it as a STREAM: the source can be a member of a zip
on the NAS (`unzip -p` pipe), so a 21 GB recording never has to be unpacked locally to keep the few GB SINS needs.
Then, in the container: `ros2 bag reindex <out_dir>` writes metadata.yaml.

usage: extract_topics.py <out_dir> [--zip Z --member M | --file F] [--profile sins-lab] [--topics t1 t2 ...]
                         [--image-hz H]   (keep image topics at most H Hz: SINS runs at 10 Hz, the RealSense at 30)
                         [--min-free-gb G] (stop cleanly, bag still valid, when the output disk gets below G GB)
"""
import argparse
import subprocess
import sys
from pathlib import Path

from mcap.stream_reader import StreamReader
from mcap.records import Channel, Message, Schema
from mcap.writer import CompressionType, Writer

# what SINS + the evaluation read from a lab-robot bag (two RealSense with aligned depth, VLP-16, odometry, TF)
SINS_LAB = ['/velodyne_points', '/tf', '/tf_static', '/robot_description', '/odometry/local', '/odometry/global',
            '/odometry/filtered', '/glim_ros/odom', '/glim_ros/odom_corrected', '/imu/data'] + [
    f'/{c}/camera/{s}' for c in ('front_camera', 'back_camera') for s in (
        'color/image_raw/compressed', 'color/camera_info', 'aligned_depth_to_color/image_raw/compressedDepth',
        'aligned_depth_to_color/camera_info', 'depth/camera_info', 'extrinsics/depth_to_color')]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out_dir')
    ap.add_argument('--zip')
    ap.add_argument('--member')
    ap.add_argument('--file')
    ap.add_argument('--profile', default='sins-lab')
    ap.add_argument('--topics', nargs='*')
    ap.add_argument('--image-hz', type=float, default=0)
    ap.add_argument('--min-free-gb', type=float, default=0)
    a = ap.parse_args()
    keep = set(a.topics or (SINS_LAB if a.profile == 'sins-lab' else []))
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=False)
    if a.zip:
        proc = subprocess.Popen(['unzip', '-p', a.zip, a.member], stdout=subprocess.PIPE, bufsize=1 << 24)
        src = proc.stdout
    else:
        src = open(a.file, 'rb')
    w = Writer(open(out / f'{out.name}_0.mcap', 'wb'), compression=CompressionType.ZSTD)
    w.start(profile='ros2', library='extract_topics.py')
    schemas, channels, kept_ch, n, nbytes = {}, {}, {}, 0, 0
    last_img, min_dt = {}, (1e9 / a.image_hz if a.image_hz > 0 else 0)
    import shutil
    for rec in StreamReader(src, skip_magic=False).records:
        if isinstance(rec, Schema):
            schemas[rec.id] = rec
        elif isinstance(rec, Channel):
            channels[rec.id] = rec
            if rec.topic in keep and rec.id not in kept_ch:
                s = schemas[rec.schema_id]
                sid = w.register_schema(s.name, s.encoding, s.data)
                kept_ch[rec.id] = w.register_channel(rec.topic, rec.message_encoding, sid, rec.metadata)
        elif isinstance(rec, Message) and rec.channel_id in kept_ch:
            if min_dt and 'image' in channels[rec.channel_id].topic and not channels[rec.channel_id].topic.endswith('camera_info'):
                # the sensor rate jitters around 30 Hz: 0.9 * period keeps a steady ~H Hz
                if rec.log_time - last_img.get(rec.channel_id, -1e30) < 0.9 * min_dt:
                    continue
                last_img[rec.channel_id] = rec.log_time
            w.add_message(kept_ch[rec.channel_id], rec.log_time, rec.data, rec.publish_time, rec.sequence)
            n += 1
            nbytes += len(rec.data)
            if n % 20000 == 0:
                print(f'{n} messages, {nbytes / 1e9:.2f} GB', flush=True)
                if a.min_free_gb and shutil.disk_usage(out).free < a.min_free_gb * 1e9:
                    print(f'STOP: less than {a.min_free_gb} GB free, bag cut at {rec.log_time}', flush=True)
                    break
    w.finish()
    missing = keep - {c.topic for c in channels.values()}
    print(f'done: {n} messages, {nbytes / 1e9:.2f} GB; topics not in the source: {sorted(missing)}')


if __name__ == '__main__':
    sys.exit(main())
