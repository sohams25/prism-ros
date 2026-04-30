#!/usr/bin/env python3
"""
bench/run.py — capture one A/B operation.

Usage:
  python3 bench/run.py --operation {resize|crop|colorconvert|rectify|chain} \
                       --video /path/to/4k.mp4 \
                       --duration 120 \
                       --output-dir bench/results/

Runs launch/A_B_comparison.launch.py with operation:=<op>, waits
10 s warmup, subscribes to /legacy/image_processed and
/accelerated/image_processed with rclpy to record per-frame
latency, and samples container CPU/RSS via psutil. Writes per-run
CSVs and a meta JSON under the output directory.

No numbers in this file leak into published docs directly —
analyze.py consumes these CSVs and produces the summary.
"""

import argparse
import csv
import hashlib
import json
import os
import platform
import shlex
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone

import psutil
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image


LEGACY_TOPIC = '/legacy/image_processed'
ACCEL_TOPIC = '/accelerated/image_processed'

LEGACY_CONTAINER = 'legacy_container'
ACCEL_CONTAINER = 'accel_container'


# --------------------------------------------------------------------------
# ffprobe helper
# --------------------------------------------------------------------------
def probe_video(path, require_4k=True):
    if not os.path.exists(path):
        raise RuntimeError(f'video not found: {path}')

    cmd = [
        'ffprobe', '-v', 'error',
        '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height,r_frame_rate,duration',
        '-of', 'json', path,
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, check=True)
    info = json.loads(r.stdout)['streams'][0]

    codec = info.get('codec_name', '').lower()
    width = int(info.get('width', 0))
    height = int(info.get('height', 0))
    fps_raw = info.get('r_frame_rate', '0/0')

    if codec not in ('h264', 'hevc', 'mpeg4', 'mjpeg'):
        raise RuntimeError(f'unsupported codec: {codec}')
    if require_4k and (width < 3840 or height < 2160):
        raise RuntimeError(f'resolution under 4K: {width}x{height}')

    try:
        num, den = fps_raw.split('/')
        fps = float(num) / float(den) if float(den) else 0.0
    except Exception:
        fps = 0.0

    return {'codec': codec, 'width': width, 'height': height, 'fps': fps,
            'duration_s': float(info.get('duration', 0.0) or 0.0)}


def md5sum(path, chunk=1 << 20):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(chunk), b''):
            h.update(block)
    return h.hexdigest()


# --------------------------------------------------------------------------
# rclpy capture node
# --------------------------------------------------------------------------
class CaptureNode(Node):
    """Subscribes to both /*_processed topics, records per-frame latency."""

    def __init__(self):
        super().__init__('prism_bench_capture')
        self.legacy_rows = []
        self.accel_rows = []

        self.create_subscription(
            Image, LEGACY_TOPIC,
            lambda m: self._record(m, self.legacy_rows), 50)
        self.create_subscription(
            Image, ACCEL_TOPIC,
            lambda m: self._record(m, self.accel_rows), 50)

    def _record(self, msg, sink):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        latency_ns = now_ns - stamp_ns
        try:
            frame_seq = int(msg.header.frame_id) if msg.header.frame_id.isdigit() \
                else len(sink)
        except (ValueError, AttributeError):
            frame_seq = len(sink)
        sink.append((frame_seq, stamp_ns, now_ns, latency_ns))


# --------------------------------------------------------------------------
# Process discovery
# --------------------------------------------------------------------------
# Per-side cmdline match predicates. A side can match multiple processes
# (e.g. stock chain = legacy_container + legacy_cvbridge Python). Resource
# samples are summed across every matched process before being written to
# the resource CSV, so the stock-side total reflects the full stock graph
# cost including any out-of-container helper nodes.
SIDE_MATCHERS = {
    'stock': [
        lambda cmd: 'component_container' in cmd and 'legacy_container' in cmd,
        lambda cmd: 'cv_bridge_subscriber_node.py' in cmd
                    and '__node:=legacy_cvbridge' in cmd,
    ],
    'prism': [
        lambda cmd: 'component_container' in cmd and 'accel_container' in cmd,
    ],
}


def find_pids_for_side(side, timeout_s=10.0):
    """Poll psutil until at least one process matches; return all matches."""
    matchers = SIDE_MATCHERS[side]
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        pids = []
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmd = ' '.join(p.info['cmdline'] or [])
                if any(m(cmd) for m in matchers):
                    pids.append(p.info['pid'])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        if pids:
            return pids
        time.sleep(0.25)
    return []


def sample_side_resources(procs, sink):
    cpu_total = 0.0
    rss_total = 0.0
    alive = []
    for p in procs:
        try:
            cpu_total += p.cpu_percent()
            rss_total += p.memory_info().rss / (1024.0 * 1024.0)
            alive.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    sink.append((time.time(), cpu_total, rss_total))
    return alive


# --------------------------------------------------------------------------
# CSV / JSON writers
# --------------------------------------------------------------------------
def write_latency_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['frame_seq', 't_sent_ns', 't_recv_ns', 'latency_ns'])
        w.writerows(rows)


def write_resource_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_sample', 'cpu_pct', 'rss_mb'])
        w.writerows(rows)


def write_meta(path, meta):
    with open(path, 'w') as f:
        json.dump(meta, f, indent=2, sort_keys=True)


# --------------------------------------------------------------------------
# GStreamer / ROS version strings
# --------------------------------------------------------------------------
def gstreamer_version():
    try:
        r = subprocess.run(
            ['gst-launch-1.0', '--version'],
            capture_output=True, text=True, check=False, timeout=4)
        for line in r.stdout.splitlines():
            if 'GStreamer' in line:
                return line.strip()
    except Exception:
        pass
    return 'unknown'


def prism_commit_sha():
    try:
        r = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            capture_output=True, text=True, check=True,
            cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        return r.stdout.strip()
    except Exception:
        return 'unknown'


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--operation', required=True,
                    choices=['resize', 'crop', 'colorconvert', 'rectify',
                             'chain'])
    ap.add_argument('--video', required=True)
    ap.add_argument('--duration', type=float, default=120.0,
                    help='capture duration in seconds (excludes warmup)')
    ap.add_argument('--warmup', type=float, default=10.0,
                    help='seconds to wait after launch before starting capture')
    ap.add_argument('--output-dir', required=True)
    ap.add_argument('--allow-non-4k', action='store_true',
                    help='Skip the 4K-source guard. Use for resolution-sensitive '
                         'experiments (e.g. v0.2 rectify, where same-size 4K '
                         'output saturates the Python rclpy receiver).')
    args = ap.parse_args()

    video = os.path.abspath(args.video)
    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    # ---- Probe video. Bail loudly on unsupported sources. ----
    print(f'[run] probing {video}...', flush=True)
    info = probe_video(video, require_4k=not args.allow_non_4k)
    print(f'[run] video OK: {info}', flush=True)

    # ---- Start the A/B launch as a subprocess. ----
    launch_cmd = [
        'ros2', 'launch', 'prism_image_proc', 'A_B_comparison.launch.py',
        f'video_path:={video}',
        f'operation:={args.operation}',
    ]
    print(f'[run] launching: {" ".join(shlex.quote(x) for x in launch_cmd)}',
          flush=True)

    launch_proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    start_iso = datetime.now(timezone.utc).isoformat()

    # ---- Find all PIDs per side; may be >1 when the stock side has
    #      out-of-container helper Nodes (colorconvert, chain). ----
    print(f'[run] warmup {args.warmup}s ...', flush=True)
    time.sleep(args.warmup)

    stock_pids = find_pids_for_side('stock', timeout_s=5.0)
    prism_pids = find_pids_for_side('prism', timeout_s=5.0)
    print(f'[run] stock pids={stock_pids}  prism pids={prism_pids}', flush=True)

    stock_procs = [psutil.Process(pid) for pid in stock_pids]
    prism_procs = [psutil.Process(pid) for pid in prism_pids]
    # Prime the psutil running-diff baseline before the first sample.
    for p in stock_procs + prism_procs:
        try:
            p.cpu_percent()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    # ---- Start rclpy capture. ----
    rclpy.init()
    node = CaptureNode()

    legacy_resources = []
    accel_resources = []

    t_end = time.time() + args.duration
    next_sample = time.time() + 1.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now >= next_sample:
            stock_procs = sample_side_resources(stock_procs, legacy_resources)
            prism_procs = sample_side_resources(prism_procs, accel_resources)
            next_sample = now + 1.0

    node.destroy_node()
    rclpy.shutdown()

    end_iso = datetime.now(timezone.utc).isoformat()

    # ---- Clean launch shutdown (process group, SIGTERM → SIGKILL). ----
    try:
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
        launch_proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
        launch_proc.wait(timeout=4)
    except ProcessLookupError:
        pass

    # ---- Write outputs. ----
    ts = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    base = os.path.join(out_dir, f'{args.operation}_{ts}')
    prism_csv = f'{base}_prism.csv'
    stock_csv = f'{base}_stock.csv'
    prism_res_csv = f'{base}_prism_resources.csv'
    stock_res_csv = f'{base}_stock_resources.csv'
    meta_path = f'{base}_meta.json'

    write_latency_csv(prism_csv, node.accel_rows)
    write_latency_csv(stock_csv, node.legacy_rows)
    write_resource_csv(prism_res_csv, accel_resources)
    write_resource_csv(stock_res_csv, legacy_resources)

    meta = {
        'operation':          args.operation,
        'video_path':         video,
        'video_md5':          md5sum(video),
        'video_info':         info,
        'duration_s':         args.duration,
        'warmup_s':           args.warmup,
        'hostname':           socket.gethostname(),
        'uname':              ' '.join(platform.uname()),
        'ros_distro':         os.environ.get('ROS_DISTRO', 'unknown'),
        'gstreamer_version':  gstreamer_version(),
        'prism_commit_sha':   prism_commit_sha(),
        'start_iso':          start_iso,
        'end_iso':            end_iso,
        'files': {
            'prism_latency':    os.path.basename(prism_csv),
            'stock_latency':    os.path.basename(stock_csv),
            'prism_resources':  os.path.basename(prism_res_csv),
            'stock_resources':  os.path.basename(stock_res_csv),
        },
        'row_counts': {
            'prism_frames':   len(node.accel_rows),
            'stock_frames':   len(node.legacy_rows),
            'prism_samples':  len(accel_resources),
            'stock_samples':  len(legacy_resources),
        },
    }
    write_meta(meta_path, meta)

    print(f'[run] wrote {prism_csv} ({len(node.accel_rows)} rows)', flush=True)
    print(f'[run] wrote {stock_csv} ({len(node.legacy_rows)} rows)', flush=True)
    print(f'[run] wrote {prism_res_csv} ({len(accel_resources)} samples)', flush=True)
    print(f'[run] wrote {stock_res_csv} ({len(legacy_resources)} samples)', flush=True)
    print(f'[run] wrote {meta_path}', flush=True)

    if len(node.accel_rows) == 0 or len(node.legacy_rows) == 0:
        print('[run] ERROR: one or both sides recorded zero frames', file=sys.stderr)
        sys.exit(2)


if __name__ == '__main__':
    main()
