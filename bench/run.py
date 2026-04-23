#!/usr/bin/env python3
"""
bench/run.py — capture one A/B operation.

Usage:
  python3 bench/run.py --operation {resize|crop|colorconvert|chain} \
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
def probe_video(path):
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
    if width < 3840 or height < 2160:
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
# Container discovery
# --------------------------------------------------------------------------
def find_container_pid(container_name, timeout_s=10.0):
    """Poll psutil for a component_container with container_name in argv."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmd = ' '.join(p.info['cmdline'] or [])
                if 'component_container' in cmd and container_name in cmd:
                    return p.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        time.sleep(0.25)
    return None


def sample_resources(proc, sink):
    try:
        cpu = proc.cpu_percent()
        rss_mb = proc.memory_info().rss / (1024.0 * 1024.0)
        sink.append((time.time(), cpu, rss_mb))
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        pass


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
                    choices=['resize', 'crop', 'colorconvert', 'chain'])
    ap.add_argument('--video', required=True)
    ap.add_argument('--duration', type=float, default=120.0,
                    help='capture duration in seconds (excludes warmup)')
    ap.add_argument('--warmup', type=float, default=10.0,
                    help='seconds to wait after launch before starting capture')
    ap.add_argument('--output-dir', required=True)
    args = ap.parse_args()

    video = os.path.abspath(args.video)
    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    # ---- Probe video. Bail loudly on unsupported sources. ----
    print(f'[run] probing {video}...', flush=True)
    info = probe_video(video)
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

    # ---- Find container PIDs (best-effort; may be absent for chain stock). ----
    print(f'[run] warmup {args.warmup}s ...', flush=True)
    time.sleep(args.warmup)

    legacy_pid = find_container_pid(LEGACY_CONTAINER, timeout_s=5.0)
    accel_pid = find_container_pid(ACCEL_CONTAINER, timeout_s=5.0)
    print(f'[run] legacy_container pid={legacy_pid}  accel_container pid={accel_pid}',
          flush=True)

    legacy_proc = psutil.Process(legacy_pid) if legacy_pid else None
    accel_proc = psutil.Process(accel_pid) if accel_pid else None
    if legacy_proc:
        legacy_proc.cpu_percent()  # prime the running-diff baseline
    if accel_proc:
        accel_proc.cpu_percent()

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
            if legacy_proc:
                sample_resources(legacy_proc, legacy_resources)
            if accel_proc:
                sample_resources(accel_proc, accel_resources)
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
