#!/usr/bin/env python3
"""
bench/analyze.py — roll per-run CSVs up into summary.json and summary.md.

Usage:
  python3 bench/analyze.py --results-dir bench/results/ \
                           --output bench/results/summary.json

Scans the results directory for <op>_<ts>_meta.json files, one per
run, and for each meta consumes its four associated CSVs. Drops the
first 10 s and last 5 s of each series as warmup/tail, computes
latency/cpu/rss/fps statistics per side, and emits:

  summary.json — nested: {operation: {prism, stock, delta}}
  summary.md   — one-row-per-operation markdown table

If more than one run exists per operation, the most recent
timestamp wins.
"""

import argparse
import csv
import glob
import json
import os
import statistics
from collections import defaultdict


WARMUP_DROP_S = 10.0
TAIL_DROP_S = 5.0


# --------------------------------------------------------------------------
# CSV readers
# --------------------------------------------------------------------------
def read_latency(path):
    rows = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                rows.append({
                    'frame_seq':   int(row['frame_seq']),
                    't_sent_ns':   int(row['t_sent_ns']),
                    't_recv_ns':   int(row['t_recv_ns']),
                    'latency_ns':  int(row['latency_ns']),
                })
            except (ValueError, KeyError):
                continue
    return rows


def read_resources(path):
    rows = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                rows.append({
                    't_sample':  float(row['t_sample']),
                    'cpu_pct':   float(row['cpu_pct']),
                    'rss_mb':    float(row['rss_mb']),
                })
            except (ValueError, KeyError):
                continue
    return rows


# --------------------------------------------------------------------------
# Trim + stats
# --------------------------------------------------------------------------
def trim_by_wallclock(rows, key, warmup, tail):
    if not rows:
        return rows
    t0 = rows[0][key]
    tN = rows[-1][key]
    lo = t0 + warmup
    hi = tN - tail
    return [r for r in rows if lo <= r[key] <= hi]


def trim_latency(rows):
    # CSV t_recv_ns is nanoseconds; convert bounds to ns.
    if not rows:
        return rows
    t0 = rows[0]['t_recv_ns']
    tN = rows[-1]['t_recv_ns']
    lo = t0 + int(WARMUP_DROP_S * 1e9)
    hi = tN - int(TAIL_DROP_S * 1e9)
    return [r for r in rows if lo <= r['t_recv_ns'] <= hi]


def trim_resources(rows):
    return trim_by_wallclock(rows, 't_sample', WARMUP_DROP_S, TAIL_DROP_S)


def pct(xs, p):
    if not xs:
        return 0.0
    s = sorted(xs)
    idx = int(round((p / 100.0) * (len(s) - 1)))
    return s[idx]


def latency_stats(rows):
    if not rows:
        return {}
    lats_ms = [r['latency_ns'] / 1e6 for r in rows]
    t0 = rows[0]['t_recv_ns']
    tN = rows[-1]['t_recv_ns']
    eff_s = max((tN - t0) / 1e9, 1e-9)
    return {
        'frames':      len(rows),
        'effective_duration_s': eff_s,
        'min_ms':      min(lats_ms),
        'p5_ms':       pct(lats_ms, 5),
        'median_ms':   statistics.median(lats_ms),
        'mean_ms':     statistics.fmean(lats_ms),
        'p95_ms':      pct(lats_ms, 95),
        'p99_ms':      pct(lats_ms, 99),
        'max_ms':      max(lats_ms),
        'stdev_ms':    statistics.pstdev(lats_ms) if len(lats_ms) > 1 else 0.0,
        'realized_fps': len(rows) / eff_s,
    }


def resource_stats(rows):
    if not rows:
        return {}
    cpu = [r['cpu_pct'] for r in rows]
    rss = [r['rss_mb'] for r in rows]
    return {
        'samples':   len(rows),
        'cpu_mean':  statistics.fmean(cpu),
        'cpu_p95':   pct(cpu, 95),
        'rss_mean':  statistics.fmean(rss),
        'rss_peak':  max(rss),
    }


def side_summary(lat_rows, res_rows):
    return {
        'latency':    latency_stats(lat_rows),
        'resources':  resource_stats(res_rows),
    }


def safe_get(d, *path, default=0.0):
    cur = d
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def compute_delta(prism, stock):
    p_med = safe_get(prism, 'latency', 'median_ms')
    s_med = safe_get(stock, 'latency', 'median_ms')
    p_p95 = safe_get(prism, 'latency', 'p95_ms')
    s_p95 = safe_get(stock, 'latency', 'p95_ms')
    p_cpu = safe_get(prism, 'resources', 'cpu_mean')
    s_cpu = safe_get(stock, 'resources', 'cpu_mean')

    def diff(p, s):
        if s == 0:
            return {'absolute': 0.0, 'percent': 0.0}
        return {'absolute': p - s, 'percent': ((p - s) / s) * 100.0}

    return {
        'median_latency':  diff(p_med, s_med),
        'p95_latency':     diff(p_p95, s_p95),
        'cpu_mean':        diff(p_cpu, s_cpu),
    }


# --------------------------------------------------------------------------
# Summary.md renderer
# --------------------------------------------------------------------------
def render_summary_md(summary):
    lines = []
    lines.append('# v0.1.0 benchmark summary')
    lines.append('')
    lines.append('| operation | frames (p/s) | median ms (p/s) | p95 ms (p/s) | p99 ms (p/s) | cpu%  mean (p/s) | rss MB mean (p/s) | fps (p/s) | median Δ | median Δ% |')
    lines.append('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    for op, side in summary.items():
        p = side.get('prism', {})
        s = side.get('stock', {})
        d = side.get('delta', {})

        def pair(pv, sv, fmt='{:.2f}'):
            p_s = fmt.format(pv) if isinstance(pv, (int, float)) else str(pv)
            s_s = fmt.format(sv) if isinstance(sv, (int, float)) else str(sv)
            return f'{p_s} / {s_s}'

        lines.append(
            '| {op} | {frm} | {med} | {p95} | {p99} | {cpu} | {rss} | {fps} | {dms} | {dpc} |'.format(
                op=op,
                frm=pair(safe_get(p, 'latency', 'frames'),
                         safe_get(s, 'latency', 'frames'), '{:.0f}'),
                med=pair(safe_get(p, 'latency', 'median_ms'),
                         safe_get(s, 'latency', 'median_ms')),
                p95=pair(safe_get(p, 'latency', 'p95_ms'),
                         safe_get(s, 'latency', 'p95_ms')),
                p99=pair(safe_get(p, 'latency', 'p99_ms'),
                         safe_get(s, 'latency', 'p99_ms')),
                cpu=pair(safe_get(p, 'resources', 'cpu_mean'),
                         safe_get(s, 'resources', 'cpu_mean'), '{:.1f}'),
                rss=pair(safe_get(p, 'resources', 'rss_mean'),
                         safe_get(s, 'resources', 'rss_mean'), '{:.0f}'),
                fps=pair(safe_get(p, 'latency', 'realized_fps'),
                         safe_get(s, 'latency', 'realized_fps'), '{:.2f}'),
                dms='{:+.2f}'.format(d.get('median_latency', {}).get('absolute', 0.0)),
                dpc='{:+.1f}%'.format(d.get('median_latency', {}).get('percent', 0.0)),
            ))
    return '\n'.join(lines) + '\n'


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results-dir', required=True)
    ap.add_argument('--output', required=True,
                    help='path to summary.json (summary.md is written beside it)')
    args = ap.parse_args()

    results_dir = os.path.abspath(args.results_dir)

    # Group meta files by operation, pick the most recent per op.
    per_op_latest = {}
    for meta_path in glob.glob(os.path.join(results_dir, '*_meta.json')):
        with open(meta_path) as f:
            meta = json.load(f)
        op = meta.get('operation')
        if not op:
            continue
        stamp = meta.get('end_iso', '')
        if op not in per_op_latest or stamp > per_op_latest[op][0]:
            per_op_latest[op] = (stamp, meta_path, meta)

    if not per_op_latest:
        raise SystemExit(f'no *_meta.json files found under {results_dir}')

    summary = {}
    for op in ('resize', 'crop', 'colorconvert', 'chain'):
        if op not in per_op_latest:
            continue
        _, meta_path, meta = per_op_latest[op]
        base_dir = os.path.dirname(meta_path)

        prism_lat = trim_latency(read_latency(os.path.join(base_dir, meta['files']['prism_latency'])))
        stock_lat = trim_latency(read_latency(os.path.join(base_dir, meta['files']['stock_latency'])))
        prism_res = trim_resources(read_resources(os.path.join(base_dir, meta['files']['prism_resources'])))
        stock_res = trim_resources(read_resources(os.path.join(base_dir, meta['files']['stock_resources'])))

        prism = side_summary(prism_lat, prism_res)
        stock = side_summary(stock_lat, stock_res)
        delta = compute_delta(prism, stock)

        summary[op] = {
            'meta_file': os.path.basename(meta_path),
            'prism':     prism,
            'stock':     stock,
            'delta':     delta,
            'source_video_md5': meta.get('video_md5'),
            'prism_commit_sha': meta.get('prism_commit_sha'),
        }

    out_json = os.path.abspath(args.output)
    with open(out_json, 'w') as f:
        json.dump(summary, f, indent=2, sort_keys=True)
    print(f'[analyze] wrote {out_json}')

    out_md = os.path.splitext(out_json)[0] + '.md'
    with open(out_md, 'w') as f:
        f.write(render_summary_md(summary))
    print(f'[analyze] wrote {out_md}')


if __name__ == '__main__':
    main()
