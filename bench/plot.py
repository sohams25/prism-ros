#!/usr/bin/env python3
"""
bench/plot.py — render dark + light SVGs for each operation.

Usage:
  python3 bench/plot.py --summary bench/results/summary.json \
                        --output-dir docs/assets/bench/

For every operation present in summary.json, emit three charts × two
themes = 6 SVGs per op. With 4 ops that's 24 files:

  <op>_latency_distribution_{dark,light}.svg
  <op>_latency_timeseries_{dark,light}.svg
  <op>_resources_{dark,light}.svg

Axis labels and legends only — no chart titles. The surrounding doc
section supplies the title. Figures are rendered transparent so the
host page background shows through cleanly in both themes.
"""

import argparse
import csv
import glob
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import style


# --------------------------------------------------------------------------
# Per-op data loading
# --------------------------------------------------------------------------
def load_run(results_dir, operation):
    """Locate the most-recent meta for `operation` and load its series."""
    latest = None
    for meta_path in glob.glob(os.path.join(results_dir, f'{operation}_*_meta.json')):
        with open(meta_path) as f:
            meta = json.load(f)
        stamp = meta.get('end_iso', '')
        if latest is None or stamp > latest[0]:
            latest = (stamp, meta_path, meta)
    if latest is None:
        return None
    _, meta_path, meta = latest
    base = os.path.dirname(meta_path)

    def read_lat(fn):
        xs = []
        with open(os.path.join(base, fn)) as f:
            r = csv.DictReader(f)
            for row in r:
                try:
                    xs.append((int(row['t_recv_ns']), int(row['latency_ns'])))
                except (ValueError, KeyError):
                    continue
        return xs

    def read_res(fn):
        xs = []
        with open(os.path.join(base, fn)) as f:
            r = csv.DictReader(f)
            for row in r:
                try:
                    xs.append((float(row['t_sample']),
                               float(row['cpu_pct']),
                               float(row['rss_mb'])))
                except (ValueError, KeyError):
                    continue
        return xs

    return {
        'meta':   meta,
        'prism_lat': read_lat(meta['files']['prism_latency']),
        'stock_lat': read_lat(meta['files']['stock_latency']),
        'prism_res': read_res(meta['files']['prism_resources']),
        'stock_res': read_res(meta['files']['stock_resources']),
    }


def trim_latency(rows, warmup_s=10.0, tail_s=5.0):
    if not rows:
        return rows
    t0 = rows[0][0]
    tN = rows[-1][0]
    lo = t0 + int(warmup_s * 1e9)
    hi = tN - int(tail_s * 1e9)
    return [r for r in rows if lo <= r[0] <= hi]


def trim_resources(rows, warmup_s=10.0, tail_s=5.0):
    if not rows:
        return rows
    t0 = rows[0][0]
    tN = rows[-1][0]
    return [r for r in rows if (t0 + warmup_s) <= r[0] <= (tN - tail_s)]


def percentile(xs, p):
    if not xs:
        return 0.0
    return float(np.percentile(xs, p))


# --------------------------------------------------------------------------
# Chart renderers (per-theme)
# --------------------------------------------------------------------------
def save_fig(fig, out_path):
    fig.savefig(out_path, format='svg', bbox_inches='tight', pad_inches=0.25,
                transparent=True)
    plt.close(fig)


def render_latency_distribution(run, op, theme_palette, out_path):
    prism_ms = [r[1] / 1e6 for r in trim_latency(run['prism_lat'])]
    stock_ms = [r[1] / 1e6 for r in trim_latency(run['stock_lat'])]

    def five(xs):
        return [min(xs), percentile(xs, 5), percentile(xs, 50),
                percentile(xs, 95), max(xs)] if xs else [0, 0, 0, 0, 0]

    prism5 = five(prism_ms)
    stock5 = five(stock_ms)

    fig, ax = plt.subplots(figsize=(8.5, 3.6))
    labels = ['min', 'p5', 'median', 'p95', 'max']
    x = np.arange(len(labels))
    w = 0.36

    ax.bar(x - w / 2, stock5, w, label='stock',
           color=theme_palette['stock_accent'], alpha=0.85)
    ax.bar(x + w / 2, prism5, w, label='prism',
           color=theme_palette['prism_accent'], alpha=0.95)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('latency (ms)')
    ax.legend(loc='upper left')
    save_fig(fig, out_path)


def render_latency_timeseries(run, op, theme_palette, out_path):
    def rolling(rows, window_s=2.0):
        if not rows:
            return [], []
        t0 = rows[0][0]
        xs = [(r[0] - t0) / 1e9 for r in rows]
        ys = [r[1] / 1e6 for r in rows]
        out_t, out_y = [], []
        win = int(window_s)
        # bucket by second and rolling-mean over a window of `win` seconds
        by_sec = {}
        for t, y in zip(xs, ys):
            s = int(t)
            by_sec.setdefault(s, []).append(y)
        secs = sorted(by_sec.keys())
        for i, s in enumerate(secs):
            lo = max(0, i - win + 1)
            bucket = []
            for ss in secs[lo:i + 1]:
                bucket.extend(by_sec[ss])
            if bucket:
                out_t.append(s)
                out_y.append(sum(bucket) / len(bucket))
        return out_t, out_y

    stock_t, stock_y = rolling(trim_latency(run['stock_lat']))
    prism_t, prism_y = rolling(trim_latency(run['prism_lat']))

    fig, ax = plt.subplots(figsize=(8.5, 3.6))
    if stock_t:
        ax.plot(stock_t, stock_y, label='stock',
                color=theme_palette['stock_accent'])
    if prism_t:
        ax.plot(prism_t, prism_y, label='prism',
                color=theme_palette['prism_accent'])
    ax.set_xlabel('time (s, capture-relative)')
    ax.set_ylabel('rolling 2s mean latency (ms)')
    ax.legend(loc='upper right')
    save_fig(fig, out_path)


def render_resources(run, op, theme_palette, out_path):
    def stats(res):
        if not res:
            return (0.0, 0.0, 0.0)
        cpu = [r[1] for r in res]
        rss = [r[2] for r in res]
        return (float(np.mean(cpu)), percentile(cpu, 95), float(np.mean(rss)))

    s_cpu_m, s_cpu_p95, s_rss_m = stats(trim_resources(run['stock_res']))
    p_cpu_m, p_cpu_p95, p_rss_m = stats(trim_resources(run['prism_res']))

    fig, (ax_cpu, ax_rss) = plt.subplots(1, 2, figsize=(8.5, 3.6),
                                         gridspec_kw={'width_ratios': [2, 1]})
    x = np.arange(2)
    w = 0.38
    ax_cpu.bar(x - w / 2, [s_cpu_m, s_cpu_p95], w, label='stock',
               color=theme_palette['stock_accent'], alpha=0.85)
    ax_cpu.bar(x + w / 2, [p_cpu_m, p_cpu_p95], w, label='prism',
               color=theme_palette['prism_accent'], alpha=0.95)
    ax_cpu.set_xticks(x)
    ax_cpu.set_xticklabels(['cpu mean', 'cpu p95'])
    ax_cpu.set_ylabel('cpu (%)')
    ax_cpu.legend(loc='upper left')

    x2 = np.arange(1)
    ax_rss.bar(x2 - w / 2, [s_rss_m], w, label='stock',
               color=theme_palette['stock_accent'], alpha=0.85)
    ax_rss.bar(x2 + w / 2, [p_rss_m], w, label='prism',
               color=theme_palette['prism_accent'], alpha=0.95)
    ax_rss.set_xticks(x2)
    ax_rss.set_xticklabels(['rss mean'])
    ax_rss.set_ylabel('rss (MB)')
    ax_rss.legend(loc='upper right')

    save_fig(fig, out_path)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def render_all_for_theme(run, op, theme, out_dir):
    if theme == 'dark':
        palette = style.apply_dark()
    else:
        palette = style.apply_light()

    render_latency_distribution(
        run, op, palette,
        os.path.join(out_dir, f'{op}_latency_distribution_{theme}.svg'))
    render_latency_timeseries(
        run, op, palette,
        os.path.join(out_dir, f'{op}_latency_timeseries_{theme}.svg'))
    render_resources(
        run, op, palette,
        os.path.join(out_dir, f'{op}_resources_{theme}.svg'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--summary', required=True)
    ap.add_argument('--output-dir', required=True)
    args = ap.parse_args()

    summary_path = os.path.abspath(args.summary)
    results_dir = os.path.dirname(summary_path)
    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    with open(summary_path) as f:
        summary = json.load(f)

    rendered = 0
    for op in ('resize', 'crop', 'colorconvert', 'chain'):
        if op not in summary:
            continue
        run = load_run(results_dir, op)
        if run is None:
            print(f'[plot] skipping {op}: no run data')
            continue
        for theme in ('dark', 'light'):
            render_all_for_theme(run, op, theme, out_dir)
            rendered += 3
        print(f'[plot] rendered {op} (6 svgs)')

    print(f'[plot] wrote {rendered} svgs to {out_dir}')


if __name__ == '__main__':
    main()
