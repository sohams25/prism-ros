#!/usr/bin/env python3
"""Render the operator's simple per-operation Markdown summary from analyze.py's summary.json.

Per-operation block emits: median, mean, p95, p99, mean CPU %, mean RSS MB, realised fps,
each as (stock vs prism). Median and mean carry absolute and percent deltas. The two-line
header records host label and GStreamer version.
"""
import argparse
import json
import sys


OPERATIONS = ('resize', 'crop', 'colorconvert', 'chain')


def fmt(v, pat='{:.2f}'):
    if v is None:
        return '—'
    return pat.format(v)


def diff_pct(p, s):
    if s in (0, None) or p is None:
        return None
    return ((p - s) / s) * 100.0


def render(summary, host_label, gst_version):
    lines = []
    lines.append(f'**Host:** {host_label}')
    lines.append(f'**GStreamer:** {gst_version}')
    lines.append('')

    for op in OPERATIONS:
        block = summary.get(op)
        if not block:
            lines.append(f'## {op}')
            lines.append('')
            lines.append('_No capture for this operation._')
            lines.append('')
            continue

        p_lat = block.get('prism', {}).get('latency', {}) or {}
        s_lat = block.get('stock', {}).get('latency', {}) or {}
        p_res = block.get('prism', {}).get('resources', {}) or {}
        s_res = block.get('stock', {}).get('resources', {}) or {}

        p_med, s_med = p_lat.get('median_ms'), s_lat.get('median_ms')
        p_mean, s_mean = p_lat.get('mean_ms'), s_lat.get('mean_ms')
        p_p95, s_p95 = p_lat.get('p95_ms'), s_lat.get('p95_ms')
        p_p99, s_p99 = p_lat.get('p99_ms'), s_lat.get('p99_ms')
        p_cpu, s_cpu = p_res.get('cpu_mean'), s_res.get('cpu_mean')
        p_rss, s_rss = p_res.get('rss_mean'), s_res.get('rss_mean')
        p_fps, s_fps = p_lat.get('realized_fps'), s_lat.get('realized_fps')

        med_dabs = (p_med - s_med) if (p_med is not None and s_med is not None) else None
        med_dpc = diff_pct(p_med, s_med)
        mean_dabs = (p_mean - s_mean) if (p_mean is not None and s_mean is not None) else None
        mean_dpc = diff_pct(p_mean, s_mean)

        lines.append(f'## {op}')
        lines.append('')
        lines.append('| metric | stock | prism | Δ | Δ % |')
        lines.append('| --- | ---: | ---: | ---: | ---: |')
        lines.append(f'| median latency (ms) | {fmt(s_med)} | {fmt(p_med)} | '
                     f'{fmt(med_dabs, "{:+.2f}")} | {fmt(med_dpc, "{:+.1f} %")} |')
        lines.append(f'| mean latency (ms)   | {fmt(s_mean)} | {fmt(p_mean)} | '
                     f'{fmt(mean_dabs, "{:+.2f}")} | {fmt(mean_dpc, "{:+.1f} %")} |')
        lines.append(f'| p95 latency (ms)    | {fmt(s_p95)} | {fmt(p_p95)} | | |')
        lines.append(f'| p99 latency (ms)    | {fmt(s_p99)} | {fmt(p_p99)} | | |')
        lines.append(f'| mean CPU (%)        | {fmt(s_cpu, "{:.1f}")} | {fmt(p_cpu, "{:.1f}")} | | |')
        lines.append(f'| mean RSS (MB)       | {fmt(s_rss, "{:.0f}")} | {fmt(p_rss, "{:.0f}")} | | |')
        lines.append(f'| realised fps        | {fmt(s_fps)} | {fmt(p_fps)} | | |')
        lines.append('')

    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--summary', required=True, help='path to analyze.py summary.json')
    ap.add_argument('--host-label', required=True,
                    help='one-line host description, e.g. "Intel desktop, GStreamer 1.20, direct-mode fallback"')
    ap.add_argument('--gst-version', default='',
                    help='GStreamer version string; if empty, omitted from header')
    ap.add_argument('--out', required=True, help='output Markdown path')
    args = ap.parse_args()

    with open(args.summary) as f:
        summary = json.load(f)

    md = render(summary, args.host_label, args.gst_version or '(unspecified)')

    with open(args.out, 'w') as f:
        f.write(md)
    print(f'[emit_simple_summary] wrote {args.out} ({len(md)} bytes)')


if __name__ == '__main__':
    main()
