"""
F2 E2.2 — kernel microbenchmark for cv::remap at 4K BGR.

Standalone, no ROS. Times the rectification kernel in isolation to
establish a per-host CPU absolute baseline before E2.3's end-to-end
A/B. 1000 iterations, drop first 100 as warmup, report p50/p95/p99
in milliseconds.

The LUT is computed once with cv2.initUndistortRectifyMap from a
representative pinhole + radial-tangential camera_info chosen to
mirror a typical robotics fisheye undistortion job. The intrinsics
are baked in below for reproducibility — no external camera_info
file to drift over time.

Usage:
    python3 cv_remap_4k_bench.py [--width 3840] [--height 2160]
                                 [--iters 1000] [--warmup 100]
                                 [--interp linear|nearest]
"""
import argparse
import statistics
import time

import cv2
import numpy as np


# Representative 4K wide-angle camera intrinsics. Chosen to keep the
# remap math non-trivial (significant radial distortion) while staying
# in the plausible-lens range for a robotics fisheye.
# Intrinsics are independent of the run; baked in for reproducibility.
DEFAULT_K = np.array([
    [1900.0,    0.0, 1920.0],
    [   0.0, 1900.0, 1080.0],
    [   0.0,    0.0,    1.0],
], dtype=np.float64)
DEFAULT_D = np.array([-0.30, 0.10, 0.0, 0.0, -0.02], dtype=np.float64)


def percentile(values, q):
    s = sorted(values)
    if not s:
        return float("nan")
    k = (len(s) - 1) * q / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    frac = k - lo
    return s[lo] + (s[hi] - s[lo]) * frac


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--width", type=int, default=3840)
    p.add_argument("--height", type=int, default=2160)
    p.add_argument("--iters", type=int, default=1000)
    p.add_argument("--warmup", type=int, default=100)
    p.add_argument("--interp", choices=["linear", "nearest"], default="linear")
    args = p.parse_args()

    interp = {
        "linear": cv2.INTER_LINEAR,
        "nearest": cv2.INTER_NEAREST,
    }[args.interp]

    K = DEFAULT_K.copy()
    D = DEFAULT_D.copy()
    new_K, _ = cv2.getOptimalNewCameraMatrix(
        K, D, (args.width, args.height), 0.0, (args.width, args.height)
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        K, D, None, new_K, (args.width, args.height), cv2.CV_16SC2
    )

    rng = np.random.default_rng(seed=42)
    src = rng.integers(0, 256, (args.height, args.width, 3), dtype=np.uint8)
    dst = np.empty_like(src)

    print(f"resolution: {args.width}x{args.height} BGR8")
    print(f"interp:     {args.interp}")
    print(f"iters:      {args.iters} (warmup drop: {args.warmup})")
    print(f"K:          fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    print(f"D:          {D.tolist()}")
    print()

    times_ms = []
    for i in range(args.iters):
        t0 = time.perf_counter_ns()
        cv2.remap(src, map1, map2, interp, dst=dst, borderMode=cv2.BORDER_CONSTANT)
        t1 = time.perf_counter_ns()
        times_ms.append((t1 - t0) / 1e6)

    sample = times_ms[args.warmup:]
    p50 = percentile(sample, 50)
    p95 = percentile(sample, 95)
    p99 = percentile(sample, 99)
    mean = statistics.fmean(sample)
    fps_ceiling = 1000.0 / p50 if p50 > 0 else float("inf")

    print(f"samples (post-warmup): {len(sample)}")
    print(f"mean:  {mean:7.3f} ms")
    print(f"p50:   {p50:7.3f} ms")
    print(f"p95:   {p95:7.3f} ms")
    print(f"p99:   {p99:7.3f} ms")
    print(f"fps ceiling at p50: {fps_ceiling:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
