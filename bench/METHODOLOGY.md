# Prism v0.1.0 — benchmark methodology

This document describes exactly how the numbers published on the
[project site](https://sohams25.github.io/prism-ros/#benchmarks)
and in the repository [README](../README.md#benchmarks) were
measured. The intent is that anyone with comparable hardware can
re-run the same scripts and cross-check.

## What we measure

For each of four operations (`resize`, `crop`, `colorconvert`,
`chain`) we run two equivalent graphs side by side in the same
process group, on the same 4K source video, and compare:

1. **Per-frame latency.** Time from `Image::header.stamp` (set by
   `prism::MediaStreamerNode` when it publishes the raw frame)
   to the moment the final `*_processed` message arrives at a
   subscriber. Measured in nanoseconds, reported in milliseconds.
2. **Container CPU %** — sampled at 1 Hz via `psutil` against
   the two `component_container` PIDs.
3. **Container RSS (MB)** — sampled at 1 Hz, same PIDs.
4. **Realized FPS** — `frame_count / effective_duration_s`, where
   `effective_duration_s` is the capture window minus the warmup
   drop and tail drop.

Each capture runs for 120 seconds. The first 10 seconds and last
5 seconds are discarded when computing statistics (the source has
to spin up; the tail is the SIGTERM settling period).

## What we do NOT measure

- **GPU path.** This hardware is an Intel desktop on stock ROS 2
  Humble (GStreamer 1.20). `vaapipostproc` is present but fails
  live validation due to a chroma regression, so the Intel iGPU
  path falls back to CPU direct mode. Jetson Orin validation is
  a separate, later release.
- **Network loss or cross-host DDS.** Both containers run on the
  same host.
- **End-to-end pipeline scenarios.** No downstream ML inference,
  no recording, no transport plugins beyond `raw`.

## What "legacy" means here

For each operation, the stock side uses the closest available
upstream ROS 2 implementation:

| Operation      | Stock-side graph                                                   |
| ---            | ---                                                                |
| `resize`       | `image_proc::ResizeNode`                                           |
| `crop`         | `image_proc::CropDecimateNode` with `decimation_{x,y}=1`           |
| `colorconvert` | `bench/helpers/cv_bridge_subscriber_node.py` (see caveat below)    |
| `chain`        | `CropDecimate` → `ResizeNode` → `cv_bridge_subscriber`, DDS-piped  |

On both sides, the source is `prism::MediaStreamerNode` publishing
a 4K video at 10 Hz (`max_fps: 10`), the subscriber is identical,
and intra-process comms are enabled. Everything between input and
output is the single variable.

## The chain-composition caveat (important)

For the `chain` operation, the **stock side runs three separate
nodes piped through DDS topics** (`CropDecimate` → `Resize` →
`cv_bridge`). The **Prism side runs one chained pipeline**
(`prism::ImageProcNode` with `action=crop,resize,colorconvert`).

The latency delta on the chain row therefore reflects Prism's
single-pipeline architecture avoiding two intermediate DDS hops.
**It is not a kernel-speed claim.** Readers should attribute the
chain-row number to architectural behaviour, not to any assertion
that Prism's resize/crop/colorconvert kernels are faster than
stock's when run in isolation — those comparisons live in the
first three rows of the table, each with the caveat that they
bypass the DDS round-trip as well.

The README summary table carries this distinction in its `Note`
column at point of claim rather than in a footnote, and the site
Benchmarks section repeats it in a `.finding` callout immediately
above the chain charts.

## The colorconvert caveat

`image_proc` has no dedicated colorconvert node. The stock side
of the colorconvert and chain operations therefore uses
`bench/helpers/cv_bridge_subscriber_node.py`: a minimal rclpy
Node that subscribes to an `Image`, calls `cv_bridge.toCvCopy`
with the target encoding, and re-publishes as an `Image`. This
is the fairest available approximation of "what a typical ROS 2
user would write if they needed a standalone colorconvert step
today," but it is not an official reference implementation.

## Sources of noise, mitigations

- **First-run cold-cache effects.** Drop the first 10 s of each
  capture as warmup.
- **Thermal throttling.** The captures are short (120 s each)
  with a 15 s settle between operations. No artificial thermal
  load is imposed; ambient room temperature is comparable across
  the four runs.
- **Other host processes.** Run captures with the desktop idle.
- **Python GIL.** Latency is captured in a rclpy subscriber
  (Python), which does introduce small GIL-related jitter on
  the subscriber side. The same subscriber is used for both
  Prism and stock, so the effect cancels in the delta. Absolute
  latency numbers are therefore slightly pessimistic for both
  sides.
- **Resource attribution.** CPU and RSS are sampled on the
  `component_container` PIDs, which includes the
  `MediaStreamerNode` source on each side. That is equal on both
  sides (same source, same rate cap) so the delta is clean,
  but the absolute CPU figures include the source's share.

## How to reproduce

Prerequisites: ROS 2 Humble, a workspace in which the
`prism_image_proc` package builds, plus `python3-matplotlib`,
`python3-numpy`, `python3-psutil`, `python3-yaml`, and `ffmpeg`
on `PATH`.

```bash
# 1. Build + source as usual.
colcon build --packages-select prism_image_proc
source install/setup.bash

# 2. Run one operation. Repeat for {resize, crop, colorconvert, chain}.
python3 src/prism-ros/bench/run.py \
  --operation resize \
  --video /path/to/4k.mp4 \
  --duration 120 \
  --output-dir src/prism-ros/bench/results/

# 3. Aggregate.
python3 src/prism-ros/bench/analyze.py \
  --results-dir src/prism-ros/bench/results/ \
  --output src/prism-ros/bench/results/summary.json

# 4. (Optional) Regenerate the site charts.
python3 src/prism-ros/bench/plot.py \
  --summary src/prism-ros/bench/results/summary.json \
  --output-dir src/prism-ros/docs/assets/bench/
```

Each `run.py` invocation writes five files under `--output-dir`:
two per-frame latency CSVs (prism + stock), two resource CSVs,
and one meta JSON that captures the video MD5, prism commit SHA,
hostname, GStreamer version, and start/end timestamps. That JSON
is sufficient to tell apart runs from different hardware or
different builds.
