# Prism

*ROS 2 perception acceleration that picks the right path through your hardware.*

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![GStreamer](https://img.shields.io/badge/GStreamer-1.x-orange.svg)](https://gstreamer.freedesktop.org/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

Hardware-agnostic ROS 2 image-processing accelerator. `prism::ResizeNode` is a **drop-in replacement for `image_proc::ResizeNode`'s resize pipeline** — same parameters, same Image output, with a scaled `CameraInfo` on the matching topic — that auto-detects host accelerators at runtime and chooses the fastest available backend: a GStreamer pipeline with **zero-copy intra-process ingest** on supported GPUs, or a direct `cv::resize` callback when no usable GPU path is present. Egress to the outgoing `sensor_msgs/Image` is a single copy; the claim is *zero-copy ingest, no DDS round-trip*, not end-to-end zero-copy. Input honours `image_transport` (default `raw`; compressed transports decode before GStreamer ingest).

Same resize parameters. Same output topic. Scaled `CameraInfo` on the paired topic. One-line launch-file swap.

```python
ComposableNode(
    package='prism_image_proc',           # was: 'image_proc'
    plugin='prism::ResizeNode', # was: 'image_proc::ResizeNode'
    name='resize',
    parameters=[{'use_scale': False, 'width': 640, 'height': 480}],
)
```

---

## Positioning / Scope

**Prism is not a NITROS replacement.** It targets the segment of the
ROS 2 fleet that Isaac ROS does not cover: Intel iGPU, AMD, older Jetson,
Jetson Orin on Humble, and non-NVIDIA embedded hosts.

**The ROS 2 hardware-acceleration landscape today:**

- **Isaac ROS 4.x** (current branch, `release-4.3`) supports only Jetson Thor
  (T5000/T4000, Blackwell), x86_64 Ampere-or-newer, and DGX Spark — on ROS 2
  Jazzy. No Humble support, no non-NVIDIA support.
  (Source: `nvidia-isaac-ros.github.io/getting_started`.)
- **Isaac ROS 3.2** is the last branch supporting Jetson Orin (Nano/NX/AGX)
  and Xavier on ROS 2 Humble.
- Isaac ROS supports **no non-NVIDIA hardware**.
- **REP-2007** (type adaptation) and **REP-2009** (type negotiation) are the
  standard ROS 2 zero-copy mechanism since Humble. NITROS is NVIDIA's
  implementation. `prism_image_proc` does **not** implement type adaptation —
  the tradeoff is deliberate: hardware portability across Intel/AMD/Jetson
  Orin-on-Humble in exchange for giving up GPU-resident data flow between
  nodes.
- **ros-gst-bridge** (`github.com/BrettRD/ros-gst-bridge`) provides
  `rosimagesrc`/`rosimagesink` GStreamer elements and is adjacent prior art.
  `prism_image_proc` differs by being a ROS 2 component node: one process,
  intra-process composition, a single parameter surface matching
  `image_proc::ResizeNode`, live GStreamer-registry validation of detected
  accelerators, and a dual-mode fallback to `cv::resize` when no usable
  pipeline validates.

**Principled hardware-stack finding.**
Hardware acceleration on Intel requires GStreamer 1.22+, i.e. Ubuntu 24.04 /
Jazzy. On stock Humble (GStreamer 1.20) the `vaapipostproc` element is
present but fails live validation due to a chroma-subsampling regression,
and the Intel iGPU path gracefully falls back to direct mode. The headline
numbers below were captured in exactly that configuration — the win comes
from eliminating the `image_transport` + `CameraSubscriber` DDS round-trip,
not from a GPU kernel. On Jazzy hosts or Jetson hosts, the GPU path runs
and the win is compounded by offloading the resize kernel.

**What Prism is not:**

- Not a type-adaptation framework. Data crosses the egress boundary as a
  normal `sensor_msgs/Image`; publishing is a single copy.
- Not a Jetson Thor or Blackwell target (use Isaac ROS 4.x).
- Not a replacement for CUDA-resident compute graphs; it is a portable
  resize node that wins on the hosts Isaac ROS does not serve.

## Current Benchmark

Measured over a **~170-second sustained A/B run** on an Intel desktop, GStreamer 1.20, direct-mode fallback. 4K (3840×2160) BGR8 source → 640×480 at 10 Hz. 2108 legacy + 2109 accel per-frame latency records; 170 samples of CPU / RSS at 1 Hz. First 10 s dropped as warmup.

| Metric | `image_proc::ResizeNode` | `prism::ResizeNode` | Delta |
|---|---|---|---|
| **Latency — median** | 12.25 ms | **4.80 ms** | **7.45 ms faster** (60.8 %) |
| **Latency — mean** | 14.28 ms | **7.68 ms** | **6.60 ms faster** (46.2 %) |
| **Latency — p95** | 21.84 ms | **14.46 ms** | **7.38 ms** |
| **Latency — p99** | 29.76 ms | **24.38 ms** | **5.38 ms** |
| **Latency — stdev** | 4.96 ms | 5.27 ms | — |
| **Frame rate** | 9.99 fps (σ 0.08) | 10.00 fps (σ 0.19) | steady |
| **Container CPU — mean** | 64.99 % | **55.54 %** | **9.4 pp** |
| **Container CPU — p95** | 74.65 % | **64.00 %** | **10.65 pp** |
| **Container RSS — mean** | 810.52 MB | **747.56 MB** | **62.96 MB** |

<p align="center">
  <img src="docs/assets/latency_distribution.svg" alt="Latency distribution: min, p5, median, p95, max" width="720"/>
</p>

<p align="center">
  <img src="docs/assets/latency_timeseries.svg" alt="Per-frame latency time series, 170s run" width="720"/>
</p>

<p align="center">
  <img src="docs/assets/resource_usage.svg" alt="Resource comparison: latency / CPU / RSS" width="720"/>
</p>

On this host the GPU path is skipped because GStreamer 1.20's `vaapipostproc` has a chroma-loss bug (Y plane only). The node detects it, falls back to direct mode, and **still wins by 7.4 ms at the median** because it eliminates the `image_transport` + `CameraSubscriber` DDS round-trip entirely. On hosts with a working GPU (NVIDIA Jetson `nvvideoconvert`, or Intel on GStreamer 1.22+ with `vapostproc`), the accelerated path additionally offloads the resize kernel to dedicated silicon and frees CPU for SLAM / perception / planning.

### Jetson (Orin) — pending

The GPU path is implemented for Jetson via `nvvideoconvert` with NVMM
buffer-types, and the fallback chain promotes it first when
`/dev/nvhost-*` and `/dev/nvmap` probe positive. A 60-second A/B
capture on Orin (Nano / NX / AGX) is pending — reproduce with:

```bash
ros2 launch prism_image_proc A_B_comparison.launch.py \
  video_path:=/path/to/4k_video.mp4
```

Numbers and a short commentary will be added to this section once the
capture is complete.

## Architecture

```
                Startup
                   │
         ┌─────────▼──────────┐
         │  HardwareDetector  │   probes /dev, runs gst-inspect
         └─────────┬──────────┘
                   │
        ┌──────────▼──────────┐
        │  PipelineFactory    │   validates gst elements exist & work
        └──────────┬──────────┘
                   │
      ┌────────────┴────────────┐
      │                         │
  ┌───▼──────────────┐  ┌───────▼────────────────┐
  │  GPU mode        │  │  Direct mode           │
  │  appsrc → GPU    │  │  cv::resize in the     │
  │  → appsink       │  │  subscriber callback   │
  │  zero-copy ingest│  │  no GStreamer at all   │
  └──────────────────┘  └────────────────────────┘
```

### Fallback chain

| Priority | Platform | Detection | Processing |
|---|---|---|---|
| 1 | NVIDIA Jetson | `/dev/nvhost-*`, `/dev/nvmap` | GStreamer `nvvideoconvert` (CUDA / NVMM) |
| 2 | Intel VA-API | `/dev/dri/renderD*` + `vapostproc` in registry | GStreamer `vapostproc` |
| 3 | CPU (always) | — | Direct `cv::resize` in callback |

The fallback is **live-validated** against the GStreamer plugin registry — an accelerator that's present but broken (like the `vaapipostproc` chroma bug) is skipped, not attempted.

## Quick Start

```bash
# Clone into a colcon workspace
cd ~/ros2_ws/src
git clone https://github.com/sohams25/prism-ros.git prism_image_proc

# Install deps
sudo apt install ros-humble-image-proc gstreamer1.0-vaapi \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
pip install flask psutil

# Build
cd ~/ros2_ws
colcon build --packages-up-to prism_image_proc
source install/setup.bash

# Run the A/B stress test (any 4K mp4)
ros2 launch prism_image_proc A_B_comparison.launch.py \
  video_path:=/path/to/4k_video.mp4
```

The launch file runs two `component_container` processes (one for each pipeline) so CPU and RAM are measured independently. It also auto-cleans stale Fast-DDS SHM locks and orphan processes from prior runs, and disables the Fast-DDS SHM transport in favor of UDPv4 (see `config/fastdds_no_shm.xml`) — this is a known-stable configuration that avoids `RTPS_TRANSPORT_SHM` init errors on machines with leftover `/dev/shm/fastrtps_*` files.

### Web Dashboard

In a second terminal:

```bash
ros2 run prism_image_proc visualize_demo.py
```

Open <http://localhost:8080>. The dashboard shows:

- **Hero delta card** — how much faster the accelerated path is right now, with a live sparkline
- **Side-by-side MJPEG streams** — 640×480 at 10 fps, with `Content-Length`-framed multipart so the stream is stable indefinitely (no truncation on JPEG/boundary collisions)
- **Per-container CPU%, RAM MB, FPS, latency** — stale-PID safe (dead zombie containers are not measured)
- **Auto-detected hardware badge** — "NVIDIA Jetson", "Intel VA-API", or "CPU fallback · direct cv::resize"

## Parameters

### `prism::ResizeNode`

Matches the `image_proc::ResizeNode` interface.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_scale` | bool | `false` | Scale by factor instead of absolute size |
| `scale_width`, `scale_height` | double | `1.0` | Scale factors when `use_scale=true` |
| `width`, `height` | int | `640`, `480` | Output size when `use_scale=false` |
| `input_topic` | string | `/camera/image_raw` | Source topic |
| `output_topic` | string | `/camera/image_processed` | Destination topic |
| `source_width`, `source_height` | int | `3840`, `2160` | Source caps (GPU mode only) |
| `input_transport` | string | `raw` | `image_transport` name (`raw`, `compressed`, `theora`, …). `raw` keeps the UniquePtr zero-copy hot path |
| `publish_camera_info` | bool | `true` | Publish a scaled `CameraInfo` alongside the processed image |
| `action` | string | `resize` | Action chain, comma- or pipe-separated (e.g. `crop,resize,colorconvert`) |
| `target_encoding` | string | `bgr8` | Only read when chain contains `colorconvert`. One of `bgr8`, `rgb8`, `mono8` |
| `crop_x`, `crop_y`, `crop_width`, `crop_height` | int | `0` | Only read when chain contains `crop`. Pixel offsets into the source |
| `flip_method` | string | `none` | Only read when chain contains `flip`. `none`, `horizontal`, or `vertical` |

### `prism::MediaStreamerNode`

Video-file publisher used by the A/B launch.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `video_path` | string | `/tmp/test_video.mp4` | Input video file |
| `loop` | bool | `true` | Restart on EOF |
| `max_fps` | double | `10.0` | Publish rate cap |
| `image_topic` | string | `/camera/image_raw` | Image topic |
| `info_topic` | string | `/camera/camera_info` | CameraInfo topic |

## Components

```
prism_image_proc
  prism::ResizeNode         # drop-in image_proc::ResizeNode (resize) + chainable crop/flip/colorconvert
  prism::MediaStreamerNode  # video-file publisher (bgr8)
  prism::Synthetic4kPubNode # synthetic 4K test source
```

Load into any `rclcpp_components::ComponentContainer` with `use_intra_process_comms: true`.

## Project Structure

```
prism_image_proc/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── config/
│   ├── demo_params.yaml
│   └── fastdds_no_shm.xml          # UDPv4-only Fast-DDS profile
├── include/prism_image_proc/          # 5 public headers
├── launch/
│   ├── A_B_comparison.launch.py    # two-container A/B stress test
│   └── prism_image_proc_demo.launch.py
├── scripts/
│   ├── dashboard.html              # served by visualize_demo.py
│   ├── visualize_demo.py           # Flask web dashboard
│   ├── latency_tracker.py          # per-frame latency logger
│   ├── cpu_monitor.py
│   ├── synthetic_4k_pub.py
│   └── generate_assets.py
└── src/                             # 6 C++ sources
```

## Dependencies

- **ROS 2 Humble** on Ubuntu 22.04
- **GStreamer 1.x** — `libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`
- **OpenCV 4.x** via `cv_bridge`
- **Python**: `flask`, `psutil` (dashboard only)
- **Optional**: `gstreamer1.0-vaapi` (Intel), `ros-humble-image-proc` (A/B comparison)

## Roadmap

High-impact items, ranked by effort-to-reward:

- [ ] **GStreamer 1.22+ `vapostproc` wire-up** — exits direct-mode fallback on modern Intel stacks; ~10× vs `cv::resize` at 4K
- [ ] **Adaptive live fallback** — EWMA p95 latency tracker; swap to direct-mode automatically on GPU regression
- [ ] **ROS 2 Lifecycle node** — runtime reconfigure of target size / action without process restart
- [ ] **Jetson NVMM dma-buf zero-copy** — real DMA buffers end-to-end instead of host-BGR ingestion
- [ ] **Multi-stream / N-camera fanout** — single pipeline, batched via `nvstreammux`
- [ ] **Dropped-frame metric** — `appsrc max-buffers` overflow currently silent; expose as a topic
- [ ] **H.264/H.265 compressed-ingest branch** — keep remote-cam frames on GPU from network → resize
- [ ] **CI integration tests**

## Migrating from GstAdaptNode

This repository was previously published as `GstAdaptNode` / `gst_adapt_node`
and is now `Prism` / `prism_image_proc`. The rename covers the repo, the
ROS 2 package, the C++ namespace, and the class identity:

| What | Before | After |
|---|---|---|
| GitHub repo | `sohams25/GstAdaptNode` | `sohams25/prism-ros` |
| ROS 2 package | `gst_adapt_node` | `prism_image_proc` |
| C++ namespace | `gst_adapt_node::` | `prism::` |
| Chainable base class | `gst_adapt_node::ResizeNode` | `prism::ImageProcNode` |
| Drop-in resize plugin | `gst_adapt_node::ResizeNode` | `prism::ResizeNode` (thin wrapper) |
| Include guards | `GST_ADAPT_NODE__*_HPP_` | `PRISM_IMAGE_PROC__*_HPP_` |

Action items for downstream consumers:

1. **Update the remote URL after the GitHub rename**:
   ```bash
   git remote set-url origin git@github.com:sohams25/prism-ros.git
   ```
2. **Package name**: `rosdep install`, `colcon build --packages-select`, and
   `source install/setup.bash` now use **`prism_image_proc`**.
3. **Includes**: `#include "gst_adapt_node/gst_adapt_node.hpp"` →
   `#include "prism_image_proc/image_proc_node.hpp"`.
4. **Class references**: replace `gst_adapt_node::ResizeNode` with either
   `prism::ImageProcNode` (the general chainable base) or `prism::ResizeNode`
   (the thin wrapper that pins `action="resize"` by default — preserves the
   one-line launch-file swap against `image_proc::ResizeNode`).
5. **New component types** now also available for drop-in use:
   `prism::CropNode` (pins `action="crop"`) and `prism::ColorConvertNode`
   (pins `action="colorconvert"`). All three wrappers accept the full
   parameter surface of `prism::ImageProcNode`.
6. **Launch files**: `package='prism_image_proc'` and a plugin string of
   your choice from the four registered components.

## License

Apache-2.0. See [LICENSE](LICENSE).
