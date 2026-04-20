# GstAdaptNode

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![GStreamer](https://img.shields.io/badge/GStreamer-1.x-orange.svg)](https://gstreamer.freedesktop.org/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

Hardware-agnostic ROS 2 image-processing accelerator. `gst_adapt_node::ResizeNode` is a **drop-in replacement for `image_proc::ResizeNode`** that auto-detects host accelerators at runtime and chooses the fastest available backend — a zero-copy GStreamer pipeline on supported GPUs, or a direct `cv::resize` callback when no usable GPU path is present.

Same parameters. Same topics. One-line launch-file swap.

```python
ComposableNode(
    package='gst_adapt_node',           # was: 'image_proc'
    plugin='gst_adapt_node::ResizeNode', # was: 'image_proc::ResizeNode'
    name='resize',
    parameters=[{'use_scale': False, 'width': 640, 'height': 480}],
)
```

---

## Current Benchmark

Measured over a **~170-second sustained A/B run** on an Intel desktop, GStreamer 1.20, direct-mode fallback. 4K (3840×2160) BGR8 source → 640×480 at 10 Hz. 2108 legacy + 2109 accel per-frame latency records; 170 samples of CPU / RSS at 1 Hz. First 10 s dropped as warmup.

| Metric | `image_proc::ResizeNode` | `gst_adapt_node::ResizeNode` | Delta |
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
  │  zero-copy       │  │  no GStreamer at all   │
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
git clone --recursive https://github.com/sohams25/GstAdaptNode.git gst_adapt_node

# Install deps
sudo apt install ros-humble-image-proc gstreamer1.0-vaapi \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
pip install flask psutil

# Build
cd ~/ros2_ws
colcon build --packages-up-to gst_adapt_node
source install/setup.bash

# Run the A/B stress test (any 4K mp4)
ros2 launch gst_adapt_node A_B_comparison.launch.py \
  video_path:=/path/to/4k_video.mp4
```

The launch file runs two `component_container` processes (one for each pipeline) so CPU and RAM are measured independently. It also auto-cleans stale Fast-DDS SHM locks and orphan processes from prior runs, and disables the Fast-DDS SHM transport in favor of UDPv4 (see `config/fastdds_no_shm.xml`) — this is a known-stable configuration that avoids `RTPS_TRANSPORT_SHM` init errors on machines with leftover `/dev/shm/fastrtps_*` files.

### Web Dashboard

In a second terminal:

```bash
ros2 run gst_adapt_node visualize_demo.py
```

Open <http://localhost:8080>. The dashboard shows:

- **Hero delta card** — how much faster the accelerated path is right now, with a live sparkline
- **Side-by-side MJPEG streams** — 640×480 at 10 fps, with `Content-Length`-framed multipart so the stream is stable indefinitely (no truncation on JPEG/boundary collisions)
- **Per-container CPU%, RAM MB, FPS, latency** — stale-PID safe (dead zombie containers are not measured)
- **Auto-detected hardware badge** — "NVIDIA Jetson", "Intel VA-API", or "CPU fallback · direct cv::resize"

## Parameters

### `gst_adapt_node::ResizeNode`

Matches the `image_proc::ResizeNode` interface.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_scale` | bool | `false` | Scale by factor instead of absolute size |
| `scale_width`, `scale_height` | double | `1.0` | Scale factors when `use_scale=true` |
| `width`, `height` | int | `640`, `480` | Output size when `use_scale=false` |
| `input_topic` | string | `/camera/image_raw` | Source topic |
| `output_topic` | string | `/camera/image_processed` | Destination topic |
| `source_width`, `source_height` | int | `3840`, `2160` | Source caps (GPU mode only) |

### `gst_adapt_node::MediaStreamerNode`

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
gst_adapt_node
  gst_adapt_node::ResizeNode         # drop-in image_proc::ResizeNode replacement
  gst_adapt_node::MediaStreamerNode  # video-file publisher (bgr8)
  gst_adapt_node::Synthetic4kPubNode # synthetic 4K test source
```

Load into any `rclcpp_components::ComponentContainer` with `use_intra_process_comms: true`.

## Project Structure

```
gst_adapt_node/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── config/
│   ├── demo_params.yaml
│   └── fastdds_no_shm.xml          # UDPv4-only Fast-DDS profile
├── include/gst_adapt_node/          # 5 public headers
├── launch/
│   ├── A_B_comparison.launch.py    # two-container A/B stress test
│   └── gst_adapt_demo.launch.py
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

## License

Apache-2.0. See [LICENSE](LICENSE).
