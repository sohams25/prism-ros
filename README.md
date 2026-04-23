# Prism
*ROS 2 perception acceleration that picks the right path through your hardware.*

[![CI](https://github.com/sohams25/prism-ros/actions/workflows/ci.yml/badge.svg)](https://github.com/sohams25/prism-ros/actions/workflows/ci.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![GStreamer](https://img.shields.io/badge/GStreamer-1.x-orange.svg)](https://gstreamer.freedesktop.org/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

Hardware-agnostic ROS 2 image-processing accelerator. `prism::ResizeNode` is a **drop-in replacement for `image_proc::ResizeNode`'s resize pipeline** — same parameters, same Image output, with a scaled `CameraInfo` on the matching topic — that auto-detects host accelerators at runtime and chooses the fastest available backend: a GStreamer pipeline with **zero-copy intra-process ingest** on supported GPUs, or a direct `cv::resize` callback when no usable GPU path is present. Egress to the outgoing `sensor_msgs/Image` is a single copy; the claim is *zero-copy ingest, no DDS round-trip*, not end-to-end zero-copy. Input honours `image_transport` (default `raw`; compressed transports decode before GStreamer ingest).

Same resize parameters. Same output topic. Scaled `CameraInfo` on the paired topic. One-line launch-file swap.

[**Documentation & benchmarks →**](https://sohams25.github.io/prism-ros/)

---

## Quick start

### Prerequisites
ROS 2 Humble on Ubuntu 22.04. GStreamer 1.20+.

### Install
```bash
cd ~/ros2_ws/src
git clone https://github.com/sohams25/prism-ros.git prism_image_proc
sudo apt install \
  ros-humble-image-transport ros-humble-image-proc \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-vaapi
cd ~/ros2_ws
colcon build --packages-select prism_image_proc
source install/setup.bash
```

### Run the demo
The demo launch file spins up a single `prism::ResizeNode` inside a component container, subscribed to `/camera/image_raw` and publishing to `/camera/image_processed`. You supply the image source — point a camera driver at `/camera/image_raw`, or run `prism::MediaStreamerNode` / `prism::Synthetic4kPubNode` in a second terminal.

```bash
ros2 launch prism_image_proc prism_image_proc_demo.launch.py
```

The resize node logs the selected backend (GPU or direct mode) and publishes processed 640×480 frames on `/camera/image_processed` with scaled `CameraInfo` on the paired topic.

For the full A/B stress test against `image_proc::ResizeNode`, see [A/B comparison launch](#ab-comparison-launch) below.

## Usage

### Drop-in replacement for image_proc::ResizeNode

```python
ComposableNode(
    package='prism_image_proc',           # was: 'image_proc'
    plugin='prism::ResizeNode',           # was: 'image_proc::ResizeNode'
    name='resize',
    parameters=[{'use_scale': False, 'width': 640, 'height': 480}],
)
```

Same resize parameters, same output topic, same `sensor_msgs/Image` on the output. Prism also publishes a scaled `CameraInfo` on the paired topic — check the `publish_camera_info` parameter.

### Action chaining

The chainable base — `prism::ImageProcNode` — accepts a comma- or pipe-separated action chain. Each action has per-backend GStreamer fragment builders (CPU / Intel VA-API / Jetson NVMM) and a corresponding CameraInfo transform so intrinsics track the image.

```python
ComposableNode(
    package='prism_image_proc',
    plugin='prism::ImageProcNode',
    name='preprocess',
    parameters=[{
        'action': 'crop,resize,colorconvert',
        'crop_x': 320, 'crop_y': 180,
        'crop_width': 1280, 'crop_height': 720,
        'width': 640, 'height': 360,
        'target_encoding': 'rgb8',
    }],
)
```

Supported actions: `resize`, `crop`, `flip`, `colorconvert`.

### A/B comparison launch

`launch/A_B_comparison.launch.py` runs `image_proc::ResizeNode` and `prism::ResizeNode` side-by-side in separate component containers over the same source video, for latency / CPU / RSS comparison.

```bash
ros2 launch prism_image_proc A_B_comparison.launch.py \
  video_path:=/path/to/4k_video.mp4
```

## Components

Registered ROS 2 components:

| Class | Purpose |
| --- | --- |
| `prism::ImageProcNode` | Chainable base. Configurable action chain; use directly when you need crop+resize+colorconvert composed. |
| `prism::ResizeNode` | Thin wrapper pinning `action="resize"`. **Drop-in replacement for `image_proc::ResizeNode`.** |
| `prism::CropNode` | Thin wrapper pinning `action="crop"`. |
| `prism::ColorConvertNode` | Thin wrapper pinning `action="colorconvert"`. Target `bgr8` / `rgb8` / `mono8`. |

Test / demo helpers:

| Class | Purpose |
| --- | --- |
| `prism::MediaStreamerNode` | Video-file publisher (used by the A/B launch). |
| `prism::Synthetic4kPubNode` | Synthetic 4K test source. |

Load any of the above into an `rclcpp_components::ComponentContainer` with `use_intra_process_comms: true` to get the zero-copy ingest path.

## Parameters

### Core resize

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_scale` | bool | `false` | Scale by factor instead of absolute size |
| `scale_width`, `scale_height` | double | `1.0` | Scale factors when `use_scale=true` |
| `width`, `height` | int | `640`, `480` | Output size when `use_scale=false` |
| `input_topic` | string | `/camera/image_raw` | Source topic |
| `output_topic` | string | `/camera/image_processed` | Destination topic |

### Action chain

| Parameter | Type | Default | Description |
|---|---|---|---|
| `action` | string | `resize` | Action chain, comma- or pipe-separated (e.g. `crop,resize,colorconvert`) |
| `target_encoding` | string | `bgr8` | Only read when chain contains `colorconvert`. One of `bgr8`, `rgb8`, `mono8` |
| `crop_x`, `crop_y`, `crop_width`, `crop_height` | int | `0` | Only read when chain contains `crop`. Pixel offsets into the source |
| `flip_method` | string | `none` | Only read when chain contains `flip`. `none`, `horizontal`, or `vertical` |

### Transport

| Parameter | Type | Default | Description |
|---|---|---|---|
| `input_transport` | string | `raw` | `image_transport` name (`raw`, `compressed`, `theora`, …). `raw` keeps the UniquePtr zero-copy hot path |
| `publish_camera_info` | bool | `true` | Publish a scaled `CameraInfo` alongside the processed image |
| `source_width`, `source_height` | int | `3840`, `2160` | Source caps (GPU mode only) |

### Media streamer parameters

`prism::MediaStreamerNode` — video-file publisher used by the A/B launch.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `video_path` | string | `/tmp/test_video.mp4` | Input video file |
| `loop` | bool | `true` | Restart on EOF |
| `max_fps` | double | `10.0` | Publish rate cap |
| `image_topic` | string | `/camera/image_raw` | Image topic |
| `info_topic` | string | `/camera/camera_info` | CameraInfo topic |

## Architecture

At startup `HardwareDetector` probes `/dev` for accelerator devices and runs `gst-inspect` against the GStreamer registry to confirm the matching elements load. `PipelineFactory` then builds a backend-specific pipeline fragment for each action in the chain, validates the complete pipeline live, and hands it to `ImageProcNode`. If no GPU pipeline validates, the node falls back to a direct `cv::resize` in the subscriber callback — no GStreamer involvement at all.

<!-- Do NOT add %%{init: {'theme': ...}}%% — GitHub's native Mermaid
     renderer auto-adapts to dark/light mode only when no theme is
     forced. Forcing a theme breaks the opposite mode. -->
```mermaid
flowchart TD
  Start([Startup]) --> HD[HardwareDetector]
  HD --> PF[PipelineFactory]
  PF --> V{gst-inspect validates?}
  V -->|Jetson path OK| NV[nvvideoconvert / NVMM]
  V -->|Intel path OK| VA[vapostproc / VA-API]
  V -->|none valid| CV[cv::resize direct mode]
  NV --> Out([sensor_msgs/Image out])
  VA --> Out
  CV --> Out
```

### Fallback chain

| Priority | Platform | Detection | Processing |
|---|---|---|---|
| 1 | NVIDIA Jetson | `/dev/nvhost-*`, `/dev/nvmap` | GStreamer `nvvideoconvert` (CUDA / NVMM) |
| 2 | Intel VA-API | `/dev/dri/renderD*` + `vapostproc` in registry | GStreamer `vapostproc` |
| 3 | CPU (always) | — | Direct `cv::resize` in callback |

The fallback is **live-validated** against the GStreamer plugin registry — an accelerator that's present but broken (for example, the `vaapipostproc` chroma bug on GStreamer 1.20) is skipped, not attempted.

## How it works

### Zero-copy ingest, single-copy egress
Input arrives via intra-process `UniquePtr` delivery (`raw` `image_transport`) or through an `image_transport::Subscriber` plugin for compressed / theora / ffmpeg encodings. On the `raw` path the pointer is moved — no copy — into a `gst_buffer_new_wrapped_full` that feeds the GStreamer pipeline. Egress (copying the processed frame into a fresh `sensor_msgs/Image` for publication) is a single copy. The claim is zero-copy ingest and no DDS round-trip, not end-to-end zero-copy.

### CameraInfo transforms
Each registered action carries a `CameraInfoTransform` functor that scales the K / P intrinsics and ROI to match the image output for that action. Chain composition multiplies the transforms in order, so downstream consumers of the paired `CameraInfo` topic see intrinsics that match the processed image — no matter how long the action chain.

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
and the Intel iGPU path gracefully falls back to direct mode. In that
configuration the win comes from eliminating the `image_transport` +
`CameraSubscriber` DDS round-trip, not from a GPU kernel. On Jazzy hosts
or Jetson hosts, the GPU path runs and the win is compounded by offloading
the resize kernel.

**What Prism is not:**

- Not a type-adaptation framework. Data crosses the egress boundary as a
  normal `sensor_msgs/Image`; publishing is a single copy.
- Not a Jetson Thor or Blackwell target (use Isaac ROS 4.x).
- Not a replacement for CUDA-resident compute graphs; it is a portable
  resize node that wins on the hosts Isaac ROS does not serve.

## Benchmarks

Benchmarks are being re-measured against the current chainable
architecture (action registry, per-action CameraInfo transforms,
and image_transport plumbing). The prior numbers predate those
layers and no longer represent the production path.

Fresh methodology and A/B results — Intel desktop first, then
Jetson Orin — will ship with the next release. Track progress
via [GitHub Releases](https://github.com/sohams25/prism-ros/releases)
or the [project site](https://sohams25.github.io/prism-ros/).

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

## Development

### Build and test
```bash
colcon build --packages-select prism_image_proc
colcon test --packages-select prism_image_proc --ctest-args -R test_direct_resize
colcon test-result --verbose
```

### Continuous integration
CI runs on the `ros:humble` container on every push and pull
request: installs the runtime deps, builds the package, runs
the gtest suite. See
[`.github/workflows/ci.yml`](.github/workflows/ci.yml).

### Contributing
See [CONTRIBUTING.md](CONTRIBUTING.md). Prism uses
[Conventional Commits](https://www.conventionalcommits.org/)
going forward.

### Roadmap

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

Release history: [CHANGELOG.md](CHANGELOG.md).
