# Prism
*ROS 2 perception acceleration that picks the right path through your hardware.*

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

<details>
<summary>Expand full parameter reference</summary>

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

</details>

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

<details>
<summary>Expand internals</summary>

### Zero-copy ingest, single-copy egress
Input arrives via intra-process `UniquePtr` delivery (`raw` `image_transport`) or through an `image_transport::Subscriber` plugin for compressed / theora / ffmpeg encodings. On the `raw` path the pointer is moved — no copy — into a `gst_buffer_new_wrapped_full` that feeds the GStreamer pipeline. Egress (copying the processed frame into a fresh `sensor_msgs/Image` for publication) is a single copy. The claim is zero-copy ingest and no DDS round-trip, not end-to-end zero-copy.

### CameraInfo transforms
Each registered action carries a `CameraInfoTransform` functor that scales the K / P intrinsics and ROI to match the image output for that action. Chain composition multiplies the transforms in order, so downstream consumers of the paired `CameraInfo` topic see intrinsics that match the processed image — no matter how long the action chain.

### Runtime reconfiguration
Parameters can be changed after the node is running via `add_on_set_parameters_callback`. The callback synchronously validates the proposed action chain against the action registry and parameter constraints; on success, the pipeline rebuild is deferred to a one-shot timer so it doesn't run inside the param callback. Validation failures reject the parameter change without touching the running pipeline.

### GStreamer 1.20 Intel caveat

On stock ROS 2 Humble (GStreamer 1.20), the `vaapipostproc` element is present but fails live validation due to a chroma-subsampling regression; the Intel iGPU path falls back to direct `cv::resize` mode. GStreamer 1.22+ (Ubuntu 24.04 / Jazzy) is required for the GPU resize kernel on Intel. NVIDIA Jetson NVMM and the CPU direct path are unaffected.

</details>

## Benchmarks

A/B captures against stock ROS 2 Humble image processing on an Intel
desktop. Source is a 3840×2160 H.264 clip streamed at 10 Hz via
`prism::MediaStreamerNode`. Both sides run in component containers;
latency is per-frame from publish stamp to subscriber receive.
Methodology, per-run CSVs, and reproduction commands live in
[`bench/`](bench/); the commit-pinned summary is
[`bench/results/summary.md`](bench/results/summary.md).

### Per-kernel latency (3840×2160 @ 10 Hz)

A/B against the closest upstream `image_proc` C++ component for each
Prism action. Both sides sustain the 10 Hz source.

| Action   | Stock plugin                       | Prism median (ms) | Stock median (ms) | Δ        |
| -------- | ---------------------------------- | ----------------: | ----------------: | -------: |
| `resize` | `image_proc::ResizeNode`           |              5.67 |             13.61 |  −58.3 % |
| `crop`   | `image_proc::CropDecimateNode`     |             13.40 |             28.80 |  −53.5 % |

Both sides are C++ component-container nodes. The delta is kernel
plus intra-process delivery cost.

### Pipeline architecture (3840×2160 @ 10 Hz)

A three-stage `crop → resize → colorconvert` chain. Stock side runs
three separate `image_proc` nodes DDS-piped together; Prism runs the
full chain inside one `prism::ImageProcNode` with intra-process
composition.

| Pipeline                       | Prism median (ms) | Stock median (ms) | Δ        |
| ------------------------------ | ----------------: | ----------------: | -------: |
| `crop → resize → colorconvert` |             12.90 |             75.47 |  −82.9 % |

The delta here is dominated by Prism's single-pipeline architecture
avoiding two intermediate DDS round-trips on the stock side, not by
per-kernel speedup. See the chain-composition caveat in
[`bench/METHODOLOGY.md`](bench/METHODOLOGY.md).

### Stock Python subscriber throughput ceiling (3840×2160 @ 10 Hz)

`image_proc` ships no dedicated colorconvert node, so the closest
available stock-side reference is a Python `rclpy` subscriber that
performs the conversion (`cv_bridge` where it works, NumPy fallback
where it does not — see methodology). At 4K BGR8 @ 10 Hz the source
publishes ~240 MB/s; the Python subscriber cannot drain it. Frames
queue in the receive buffer with unbounded latency.

| Side                                          | Realised fps | Median latency (ms) |
| --------------------------------------------- | -----------: | ------------------: |
| Prism (`prism::ColorConvertNode`, C++)        |        10.01 |               12.57 |
| Stock (`rclpy` Image subscriber, NumPy back)  |         0.85 |             2173.09 |

This is **not** a kernel-level efficiency comparison. It is a
structural finding about the ROS 2 Python subscriber path on large
`sensor_msgs/Image` messages: GIL plus `rclpy` deserialization plus
per-callback memory copies cannot keep up with 4K @ 10 Hz on a single
core, regardless of how fast the conversion arithmetic itself runs.
Prism's C++ component path side-steps the bottleneck entirely. A C++
stock-side colorconvert reference would close this gap; `image_proc`
does not ship one. Methodology, the `cv_bridge` segfault context, and
the abandoned 1080p re-measurement are recorded in
[`bench/METHODOLOGY.md`](bench/METHODOLOGY.md).

### Reproducing

```bash
python3 bench/run.py --operation resize --video /path/to/4k.mp4 \
  --duration 120 --output-dir bench/results/
python3 bench/analyze.py --results-dir bench/results/ \
  --output bench/results/summary.json
```

Repeat with `--operation {crop,colorconvert,chain}`.

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

<details>
<summary>Contributing & Roadmap</summary>

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

</details>

## License

Apache-2.0. See [LICENSE](LICENSE).

Release history: [CHANGELOG.md](CHANGELOG.md).
