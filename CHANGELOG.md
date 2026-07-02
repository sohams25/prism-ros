# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-07-02

Production-hardening pass. See [`docs/PRODUCTION_READINESS.md`](docs/PRODUCTION_READINESS.md)
for the full audit, fixes, and remaining roadmap.

### Fixed

- **CameraInfo self-echo** — with default topics, the input and output
  CameraInfo both derived to `/camera/camera_info`, so the node subscribed to
  its own transformed publication (for `crop`, each echo re-shifted the
  principal point). A derived collision now publishes the output CameraInfo
  under the output image topic (`/camera/image_processed/camera_info` by
  default); explicit `camera_info_output_topic` values are honored unchanged.
- **`Synthetic4kPubNode` topics** — published on private `~/image_raw` /
  `~/camera_info` while its log line and the README documented
  `/camera/image_raw`; it now publishes the documented absolute topics.
- **GPU-path egress `frame_id`** — the appsink handler stamped every published
  frame `frame_id="media_frame"`; it now echoes the ingest frame's `frame_id`
  (direct mode already preserved the full input header), keeping TF lookups
  intact when a GPU backend engages.

- **Heap overflow** in the I420/YV12 appsink egress handler — the V-plane copy
  wrote past the assembly `cv::Mat`. Rewrote chroma packing to stay within the
  `h*3/2*w` budget and added an even-dimension guard.
- **Silent corruption / out-of-bounds read** on non-`bgr8` input — `on_image`
  and `on_image_direct` now validate encoding, step, and data size and
  warn-and-drop malformed frames.
- **Node crash via `ros2 param set`** — invalid `target_encoding` / `crop_*` /
  `flip_method` values are now rejected synchronously (dry-run build), and the
  deferred rebuild degrades to direct mode on error instead of throwing out of
  the timer callback.
- **`use_scale` integer overflow** when narrowing scaled dimensions to `int`.
- **`rosdep install` failure** — removed the spurious non-existent `gst_bridge`
  dependency and declared the actual GStreamer dev (build) + runtime plugin deps.
- **`GstElementFactory` reference leak** on every registry probe (≈12 sites) via
  a new `factory_exists()` helper.
- **GStreamer teardown ordering** (NULL-state before element unref) and a
  `gst_parse_launch` partial-parse pipeline leak.
- **Stereo `CameraInfo`** — `resize` now scales the projection `Tx`/`Ty` terms.
- **Cross-distro `cv_bridge` include** — use `cv_bridge.hpp` where present
  (Jazzy) and fall back to `cv_bridge.h` (Humble), so the package builds on both;
  ROS 2 Jazzy build + tests are now a blocking CI gate alongside Humble.

### Added

- Fallback to direct mode (and bounded GPU-pipeline recovery on bus ERROR/EOS)
  instead of a silent, output-less zombie node on pipeline failure.
- `SensorDataQoS` default for image / CameraInfo I/O (camera-driver
  interoperability) with a `reliable_qos` parameter escape hatch.
- `ParameterDescriptor` ranges on dimension / scale / crop parameters.
- Distinct default node names per wrapper; `ament_export_*` so downstream
  packages can link `prism_core`.
- Test suite expanded 11 → 23 unit cases plus a `launch_testing` integration
  test that drives a running node end-to-end.
- CI rebuilt as a real gate (`rosdep` deps, `--return-code-on-test-failure`,
  least-privilege token) with ASan/UBSan, advisory lint, Jazzy, and a native
  arm64 (Jetson-target) build+test job.

### Changed

- `MediaStreamerNode` now throws on video-open failure (visible component-load
  failure) rather than constructing a silent idle node.
- Real maintainer contact in `package.xml`; GitHub Actions pinned to commit
  SHAs (supply-chain hardening).

### Removed

- Dead dashboard-era tooling (`scripts/dashboard.html`, `visualize_demo.py`,
  `cpu_monitor.py`, and the Python `synthetic_4k_pub.py` superseded by the
  C++ `prism::Synthetic4kPubNode`) and their install rules — ~1,200 lines
  with no remaining references.

## [0.1.0] - 2026-04-27

Initial public release of `prism_image_proc` — a hardware-agnostic
ROS 2 image-processing accelerator with runtime-dispatched GStreamer
pipelines (Intel VA-API, NVIDIA Jetson NVMM) and a `cv::resize`
direct-mode fallback. Drop-in replacement for
`image_proc::ResizeNode`'s resize pipeline.

### Added

- `prism::ImageProcNode`: chainable image-processing base node with
  configurable action chain (comma- or pipe-separated).
- `prism::ResizeNode` / `prism::CropNode` / `prism::ColorConvertNode`:
  thin wrappers pinning the `action` parameter via `NodeOptions`
  overrides at construction time.
- Action registry with per-backend fragment builders (CPU / Intel
  VA-API / Jetson NVMM) for resize, crop, flip, and colorconvert.
- `CameraInfoTransform` functors per action; K / P intrinsics and
  ROI track the image output through any chain composition.
- `image_transport` input support: `raw` keeps the intra-process
  `UniquePtr` zero-copy ingest path; other transports decode via
  `image_transport::Subscriber` before GStreamer ingest.
- Runtime reconfiguration via `add_on_set_parameters_callback`
  with deferred-timer pipeline rebuild.
- `prism::MediaStreamerNode` (video-file publisher) and
  `prism::Synthetic4kPubNode` (synthetic 4K test source) as
  test / demo helpers.
- `launch/A_B_comparison.launch.py`: two-container side-by-side
  stress test against `image_proc::ResizeNode`.
- 11-case gtest suite (`test_pipeline_factory`) covering the action
  chain parser, each transform, chain composition, and pipeline
  build strings.
- GitHub Actions CI on the `ros:humble` container: build + gtest
  on every push and PR.
- Project site at https://sohams25.github.io/prism-ros/ with
  dark-mode-first developer-tool styling (Tokyo Night palette,
  prism-refraction hero motif).

### Known limitations

- GStreamer 1.20 `vaapipostproc` has a chroma-subsampling regression;
  the Intel iGPU path is detected but falls back to direct mode on
  stock Humble. GStreamer 1.22+ (Ubuntu 24.04 / Jazzy) required for
  the GPU resize kernel on Intel.
- Jetson Orin A/B captures are in [`bench/results/orin_simple_summary.md`](bench/results/orin_simple_summary.md);
  the `colorconvert` row's Δ% is intentionally omitted (em-dash) — the Python NumPy
  stock baseline is a throughput ceiling, not a kernel comparison — and the Round-3
  contention finding (bench-harness CPU saturation, not BGR-adapter dominance) is
  documented alongside the per-percentile data.
- `image_proc::ResizeNode` "drop-in" replacement covers the resize
  pipeline — rectification is out of scope for this release.

[Unreleased]: https://github.com/sohams25/prism-ros/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/sohams25/prism-ros/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/sohams25/prism-ros/releases/tag/v0.1.0
