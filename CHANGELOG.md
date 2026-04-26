# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/sohams25/prism-ros/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/sohams25/prism-ros/releases/tag/v0.1.0
