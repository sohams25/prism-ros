# Production Readiness — `prism_image_proc`

Status of the hardening effort that takes `prism_image_proc` from a working
v0.1.0 prototype toward a fleet-deployable ROS 2 package. This document is the
durable record of the audit findings, what has been fixed, and what remains.

> **Method.** A 10-dimension audit (memory safety, concurrency, ROS 2 idioms,
> GStreamer correctness, input validation/security, error handling, build/
> packaging, CI/release, tests, docs-vs-code) with adversarial per-finding
> verification produced 61 raw findings → **51 confirmed, 10 refuted**. Findings
> were de-duplicated (the same defect was often reported by several dimensions).
> Each fix below was verified by build + unit tests + a runtime smoke test on an
> Intel/GStreamer-1.20.3 host (direct-mode path) and on a Jetson Orin Nano
> (JetPack 6.2, legacy `nvvidconv` GPU path).

## Baseline (entry state)

- Builds clean, zero warnings under `-Wall -Wextra -Wpedantic`.
- 11/11 `PipelineFactory` unit tests pass.
- **But** the entire automated net was one gtest covering string generation +
  CameraInfo transforms. CI ran only the CPU path on an Intel host, so the GPU
  egress code, node lifecycle, parameter reconfiguration, and every per-backend
  fragment builder were unexercised — and that is exactly where the confirmed
  memory-safety and crash defects lived.

---

## Phase 1 — Stabilize (correctness & memory-safety) ✅ done

| Fix | Finding | Where |
|---|---|---|
| I420/YV12 appsink handler heap overflow — V-plane was written ~h/2 rows past the `cv::Mat`; rewrote to contiguous U/V packing inside the `h*3/2*w` budget + even-dimension guard | `i420-yv12-v-plane-heap-overflow` | `src/image_proc_node.cpp` (on_new_sample) |
| Input encoding/step/size validation on both ingest paths (`on_image`, `on_image_direct`) — non-`bgr8` / malformed frames are warn-and-dropped instead of silently corrupting output or over-reading the heap | `direct-mode-no-encoding-check`, `appsrc-no-encoding-dim-validation` | `src/image_proc_node.cpp` |
| Runtime-reconfig crash hardening — `on_set_parameters` dry-runs the proposed config through the factory (rejects bad `target_encoding`/`crop_*`/`flip_method`/combinations with a reason); `rebuild_from_params` wrapped in try/catch that degrades to direct mode instead of throwing out of the timer | `target-encoding-unvalidated-crash`, `crop-params-unvalidated-crash`, `flip_method` crasher | `src/image_proc_node.cpp` |
| `use_scale` integer-overflow guard — scaled dims computed in double and clamped before narrowing | `scale-product-signed-overflow` | `src/image_proc_node.cpp` |
| Packaging: removed the spurious, unresolvable `gst_bridge` exec_depend (hard-failed `rosdep install`); declared the GStreamer dev build deps + runtime plugins; marked `image_proc` bench-only; pinned `CMAKE_CXX_STANDARD 17` | `spurious-gst-bridge-exec-depend`, `missing-gstreamer-build-depend` | `package.xml`, `CMakeLists.txt` |

## Phase 2 — Harden (concurrency, errors, lifecycle) ✅ done

| Fix | Finding | Where |
|---|---|---|
| `GstElementFactory` ref-leak at ~12 boolean probe sites → single `factory_exists()` find-then-unref helper | `gst-factory-find-leak-*` | `pipeline_factory.{hpp,cpp}`, `image_proc_node.cpp` |
| Teardown ordering — drive pipeline to `GST_STATE_NULL` (joins the streaming thread) **before** unref'ing appsrc/appsink; added `it_sub_.shutdown()` for symmetric teardown | `shutdown-pipeline-element-unref-before-null-state` | `src/image_proc_node.cpp` |
| `gst_parse_launch` partial-parse leak — unref a non-NULL pipeline before discarding on parse error | `gst-parse-launch-pipeline-leak-on-partial-parse` | `src/image_proc_node.cpp` |
| Visible failure — `launch_pipeline()` now returns `bool` and starts the pipeline before wiring ROS I/O; on failure the node falls back to direct mode instead of becoming a live-but-silent zombie | `launch-pipeline-dead-node-no-fallback` | `src/image_proc_node.cpp` |
| Bounded bus recovery — `poll_bus` retries the GPU pipeline up to 3× on ERROR/EOS, then degrades to direct mode permanently (no permanent death, no flapping) | `poll-bus-eos-permanent-death` | `src/image_proc_node.cpp` |
| `MediaStreamerNode` throws on video-open failure (component-load failure) instead of a silent timer-less zombie; bench `cv_bridge` helper raises on unsupported conversion | `media-streamer-ctor-silent-partial-construct`, `cv-bridge-subscriber-silent-encoding-passthrough` | `src/media_streamer_node.cpp`, `bench/helpers/…` |
| `PlatformInfo::platform` default-initialized to `CPU_FALLBACK` | `platform-info-uninitialized-member` | `include/.../hardware_detector.hpp` |

## Phase 3 — Productionize ✅ mostly done

| Fix | Finding | Where |
|---|---|---|
| `SensorDataQoS` (BEST_EFFORT) default for image + CameraInfo I/O so the node connects to standard camera drivers; `reliable_qos` parameter escape hatch | `qos-reliable-incompatible-camera-drivers` | `src/image_proc_node.cpp` |
| `ParameterDescriptor` integer/float ranges on all dimension/scale/crop params (rcl-level rejection) | `missing-parameter-descriptors-ranges` | `src/image_proc_node.cpp` |
| Distinct default node names per wrapper (`resize_node`/`crop_node`/`color_convert_node`; base `image_proc_node`) via a protected constructor | `hardcoded-node-name-resize-node` | `*_node.cpp`, header |
| `ament_export_targets/libraries/include_directories/dependencies` + `EXPORT` install so downstream packages can link `prism_core` | `missing-ament-export-for-prism-core` | `CMakeLists.txt` |
| Stereo `CameraInfo` correctness — `resize_camera_info` now scales `P[3]`/`P[7]` (Tx/Ty), fixing silent disparity-to-depth corruption on rectified stereo | `camera-info-stereo-tx-untested` | `src/pipeline_factory.cpp` |
| Tests — unit suite 11 → **21** (per-backend fragment build strings, `validate_platform`/`factory_exists`, vertical flip, stereo Tx, crop/encoding/flip throw paths) **plus** a `launch_testing` integration test that drives a live `ImageProcNode` in direct mode and asserts output dims/encoding + transformed CameraInfo | `no-integration-test-node-runtime`, `*-untested` | `test/` |
| CI rebuilt into a real gate: `rosdep`-driven deps, `--return-code-on-test-failure` (the old gate could not fail), least-privilege `permissions`, **+ jobs** for ASan/UBSan, advisory `ament_lint`, a **blocking Jazzy build+test** (cross-distro `cv_bridge` include), and a **native arm64 build+test** (Jetson target, no QEMU) | `ci-test-result-no-exit-code`, `ci-no-sanitizer-run`, `ci-no-rosdep-install`, `ci-no-distro-matrix`, `ci-no-arm64-build` | `.github/workflows/ci.yml` |
| Docs-vs-code drift — qualified the "drop-in / same parameters" claim (topics, CameraInfo derivation, QoS), corrected the `vaapipostproc`→direct-mode prose, documented the direct-mode action-dropping limitation and the CameraInfo-topic collision footgun | `drop-in-topic-param-mismatch`, `vaapipostproc-arch-claim-misleading`, `direct-mode-chain-silent-drop-undocumented` | `README.md` |

### Phase 3 — remaining (tracked follow-ups)

- **SHA-pin GitHub Actions.** `permissions: contents: read` is set; actions are
  still on mutable `@v4` tags. Pin to commit digests (`pinact`/Dependabot) — not
  done here because it requires fetching the real digests, and a wrong SHA
  breaks CI. (`ci-actions-not-sha-pinned`)
- **ament_lint debt.** Lint runs as a non-blocking job. The tree predates ament
  house style (copyright headers, include order, uncrustify brace style). Bring
  the codebase to ament style (or codify the project style and scope exclusions)
  before promoting `lint` to a required check. (`ci-ament-lint-permanently-bypassed`)
- **Maintainer email.** `package.xml` still has a placeholder
  (`baremetal@todo.todo`, flagged with a TODO). Set a real, monitored address
  before any bloom / rosdistro submission. (`maintainer-email-placeholder`)
- **arm64 GPU-path coverage.** The arm64 job (native runner, build + unit
  tests) is a blocking gate, but the accelerated `nvvidconv` path itself is only
  exercisable on real Jetson hardware (validated manually on the Orin) — wire a
  self-hosted Jetson runner if continuous GPU-path CI is wanted.

## Phase 4 — Extend (post-stabilization)

- **Observability + managed lifecycle.** Publish `diagnostic_msgs/DiagnosticArray`
  (current backend, processed/dropped frame counts, pipeline state); consider a
  `rclcpp_lifecycle::LifecycleNode` variant so orchestrators can observe/recover
  degraded nodes instead of scraping logs.
- **Generalize input encoding.** Now that ingest is validated, accept `rgb8`/
  `mono8` via proper conversion (cv_bridge) rather than reject.
- **PTS running-time semantics.** Convert the tunneled ROS Unix-epoch PTS to
  GStreamer running-time (subtract `base_time`) so future clock-based/`videorate`
  elements behave correctly. Benign today (`sync=false`). (`pts-unix-epoch-vs-gst-running-time`)
- **NV12/I420 odd-dimension handling**, **MultiThreadedExecutor** safety review
  (the shipped single-threaded executor makes the rebuild/streaming-thread access
  safe today; a mutex/atomic-generation swap would be needed under an MT executor).
- **New backends/ops** (per the README Roadmap): per-action `prism::FlipNode`,
  rectify/debayer, GStreamer 1.22+ Intel VA-API capture, Rockchip RK3588.

---

## Refuted / down-rated (recorded so they are not re-raised)

- **"CRITICAL use-after-free race"** between `on_new_sample` (streaming thread)
  and `rebuild_from_params` (executor thread): **over-stated for the shipped
  configuration.** The executor is single-threaded (`rclcpp::spin` /
  `component_container`) and `gst_element_set_state(NULL)` joins the streaming
  thread before publishers are reset. Addressed defensively (teardown reorder);
  a lock is only needed under a `MultiThreadedExecutor`, which the package does
  not use by default. Kept as a Phase 4 review item, not a live blocker.

## Verification evidence

- **Intel desktop (GStreamer 1.20.3, direct mode):** clean build (zero
  warnings), 21/21 unit tests, `launch_testing` integration test green, runtime
  smoke test (1280×720 → 640×480 bgr8 + scaled CameraInfo) green.
- **Jetson Orin Nano (JetPack 6.2, aarch64, legacy `nvvidconv`)** — built and
  tested inside `dustynv/ros:humble-desktop-l4t-r36.2.0` (`--runtime nvidia
  --privileged -v /dev:/dev`), the one environment where the accelerated
  fragment builders actually execute:
  - Builds clean on aarch64; **21/21 unit tests pass** on-device.
  - `HardwareDetector` reports `NVIDIA_JETSON`; the leak-free `factory_exists`
    probe validates the legacy `nvvidconv` element (confirming the refactor did
    not break detection).
  - The generated GPU fragment is the expected legacy BGRx adapter
    (`videoconvert n-threads=4 ! BGRx ! nvvidconv compute-hw=1 ! BGRx`), and the
    pipeline reaches PLAYING (`Pipeline launched`).
  - When `nvvidconv` intermittently failed to reach PLAYING (an NVMM/container
    init issue, not a prism defect), the **new fallback path engaged correctly**
    — the node logged the failure and switched to direct mode rather than
    becoming a silent, output-less node. This validates the Phase 2
    `launch-pipeline-dead-node-no-fallback` fix on real hardware.
  - Not yet verified end-to-end on-device: full 4K-frame egress through
    `nvvidconv` (blocked by container-local DDS discovery limits and the
    intermittent `nvvidconv` PLAYING state, both environmental). Recommended as
    a bare-metal (non-container) follow-up on the Jetson.
