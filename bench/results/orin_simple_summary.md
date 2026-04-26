**Host:** Jetson Orin Nano Super, JetPack 6.2, ROS 2 Humble in `dustynv/ros:humble-desktop-l4t-r36.2.0` container; per-action backend on legacy `nvvidconv` (`resize`/`chain` GPU, `crop` CPU `videocrop`, `colorconvert` GPU)
**GStreamer:** GStreamer 1.20.3

## Methodology

4K BGR8 input (3840×2160) decoded from H.264 by `prism::MediaStreamerNode`, downscaled per-operation at 10 Hz, sustained for 120 s per operation (preceded by a 10 s warmup); the first 10 s and last 5 s of latency samples are dropped at analysis time, leaving a ≈105 s analysis window. Two `component_container` processes per run (`legacy_container` + `accel_container`); latency is per-frame from publisher stamp to subscriber receive. Fast-DDS UDPv4-only profile (`config/fastdds_no_shm.xml`) to keep shared-memory transport from quietly changing the comparison. NumPy on the stock side for the colorconvert reference (cv_bridge segfaulted locally; see `bench/METHODOLOGY.md` for that finding).

Zero-copy intra-process ingest verified empirically by pointer-address comparison between `MediaStreamerNode::publish_image` and `ImageProcNode::on_image`: addresses match per frame and vary per frame (ruling out allocator coincidence). Verification was a one-line probe added and reverted; HEAD source files have zero diff. The settled wording's "single-copy egress" qualifier remains correct — the GStreamer appsink callback still copies on the way back to `sensor_msgs/Image`.

## Per-action backend routing on legacy `nvvidconv`

`nvvideoconvert` is not packaged in this container image, but the legacy `nvvidconv` element is. Prism's `validate_platform()` two-step probe (modern element preferred, legacy element accepted) selects `nvvidconv`. `nvvidconv`'s sink/src caps do not list `BGR`, so any GPU stage on this image carries a CPU `videoconvert ↔ BGRx` adapter on the ingress boundary at full source resolution. The per-action routing below is a hand-coded table derived from operator A/B measurement on this exact hardware and element combination, **not an autonomous runtime optimiser**:

- **`resize` and `chain`** keep the GPU path (egress at 640×480 amortises the BGRx tax).
- **`crop`** routes to CPU `videocrop` directly. Cropping is a sub-rectangle memcpy in BGR; the GPU offers nothing the CPU cannot do faster on a non-shrinking 4K-input action because the BGR adapter dominates (this is a crop-specific finding from Round-1/2 A/B measurement on this image; Round-3 falsified the corresponding adapter-dominance hypothesis for `colorconvert` — see below). Same principled finding pattern as the VAAPI 1.20 fallback on Intel: the GPU element is detected and validated, but the measured cost-benefit declines to use it for this action on this image.
- **`colorconvert`** keeps the GPU path on this image (BGR → BGRx → GPU RGBA → `cv::cvtColor` at egress) and is the legitimately backpressured op. Round-3 verification: a follow-up A/B that routed colorconvert through pure CPU `videoconvert n-threads=4` (no `nvvidconv` stage) produced statistically identical numbers (1188 ms / 2.38 fps), falsifying the BGR-adapter hypothesis. The colorconvert ceiling on this image is dual-container CPU contention from the bench harness — the stock-side Python NumPy baseline pins ~2.5 of the 6 Orin cores at 250 % CPU, leaving Prism ~2 cores; 4K colorspace work cannot fit. Single-process direct-mode colorconvert measured 9.27 ms on this same hardware on April 25 (`bench/results/orin_20260425T132225Z/`); production deployments without that contention should not see this ceiling. The proposal's Section 9 `nvvideoconvert` capture (CUDA-resident colorspace, off the contended CPU pool) closes both the production case (BGR-native CAPS, no adapter) and the bench case.

The `chain` row uses `crop → resize → colorconvert`. The lead `crop` step shrinks 4K → 2560×1440 in CPU `videocrop` before any BGR↔BGRx adapter; the `resize` step then downsamples to 640×480 before the closing `colorconvert`'s GPU stage runs. Net: ~10 Hz at 17 ms median.

For reference, the prior April-25 direct-mode capture on this same Orin (when neither GPU element was registered in the container — CSVs preserved at `bench/results/orin_20260425T132225Z/`) showed `cv::resize` medians of 7.61 ms (resize), 9.47 ms (crop), 9.27 ms (colorconvert), 9.00 ms (chain), all at ≈10 fps. Direct-mode wins on this Orin in absolute terms, and the proposal narrative reflects that.

## resize (3840×2160 → 640×480 @ 10 Hz, GPU path; stock unavailable in this container)

`image_proc::ResizeNode` does not publish frames inside this container (an `image_proc` packaging issue specific to the dustynv image, not a Prism finding); A/B delta omitted.

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 21.77 | — | — |
| mean latency (ms)   | — | 22.64 | — | — |
| p95 latency (ms)    | — | 28.14 | | |
| p99 latency (ms)    | — | 38.07 | | |
| mean CPU (%)        | 117.0 | 149.4 | | |
| mean RSS (MB)       | 361   | 668   | | |
| realised fps        | — | 9.97 | | |

## crop (3840×2160 → 2560×1440 @ 10 Hz, CPU `videocrop`, A/B)

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 868.65  | 716.56 | -152.09 | -17.5 % |
| mean latency (ms)   | 834.38  | 647.43 | -186.95 | -22.4 % |
| p95 latency (ms)    | 999.92  | 975.82 | | |
| p99 latency (ms)    | 1068.05 | 1057.99 | | |
| mean CPU (%)        | 163.0   | 157.4 | | |
| mean RSS (MB)       | 520     | 531   | | |
| realised fps        | 6.88    | 7.77  | | |

Both stock and Prism saturate at ≈7 fps under this container's CPU contention (the bench's twin-component-container layout pins the Orin Nano's 6 cores hard); Prism's median is below stock by ~17 %, so the intra-process + DDS-roundtrip-elimination architecture still pays off on the legacy GPU image even when the GPU element itself does not contribute.

## colorconvert (BGR8 → RGB8 @ 10 Hz, GPU path, A/B)

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 14295.86 | 1194.61 | -13101.26 | -91.6 % |
| mean latency (ms)   | 14634.97 | 1200.36 | -13434.61 | -91.8 % |
| p95 latency (ms)    | 18757.00 | 1398.53 | | |
| p99 latency (ms)    | 18940.77 | 1469.23 | | |
| mean CPU (%)        | 249.9    | 204.1   | | |
| mean RSS (MB)       | 1393     | 802     | | |
| realised fps        | 0.21     | 2.70    | | |

See the per-action routing notes above for the Round-3 contention finding.

## chain (crop → resize → colorconvert @ 10 Hz, mixed CPU `videocrop` + GPU resize/colorconvert; stock unavailable in this container)

`image_proc::ResizeNode` is the second step in the stock-side launch and does not publish; A/B delta omitted, Prism throughput reported standalone.

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 17.11 | — | — |
| mean latency (ms)   | — | 18.04 | — | — |
| p95 latency (ms)    | — | 21.30 | | |
| p99 latency (ms)    | — | 24.95 | | |
| mean CPU (%)        | 131.9 | 137.1 | | |
| mean RSS (MB)       | 389   | 647   | | |
| realised fps        | — | 9.98 | | |
