# F2A — rectify+resize chain A/B results

Date: 2026-04-30. Branch: `exp/v0.2-validation`. Commits:
prototype `cb6a99f`, F2 harness `910c313`, F2A direct-mode chain
+ harness `1163179`.

## Why F2A

E2.3 showed standalone rectify is a wash vs `image_proc::RectifyNode`
(see `F2_E2.3_a_b.md`): rectify preserves frame size, the
Python rclpy CaptureNode receive path becomes the ceiling, and
image_transport in Humble appears to do intra-process inside a
composable container — so the v0.1.0 source → processor
DDS-elimination mechanism does not apply.

F2A applies the v0.1.0 *chain* mechanism instead. Stock side is
two image_proc nodes piped through DDS; Prism side is one
chained pipeline. The intermediate DDS hop on stock is the
mechanism Prism eliminates. Output is 640×480 (small, no
receiver saturation), matching the v0.1.0 chain row shape.

## Method

- Source: `prism::MediaStreamerNode` publishing BGR8 at 10 Hz with
  `focal_x/y=1900`, `D=[-0.30, 0.10, 0, 0, -0.02]` (same intrinsics
  as the E2.2 kernel microbench).
- Stock graph: `image_proc::RectifyNode → image_proc::ResizeNode`,
  both with `use_intra_process_comms: False` so the intermediate
  hop serialises through DDS — same shape as the existing v0.1.0
  `chain` operation's stock side.
- Prism graph: one `prism::ImageProcNode` running
  `action="rectify,resize"` with `target = 640×480`. Direct-mode
  pipeline: `cv::remap` against the LUT computed from runtime
  CameraInfo, then `cv::resize` to target. Single DDS hop on
  egress. CPU-only by design.
- Latency: `recv_ns − msg.header.stamp_ns` at the same Python
  rclpy CaptureNode for both sides.
- Capture: 120 s, 10 s warmup drop, 5 s tail drop (same as
  v0.1.0 anchor methodology).

## Results

### Intel desktop, 4K source

`bench/results/v0.2_F2A_intel_4k_20260430/`

| Side | n | p50 ms | p95 ms | p99 ms | mean ms | fps |
| --- | --- | --- | --- | --- | --- | --- |
| Prism | 1200 | **17.75** | 28.77 | 35.63 | 17.95 | 10.00 |
| Stock | 1200 | 40.28 | 55.05 | 62.00 | 40.82 | 10.00 |

**Δ p50: −55.9% (Prism saves 22.53 ms per frame).**

Same architectural family as the v0.1.0 chain row anchor
(Intel chain: 12.86 / 76.28 = −83.1%). F2A's −55.9% is somewhat
smaller because rectify+resize does less staged work than
v0.1.0's three-stage `crop,resize,colorconvert` chain — fewer
intermediate DDS hops to eliminate.

### Orin Nano, 1080p source

`bench/results/v0.2_F2A_orin_1080p_20260430/`

| Side | n | p50 ms | p95 ms | p99 ms | mean ms | fps |
| --- | --- | --- | --- | --- | --- | --- |
| Prism | 1200 | **37.49** | 43.31 | 57.21 | 38.60 | 10.00 |
| Stock | 0 | — | — | — | — | 0 |

**Δ p50: — (stock side does not publish on this host).**

Same `—` outcome as v0.1.0's Orin resize and chain rows.
`image_proc::ResizeNode` does not publish frames inside the
dustynv L4T container — a packaging issue, not a Prism finding;
honestly attributed in the existing `orin_simple_summary.md`.

Resolution choice on Orin: 1080p, not 4K. Justified by the
F2_E2.2_kernel.md microbench (cv::remap on Orin is 63 ms p50
at 4K — barely 1.5× headroom over 10 Hz, and the chain stage
adds further work that pushes the pipeline above the 100 ms
budget). At 1080p the kernel cost projection (~16 ms) plus
the resize stage plus path overhead lands at the measured
37.49 ms p50, sustaining 10 Hz with comfortable headroom.
This is the deployable point on Orin Nano for this chain.

A 4K capture on Orin (`bench/results/v0.2_F2A_orin_smoke/`) ran
30 s at p50 = 235.72 ms, sustained 7.5 fps, frames dropped above
the 100 ms budget — the predicted kernel-ceiling outcome.

## What this row says

- **Intel desktop @ 4K**: Prism wins by 22.53 ms / −55.9 % via
  the v0.1.0 chain mechanism (single pipeline avoiding
  intermediate DDS hop between rectify and resize stages).
  Output is 640×480 BGR; intra-process composition + chain
  collapsing.
- **Orin Nano @ 1080p**: Prism sustains the rectify+resize chain
  at 10 Hz with p50 = 37.49 ms. Stock side is `—` due to the
  same documented L4T packaging issue as v0.1.0. The Prism
  number alone is honest evidence that the v0.2 chain works on
  Prism's segment.
- **CPU/GPU compartmentalization**: this entire chain runs on
  CPU. The GPU on either host stays free for whatever
  perception model the user runs downstream — the operator's
  positioning line, validated by construction (no GPU
  allocation in the prototype path).

## Implication for v0.2

F2A is publishable. The deploy decision is now between:

- **Deploy F2A as-is.** New `prism::RectifyNode` component plus the
  `rectify,resize` chain capability. Add an Intel `rectify_chain`
  row to the README + site benchmark tables alongside the
  existing four operations. Add an Orin `rectify_chain` row with
  `—` stock. Settle the joint surface accordingly.
- **Deploy F2A with caveats first.** Land the code under v0.2 but
  hold the README/site row until either (a) a 4K Orin sustain
  scenario gets a separate measured row, or (b) a non-dustynv L4T
  container is built so Orin gets a real stock comparand.

The architectural mechanism (chain DDS-hop elimination) is
sound. The numbers are real. The L4T packaging gap is
pre-existing v0.1.0 territory.

## Out of scope, recorded for completeness

- E1.2 (NV12 detour-cost) and E1.3 (FormatBridge prototype):
  not run; F1 deferred at E1.1 sign-off.
- 4K Orin sustained capture: smoke shows kernel ceiling, full
  120 s run not commissioned. The 1080p capture is the
  deployable point.
- Standalone `prism::RectifyNode` component still ships (built
  in `cb6a99f`); F2A just uses the chain composition as the
  measured surface. Standalone rectify is publishable as
  "wash" — same kernel as image_proc::RectifyNode, no DDS-elim
  win without a chain — and that finding matches the operator's
  CPU/GPU compartmentalization framing where rectify being
  CPU-bound is the *point*, not a fallback.
