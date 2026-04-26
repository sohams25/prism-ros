**Host:** Intel desktop (i7-class), Ubuntu 22.04, ROS 2 Humble, direct-mode `cv::resize` fallback (`vapostproc` fails live validation on GStreamer 1.20)
**GStreamer:** gst-launch-1.0 version 1.20.3

## Methodology

4K BGR8 input (3840×2160) decoded from H.264 by `prism::MediaStreamerNode`, downscaled per-operation at 10 Hz, sustained for 120 s per operation (preceded by a 10 s warmup); the first 10 s and last 5 s of latency samples are dropped at analysis time, leaving a ≈105 s analysis window. Two `component_container` processes per run (`legacy_container` + `accel_container`); latency is per-frame from publisher stamp to subscriber receive. Fast-DDS UDPv4-only profile (`config/fastdds_no_shm.xml`) to keep shared-memory transport from quietly changing the comparison. NumPy on the stock side for the colorconvert reference (cv_bridge segfaulted locally; see `bench/METHODOLOGY.md` for that finding).

## Why direct-mode on Intel

On stock ROS 2 Humble (GStreamer 1.20), the `vapostproc` element is present but fails live validation due to a chroma-subsampling regression in `vaapipostproc`. Prism's GPU detection probes positive on `/dev/dri/renderD*`, but the subsequent live registry check rejects the candidate, and the node falls back to a direct `cv::resize` in the subscriber callback (no GStreamer involvement at all in fallback mode). GStreamer 1.22+ (Ubuntu 24.04 / Jazzy) is required for the GPU resize kernel on Intel; that capture is in the proposal's Section 9 forward-looking items.

The wins below are intra-process composition + DDS round-trip elimination, not GPU offload. The `chain` row is the architecturally interesting one: three DDS-piped `image_proc` / `rclpy` nodes on the stock side versus one `prism::ImageProcNode` with intra-process composition on ours. The `colorconvert` row is structural, not a kernel comparison: `image_proc` ships no C++ colorconvert node; the closest stock-side reference is a Python `rclpy` subscriber, which cannot drain a 4K BGR8 topic at 10 Hz (realised 0.76 fps, 1221 MB RSS).

## resize (3840×2160 → 640×480 @ 10 Hz, A/B)

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 10.77 | 4.55 | -6.22 | -57.8 % |
| mean latency (ms)   | 13.42 | 7.74 | -5.68 | -42.4 % |
| p95 latency (ms)    | 20.39 | 13.93 | | |
| p99 latency (ms)    | 26.66 | 23.69 | | |
| mean CPU (%)        | 60.8  | 56.2  | | |
| mean RSS (MB)       | 806   | 748   | | |
| realised fps        | 10.01 | 10.00 | | |

## crop (3840×2160 → 2560×1440 @ 10 Hz, A/B)

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 22.65 | 4.27 | -18.38 | -81.1 % |
| mean latency (ms)   | 24.76 | 8.68 | -16.09 | -65.0 % |
| p95 latency (ms)    | 38.27 | 20.56 | | |
| p99 latency (ms)    | 49.21 | 34.55 | | |
| mean CPU (%)        | 63.9  | 57.7  | | |
| mean RSS (MB)       | 830   | 747   | | |
| realised fps        | 10.01 | 10.01 | | |

## colorconvert (BGR8 → RGB8 @ 10 Hz, A/B)

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 2323.65 | 2.99  | -2320.66 | -99.9 % |
| mean latency (ms)   | 2317.22 | 7.33  | -2309.89 | -99.7 % |
| p95 latency (ms)    | 2397.21 | 13.34 | | |
| p99 latency (ms)    | 2430.18 | 23.37 | | |
| mean CPU (%)        | 157.5   | 49.9  | | |
| mean RSS (MB)       | 1221    | 747   | | |
| realised fps        | 0.76    | 10.01 | | |

## chain (crop → resize → colorconvert @ 10 Hz, A/B)

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 76.28  | 12.86 | -63.43 | -83.1 % |
| mean latency (ms)   | 76.43  | 8.86  | -67.57 | -88.4 % |
| p95 latency (ms)    | 97.74  | 14.09 | | |
| p99 latency (ms)    | 107.56 | 24.38 | | |
| mean CPU (%)        | 103.2  | 49.2  | | |
| mean RSS (MB)       | 885    | 747   | | |
| realised fps        | 10.01  | 10.01 | | |
