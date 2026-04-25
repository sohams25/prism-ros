**Host:** Intel desktop (i7-class), Ubuntu 22.04, ROS 2 Humble, direct-mode fallback (vapostproc fails live validation on GStreamer 1.20)
**GStreamer:** gst-launch-1.0 version 1.20.3

## resize

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 10.77 | 4.55 | -6.22 | -57.8 % |
| mean latency (ms)   | 13.42 | 7.74 | -5.68 | -42.4 % |
| p95 latency (ms)    | 20.39 | 13.93 | | |
| p99 latency (ms)    | 26.66 | 23.69 | | |
| mean CPU (%)        | 60.8 | 56.2 | | |
| mean RSS (MB)       | 806 | 748 | | |
| realised fps        | 10.01 | 10.00 | | |

## crop

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 22.65 | 4.27 | -18.38 | -81.1 % |
| mean latency (ms)   | 24.76 | 8.68 | -16.09 | -65.0 % |
| p95 latency (ms)    | 38.27 | 20.56 | | |
| p99 latency (ms)    | 49.21 | 34.55 | | |
| mean CPU (%)        | 63.9 | 57.7 | | |
| mean RSS (MB)       | 830 | 747 | | |
| realised fps        | 10.01 | 10.01 | | |

## colorconvert

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 2323.65 | 2.99 | -2320.66 | -99.9 % |
| mean latency (ms)   | 2317.22 | 7.33 | -2309.89 | -99.7 % |
| p95 latency (ms)    | 2397.21 | 13.34 | | |
| p99 latency (ms)    | 2430.18 | 23.37 | | |
| mean CPU (%)        | 157.5 | 49.9 | | |
| mean RSS (MB)       | 1221 | 747 | | |
| realised fps        | 0.76 | 10.01 | | |

## chain

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 76.28 | 12.86 | -63.43 | -83.1 % |
| mean latency (ms)   | 76.43 | 8.86 | -67.57 | -88.4 % |
| p95 latency (ms)    | 97.74 | 14.09 | | |
| p99 latency (ms)    | 107.56 | 24.38 | | |
| mean CPU (%)        | 103.2 | 49.2 | | |
| mean RSS (MB)       | 885 | 747 | | |
| realised fps        | 10.01 | 10.01 | | |

