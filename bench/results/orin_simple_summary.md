**Host:** Jetson Orin Nano Super, JetPack 6.2 host running dustynv/ros:humble-desktop-l4t-r36.2.0 container, GPU path via nvvidconv (legacy element with BGR adapter)
**GStreamer:** GStreamer 1.20.3

## resize

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 32.80 | — | — |
| mean latency (ms)   | — | 35.17 | — | — |
| p95 latency (ms)    | — | 44.88 | | |
| p99 latency (ms)    | — | 56.16 | | |
| mean CPU (%)        | 116.1 | 146.3 | | |
| mean RSS (MB)       | 356 | 671 | | |
| realised fps        | — | 9.97 | | |

## crop

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 45.93 | 14306.33 | +14260.40 | +31046.8 % |
| mean latency (ms)   | 399.91 | 14256.81 | +13856.90 | +3465.0 % |
| p95 latency (ms)    | 1217.81 | 19409.17 | | |
| p99 latency (ms)    | 1924.88 | 20575.15 | | |
| mean CPU (%)        | 153.7 | 114.4 | | |
| mean RSS (MB)       | 470 | 2049 | | |
| realised fps        | 7.21 | 1.63 | | |

## colorconvert

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 12871.20 | 12102.25 | -768.95 | -6.0 % |
| mean latency (ms)   | 12466.50 | 12040.61 | -425.89 | -3.4 % |
| p95 latency (ms)    | 15719.46 | 19372.44 | | |
| p99 latency (ms)    | 15841.43 | 22609.29 | | |
| mean CPU (%)        | 247.0 | 198.7 | | |
| mean RSS (MB)       | 1339 | 3172 | | |
| realised fps        | 0.21 | 2.05 | | |

## chain

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 35.51 | — | — |
| mean latency (ms)   | — | 37.26 | — | — |
| p95 latency (ms)    | — | 44.92 | | |
| p99 latency (ms)    | — | 53.00 | | |
| mean CPU (%)        | 134.4 | 150.7 | | |
| mean RSS (MB)       | 397 | 682 | | |
| realised fps        | — | 10.00 | | |

