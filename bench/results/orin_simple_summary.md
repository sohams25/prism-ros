**Host:** Jetson Orin Nano Super, JetPack 6.2 host running dustynv/ros:humble-desktop-l4t-r36.2.0 container, direct-mode fallback (nvvideoconvert plugin not in this container)
**GStreamer:** 1.20.3 (host)

## resize

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 7.61 | — | — |
| mean latency (ms)   | — | 8.88 | — | — |
| p95 latency (ms)    | — | 11.43 | | |
| p99 latency (ms)    | — | 22.90 | | |
| mean CPU (%)        | 116.3 | 124.9 | | |
| mean RSS (MB)       | 368 | 374 | | |
| realised fps        | — | 9.99 | | |

## crop

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 53.48 | 9.47 | -44.01 | -82.3 % |
| mean latency (ms)   | 108.59 | 18.16 | -90.43 | -83.3 % |
| p95 latency (ms)    | 486.50 | 48.22 | | |
| p99 latency (ms)    | 816.97 | 211.48 | | |
| mean CPU (%)        | 155.8 | 128.2 | | |
| mean RSS (MB)       | 375 | 373 | | |
| realised fps        | 9.91 | 9.98 | | |

## colorconvert

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 8170.05 | 9.27 | -8160.78 | -99.9 % |
| mean latency (ms)   | 8353.75 | 15.98 | -8337.77 | -99.8 % |
| p95 latency (ms)    | 10119.81 | 30.95 | | |
| p99 latency (ms)    | 10151.45 | 212.35 | | |
| mean CPU (%)        | 262.9 | 127.9 | | |
| mean RSS (MB)       | 1345 | 371 | | |
| realised fps        | 0.25 | 9.97 | | |

## chain

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 9.00 | — | — |
| mean latency (ms)   | — | 11.76 | — | — |
| p95 latency (ms)    | — | 21.78 | | |
| p99 latency (ms)    | — | 32.06 | | |
| mean CPU (%)        | 133.5 | 128.7 | | |
| mean RSS (MB)       | 398 | 363 | | |
| realised fps        | — | 9.97 | | |

