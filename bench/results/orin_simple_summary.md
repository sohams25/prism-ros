**Host:** Jetson Orin Nano Super, dustynv L4T container, GPU path via nvvidconv (legacy element with BGR adapter); crop routes through videocrop (CPU) — legacy nvvidconv BGR-CAPS gap
**GStreamer:** GStreamer 1.20.3

## resize

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 21.77 | — | — |
| mean latency (ms)   | — | 22.64 | — | — |
| p95 latency (ms)    | — | 28.14 | | |
| p99 latency (ms)    | — | 38.07 | | |
| mean CPU (%)        | 117.0 | 149.4 | | |
| mean RSS (MB)       | 361 | 668 | | |
| realised fps        | — | 9.97 | | |

## crop

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 868.65 | 716.56 | -152.09 | -17.5 % |
| mean latency (ms)   | 834.38 | 647.43 | -186.95 | -22.4 % |
| p95 latency (ms)    | 999.92 | 975.82 | | |
| p99 latency (ms)    | 1068.05 | 1057.99 | | |
| mean CPU (%)        | 163.0 | 157.4 | | |
| mean RSS (MB)       | 520 | 531 | | |
| realised fps        | 6.88 | 7.77 | | |

## colorconvert

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | 14295.86 | 1194.61 | -13101.26 | -91.6 % |
| mean latency (ms)   | 14634.97 | 1200.36 | -13434.61 | -91.8 % |
| p95 latency (ms)    | 18757.00 | 1398.53 | | |
| p99 latency (ms)    | 18940.77 | 1469.23 | | |
| mean CPU (%)        | 249.9 | 204.1 | | |
| mean RSS (MB)       | 1393 | 802 | | |
| realised fps        | 0.21 | 2.70 | | |

## chain

| metric | stock | prism | Δ | Δ % |
| --- | ---: | ---: | ---: | ---: |
| median latency (ms) | — | 17.11 | — | — |
| mean latency (ms)   | — | 18.04 | — | — |
| p95 latency (ms)    | — | 21.30 | | |
| p99 latency (ms)    | — | 24.95 | | |
| mean CPU (%)        | 131.9 | 137.1 | | |
| mean RSS (MB)       | 389 | 647 | | |
| realised fps        | — | 9.98 | | |

