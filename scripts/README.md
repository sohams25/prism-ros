# scripts/

This directory mixes user-facing demo tooling, dev utilities, and
bench-harness helpers. Audience-grouped subdirectory layout
(`demo/`, `dev/`, `bench/`) is on the post-submission cleanup list.

| File | Audience | Role |
|---|---|---|
| `dashboard.html` | User-facing demo | Static HTML shell for the A/B benchmark dashboard; served by and paired with `visualize_demo.py` |
| `visualize_demo.py` | User-facing demo | MJPEG-streaming visualizer that serves `dashboard.html` |
| `generate_assets.py` | Dev utility | Asset / banner generator |
| `synthetic_4k_pub.py` | Dev utility | Standalone Python 4K source for inter-process demo use; topic `/camera/image_raw`, no intra-process comms (use the C++ component for benchmark runs) |
| `cpu_monitor.py` | Bench-harness internal | CPU monitor helper |
| `latency_tracker.py` | Bench-harness internal | Latency tracker; used by `A_B_comparison.launch.py` |
