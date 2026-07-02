# scripts/

Dev utilities and bench-harness helpers.

| File | Role |
|---|---|
| `latency_tracker.py` | Per-frame latency tracker; launched by `launch/A_B_comparison.launch.py` |
| `generate_assets.py` | Generates `assets/banner.png` (repo art provenance) |

The bench harness itself lives in [`bench/`](../bench/); its subscriber
helper is `bench/helpers/cv_bridge_subscriber_node.py`.
