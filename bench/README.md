# bench/

Reproducible benchmark harness for Prism.

This directory contains the scripts used to capture the numbers
published on the [project site](https://sohams25.github.io/prism-ros/#benchmarks)
and in the repository [README](../README.md#benchmarks). Four
operations are covered: `resize`, `crop`, `colorconvert`, and the
full `crop,resize,colorconvert` chain.

## Layout

```
bench/
  operations.yaml   # declarative description of the four A/B ops
  run.py            # one capture: launches A/B, records per-frame + resources
  analyze.py        # aggregates CSVs into summary.json + summary.md
  plot.py           # renders 6 SVGs per operation (dark + light themes)
  style.py          # Tokyo-Night matplotlib palette, matches the site
  METHODOLOGY.md    # full accounting of what is measured and why
  requirements.txt  # matplotlib / numpy / psutil / pyyaml
  helpers/
    cv_bridge_subscriber_node.py   # benchmark partner for colorconvert
  results/          # CSVs + meta JSON land here
```

## Quick reproduction

```bash
# From the workspace root, with prism_image_proc built and sourced:
python3 src/prism-ros/bench/run.py --operation resize \
  --video /path/to/4k.mp4 --duration 120 \
  --output-dir src/prism-ros/bench/results/

python3 src/prism-ros/bench/analyze.py \
  --results-dir src/prism-ros/bench/results/ \
  --output src/prism-ros/bench/results/summary.json
```

Repeat step 1 with `--operation {crop,colorconvert,chain}` to
cover all four.

See [`METHODOLOGY.md`](METHODOLOGY.md) for the full methodology,
including the important caveat on the `chain` row (architectural
win, not a kernel-speed claim) and the `colorconvert` stock-side
approximation.
