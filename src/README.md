# src/

This directory is a flat mix of library, node, helper, and
entry-point sources. Subdirectory grouping (`core/`, `nodes/`) is
on the post-submission cleanup list.

| File | CMake target | Role |
|---|---|---|
| `hardware_detector.cpp` | `prism_core` | `/dev/*` glob + GStreamer-registry inspect (platform enum only; no fallback) |
| `pipeline_factory.cpp` | `prism_core` | Pipeline construction + action chain parsing; `validate_platform()` overrides to CPU_FALLBACK if a required element is absent |
| `image_proc_node.cpp` | `prism_image_proc_component` | `ImageProcNode` base — GStreamer orchestration + direct fallback |
| `resize_node.cpp` | `prism_image_proc_component` | Thin wrapper, `action="resize"` |
| `crop_node.cpp` | `prism_image_proc_component` | Thin wrapper, `action="crop"` |
| `color_convert_node.cpp` | `prism_image_proc_component` | Thin wrapper, `action="colorconvert"` |
| `media_streamer_node.cpp` | `media_streamer_component` | Video-file publisher (test / demo helper) |
| `synthetic_4k_pub_node.cpp` | `synthetic_4k_pub_component` | Synthetic 4K source (test / demo helper) |
| `main.cpp` | `prism_image_proc` (binary) | Standalone executable entry |
