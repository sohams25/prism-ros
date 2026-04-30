#ifndef PRISM_IMAGE_PROC__RECTIFY_NODE_HPP_
#define PRISM_IMAGE_PROC__RECTIFY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "prism_image_proc/image_proc_node.hpp"

namespace prism
{

// Thin wrapper around ImageProcNode that pins action="rectify" at
// construction time. Drop-in replacement for `image_proc::RectifyNode`'s
// rectification pipeline. Rectify always runs CPU-side (cv::remap on the
// direct-mode path) — see CLAUDE.md "CPU/GPU compartmentalization" and
// the v0.2 E2.1 capability probe for the architectural rationale.
class RectifyNode : public ImageProcNode
{
public:
  explicit RectifyNode(const rclcpp::NodeOptions & options);
};

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__RECTIFY_NODE_HPP_
