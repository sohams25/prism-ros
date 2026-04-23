#ifndef PRISM_IMAGE_PROC__COLOR_CONVERT_NODE_HPP_
#define PRISM_IMAGE_PROC__COLOR_CONVERT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "prism_image_proc/image_proc_node.hpp"

namespace prism
{

// Thin wrapper around ImageProcNode that pins action="colorconvert" at
// construction time. See ResizeNode for the wrapper rationale.
class ColorConvertNode : public ImageProcNode
{
public:
  explicit ColorConvertNode(const rclcpp::NodeOptions & options);
};

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__COLOR_CONVERT_NODE_HPP_
