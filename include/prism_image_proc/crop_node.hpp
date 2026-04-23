#ifndef PRISM_IMAGE_PROC__CROP_NODE_HPP_
#define PRISM_IMAGE_PROC__CROP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "prism_image_proc/image_proc_node.hpp"

namespace prism
{

// Thin wrapper around ImageProcNode that pins action="crop" at
// construction time. See ResizeNode for the wrapper rationale.
class CropNode : public ImageProcNode
{
public:
  explicit CropNode(const rclcpp::NodeOptions & options);
};

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__CROP_NODE_HPP_
