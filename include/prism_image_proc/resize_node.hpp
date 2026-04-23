#ifndef PRISM_IMAGE_PROC__RESIZE_NODE_HPP_
#define PRISM_IMAGE_PROC__RESIZE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "prism_image_proc/image_proc_node.hpp"

namespace prism
{

// Thin wrapper around ImageProcNode that pins action="resize" at
// construction time. Exists so launch files and downstream packages that
// migrated from `image_proc::ResizeNode` can continue to write
// `plugin='prism::ResizeNode'` and get the expected default behaviour
// without having to also set the `action` parameter.
class ResizeNode : public ImageProcNode
{
public:
  explicit ResizeNode(const rclcpp::NodeOptions & options);
};

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__RESIZE_NODE_HPP_
