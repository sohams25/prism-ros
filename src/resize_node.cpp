#include "prism_image_proc/resize_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "prism_image_proc/node_options.hpp"

namespace prism
{

ResizeNode::ResizeNode(const rclcpp::NodeOptions & options)
: ImageProcNode(with_default_action(options, "resize"), "resize_node")
{
}

}  // namespace prism

RCLCPP_COMPONENTS_REGISTER_NODE(prism::ResizeNode)
