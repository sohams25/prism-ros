#include "prism_image_proc/crop_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "prism_image_proc/node_options.hpp"

namespace prism
{

CropNode::CropNode(const rclcpp::NodeOptions & options)
: ImageProcNode(with_default_action(options, "crop"), "crop_node")
{
}

}  // namespace prism

RCLCPP_COMPONENTS_REGISTER_NODE(prism::CropNode)
