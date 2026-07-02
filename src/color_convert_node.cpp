#include "prism_image_proc/color_convert_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "prism_image_proc/node_options.hpp"

namespace prism
{

ColorConvertNode::ColorConvertNode(const rclcpp::NodeOptions & options)
: ImageProcNode(with_default_action(options, "colorconvert"), "color_convert_node")
{
}

}  // namespace prism

RCLCPP_COMPONENTS_REGISTER_NODE(prism::ColorConvertNode)
