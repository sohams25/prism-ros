#include "prism_image_proc/color_convert_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace prism
{

namespace
{

rclcpp::NodeOptions with_default_action(
  rclcpp::NodeOptions options, const std::string & action)
{
  auto overrides = options.parameter_overrides();
  for (const auto & p : overrides) {
    if (p.get_name() == "action") {
      return options;
    }
  }
  overrides.emplace_back("action", action);
  options.parameter_overrides(overrides);
  return options;
}

}  // namespace

ColorConvertNode::ColorConvertNode(const rclcpp::NodeOptions & options)
: ImageProcNode(with_default_action(options, "colorconvert"))
{
}

}  // namespace prism

RCLCPP_COMPONENTS_REGISTER_NODE(prism::ColorConvertNode)
