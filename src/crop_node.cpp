#include "prism_image_proc/crop_node.hpp"

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

CropNode::CropNode(const rclcpp::NodeOptions & options)
: ImageProcNode(with_default_action(options, "crop"))
{
}

}  // namespace prism

RCLCPP_COMPONENTS_REGISTER_NODE(prism::CropNode)
