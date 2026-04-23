#include "prism_image_proc/resize_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace prism
{

namespace
{

// Returns the passed NodeOptions with "action" pre-set to the wrapper's
// default, unless the caller has already supplied an override for
// "action" — in which case the caller's value wins. The override is
// applied before ImageProcNode's declare_parameter("action", ...) runs,
// so by the time the base ctor builds the pipeline the right action is
// in effect.
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

ResizeNode::ResizeNode(const rclcpp::NodeOptions & options)
: ImageProcNode(with_default_action(options, "resize"))
{
}

}  // namespace prism

RCLCPP_COMPONENTS_REGISTER_NODE(prism::ResizeNode)
