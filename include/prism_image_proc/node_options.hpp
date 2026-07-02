// Copyright 2026 Prism contributors. Apache-2.0.
#ifndef PRISM_IMAGE_PROC__NODE_OPTIONS_HPP_
#define PRISM_IMAGE_PROC__NODE_OPTIONS_HPP_

#include <string>

#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>

namespace prism
{

// Returns the passed NodeOptions with "action" pre-set to the wrapper's
// default, unless the caller has already supplied an override for
// "action" — in which case the caller's value wins. The override is
// applied before ImageProcNode's declare_parameter("action", ...) runs,
// so by the time the base ctor builds the pipeline the right action is
// in effect.
inline rclcpp::NodeOptions with_default_action(
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

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__NODE_OPTIONS_HPP_
