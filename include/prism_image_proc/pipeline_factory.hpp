#ifndef PRISM_IMAGE_PROC__PIPELINE_FACTORY_HPP_
#define PRISM_IMAGE_PROC__PIPELINE_FACTORY_HPP_

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "prism_image_proc/hardware_detector.hpp"

namespace prism
{

struct PipelineConfig
{
  std::string input_topic  = "/camera/image_raw";
  std::string output_topic = "/camera/image_processed";
  std::string action       = "resize";
  int source_width         = 3840;
  int source_height        = 2160;
  int target_width         = 640;
  int target_height        = 480;

  // Reserved for additional actions (colorconvert, crop, flip) — Phase 3.5.
  std::string target_encoding = "bgr8";
  int crop_x = 0;
  int crop_y = 0;
  int crop_width = 0;
  int crop_height = 0;
  std::string flip_method = "none";
};

// Confirms that the elements required by the detected platform exist in the
// live GStreamer registry. Demotes to CPU_FALLBACK if not.
PlatformInfo validate_platform(
  const PlatformInfo & detected, const rclcpp::Logger & logger);

// A CameraInfoTransform mutates a CameraInfo in place to reflect the image
// transformation performed by one action in the chain. Transforms compose:
// each reads the current CameraInfo state (e.g. width/height) and updates
// it, so chaining is simply "apply each in order."
using CameraInfoTransform =
  std::function<void(sensor_msgs::msg::CameraInfo &, const PipelineConfig &)>;

struct ActionDescriptor
{
  using FragmentBuilder =
    std::function<std::string(const PlatformInfo &, const PipelineConfig &)>;
  FragmentBuilder fragment;
  CameraInfoTransform transform;  // may be empty (encoding-only action)
};

class PipelineFactory
{
public:
  PipelineFactory(const PlatformInfo & platform, const PipelineConfig & config);

  // Composes the full gst-launch pipeline for the action chain parsed from
  // config.action. Throws std::invalid_argument on unknown action names or
  // a malformed chain.
  std::string build() const;

  // Returns the CameraInfoTransforms for each action in the chain, in order.
  // Callers apply them left-to-right on a copy of the input CameraInfo to
  // produce the egress CameraInfo.
  std::vector<CameraInfoTransform> camera_info_transforms() const;

  // Parsed action names in chain order.
  const std::vector<std::string> & action_chain() const { return action_names_; }

  // Parses a comma- or pipe-separated action list, trims whitespace, and
  // validates each name is registered. Exposed for tests.
  static std::vector<std::string> parse_action_chain(const std::string & action_str);

  // Registered action names (for diagnostics and error messages).
  static std::vector<std::string> registered_actions();

private:
  PlatformInfo platform_;
  PipelineConfig config_;
  std::vector<std::string> action_names_;

  static const std::unordered_map<std::string, ActionDescriptor> & registry();

  std::string source_element() const;
  std::string sink_element() const;
};

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__PIPELINE_FACTORY_HPP_
