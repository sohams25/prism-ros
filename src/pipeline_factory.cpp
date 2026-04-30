#include "prism_image_proc/pipeline_factory.hpp"

#include <gst/gst.h>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace prism
{

PlatformInfo validate_platform(
  const PlatformInfo & detected, const rclcpp::Logger & logger)
{
  if (detected.platform == HardwarePlatform::CPU_FALLBACK) {
    return detected;
  }

  const char * required_element = nullptr;
  switch (detected.platform) {
    case HardwarePlatform::INTEL_VAAPI:
      // Try the newer va plugin first, fall back to legacy vaapi plugin
      if (gst_element_factory_find("vapostproc")) {
        RCLCPP_INFO(logger, "Plugin validated: 'vapostproc' found (GStreamer va plugin)");
        return detected;
      }
      required_element = "vaapipostproc";
      break;
    case HardwarePlatform::NVIDIA_JETSON:
      // Prefer the modern element; fall back to the legacy nvvidconv
      // shipped on Tegra L4T images that pre-date nvvideoconvert.
      if (gst_element_factory_find("nvvideoconvert")) {
        RCLCPP_INFO(logger, "Plugin validated: 'nvvideoconvert' found");
        return detected;
      }
      if (gst_element_factory_find("nvvidconv")) {
        RCLCPP_INFO(
          logger,
          "Plugin validated: 'nvvidconv' (legacy) found "
          "— GPU path will use legacy element with BGR adapter");
        return detected;
      }
      required_element = "nvvideoconvert";
      break;
    default:
      return detected;
  }

  GstElementFactory * factory = gst_element_factory_find(required_element);
  if (factory) {
    gst_object_unref(factory);
    RCLCPP_INFO(logger, "Plugin validated: '%s' found in registry", required_element);
    return detected;
  }

  RCLCPP_WARN(
    logger,
    "*** PLUGIN MISSING: '%s' not found in GStreamer registry "
    "— falling back to CPU_FALLBACK ***",
    required_element);

  PlatformInfo fallback{};
  fallback.platform = HardwarePlatform::CPU_FALLBACK;
  return fallback;
}

// ---------------------------------------------------------------------------
// Action: resize
// ---------------------------------------------------------------------------

namespace
{

std::string resize_caps(const PipelineConfig & c)
{
  std::ostringstream ss;
  ss << "video/x-raw,width=" << c.target_width
     << ",height=" << c.target_height;
  return ss.str();
}

std::string resize_vaapi(const PipelineConfig & c)
{
  // GStreamer 1.22+ with va plugin: full GL bridge, no CPU conversion.
  if (gst_element_factory_find("vapostproc")) {
    std::ostringstream ss;
    ss << "glupload ! glcolorconvert"
       << " ! video/x-raw(memory:GLMemory),format=RGBA"
       << " ! vapostproc"
       << " ! video/x-raw(memory:VAMemory),format=NV12"
       << ",width=" << c.target_width
       << ",height=" << c.target_height
       << " ! vapostproc"
       << " ! video/x-raw(memory:VAMemory),format=BGRA"
       << " ! gldownload ! glcolorconvert"
       << " ! video/x-raw,format=BGR";
    return ss.str();
  }

  // GStreamer 1.20 with legacy vaapi: vaapipostproc chroma-loss regression.
  // Use CPU videoscale for correctness. The appsrc/appsink architecture
  // still buys zero-copy intra-process ingest (egress is a single copy).
  return "videoscale ! videoconvert ! " + resize_caps(c);
}

std::string resize_jetson(const PipelineConfig & c)
{
  std::ostringstream ss;
  if (gst_element_factory_find("nvvideoconvert")) {
    ss << "nvvideoconvert compute-hw=1 nvbuf-memory-type=2"
       << " ! video/x-raw(memory:NVMM),format=NV12"
       << " ! nvvideoconvert compute-hw=1 nvbuf-memory-type=0 interpolation-method=1"
       << " ! video/x-raw,format=BGR"
       << ",width=" << c.target_width
       << ",height=" << c.target_height;
    return ss.str();
  }
  // Legacy nvvidconv: BGR is not in its CAPS, so we adapt to BGRx on the
  // ingress boundary (CPU videoconvert with n-threads=4) and let the GPU
  // do the actual scale. Egress stays in BGRx; image_proc_node's appsink
  // callback already handles BGRx -> BGR via cv::cvtColor in the same
  // single-copy egress, which avoids a second full-frame CPU videoconvert.
  ss << "videoconvert n-threads=4 ! video/x-raw,format=BGRx"
     << " ! nvvidconv compute-hw=1 interpolation-method=1"
     << " ! video/x-raw,format=BGRx"
     << ",width=" << c.target_width
     << ",height=" << c.target_height;
  return ss.str();
}

std::string resize_cpu(const PipelineConfig & c)
{
  return "videoscale ! videoconvert ! " + resize_caps(c);
}

std::string resize_fragment(const PlatformInfo & p, const PipelineConfig & c)
{
  switch (p.platform) {
    case HardwarePlatform::NVIDIA_JETSON: return resize_jetson(c);
    case HardwarePlatform::INTEL_VAAPI:   return resize_vaapi(c);
    case HardwarePlatform::CPU_FALLBACK:  return resize_cpu(c);
  }
  return resize_cpu(c);
}

// Scales the intrinsics of the input CameraInfo so the published CameraInfo
// matches the resized image. Reads the current width/height as the "in"
// dimensions so the transform composes correctly after a preceding crop.
void resize_camera_info(
  sensor_msgs::msg::CameraInfo & info, const PipelineConfig & c)
{
  if (info.width == 0 || info.height == 0 ||
      c.target_width == 0 || c.target_height == 0)
  {
    return;
  }
  const double sx = static_cast<double>(c.target_width)  / info.width;
  const double sy = static_cast<double>(c.target_height) / info.height;

  info.k[0] *= sx;  info.k[2] *= sx;
  info.k[4] *= sy;  info.k[5] *= sy;

  info.p[0] *= sx;  info.p[2] *= sx;
  info.p[5] *= sy;  info.p[6] *= sy;

  info.roi.x_offset = static_cast<uint32_t>(info.roi.x_offset * sx);
  info.roi.y_offset = static_cast<uint32_t>(info.roi.y_offset * sy);
  info.roi.width    = static_cast<uint32_t>(info.roi.width    * sx);
  info.roi.height   = static_cast<uint32_t>(info.roi.height   * sy);

  info.width  = static_cast<uint32_t>(c.target_width);
  info.height = static_cast<uint32_t>(c.target_height);
  // distortion_model, d, r preserved unchanged.
}

// ---------------------------------------------------------------------------
// Action: colorconvert
// ---------------------------------------------------------------------------

// Maps a sensor_msgs encoding string to the GStreamer format string.
// Allow-list is intentionally narrow (bgr8, rgb8, mono8) — the egress code
// in prism_image_proc.cpp publishes these three as sensor_msgs/Image.
std::string encoding_to_gst_format(const std::string & enc)
{
  if (enc == "bgr8")   { return "BGR"; }
  if (enc == "rgb8")   { return "RGB"; }
  if (enc == "mono8")  { return "GRAY8"; }
  throw std::invalid_argument(
    "Unsupported target_encoding '" + enc +
    "'. Supported: bgr8, rgb8, mono8.");
}

std::string colorconvert_fragment(const PlatformInfo & p, const PipelineConfig & c)
{
  const std::string fmt = encoding_to_gst_format(c.target_encoding);
  const char * element = nullptr;
  switch (p.platform) {
    case HardwarePlatform::NVIDIA_JETSON:
      if (gst_element_factory_find("nvvideoconvert")) {
        element = "nvvideoconvert";  // emits BGR/RGB/GRAY8 directly
      } else {
        // Legacy nvvidconv supports BGRx, RGBA, GRAY8 — not BGR/RGB.
        // Use a 4-channel intermediate; CPU videoconvert lands in the
        // requested 3-channel encoding (or stays at GRAY8).
        const std::string intermediate = (fmt == "GRAY8") ? "GRAY8"
                                       : (fmt == "RGB"  ) ? "RGBA"
                                                          : "BGRx";
        std::ostringstream ssj;
        // Drop the trailing CPU videoconvert: image_proc_node's appsink
        // egress maps BGRx -> bgr8, RGBA -> rgb8, GRAY8 -> mono8 directly
        // via cv::cvtColor in the same copy that already happens.
        ssj << "videoconvert n-threads=4 ! video/x-raw,format=" << intermediate
            << " ! nvvidconv compute-hw=1"
            << " ! video/x-raw,format=" << intermediate;
        return ssj.str();
      }
      break;
    case HardwarePlatform::INTEL_VAAPI:
      element = gst_element_factory_find("vapostproc") ? "vapostproc" : "videoconvert";
      break;
    case HardwarePlatform::CPU_FALLBACK:  element = "videoconvert"; break;
  }
  std::ostringstream ss;
  ss << element << " ! video/x-raw,format=" << fmt;
  return ss.str();
}

// No-op for intrinsics — an encoding change preserves geometry.
void colorconvert_camera_info(
  sensor_msgs::msg::CameraInfo & /*info*/, const PipelineConfig & /*c*/) {}

// ---------------------------------------------------------------------------
// Action: crop
// ---------------------------------------------------------------------------

std::string crop_fragment(const PlatformInfo & p, const PipelineConfig & c)
{
  if (c.crop_width <= 0 || c.crop_height <= 0 ||
      c.crop_x < 0 || c.crop_y < 0)
  {
    throw std::invalid_argument(
      "crop requires positive crop_width/crop_height and non-negative "
      "crop_x/crop_y");
  }

  std::ostringstream ss;
  switch (p.platform) {
    case HardwarePlatform::NVIDIA_JETSON:
      if (gst_element_factory_find("nvvideoconvert")) {
        // nvvideoconvert supports src-* properties for input-side cropping.
        ss << "nvvideoconvert"
           << " src-x=" << c.crop_x
           << " src-y=" << c.crop_y
           << " src-width="  << c.crop_width
           << " src-height=" << c.crop_height
           << " ! video/x-raw,format=BGR"
           << ",width=" << c.crop_width
           << ",height=" << c.crop_height;
        return ss.str();
      }
      // Legacy nvvidconv requires BGR<->BGRx adapters at full source
      // resolution because BGR is not in its CAPS. For crop alone (no
      // resize, no colorspace change downstream), the adapter tax dwarfs
      // any benefit from running the crop on the VIC engine. videocrop
      // operates directly on BGR system memory with stride-only work
      // (effectively a sub-rectangle memcpy), so we route legacy crop
      // through it and skip the GPU stage entirely. Same principled
      // finding as the VAAPI 1.20 fallback: when the GPU element is
      // structurally not paying off, the CPU path is the right answer.
      {
        const int right_l  = c.source_width  - (c.crop_x + c.crop_width);
        const int bottom_l = c.source_height - (c.crop_y + c.crop_height);
        if (right_l < 0 || bottom_l < 0) {
          throw std::invalid_argument(
            "crop rect extends beyond source_width/source_height");
        }
        ss << "videocrop"
           << " left="   << c.crop_x
           << " right="  << right_l
           << " top="    << c.crop_y
           << " bottom=" << bottom_l;
        return ss.str();
      }

    case HardwarePlatform::INTEL_VAAPI:
    case HardwarePlatform::CPU_FALLBACK:
      // videocrop uses left/right/top/bottom. We translate from x,y,w,h by
      // taking the source dims from the PipelineConfig source caps.
      {
        const int right  = c.source_width  - (c.crop_x + c.crop_width);
        const int bottom = c.source_height - (c.crop_y + c.crop_height);
        if (right < 0 || bottom < 0) {
          throw std::invalid_argument(
            "crop rect extends beyond source_width/source_height");
        }
        ss << "videocrop"
           << " left="   << c.crop_x
           << " right="  << right
           << " top="    << c.crop_y
           << " bottom=" << bottom;
        return ss.str();
      }
  }
  return ss.str();
}

void crop_camera_info(
  sensor_msgs::msg::CameraInfo & info, const PipelineConfig & c)
{
  info.k[2] -= c.crop_x; info.k[5] -= c.crop_y;
  info.p[2] -= c.crop_x; info.p[6] -= c.crop_y;
  info.width  = static_cast<uint32_t>(c.crop_width);
  info.height = static_cast<uint32_t>(c.crop_height);
  info.roi.x_offset += static_cast<uint32_t>(c.crop_x);
  info.roi.y_offset += static_cast<uint32_t>(c.crop_y);
  info.roi.width  = static_cast<uint32_t>(c.crop_width);
  info.roi.height = static_cast<uint32_t>(c.crop_height);
}

// ---------------------------------------------------------------------------
// Action: flip
// ---------------------------------------------------------------------------

std::string flip_fragment(const PlatformInfo & p, const PipelineConfig & c)
{
  static const std::unordered_set<std::string> allowed = {
    "none", "horizontal", "vertical"};
  if (!allowed.count(c.flip_method)) {
    throw std::invalid_argument(
      "flip_method must be one of: none, horizontal, vertical (got '" +
      c.flip_method + "')");
  }
  // 'none' still emits an identity convert — keeps the chain well-formed.
  if (c.flip_method == "none") {
    return "identity";
  }

  switch (p.platform) {
    case HardwarePlatform::NVIDIA_JETSON: {
      // flip-method enum: 4=horizontal-flip, 6=vertical-flip on both
      // nvvideoconvert and the legacy nvvidconv element.
      const int method = c.flip_method == "horizontal" ? 4 : 6;
      if (gst_element_factory_find("nvvideoconvert")) {
        return c.flip_method == "horizontal"
          ? "nvvideoconvert flip-method=4"
          : "nvvideoconvert flip-method=6";
      }
      // Legacy nvvidconv: same BGR boundary adapter as the other actions.
      std::ostringstream ssj;
      // Egress stays in BGRx; appsink callback maps BGRx -> bgr8.
      ssj << "videoconvert n-threads=4 ! video/x-raw,format=BGRx"
          << " ! nvvidconv flip-method=" << method
          << " ! video/x-raw,format=BGRx";
      return ssj.str();
    }

    case HardwarePlatform::INTEL_VAAPI:
      if (gst_element_factory_find("vapostproc")) {
        return c.flip_method == "horizontal"
          ? "vapostproc video-direction=horiz"
          : "vapostproc video-direction=vert";
      }
      // Fall through to CPU videoflip.
      [[fallthrough]];
    case HardwarePlatform::CPU_FALLBACK:
      return c.flip_method == "horizontal"
        ? "videoflip method=horizontal-flip"
        : "videoflip method=vertical-flip";
  }
  return "identity";
}

void flip_camera_info(
  sensor_msgs::msg::CameraInfo & info, const PipelineConfig & c)
{
  if (c.flip_method == "horizontal") {
    info.k[2] = static_cast<double>(info.width) - info.k[2];
    info.p[2] = static_cast<double>(info.width) - info.p[2];
  } else if (c.flip_method == "vertical") {
    info.k[5] = static_cast<double>(info.height) - info.k[5];
    info.p[5] = static_cast<double>(info.height) - info.p[5];
  }
  // 'none': no-op.
}

// ---------------------------------------------------------------------------
// Action: rectify
// ---------------------------------------------------------------------------
// Rectify is a CPU-only direct-mode action by design. The GStreamer
// fragment is "identity" — when a working GPU is detected the chain
// containing rectify still falls back to direct mode in image_proc_node,
// because the segment Prism targets has no hardware-accelerated remap
// element with a BGR contract (validated in the v0.2 E2.1 capability
// probe). Keeping rectify CPU-bound is also Prism's positioning line:
// image_proc on CPU, GPU stays free for downstream perception.

std::string rectify_fragment(
  const PlatformInfo & /*p*/, const PipelineConfig & /*c*/)
{
  return "identity";
}

// Rectified output uses the new K returned by getOptimalNewCameraMatrix
// with R = identity and zero distortion. The actual K transform happens
// in image_proc_node (it has the matching cv::Mat handy from LUT
// computation); here we only zero the distortion model, since
// distortion_model + d are not part of PipelineConfig.
void rectify_camera_info(
  sensor_msgs::msg::CameraInfo & info, const PipelineConfig & /*c*/)
{
  info.distortion_model = "plumb_bob";
  std::fill(info.d.begin(), info.d.end(), 0.0);
  // R = identity (rectified frame aligns with the original)
  info.r = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
}

}  // namespace

// ---------------------------------------------------------------------------
// Action registry
// ---------------------------------------------------------------------------

const std::unordered_map<std::string, ActionDescriptor> &
PipelineFactory::registry()
{
  static const std::unordered_map<std::string, ActionDescriptor> r = {
    {"resize",       ActionDescriptor{&resize_fragment,       &resize_camera_info}},
    {"colorconvert", ActionDescriptor{&colorconvert_fragment, &colorconvert_camera_info}},
    {"crop",         ActionDescriptor{&crop_fragment,         &crop_camera_info}},
    {"flip",         ActionDescriptor{&flip_fragment,         &flip_camera_info}},
    {"rectify",      ActionDescriptor{&rectify_fragment,      &rectify_camera_info}},
  };
  return r;
}

std::vector<std::string> PipelineFactory::registered_actions()
{
  std::vector<std::string> names;
  names.reserve(registry().size());
  for (const auto & kv : registry()) { names.push_back(kv.first); }
  std::sort(names.begin(), names.end());
  return names;
}

// ---------------------------------------------------------------------------
// Chain parser
// ---------------------------------------------------------------------------

std::vector<std::string>
PipelineFactory::parse_action_chain(const std::string & action_str)
{
  auto trim = [](std::string s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { return std::string{}; }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
  };

  std::vector<std::string> names;
  std::string current;
  for (char ch : action_str) {
    if (ch == ',' || ch == '|') {
      auto t = trim(current);
      if (!t.empty()) { names.push_back(std::move(t)); }
      current.clear();
    } else {
      current.push_back(ch);
    }
  }
  auto tail = trim(current);
  if (!tail.empty()) { names.push_back(std::move(tail)); }

  if (names.empty()) {
    throw std::invalid_argument(
      "Action chain is empty. Expected one or more of: " +
      [](){
        std::ostringstream ss;
        bool first = true;
        for (const auto & n : registered_actions()) {
          if (!first) { ss << ", "; }
          first = false;
          ss << n;
        }
        return ss.str();
      }());
  }

  const auto & r = registry();
  for (const auto & name : names) {
    if (r.find(name) == r.end()) {
      std::ostringstream ss;
      ss << "Unsupported action '" << name << "'. Supported: ";
      bool first = true;
      for (const auto & n : registered_actions()) {
        if (!first) { ss << ", "; }
        first = false;
        ss << n;
      }
      throw std::invalid_argument(ss.str());
    }
  }
  return names;
}

// ---------------------------------------------------------------------------
// PipelineFactory
// ---------------------------------------------------------------------------

PipelineFactory::PipelineFactory(
  const PlatformInfo & platform, const PipelineConfig & config)
: platform_(platform), config_(config),
  action_names_(parse_action_chain(config.action))
{
}

std::string PipelineFactory::source_element() const
{
  std::ostringstream ss;
  ss << "appsrc name=ros_ingest is-live=true block=false format=3"
     << " ! video/x-raw,format=BGR"
     << ",width=" << config_.source_width
     << ",height=" << config_.source_height
     << ",framerate=30/1"
     << ",interlace-mode=progressive";
  return ss.str();
}

std::string PipelineFactory::sink_element() const
{
  // No caps= restriction: the appsink accepts whatever format the last
  // action in the chain produces (BGR for resize/crop/flip; whatever
  // target_encoding specifies for colorconvert). The node-side egress
  // code inspects the actual GstVideoFormat and publishes an accordingly
  // encoded sensor_msgs/Image.
  return "appsink name=ros_emit sync=false max-buffers=1 drop=true";
}

std::string PipelineFactory::build() const
{
  std::ostringstream ss;
  ss << source_element();

  const auto & r = registry();
  for (const auto & name : action_names_) {
    const auto & desc = r.at(name);
    ss << " ! " << desc.fragment(platform_, config_);
  }

  ss << " ! " << sink_element();
  return ss.str();
}

std::vector<CameraInfoTransform>
PipelineFactory::camera_info_transforms() const
{
  std::vector<CameraInfoTransform> out;
  out.reserve(action_names_.size());
  const auto & r = registry();
  for (const auto & name : action_names_) {
    const auto & desc = r.at(name);
    if (desc.transform) {
      out.push_back(desc.transform);
    }
  }
  return out;
}

}  // namespace prism
