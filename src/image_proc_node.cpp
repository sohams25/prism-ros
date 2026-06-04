#include "prism_image_proc/image_proc_node.hpp"
#include "prism_image_proc/pipeline_factory.hpp"

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <gst/video/video.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <unordered_set>

using namespace std::chrono_literals;

namespace prism
{

ImageProcNode::ImageProcNode(const rclcpp::NodeOptions & options)
: ImageProcNode(options, "image_proc_node")
{
}

ImageProcNode::ImageProcNode(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: Node(node_name, rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  gst_init(nullptr, nullptr);

  declare_parameters();
  detect_hardware();

  const bool has_working_gpu = gpu_path_available();

  if (has_working_gpu) {
    build_pipeline();
    if (!launch_pipeline()) {
      RCLCPP_WARN(get_logger(),
        "GPU pipeline launch failed at startup — falling back to direct mode.");
      launch_direct();
    }
  } else {
    launch_direct();
  }

  param_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return on_set_parameters(params);
    });
}

ImageProcNode::~ImageProcNode()
{
  shutdown_pipeline();
}

namespace
{

// Build a ParameterDescriptor with an inclusive integer range so out-of-range
// values are rejected by rcl before reaching our callback.
rcl_interfaces::msg::ParameterDescriptor int_range_desc(
  const std::string & description, int64_t lo, int64_t hi)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  rcl_interfaces::msg::IntegerRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = 1;
  d.integer_range.push_back(r);
  return d;
}

rcl_interfaces::msg::ParameterDescriptor dbl_range_desc(
  const std::string & description, double lo, double hi)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  rcl_interfaces::msg::FloatingPointRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = 0.0;  // continuous
  d.floating_point_range.push_back(r);
  return d;
}

constexpr int64_t kMaxPixelDim = 16384;  // generous upper bound (2x 8K)

}  // namespace

void ImageProcNode::declare_parameters()
{
  declare_parameter("input_topic", "/camera/image_raw");
  declare_parameter("output_topic", "/camera/image_processed");
  declare_parameter("action", "resize");
  declare_parameter("source_width", 3840,
    int_range_desc("Source frame width (px); fixes the GPU appsrc caps", 1, kMaxPixelDim));
  declare_parameter("source_height", 2160,
    int_range_desc("Source frame height (px); fixes the GPU appsrc caps", 1, kMaxPixelDim));
  declare_parameter("use_scale", false);
  declare_parameter("scale_height", 1.0,
    dbl_range_desc("Vertical scale factor when use_scale=true", 0.001, 100.0));
  declare_parameter("scale_width", 1.0,
    dbl_range_desc("Horizontal scale factor when use_scale=true", 0.001, 100.0));
  declare_parameter("height", 480,
    int_range_desc("Output height (px) when use_scale=false", 1, kMaxPixelDim));
  declare_parameter("width", 640,
    int_range_desc("Output width (px) when use_scale=false", 1, kMaxPixelDim));
  declare_parameter("publish_camera_info", true);
  declare_parameter("camera_info_input_topic", "");
  declare_parameter("camera_info_output_topic", "");
  declare_parameter("input_transport", "raw");
  declare_parameter("reliable_qos", false);

  // Action-specific parameters (only read when the corresponding action is
  // present in the action chain).
  declare_parameter("target_encoding", "bgr8");
  declare_parameter("crop_x", 0,
    int_range_desc("Crop left offset (px) into the source", 0, kMaxPixelDim));
  declare_parameter("crop_y", 0,
    int_range_desc("Crop top offset (px) into the source", 0, kMaxPixelDim));
  declare_parameter("crop_width", 0,
    int_range_desc("Crop width (px); must be >0 when 'crop' is in the chain", 0, kMaxPixelDim));
  declare_parameter("crop_height", 0,
    int_range_desc("Crop height (px); must be >0 when 'crop' is in the chain", 0, kMaxPixelDim));
  declare_parameter("flip_method", "none");
}

// ---------------------------------------------------------------------------
// Config / CameraInfo helpers
// ---------------------------------------------------------------------------

namespace
{

// Follows the ROS convention used by image_transport::CameraSubscriber:
// the CameraInfo topic for an image at <ns>/<image_name> is <ns>/camera_info.
std::string derive_camera_info_topic(const std::string & image_topic)
{
  const auto slash = image_topic.find_last_of('/');
  if (slash == std::string::npos) {
    return "camera_info";
  }
  return image_topic.substr(0, slash) + "/camera_info";
}

}  // namespace

rclcpp::QoS ImageProcNode::image_qos() const
{
  // Image and CameraInfo streams default to SensorDataQoS (BEST_EFFORT,
  // KEEP_LAST depth 5) so the node interoperates with the standard camera
  // drivers, which publish sensor data BEST_EFFORT. A RELIABLE subscription
  // would silently fail to connect to those publishers. Set reliable_qos:=true
  // for lossless delivery against a RELIABLE source (e.g. a file/replay node).
  if (get_parameter("reliable_qos").as_bool()) {
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  }
  return rclcpp::SensorDataQoS();
}

PipelineConfig ImageProcNode::load_config_from_params() const
{
  PipelineConfig c;
  c.input_topic   = get_parameter("input_topic").as_string();
  c.output_topic  = get_parameter("output_topic").as_string();
  c.action        = get_parameter("action").as_string();
  c.source_width  = get_parameter("source_width").as_int();
  c.source_height = get_parameter("source_height").as_int();

  const bool use_scale = get_parameter("use_scale").as_bool();
  if (use_scale) {
    // Compute scaled dimensions in double and clamp to a sane pixel range
    // before narrowing. An unbounded scale factor would otherwise overflow
    // the double->int conversion (undefined behavior) and yield negative or
    // garbage target dimensions that crash cv::resize / GStreamer caps.
    constexpr double kMinDim = 1.0;
    constexpr double kMaxDim = 16384.0;  // generous: 2x 8K
    const double tw = c.source_width  * get_parameter("scale_width").as_double();
    const double th = c.source_height * get_parameter("scale_height").as_double();
    if (tw > kMaxDim || th > kMaxDim) {
      RCLCPP_WARN(get_logger(),
        "Scaled target %.0fx%.0f exceeds the %.0f-px clamp; clamping. "
        "Reduce scale_width/scale_height.", tw, th, kMaxDim);
    }
    c.target_width  = static_cast<int>(std::clamp(tw, kMinDim, kMaxDim));
    c.target_height = static_cast<int>(std::clamp(th, kMinDim, kMaxDim));
  } else {
    c.target_width  = get_parameter("width").as_int();
    c.target_height = get_parameter("height").as_int();
  }

  c.target_encoding = get_parameter("target_encoding").as_string();
  c.crop_x          = get_parameter("crop_x").as_int();
  c.crop_y          = get_parameter("crop_y").as_int();
  c.crop_width      = get_parameter("crop_width").as_int();
  c.crop_height     = get_parameter("crop_height").as_int();
  c.flip_method     = get_parameter("flip_method").as_string();
  return c;
}

void ImageProcNode::setup_camera_info_io(
  const std::string & image_input_topic,
  const std::string & image_output_topic)
{
  publish_camera_info_ = get_parameter("publish_camera_info").as_bool();
  if (!publish_camera_info_) { return; }

  std::string input_info_topic  = get_parameter("camera_info_input_topic").as_string();
  std::string output_info_topic = get_parameter("camera_info_output_topic").as_string();
  if (input_info_topic.empty())  { input_info_topic  = derive_camera_info_topic(image_input_topic); }
  if (output_info_topic.empty()) { output_info_topic = derive_camera_info_topic(image_output_topic); }

  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    input_info_topic, image_qos(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
      on_camera_info(std::move(msg));
    });
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_info_topic, image_qos());

  RCLCPP_INFO(get_logger(),
    "CameraInfo: subscribing %s, publishing %s "
    "(transforms applied: %zu)",
    input_info_topic.c_str(), output_info_topic.c_str(),
    info_transforms_.size());
}

void ImageProcNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  last_info_ = std::move(msg);
}

// ---------------------------------------------------------------------------
// Runtime reconfiguration
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult ImageProcNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  static const std::unordered_set<std::string> rebuild_triggers = {
    "width", "height", "use_scale", "scale_width", "scale_height",
    "action", "target_encoding",
    "crop_x", "crop_y", "crop_width", "crop_height",
    "flip_method",
    "input_topic", "output_topic",
    "camera_info_input_topic", "camera_info_output_topic",
    "publish_camera_info",
    "input_transport",
    "reliable_qos",
    "source_width", "source_height",
  };

  bool needs_rebuild = false;
  for (const auto & p : params) {
    if (rebuild_triggers.count(p.get_name())) { needs_rebuild = true; }

    if (p.get_name() == "action") {
      try {
        (void)PipelineFactory::parse_action_chain(p.as_string());
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = e.what();
        return result;
      }
    } else if (p.get_name() == "width" || p.get_name() == "height" ||
               p.get_name() == "source_width" || p.get_name() == "source_height")
    {
      if (p.as_int() <= 0) {
        result.successful = false;
        result.reason = p.get_name() + " must be positive";
        return result;
      }
    } else if (p.get_name() == "scale_width" || p.get_name() == "scale_height") {
      if (p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = p.get_name() + " must be positive";
        return result;
      }
    }
  }

  // Dry-run the full proposed configuration through the pipeline factory so
  // invalid *combinations* — an unsupported target_encoding, a bad flip_method,
  // or a crop action with non-positive / out-of-bounds crop dimensions — are
  // rejected here (successful=false, with the factory's reason) instead of
  // being accepted now and then throwing an uncaught std::invalid_argument
  // from the deferred rebuild timer, which would terminate the node.
  if (needs_rebuild) {
    PipelineConfig trial = load_config_from_params();
    // Track the inputs that determine the output dimensions so we can recompute
    // target_width/height exactly the way load_config_from_params would, but
    // against the *proposed* values (it only sees the currently-committed ones).
    bool use_scale = get_parameter("use_scale").as_bool();
    double scale_w = get_parameter("scale_width").as_double();
    double scale_h = get_parameter("scale_height").as_double();
    int abs_w = get_parameter("width").as_int();
    int abs_h = get_parameter("height").as_int();
    for (const auto & p : params) {
      const auto & n = p.get_name();
      if (n == "action")              { trial.action = p.as_string(); }
      else if (n == "target_encoding") { trial.target_encoding = p.as_string(); }
      else if (n == "flip_method")     { trial.flip_method = p.as_string(); }
      else if (n == "crop_x")          { trial.crop_x = static_cast<int>(p.as_int()); }
      else if (n == "crop_y")          { trial.crop_y = static_cast<int>(p.as_int()); }
      else if (n == "crop_width")      { trial.crop_width = static_cast<int>(p.as_int()); }
      else if (n == "crop_height")     { trial.crop_height = static_cast<int>(p.as_int()); }
      else if (n == "source_width")    { trial.source_width = static_cast<int>(p.as_int()); }
      else if (n == "source_height")   { trial.source_height = static_cast<int>(p.as_int()); }
      else if (n == "use_scale")       { use_scale = p.as_bool(); }
      else if (n == "scale_width")     { scale_w = p.as_double(); }
      else if (n == "scale_height")    { scale_h = p.as_double(); }
      else if (n == "width")           { abs_w = static_cast<int>(p.as_int()); }
      else if (n == "height")          { abs_h = static_cast<int>(p.as_int()); }
    }
    if (use_scale) {
      trial.target_width  = static_cast<int>(trial.source_width  * scale_w);
      trial.target_height = static_cast<int>(trial.source_height * scale_h);
    } else {
      trial.target_width  = abs_w;
      trial.target_height = abs_h;
    }
    try {
      PipelineFactory trial_factory(platform_info_, trial);
      (void)trial_factory.build();
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
      return result;
    }
  }

  if (needs_rebuild) {
    schedule_rebuild();
  }
  return result;
}

void ImageProcNode::schedule_rebuild()
{
  if (rebuild_timer_) { return; }  // a rebuild is already pending
  rebuild_timer_ = create_wall_timer(
    std::chrono::milliseconds(1),
    [this]() {
      if (rebuild_timer_) { rebuild_timer_->cancel(); }
      rebuild_timer_.reset();
      rebuild_from_params();
    });
}

void ImageProcNode::rebuild_from_params()
{
  RCLCPP_INFO(get_logger(), "Parameter change detected — rebuilding pipeline");

  // Teardown order: subscriptions first (stop accepting new work),
  // then GStreamer pipeline, then publishers.
  image_sub_.reset();
  it_sub_.shutdown();
  info_sub_.reset();
  shutdown_pipeline();
  output_pub_.reset();
  info_pub_.reset();
  info_transforms_.clear();

  const bool has_working_gpu = gpu_path_available();
  direct_mode_ = !has_working_gpu;

  // on_set_parameters dry-runs the pipeline build, so a rebuild reaching this
  // point is expected to succeed. The try/catch is a backstop: a build/launch
  // exception here runs on the one-shot timer thread, where an escape would
  // terminate the executor. Degrade to direct mode (which constructs no
  // GStreamer fragment and so cannot re-trigger per-action validation) so the
  // node stays alive and observable.
  try {
    if (has_working_gpu) {
      build_pipeline();
      if (!launch_pipeline()) {
        RCLCPP_WARN(get_logger(),
          "GPU pipeline launch failed on rebuild — falling back to direct mode.");
        direct_mode_ = true;
        launch_direct();
      }
    } else {
      launch_direct();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(),
      "Pipeline rebuild failed (%s) — falling back to direct mode.", e.what());
    image_sub_.reset();
    it_sub_.shutdown();
    shutdown_pipeline();
    output_pub_.reset();
    info_pub_.reset();
    info_sub_.reset();
    info_transforms_.clear();
    direct_mode_ = true;
    try {
      launch_direct();
    } catch (const std::exception & e2) {
      RCLCPP_FATAL(get_logger(),
        "Direct-mode fallback also failed (%s); node is idle until "
        "reconfigured.", e2.what());
    }
  }
}

void ImageProcNode::publish_transformed_camera_info(
  const std_msgs::msg::Header & image_header)
{
  if (!publish_camera_info_ || !info_pub_) { return; }

  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_in;
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    info_in = last_info_;
  }
  if (!info_in) { return; }

  // Drop the stale-info case: if the cached CameraInfo is more than one
  // frame period older than the image, skip (the upstream publisher will
  // send a fresh one shortly).
  sensor_msgs::msg::CameraInfo out = *info_in;
  out.header = image_header;

  for (const auto & transform : info_transforms_) {
    transform(out, config_);
  }
  info_pub_->publish(out);
}

void ImageProcNode::detect_hardware()
{
  HardwareDetector detector;
  platform_info_ = detector.detect_platform();

  RCLCPP_INFO(get_logger(), "Detected platform: %s",
    to_string(platform_info_.platform));

  if (!platform_info_.render_device.empty()) {
    RCLCPP_INFO(get_logger(), "Render device: %s",
      platform_info_.render_device.c_str());
  }

  for (const auto & path : platform_info_.evidence) {
    RCLCPP_DEBUG(get_logger(), "  evidence: %s", path.c_str());
  }

  platform_info_ = validate_platform(platform_info_, get_logger());
}

bool ImageProcNode::gpu_path_available() const
{
  // On GStreamer 1.20 vaapipostproc exists but has a chroma-loss bug; only
  // vapostproc (1.22+) or the Jetson NVMM path are treated as working GPUs.
  return (platform_info_.platform == HardwarePlatform::NVIDIA_JETSON) ||
         (platform_info_.platform == HardwarePlatform::INTEL_VAAPI &&
          factory_exists("vapostproc"));
}

void ImageProcNode::build_pipeline()
{
  config_ = load_config_from_params();
  target_w_ = config_.target_width;
  target_h_ = config_.target_height;

  PipelineFactory factory(platform_info_, config_);
  pipeline_string_ = factory.build();
  info_transforms_ = factory.camera_info_transforms();

  RCLCPP_INFO(get_logger(), "Generated pipeline: %s", pipeline_string_.c_str());
}

// ---------------------------------------------------------------------------
// GStreamer execution: appsrc (input) + appsink (output)
// ---------------------------------------------------------------------------

// Returns true if the GStreamer pipeline started and ROS I/O is wired.
// Returns false (after fully releasing any partial GStreamer/ROS state) if the
// pipeline could not be parsed, the appsrc/appsink could not be found, or the
// state change to PLAYING failed — letting the caller fall back to direct mode
// instead of leaving a live-but-silent node.
bool ImageProcNode::launch_pipeline()
{
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_string_.c_str(), &error);

  if (error) {
    RCLCPP_ERROR(get_logger(), "Pipeline parse error: %s", error->message);
    g_error_free(error);
    // gst_parse_launch may return a partially-built pipeline alongside the
    // error; unref it before discarding so it (and its children) are freed.
    if (pipeline_) {
      gst_object_unref(pipeline_);
    }
    pipeline_ = nullptr;
    return false;
  }

  // Get appsrc (input)
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_ingest");
  if (!appsrc_) {
    RCLCPP_ERROR(get_logger(), "Failed to find appsrc 'ros_ingest'");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }

  g_object_set(G_OBJECT(appsrc_),
    "is-live", TRUE,
    "format", GST_FORMAT_TIME,
    "block", FALSE,
    "max-bytes", static_cast<guint64>(75 * 1024 * 1024),  // ~3 BGR 4K frames
    "max-buffers", static_cast<guint64>(3),
    nullptr);

  // Get appsink (output) and wire up the new-sample signal
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_emit");
  if (!appsink_) {
    RCLCPP_ERROR(get_logger(), "Failed to find appsink 'ros_emit'");
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }

  g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, nullptr);
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);

  // Start the pipeline BEFORE wiring ROS publishers/subscribers. A state-change
  // failure then happens with no ROS endpoints created, so the failure path is
  // a clean GStreamer-only teardown and the caller can fall back cleanly.
  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING");
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(appsink_);
    appsink_ = nullptr;
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }

  // Pipeline is live — now create the ROS output publisher, CameraInfo I/O,
  // and the input subscription that feeds appsrc.
  auto output_topic = get_parameter("output_topic").as_string();
  output_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, image_qos());

  setup_camera_info_io(config_.input_topic, config_.output_topic);

  auto input_topic = get_parameter("input_topic").as_string();
  const auto transport = get_parameter("input_transport").as_string();
  if (transport == "raw") {
    // Fast path: UniquePtr intra-process zero-copy ingest.
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic, image_qos(),
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        on_image(std::move(msg));
      });
  } else {
    // image_transport hands us a ConstSharedPtr (the transport plugin has
    // already decoded compressed/theora/etc.). We copy once into a
    // UniquePtr to re-enter the fast path; the copy is inherent to
    // decompression.
    it_sub_ = image_transport::create_subscription(
      this, input_topic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        auto owned = std::make_unique<sensor_msgs::msg::Image>(*msg);
        on_image(std::move(owned));
      },
      transport, image_qos().get_rmw_qos_profile());
  }

  RCLCPP_INFO(get_logger(),
    "Pipeline launched — appsrc(%s, transport=%s) -> appsink(%s) "
    "(raw ingest is zero-copy; egress is a copy)",
    input_topic.c_str(), transport.c_str(), output_topic.c_str());

  bus_timer_ = create_wall_timer(100ms, std::bind(&ImageProcNode::poll_bus, this));
  return true;
}

// ---------------------------------------------------------------------------
// appsrc input: zero-copy buffer wrapping (intra-process ingest).
// Egress out of the appsink (see on_new_sample) performs a single copy.
// ---------------------------------------------------------------------------

void ImageProcNode::destroy_ros_image(gpointer user_data)
{
  auto * msg = static_cast<sensor_msgs::msg::Image *>(user_data);
  delete msg;
}

void ImageProcNode::on_image(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!appsrc_) { return; }

  // The appsrc caps are fixed at video/x-raw,format=BGR with the source
  // width/height (see PipelineFactory::source_element). Pushing a buffer
  // whose encoding or byte layout disagrees with those caps either silently
  // corrupts the output (wrong encoding, same size) or errors the whole
  // pipeline (size mismatch). Validate at the boundary and drop loudly.
  if (msg->encoding != "bgr8") {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Ingest expects bgr8 (appsrc caps are format=BGR); got '%s' — dropping frame",
      msg->encoding.c_str());
    return;
  }
  // The appsrc caps describe tightly-packed BGR (stride == width*3). Require an
  // exact stride: a row-padded frame (step > width*3) would feed misaligned
  // rows into the pipeline (corruption) or trip a buffer-size/caps mismatch.
  const size_t expected_step = static_cast<size_t>(msg->width) * 3u;
  const size_t expected_size = expected_step * msg->height;
  if (msg->width == 0 || msg->height == 0 ||
      msg->step != expected_step || msg->data.size() < expected_size)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Malformed or row-padded bgr8 frame (w=%u h=%u step=%u, expected step=%zu, "
      "size=%zu) — dropping; the GPU appsrc caps require tightly-packed BGR rows",
      msg->width, msg->height, msg->step, expected_step, msg->data.size());
    return;
  }
  if (msg->width != static_cast<uint32_t>(config_.source_width) ||
      msg->height != static_cast<uint32_t>(config_.source_height))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Frame %ux%u does not match configured source_width/source_height "
      "(%dx%d); the GPU pipeline caps are fixed — dropping. Set source_width/"
      "source_height to the camera resolution.",
      msg->width, msg->height, config_.source_width, config_.source_height);
    return;
  }

  // Extract the ROS timestamp BEFORE releasing ownership
  GstClockTime pts =
    static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
    msg->header.stamp.nanosec;

  auto * raw_msg = msg.release();
  uint8_t * data = raw_msg->data.data();
  gsize size = raw_msg->data.size();

  GstBuffer * buffer = gst_buffer_new_wrapped_full(
    static_cast<GstMemoryFlags>(0),
    data, size, 0, size,
    raw_msg, destroy_ros_image);

  if (!buffer) {
    // Wrap failed — no GstBuffer took ownership, so free the released payload
    // ourselves to avoid leaking it, and skip the null deref of GST_BUFFER_PTS.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to wrap frame into a GstBuffer — dropping frame");
    destroy_ros_image(raw_msg);
    return;
  }

  // Tunnel the ROS timestamp through GStreamer as the buffer PTS
  GST_BUFFER_PTS(buffer) = pts;

  GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if (flow != GST_FLOW_OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "appsrc push failed: %s", gst_flow_get_name(flow));
  }
}

// ---------------------------------------------------------------------------
// appsink output: pull processed frames and publish as ROS Image.
// Called on GStreamer's streaming thread — publishers are thread-safe.
// ---------------------------------------------------------------------------

GstFlowReturn ImageProcNode::on_new_sample(GstAppSink * sink, gpointer user_data)
{
  auto * self = static_cast<ImageProcNode *>(user_data);

  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (!sample) { return GST_FLOW_ERROR; }

  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstCaps * caps = gst_sample_get_caps(sample);

  GstVideoInfo info;
  if (!gst_video_info_from_caps(&info, caps)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  int w = info.width;
  int h = info.height;
  GstVideoFormat fmt = GST_VIDEO_INFO_FORMAT(&info);

  // Where formats map 1:1 to a sensor_msgs encoding (BGR, RGB, GRAY8), we
  // publish them as-is so colorconvert actually takes effect end-to-end.
  // All other formats are converted to BGR via OpenCV and published as bgr8.
  cv::Mat out_mat;
  std::string out_encoding;
  int out_channels = 0;  // bytes per pixel (used to compute step)

  if (fmt == GST_VIDEO_FORMAT_BGR) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC3,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    out_mat = raw.clone();
    out_encoding = "bgr8";
    out_channels = 3;
  } else if (fmt == GST_VIDEO_FORMAT_RGB) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC3,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    out_mat = raw.clone();
    out_encoding = "rgb8";
    out_channels = 3;
  } else if (fmt == GST_VIDEO_FORMAT_GRAY8) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC1,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    out_mat = raw.clone();
    out_encoding = "mono8";
    out_channels = 1;
  } else if (fmt == GST_VIDEO_FORMAT_BGRA || fmt == GST_VIDEO_FORMAT_BGRx) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC4,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    cv::cvtColor(raw, out_mat, cv::COLOR_BGRA2BGR);
    out_encoding = "bgr8";
    out_channels = 3;
  } else if (fmt == GST_VIDEO_FORMAT_RGBA || fmt == GST_VIDEO_FORMAT_RGBx) {
    // RGBA at egress means the upstream stage produced 4-channel R,G,B,A
    // and the user's downstream contract is rgb8 (drop alpha, keep order).
    // cv::cvtColor with COLOR_RGBA2RGB performs the channel drop in place.
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC4,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    cv::cvtColor(raw, out_mat, cv::COLOR_RGBA2RGB);
    out_encoding = "rgb8";
    out_channels = 3;
  } else if (fmt == GST_VIDEO_FORMAT_NV12 ||
             fmt == GST_VIDEO_FORMAT_I420 ||
             fmt == GST_VIDEO_FORMAT_YV12) {
    // Planar 4:2:0 formats require even width/height; odd dimensions would
    // truncate a chroma row/column and (pre-fix) overran the assembly Mat.
    // GStreamer caps negotiation enforces even dims for these formats, so an
    // odd value here signals a malformed buffer — drop it loudly rather than
    // read/write out of bounds.
    if ((w & 1) || (h & 1)) {
      RCLCPP_WARN_THROTTLE(self->get_logger(), *self->get_clock(), 2000,
        "Planar YUV egress with odd dimensions %dx%d — skipping frame", w, h);
      gst_video_frame_unmap(&frame);
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }
    const int cw = w / 2;  // chroma width
    const int ch = h / 2;  // chroma height
    // Assemble into the contiguous (h*3/2 x w) layout OpenCV's YUV2BGR
    // conversions expect: Y (h rows of w), then the two 4:2:0 chroma planes
    // packed back-to-back. Writing through a raw base pointer keeps every
    // byte inside the h*3/2*w allocation (the previous ptr(row)-per-line
    // packing wrote the V plane up to h/2 rows past the Mat — a heap
    // overflow — and left half of each chroma row uninitialised).
    cv::Mat yuv(h * 3 / 2, w, CV_8UC1);
    uint8_t * dst = yuv.data;
    const auto * y_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    const int ys = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    for (int r = 0; r < h; ++r) {
      std::memcpy(dst + static_cast<size_t>(r) * w, y_src + static_cast<size_t>(r) * ys, w);
    }

    if (fmt == GST_VIDEO_FORMAT_NV12) {
      // Single interleaved UV plane: ch rows of w bytes (cw UV pairs).
      const auto * uv_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 1));
      const int uvs = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
      uint8_t * uv_dst = dst + static_cast<size_t>(h) * w;
      for (int r = 0; r < ch; ++r) {
        std::memcpy(uv_dst + static_cast<size_t>(r) * w, uv_src + static_cast<size_t>(r) * uvs, w);
      }
      cv::cvtColor(yuv, out_mat, cv::COLOR_YUV2BGR_NV12);
    } else {
      // I420 (Y,U,V) / YV12 (Y,V,U): two separate ch x cw chroma planes.
      // GstVideoFrame plane indices follow the format's native plane order,
      // and the matching cv code expects that same order, so packing
      // plane(1) then plane(2) contiguously is correct for both.
      const auto * c0_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 1));
      const auto * c1_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 2));
      const int c0s = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
      const int c1s = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 2);
      uint8_t * c0_dst = dst + static_cast<size_t>(h) * w;
      uint8_t * c1_dst = c0_dst + static_cast<size_t>(ch) * cw;
      for (int r = 0; r < ch; ++r) {
        std::memcpy(c0_dst + static_cast<size_t>(r) * cw, c0_src + static_cast<size_t>(r) * c0s, cw);
      }
      for (int r = 0; r < ch; ++r) {
        std::memcpy(c1_dst + static_cast<size_t>(r) * cw, c1_src + static_cast<size_t>(r) * c1s, cw);
      }
      const int code = (fmt == GST_VIDEO_FORMAT_I420) ?
        cv::COLOR_YUV2BGR_I420 : cv::COLOR_YUV2BGR_YV12;
      cv::cvtColor(yuv, out_mat, code);
    }
    out_encoding = "bgr8";
    out_channels = 3;
  } else {
    RCLCPP_WARN_THROTTLE(self->get_logger(), *self->get_clock(), 2000,
      "Unsupported appsink format: %s — skipping",
      gst_video_format_to_string(fmt));
    gst_video_frame_unmap(&frame);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  // Extract PTS before unmapping
  GstClockTime pts = GST_BUFFER_PTS(buffer);

  gst_video_frame_unmap(&frame);
  gst_sample_unref(sample);

  // Publish with the encoding produced by the pipeline (bgr8, rgb8, or
  // mono8 if colorconvert was in the chain; bgr8 otherwise).
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  if (GST_CLOCK_TIME_IS_VALID(pts)) {
    msg->header.stamp.sec = static_cast<int32_t>(pts / 1000000000ULL);
    msg->header.stamp.nanosec = static_cast<uint32_t>(pts % 1000000000ULL);
  } else {
    msg->header.stamp = self->now();
  }
  msg->header.frame_id = "media_frame";
  msg->width = out_mat.cols;
  msg->height = out_mat.rows;
  msg->encoding = out_encoding;
  msg->step = out_mat.cols * out_channels;
  msg->is_bigendian = false;
  msg->data.assign(out_mat.data, out_mat.data + out_mat.total() * out_mat.elemSize());

  const auto image_header = msg->header;
  self->output_pub_->publish(std::move(msg));
  self->publish_transformed_camera_info(image_header);
  // A delivered frame proves the pipeline is healthy: clear the recovery
  // budget so transient errors that later recover don't accumulate across the
  // node's lifetime toward permanent direct-mode degradation.
  self->bus_error_count_.store(0, std::memory_order_relaxed);
  return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Bus message pump
// ---------------------------------------------------------------------------

void ImageProcNode::poll_bus()
{
  if (!pipeline_) { return; }

  // Total GPU-pipeline recovery attempts over the node's lifetime. Once
  // exhausted, the node degrades to direct mode permanently rather than
  // flapping or dying silently.
  constexpr int kMaxBusErrorRetries = 3;

  GstBus * bus = gst_element_get_bus(pipeline_);
  GstMessage * msg = gst_bus_pop_filtered(
    bus,
    static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  if (msg) {
    bool recoverable = false;
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        RCLCPP_ERROR(get_logger(), "GStreamer error: %s", err->message);
        if (debug) {
          RCLCPP_DEBUG(get_logger(), "  debug: %s", debug);
          g_free(debug);
        }
        g_error_free(err);
        recoverable = true;
        break;
      }
      case GST_MESSAGE_EOS:
        // appsrc is a live source we never send EOS to, so EOS on this bus is
        // abnormal — treat it as a recoverable pipeline fault.
        RCLCPP_WARN(get_logger(), "GStreamer end-of-stream (unexpected for a live source)");
        recoverable = true;
        break;
      default:
        break;
    }
    gst_message_unref(msg);
    gst_object_unref(bus);

    if (recoverable) {
      const int errors = bus_error_count_.fetch_add(1) + 1;
      if (errors < kMaxBusErrorRetries) {
        RCLCPP_WARN(get_logger(),
          "Attempting GPU pipeline recovery (%d/%d)…",
          errors, kMaxBusErrorRetries);
        schedule_rebuild();  // tears down and rebuilds from current params
      } else {
        RCLCPP_ERROR(get_logger(),
          "GPU pipeline failed %d times without recovering — degrading to "
          "direct mode permanently.", kMaxBusErrorRetries);
        // Cancel any rebuild already queued by a prior error, or it would fire
        // after this fallback and tear down the direct-mode pipeline we install.
        if (rebuild_timer_) {
          rebuild_timer_->cancel();
          rebuild_timer_.reset();
        }
        image_sub_.reset();
        it_sub_.shutdown();
        shutdown_pipeline();
        output_pub_.reset();
        info_pub_.reset();
        info_sub_.reset();
        info_transforms_.clear();
        direct_mode_ = true;
        try {
          launch_direct();
        } catch (const std::exception & e) {
          RCLCPP_FATAL(get_logger(),
            "Direct-mode fallback failed (%s); node is idle.", e.what());
        }
      }
    }
    return;
  }
  gst_object_unref(bus);
}

// ---------------------------------------------------------------------------
// Teardown
// ---------------------------------------------------------------------------

void ImageProcNode::shutdown_pipeline()
{
  if (bus_timer_) {
    bus_timer_->cancel();
    bus_timer_.reset();
  }

  image_sub_.reset();
  it_sub_.shutdown();  // symmetric teardown for the image_transport ingest path

  // Drive the pipeline to NULL FIRST. The downward transition stops and joins
  // the appsink streaming thread, so no further on_new_sample callback can run
  // before we drop our element references — eliminating any window where the
  // streaming thread touches appsrc_/appsink_ after they have been unref'd.
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }

  if (appsink_) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }

  if (appsrc_) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }

  if (pipeline_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    RCLCPP_INFO(get_logger(), "Pipeline shut down");
  }
}

// ---------------------------------------------------------------------------
// Direct mode — OpenCV resize, zero GStreamer overhead.
// Used when no working GPU accelerator is available.
// ---------------------------------------------------------------------------

void ImageProcNode::launch_direct()
{
  direct_mode_ = true;

  config_ = load_config_from_params();
  target_w_ = config_.target_width;
  target_h_ = config_.target_height;

  // Direct mode implements only the 'resize' action (cv::resize). If the
  // configured action chain contains anything else, warn and fall through
  // to resize-only behavior — the action chain is a GPU-path feature.
  PipelineFactory factory(platform_info_, config_);
  const auto & chain = factory.action_chain();
  if (chain.size() != 1 || chain[0] != "resize") {
    RCLCPP_WARN(get_logger(),
      "Action chain '%s' contains non-resize operations; direct mode "
      "(no GPU) supports only 'resize'. Running cv::resize only and "
      "skipping the other actions. Enable a GPU backend to use the full chain.",
      config_.action.c_str());
    // Collapse info_transforms_ to the resize transform only, so the
    // published CameraInfo matches what direct mode actually does.
    PipelineConfig resize_only = config_;
    resize_only.action = "resize";
    PipelineFactory resize_factory(platform_info_, resize_only);
    info_transforms_ = resize_factory.camera_info_transforms();
  } else {
    info_transforms_ = factory.camera_info_transforms();
  }

  output_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.output_topic, image_qos());

  const auto transport = get_parameter("input_transport").as_string();
  if (transport == "raw") {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      config_.input_topic, image_qos(),
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        on_image_direct(std::move(msg));
      });
  } else {
    it_sub_ = image_transport::create_subscription(
      this, config_.input_topic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        auto owned = std::make_unique<sensor_msgs::msg::Image>(*msg);
        on_image_direct(std::move(owned));
      },
      transport, image_qos().get_rmw_qos_profile());
  }

  setup_camera_info_io(config_.input_topic, config_.output_topic);

  RCLCPP_INFO(get_logger(),
    "Direct mode — cv::resize(%s, transport=%s → %dx%d) on %s (no GStreamer overhead)",
    config_.input_topic.c_str(), transport.c_str(),
    target_w_, target_h_, config_.output_topic.c_str());
}

void ImageProcNode::on_image_direct(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!output_pub_) { return; }
  if (msg->width == 0 || msg->height == 0) { return; }

  // Direct mode reinterprets the payload as a 3-channel BGR cv::Mat. Without
  // this guard a non-bgr8 publisher causes silent channel corruption (rgb8)
  // or, for sub-3-byte encodings (mono8, step==0), an out-of-bounds read in
  // cv::resize. Reject anything that is not a well-formed bgr8 frame.
  if (msg->encoding != "bgr8") {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Direct mode expects bgr8 input; got '%s' — dropping frame "
      "(enable a GPU backend or republish as bgr8)", msg->encoding.c_str());
    return;
  }
  const size_t expected_step = static_cast<size_t>(msg->width) * 3u;
  const size_t expected_size = static_cast<size_t>(msg->step) * msg->height;
  if (msg->step < expected_step || msg->data.size() < expected_size) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Malformed bgr8 frame (w=%u h=%u step=%u size=%zu) — dropping",
      msg->width, msg->height, msg->step, msg->data.size());
    return;
  }

  // Wrap the incoming BGR data as a cv::Mat (zero-copy reference into
  // the intra-process UniquePtr; the cv::resize call below writes to a
  // fresh dst, and the publish step then copies dst into the outgoing
  // Image message).
  cv::Mat src(msg->height, msg->width, CV_8UC3, msg->data.data(), msg->step);

  // Resize
  cv::Mat dst;
  cv::resize(src, dst, cv::Size(target_w_, target_h_), 0, 0, cv::INTER_LINEAR);

  // Publish
  auto out = std::make_unique<sensor_msgs::msg::Image>();
  out->header = msg->header;
  out->width = dst.cols;
  out->height = dst.rows;
  out->encoding = "bgr8";
  out->step = dst.cols * 3;
  out->is_bigendian = false;
  out->data.assign(dst.data, dst.data + dst.total() * dst.elemSize());

  const auto image_header = out->header;
  output_pub_->publish(std::move(out));
  publish_transformed_camera_info(image_header);
}

}  // namespace prism

RCLCPP_COMPONENTS_REGISTER_NODE(prism::ImageProcNode)
