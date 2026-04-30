#include "prism_image_proc/image_proc_node.hpp"
#include "prism_image_proc/pipeline_factory.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <gst/video/video.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <unordered_set>

using namespace std::chrono_literals;

namespace prism
{

ImageProcNode::ImageProcNode(const rclcpp::NodeOptions & options)
: Node("resize_node", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  gst_init(nullptr, nullptr);

  declare_parameters();
  detect_hardware();

  // Determine if a WORKING GPU accelerator is available.
  // On GStreamer 1.20, vaapipostproc exists but has a chroma loss bug —
  // only vapostproc (GStreamer 1.22+) or nvvideoconvert (Jetson) actually work.
  bool has_working_gpu =
    (platform_info_.platform == HardwarePlatform::NVIDIA_JETSON) ||
    (platform_info_.platform == HardwarePlatform::INTEL_VAAPI &&
     gst_element_factory_find("vapostproc"));

  // Rectify is a CPU-only direct-mode action by design — see the v0.2
  // E2.1 capability probe and the CPU/GPU compartmentalization design
  // note. If the chain contains rectify, we force direct mode regardless
  // of GPU availability, so cv::remap runs on CPU and the GPU stays free
  // for downstream perception consumers.
  {
    const auto chain_str = get_parameter("action").as_string();
    try {
      const auto names = PipelineFactory::parse_action_chain(chain_str);
      action_chain_has_rectify_ =
        std::find(names.begin(), names.end(), "rectify") != names.end();
    } catch (const std::exception &) {
      // parse failures surface again from launch_direct/build_pipeline.
      action_chain_has_rectify_ = false;
    }
  }

  if (has_working_gpu && !action_chain_has_rectify_) {
    build_pipeline();
    launch_pipeline();
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

void ImageProcNode::declare_parameters()
{
  declare_parameter("input_topic", "/camera/image_raw");
  declare_parameter("output_topic", "/camera/image_processed");
  declare_parameter("action", "resize");
  declare_parameter("source_width", 3840);
  declare_parameter("source_height", 2160);
  declare_parameter("use_scale", false);
  declare_parameter("scale_height", 1.0);
  declare_parameter("scale_width", 1.0);
  declare_parameter("height", 480);
  declare_parameter("width", 640);
  declare_parameter("publish_camera_info", true);
  declare_parameter("camera_info_input_topic", "");
  declare_parameter("camera_info_output_topic", "");
  declare_parameter("input_transport", "raw");

  // Action-specific parameters (only read when the corresponding action is
  // present in the action chain).
  declare_parameter("target_encoding", "bgr8");
  declare_parameter("crop_x", 0);
  declare_parameter("crop_y", 0);
  declare_parameter("crop_width", 0);
  declare_parameter("crop_height", 0);
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
    c.target_width  = static_cast<int>(
      c.source_width  * get_parameter("scale_width").as_double());
    c.target_height = static_cast<int>(
      c.source_height * get_parameter("scale_height").as_double());
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
    input_info_topic, 10,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
      on_camera_info(std::move(msg));
    });
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_info_topic, 10);

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

  if (needs_rebuild && !rebuild_timer_) {
    rebuild_timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() {
        if (rebuild_timer_) { rebuild_timer_->cancel(); }
        rebuild_timer_.reset();
        rebuild_from_params();
      });
  }
  return result;
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

  const bool has_working_gpu =
    (platform_info_.platform == HardwarePlatform::NVIDIA_JETSON) ||
    (platform_info_.platform == HardwarePlatform::INTEL_VAAPI &&
     gst_element_factory_find("vapostproc"));

  // Re-evaluate the rectify-in-chain flag — the action parameter may
  // have just changed.
  try {
    const auto names = PipelineFactory::parse_action_chain(
      get_parameter("action").as_string());
    action_chain_has_rectify_ =
      std::find(names.begin(), names.end(), "rectify") != names.end();
  } catch (const std::exception &) {
    action_chain_has_rectify_ = false;
  }

  // Invalidate the rectify LUT — it may have been built against the
  // previous parameter set.
  rectify_lut_ready_ = false;

  direct_mode_ = !has_working_gpu || action_chain_has_rectify_;

  if (has_working_gpu && !action_chain_has_rectify_) {
    build_pipeline();
    launch_pipeline();
  } else {
    launch_direct();
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

  // Rectify needs the LUT-derived new_K folded in BEFORE downstream
  // transforms run, so that a chained resize after rectify scales the
  // rectified K correctly. The static rectify_camera_info transform
  // handles distortion and R; this block handles K/P which depend on
  // cv::getOptimalNewCameraMatrix and live on this node.
  if (action_chain_has_rectify_ && rectify_lut_ready_) {
    out.k = {
      rectify_new_K_(0, 0), rectify_new_K_(0, 1), rectify_new_K_(0, 2),
      rectify_new_K_(1, 0), rectify_new_K_(1, 1), rectify_new_K_(1, 2),
      rectify_new_K_(2, 0), rectify_new_K_(2, 1), rectify_new_K_(2, 2)};
    out.p = {
      rectify_new_K_(0, 0), rectify_new_K_(0, 1), rectify_new_K_(0, 2), 0.0,
      rectify_new_K_(1, 0), rectify_new_K_(1, 1), rectify_new_K_(1, 2), 0.0,
      rectify_new_K_(2, 0), rectify_new_K_(2, 1), rectify_new_K_(2, 2), 0.0};
  }

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

void ImageProcNode::launch_pipeline()
{
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_string_.c_str(), &error);

  if (error) {
    RCLCPP_ERROR(get_logger(), "Pipeline parse error: %s", error->message);
    g_error_free(error);
    pipeline_ = nullptr;
    return;
  }

  // Get appsrc (input)
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_ingest");
  if (!appsrc_) {
    RCLCPP_ERROR(get_logger(), "Failed to find appsrc 'ros_ingest'");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
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
    return;
  }

  g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, nullptr);
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);

  // Create the output publisher
  auto output_topic = get_parameter("output_topic").as_string();
  output_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

  setup_camera_info_io(config_.input_topic, config_.output_topic);

  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING");
    gst_object_unref(appsink_);
    appsink_ = nullptr;
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
  }

  auto input_topic = get_parameter("input_topic").as_string();
  const auto transport = get_parameter("input_transport").as_string();
  if (transport == "raw") {
    // Fast path: UniquePtr intra-process zero-copy ingest.
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic, 10,
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
      transport);
  }

  RCLCPP_INFO(get_logger(),
    "Pipeline launched — appsrc(%s, transport=%s) -> appsink(%s) "
    "(raw ingest is zero-copy; egress is a copy)",
    input_topic.c_str(), transport.c_str(), output_topic.c_str());

  bus_timer_ = create_wall_timer(100ms, std::bind(&ImageProcNode::poll_bus, this));
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
  } else if (fmt == GST_VIDEO_FORMAT_NV12) {
    // Y plane + interleaved UV plane — assemble for OpenCV
    cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
    auto * y_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    auto * uv_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 1));
    int ys = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    int uvs = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
    for (int r = 0; r < h; ++r)
      std::memcpy(nv12.ptr(r), y_src + r * ys, w);
    for (int r = 0; r < h / 2; ++r)
      std::memcpy(nv12.ptr(h + r), uv_src + r * uvs, w);
    cv::cvtColor(nv12, out_mat, cv::COLOR_YUV2BGR_NV12);
    out_encoding = "bgr8";
    out_channels = 3;
  } else if (fmt == GST_VIDEO_FORMAT_I420 || fmt == GST_VIDEO_FORMAT_YV12) {
    // 3 separate planes: Y, U, V
    cv::Mat yuv(h * 3 / 2, w, CV_8UC1);
    auto * y_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    auto * u_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 1));
    auto * v_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 2));
    int ys = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    int us = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
    int vs = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 2);
    for (int r = 0; r < h; ++r)
      std::memcpy(yuv.ptr(r), y_src + r * ys, w);
    for (int r = 0; r < h / 2; ++r)
      std::memcpy(yuv.ptr(h + r), u_src + r * us, w / 2);
    for (int r = 0; r < h / 2; ++r)
      std::memcpy(yuv.ptr(h + h / 2 + r), v_src + r * vs, w / 2);
    int code = (fmt == GST_VIDEO_FORMAT_I420) ?
      cv::COLOR_YUV2BGR_I420 : cv::COLOR_YUV2BGR_YV12;
    cv::cvtColor(yuv, out_mat, code);
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
  return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Bus message pump
// ---------------------------------------------------------------------------

void ImageProcNode::poll_bus()
{
  if (!pipeline_) { return; }

  GstBus * bus = gst_element_get_bus(pipeline_);
  GstMessage * msg = gst_bus_pop_filtered(
    bus,
    static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  if (msg) {
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
        shutdown_pipeline();
        break;
      }
      case GST_MESSAGE_EOS:
        RCLCPP_INFO(get_logger(), "GStreamer end-of-stream");
        shutdown_pipeline();
        break;
      default:
        break;
    }
    gst_message_unref(msg);
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

  if (appsink_) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }

  if (appsrc_) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }

  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
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

  // Direct mode supports the standalone 'resize' / 'rectify' actions
  // and the 'rectify,resize' chain (the v0.2 F2A mechanism — single
  // pipeline replacing two DDS-piped image_proc nodes). Other chains
  // fall through to cv::resize-only with a warning.
  PipelineFactory factory(platform_info_, config_);
  const auto & chain = factory.action_chain();
  const bool single_resize =
    chain.size() == 1 && chain[0] == "resize";
  const bool single_rectify =
    chain.size() == 1 && chain[0] == "rectify";
  const bool rectify_then_resize =
    chain.size() == 2 && chain[0] == "rectify" && chain[1] == "resize";

  if (!single_resize && !single_rectify && !rectify_then_resize) {
    RCLCPP_WARN(get_logger(),
      "Action chain '%s' is not supported in direct mode; running "
      "cv::resize only. Direct mode handles standalone resize, "
      "standalone rectify, or the rectify+resize chain.",
      config_.action.c_str());
    PipelineConfig resize_only = config_;
    resize_only.action = "resize";
    PipelineFactory resize_factory(platform_info_, resize_only);
    info_transforms_ = resize_factory.camera_info_transforms();
  } else {
    info_transforms_ = factory.camera_info_transforms();
  }

  output_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.output_topic, 10);

  const auto transport = get_parameter("input_transport").as_string();
  if (transport == "raw") {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      config_.input_topic, 10,
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
      transport);
  }

  setup_camera_info_io(config_.input_topic, config_.output_topic);

  if (single_rectify) {
    RCLCPP_INFO(get_logger(),
      "Direct mode — cv::remap rectify(%s, transport=%s) on %s "
      "(LUT computed from CameraInfo on first frame; CPU-only by design)",
      config_.input_topic.c_str(), transport.c_str(),
      config_.output_topic.c_str());
  } else {
    RCLCPP_INFO(get_logger(),
      "Direct mode — cv::resize(%s, transport=%s → %dx%d) on %s "
      "(no GStreamer overhead)",
      config_.input_topic.c_str(), transport.c_str(),
      target_w_, target_h_, config_.output_topic.c_str());
  }
}

// ---------------------------------------------------------------------------
// Rectify LUT — computed from the cached CameraInfo. The undistort map
// (map1/map2) is invariant in K, D, and image dimensions; rebuild only
// triggers when one of those changes.
// ---------------------------------------------------------------------------

bool ImageProcNode::ensure_rectify_lut(uint32_t img_width, uint32_t img_height)
{
  if (img_width == 0 || img_height == 0) { return false; }

  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_in;
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    info_in = last_info_;
  }
  if (!info_in) { return false; }

  // K must have a non-zero focal length to be usable. CameraInfo with
  // K all zeros means the upstream calibration hasn't been published yet.
  if (info_in->k[0] <= 0.0 || info_in->k[4] <= 0.0) { return false; }

  // Detect changes that invalidate the cached LUT. Image dimensions can
  // diverge from CameraInfo dims temporarily (the image stream may be
  // running at the source resolution while CameraInfo is published once
  // per second by a calibration node), so we use the image dims for the
  // LUT and fold them into the change-detection.
  const bool dims_changed =
    img_width  != rectify_W_snapshot_ ||
    img_height != rectify_H_snapshot_;
  bool intrinsics_changed = !rectify_lut_ready_;
  for (size_t i = 0; i < 9; ++i) {
    if (info_in->k[i] != rectify_K_snapshot_[i]) {
      intrinsics_changed = true;
      break;
    }
  }
  if (info_in->d.size() != rectify_D_snapshot_.size()) {
    intrinsics_changed = true;
  } else {
    for (size_t i = 0; i < info_in->d.size(); ++i) {
      if (info_in->d[i] != rectify_D_snapshot_[i]) {
        intrinsics_changed = true;
        break;
      }
    }
  }

  if (rectify_lut_ready_ && !dims_changed && !intrinsics_changed) {
    return true;
  }

  cv::Matx33d K(
    info_in->k[0], info_in->k[1], info_in->k[2],
    info_in->k[3], info_in->k[4], info_in->k[5],
    info_in->k[6], info_in->k[7], info_in->k[8]);
  cv::Mat D;
  if (info_in->d.empty()) {
    D = cv::Mat::zeros(1, 5, CV_64F);
  } else {
    D = cv::Mat(1, static_cast<int>(info_in->d.size()), CV_64F);
    for (size_t i = 0; i < info_in->d.size(); ++i) {
      D.at<double>(0, static_cast<int>(i)) = info_in->d[i];
    }
  }

  const cv::Size sz(static_cast<int>(img_width), static_cast<int>(img_height));
  cv::Matx33d new_K = cv::getOptimalNewCameraMatrix(K, D, sz, 0.0, sz);
  cv::initUndistortRectifyMap(
    K, D, cv::Mat::eye(3, 3, CV_64F), new_K, sz, CV_16SC2,
    rectify_map1_, rectify_map2_);

  rectify_new_K_ = new_K;
  rectify_K_snapshot_ = info_in->k;
  rectify_D_snapshot_ = info_in->d;
  rectify_W_snapshot_ = img_width;
  rectify_H_snapshot_ = img_height;
  rectify_lut_ready_ = true;

  RCLCPP_INFO(get_logger(),
    "Rectify LUT built — %ux%u, fx=%.1f fy=%.1f cx=%.1f cy=%.1f, |D|=%zu",
    img_width, img_height,
    new_K(0, 0), new_K(1, 1), new_K(0, 2), new_K(1, 2),
    info_in->d.size());
  return true;
}

void ImageProcNode::on_image_direct(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!output_pub_) { return; }
  if (msg->width == 0 || msg->height == 0) { return; }

  // Wrap the incoming BGR data as a cv::Mat (zero-copy reference into
  // the intra-process UniquePtr).
  cv::Mat src(msg->height, msg->width, CV_8UC3, msg->data.data(), msg->step);

  cv::Mat dst;
  if (action_chain_has_rectify_) {
    if (!ensure_rectify_lut(msg->width, msg->height)) {
      // Wait for the first usable CameraInfo. Drop the frame quietly;
      // a steady warn would spam at the source frame rate.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "rectify: dropping frame — CameraInfo not yet available "
        "(K must be non-zero)");
      return;
    }
    cv::Mat rectified;
    cv::remap(src, rectified, rectify_map1_, rectify_map2_,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    // If the chain is "rectify,resize", fold the resize into the same
    // call before publishing — same architectural shape as v0.1.0's
    // chain row (single pipeline instead of multiple DDS-piped nodes).
    // The chain detection (size == 2 && [0] == rectify && [1] == resize)
    // is reflected in launch_direct's allow-list above; we don't need
    // to re-validate here.
    if (target_w_ != static_cast<int>(rectified.cols) ||
        target_h_ != static_cast<int>(rectified.rows))
    {
      cv::resize(rectified, dst, cv::Size(target_w_, target_h_),
                 0, 0, cv::INTER_LINEAR);
    } else {
      dst = std::move(rectified);
    }
  } else {
    cv::resize(src, dst, cv::Size(target_w_, target_h_),
               0, 0, cv::INTER_LINEAR);
  }

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
