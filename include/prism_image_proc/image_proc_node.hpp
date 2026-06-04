#ifndef PRISM_IMAGE_PROC__IMAGE_PROC_NODE_HPP_
#define PRISM_IMAGE_PROC__IMAGE_PROC_NODE_HPP_

#include <atomic>
#include <mutex>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include "prism_image_proc/hardware_detector.hpp"
#include "prism_image_proc/pipeline_factory.hpp"

namespace prism
{

class ImageProcNode : public rclcpp::Node
{
public:
  explicit ImageProcNode(const rclcpp::NodeOptions & options);
  ~ImageProcNode() override;

protected:
  // Thin wrappers (ResizeNode / CropNode / ColorConvertNode) delegate here with
  // a distinct default node name, so that when a component is loaded
  // programmatically without an explicit name override each one reports a
  // unique name instead of all colliding on a single shared default.
  ImageProcNode(const rclcpp::NodeOptions & options, const std::string & node_name);

private:
  PlatformInfo platform_info_;
  std::string pipeline_string_;
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * appsink_ = nullptr;
  rclcpp::TimerBase::SharedPtr bus_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  // Used when input_transport != "raw". Falls back to image_sub_ for the
  // "raw" case so the hot path keeps its UniquePtr zero-copy IPC.
  image_transport::Subscriber it_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_pub_;

  // CameraInfo plumbing (both paths).
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  std::mutex info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_info_;
  bool publish_camera_info_ = true;

  // Config snapshot + CameraInfo transform chain, populated by build_pipeline()
  // or launch_direct(). The transforms apply to an input CameraInfo copy in
  // chain order to produce the egress CameraInfo.
  PipelineConfig config_;
  std::vector<CameraInfoTransform> info_transforms_;

  bool direct_mode_ = false;
  int target_w_ = 640;
  int target_h_ = 480;

  void declare_parameters();
  PipelineConfig load_config_from_params() const;
  void detect_hardware();
  // True when a GPU GStreamer backend that actually works is available for the
  // detected platform (Jetson NVMM, or Intel only when vapostproc — GStreamer
  // 1.22+ — is present; vaapipostproc on 1.20 has a chroma bug and is skipped).
  bool gpu_path_available() const;
  void build_pipeline();
  bool launch_pipeline();
  void launch_direct();
  void poll_bus();
  void shutdown_pipeline();

  // QoS for image / CameraInfo endpoints. Defaults to SensorDataQoS
  // (BEST_EFFORT) for camera-driver interoperability; reliable_qos:=true
  // selects a RELIABLE depth-10 profile instead.
  rclcpp::QoS image_qos() const;

  // Count of GStreamer-bus errors since the pipeline last produced a frame;
  // bounds automatic recovery in poll_bus before degrading to direct mode (see
  // kMaxBusErrorRetries). Reset to 0 on a successful egress sample (on the
  // streaming thread), so transient faults that recover do not accumulate over
  // the node's lifetime — hence atomic for the cross-thread reset.
  std::atomic<int> bus_error_count_{0};

  // Runtime reconfiguration. The param callback validates proposed changes
  // synchronously (rejecting unknown actions / non-positive dims) and, if
  // successful, schedules a one-shot timer to rebuild the pipeline after
  // the new values commit.
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);
  void rebuild_from_params();
  // Schedules a single deferred rebuild on a one-shot timer (idempotent: a
  // pending rebuild is not re-scheduled). Used both by parameter changes and
  // by bounded GStreamer-bus error recovery.
  void schedule_rebuild();
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_callback_handle_;
  rclcpp::TimerBase::SharedPtr rebuild_timer_;

  // Subscribes to the input-side CameraInfo topic and advertises the output-
  // side one. Safe to call after config_ is populated.
  void setup_camera_info_io(
    const std::string & image_input_topic,
    const std::string & image_output_topic);
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void publish_transformed_camera_info(const std_msgs::msg::Header & image_header);

  void on_image(sensor_msgs::msg::Image::UniquePtr msg);
  void on_image_direct(sensor_msgs::msg::Image::UniquePtr msg);
  static void destroy_ros_image(gpointer user_data);
  static GstFlowReturn on_new_sample(GstAppSink * sink, gpointer user_data);
};

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__IMAGE_PROC_NODE_HPP_
