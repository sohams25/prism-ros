#ifndef PRISM_IMAGE_PROC__IMAGE_PROC_NODE_HPP_
#define PRISM_IMAGE_PROC__IMAGE_PROC_NODE_HPP_

#include <mutex>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
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

  // Rectify-action state. The LUT is computed lazily on the first
  // CameraInfo with non-zero K, then re-used for every subsequent frame.
  // Recompute is triggered when K, D, or image dimensions change.
  bool action_chain_has_rectify_ = false;
  cv::Mat rectify_map1_;
  cv::Mat rectify_map2_;
  cv::Matx33d rectify_new_K_{1, 0, 0, 0, 1, 0, 0, 0, 1};
  bool rectify_lut_ready_ = false;
  // Snapshot of the inputs to the LUT, so we can detect when CameraInfo
  // changes enough to require recomputation.
  std::array<double, 9> rectify_K_snapshot_{};
  std::vector<double> rectify_D_snapshot_;
  uint32_t rectify_W_snapshot_ = 0;
  uint32_t rectify_H_snapshot_ = 0;

  void declare_parameters();
  PipelineConfig load_config_from_params() const;
  void detect_hardware();
  void build_pipeline();
  void launch_pipeline();
  void launch_direct();
  void poll_bus();
  void shutdown_pipeline();

  // Runtime reconfiguration. The param callback validates proposed changes
  // synchronously (rejecting unknown actions / non-positive dims) and, if
  // successful, schedules a one-shot timer to rebuild the pipeline after
  // the new values commit.
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);
  void rebuild_from_params();
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

  // Rectify helpers. ensure_rectify_lut returns true if the LUT is ready
  // (either it already was, or it was successfully computed from the
  // cached CameraInfo). Recompute triggers if K, D, or image dimensions
  // changed since the last LUT build.
  bool ensure_rectify_lut(uint32_t img_width, uint32_t img_height);
};

}  // namespace prism

#endif  // PRISM_IMAGE_PROC__IMAGE_PROC_NODE_HPP_
