// Unit tests for PipelineFactory action chain parsing and the registered
// CameraInfoTransform functors. These verify the pieces of logic that are
// easy to reason about out-of-process; an integration test that drives a
// running prism::ImageProcNode is left for Future Work — it needs a
// rclcpp executor and a synthetic Image+CameraInfo publisher.
//
// The CameraInfoTransforms are tested via the public factory API:
// constructing a PipelineFactory with a given action chain, fetching its
// camera_info_transforms(), and asserting that applying them in order to
// a known CameraInfo produces the expected output intrinsics.

#include <gtest/gtest.h>
#include <gst/gst.h>

#include <sensor_msgs/msg/camera_info.hpp>

#include "prism_image_proc/pipeline_factory.hpp"

using prism::HardwarePlatform;
using prism::PipelineConfig;
using prism::PipelineFactory;
using prism::PlatformInfo;
using sensor_msgs::msg::CameraInfo;

namespace
{

PlatformInfo cpu_platform()
{
  PlatformInfo p;
  p.platform = HardwarePlatform::CPU_FALLBACK;
  return p;
}

CameraInfo make_info(uint32_t w, uint32_t h, double fx, double fy,
                     double cx, double cy)
{
  CameraInfo info;
  info.width = w;
  info.height = h;
  info.distortion_model = "plumb_bob";
  info.k = {fx, 0.0, cx,  0.0, fy, cy,  0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, 0.0,  0.0, fy, cy, 0.0,  0.0, 0.0, 1.0, 0.0};
  info.d = {0.1, -0.2, 0.0, 0.0, 0.0};
  info.r = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
  info.roi.x_offset = 0;
  info.roi.y_offset = 0;
  info.roi.width = w;
  info.roi.height = h;
  return info;
}

void apply(const std::vector<prism::CameraInfoTransform> & ts,
           CameraInfo & info, const PipelineConfig & c)
{
  for (const auto & t : ts) { t(info, c); }
}

}  // namespace

TEST(ActionChainParser, AcceptsCommaSeparated)
{
  const auto names = PipelineFactory::parse_action_chain("crop,resize,colorconvert");
  ASSERT_EQ(names.size(), 3u);
  EXPECT_EQ(names[0], "crop");
  EXPECT_EQ(names[1], "resize");
  EXPECT_EQ(names[2], "colorconvert");
}

TEST(ActionChainParser, AcceptsPipeSeparated)
{
  const auto names = PipelineFactory::parse_action_chain("flip|resize");
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "flip");
  EXPECT_EQ(names[1], "resize");
}

TEST(ActionChainParser, TrimsWhitespace)
{
  const auto names = PipelineFactory::parse_action_chain(" resize , flip ");
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "resize");
  EXPECT_EQ(names[1], "flip");
}

TEST(ActionChainParser, RejectsUnknown)
{
  EXPECT_THROW(
    PipelineFactory::parse_action_chain("debayer"),
    std::invalid_argument);
}

TEST(ActionChainParser, AcceptsRectify)
{
  const auto names = PipelineFactory::parse_action_chain("rectify");
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "rectify");
}

TEST(RectifyCameraInfoTransform, ZeroesDistortionAndSetsIdentityR)
{
  PipelineConfig c;
  c.action = "rectify";

  PipelineFactory factory(cpu_platform(), c);
  auto transforms = factory.camera_info_transforms();
  ASSERT_EQ(transforms.size(), 1u);

  CameraInfo info = make_info(1920, 1080, 1900.0, 1900.0, 960.0, 540.0);
  // Pre-condition: distortion is non-zero, R may be anything.
  ASSERT_FALSE(info.d.empty());
  EXPECT_NE(info.d[0], 0.0);

  apply(transforms, info, c);

  // Post-condition: distortion zeroed, R = identity. K/P are NOT updated by
  // the transform alone — that happens in image_proc_node when the LUT
  // is built from the runtime CameraInfo (the new_K depends on
  // cv::getOptimalNewCameraMatrix and lives on the node, not in the
  // factory). Width and height pass through unchanged.
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  ASSERT_EQ(info.d.size(), 5u);
  for (double v : info.d) {
    EXPECT_DOUBLE_EQ(v, 0.0);
  }
  EXPECT_DOUBLE_EQ(info.r[0], 1.0);
  EXPECT_DOUBLE_EQ(info.r[4], 1.0);
  EXPECT_DOUBLE_EQ(info.r[8], 1.0);
  EXPECT_DOUBLE_EQ(info.r[1], 0.0);
  EXPECT_DOUBLE_EQ(info.r[3], 0.0);
  EXPECT_EQ(info.width, 1920u);
  EXPECT_EQ(info.height, 1080u);
}

TEST(ActionChainParser, RejectsEmpty)
{
  EXPECT_THROW(
    PipelineFactory::parse_action_chain(""),
    std::invalid_argument);
  EXPECT_THROW(
    PipelineFactory::parse_action_chain(" , "),
    std::invalid_argument);
}

TEST(ResizeCameraInfoTransform, ScalesIntrinsicsRoiAndDims)
{
  PipelineConfig c;
  c.action = "resize";
  c.source_width = 3840;
  c.source_height = 2160;
  c.target_width = 640;
  c.target_height = 480;

  PipelineFactory factory(cpu_platform(), c);
  auto transforms = factory.camera_info_transforms();
  ASSERT_EQ(transforms.size(), 1u);

  CameraInfo info = make_info(3840, 2160, 1920.0, 1080.0, 1920.0, 1080.0);
  apply(transforms, info, c);

  const double sx = 640.0 / 3840.0;
  const double sy = 480.0 / 2160.0;
  EXPECT_NEAR(info.k[0], 1920.0 * sx, 1e-9);  // fx
  EXPECT_NEAR(info.k[2], 1920.0 * sx, 1e-9);  // cx
  EXPECT_NEAR(info.k[4], 1080.0 * sy, 1e-9);  // fy
  EXPECT_NEAR(info.k[5], 1080.0 * sy, 1e-9);  // cy
  EXPECT_NEAR(info.p[0], 1920.0 * sx, 1e-9);
  EXPECT_NEAR(info.p[2], 1920.0 * sx, 1e-9);
  EXPECT_NEAR(info.p[5], 1080.0 * sy, 1e-9);
  EXPECT_NEAR(info.p[6], 1080.0 * sy, 1e-9);
  EXPECT_EQ(info.width, 640u);
  EXPECT_EQ(info.height, 480u);
  EXPECT_EQ(info.roi.width, 640u);
  EXPECT_EQ(info.roi.height, 480u);
  // Distortion untouched.
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  ASSERT_EQ(info.d.size(), 5u);
  EXPECT_DOUBLE_EQ(info.d[0], 0.1);
  EXPECT_DOUBLE_EQ(info.d[1], -0.2);
}

TEST(CropCameraInfoTransform, OffsetsPrincipalPointAndUpdatesRoi)
{
  PipelineConfig c;
  c.action = "crop";
  c.source_width = 1920;
  c.source_height = 1080;
  c.crop_x = 100;
  c.crop_y = 50;
  c.crop_width = 800;
  c.crop_height = 600;
  // target_width/height unused by crop.

  PipelineFactory factory(cpu_platform(), c);
  auto transforms = factory.camera_info_transforms();
  ASSERT_EQ(transforms.size(), 1u);

  CameraInfo info = make_info(1920, 1080, 1500.0, 1500.0, 960.0, 540.0);
  apply(transforms, info, c);

  EXPECT_DOUBLE_EQ(info.k[2], 960.0 - 100.0);  // cx shifted
  EXPECT_DOUBLE_EQ(info.k[5], 540.0 - 50.0);   // cy shifted
  EXPECT_DOUBLE_EQ(info.p[2], 960.0 - 100.0);
  EXPECT_DOUBLE_EQ(info.p[6], 540.0 - 50.0);
  EXPECT_EQ(info.width, 800u);
  EXPECT_EQ(info.height, 600u);
  EXPECT_EQ(info.roi.x_offset, 100u);
  EXPECT_EQ(info.roi.y_offset, 50u);
  EXPECT_EQ(info.roi.width, 800u);
  EXPECT_EQ(info.roi.height, 600u);
}

TEST(FlipCameraInfoTransform, HorizontalMirrorsPrincipalX)
{
  PipelineConfig c;
  c.action = "flip";
  c.flip_method = "horizontal";
  c.source_width = 640;
  c.source_height = 480;

  PipelineFactory factory(cpu_platform(), c);
  auto transforms = factory.camera_info_transforms();
  ASSERT_EQ(transforms.size(), 1u);

  CameraInfo info = make_info(640, 480, 500.0, 500.0, 300.0, 240.0);
  apply(transforms, info, c);

  EXPECT_DOUBLE_EQ(info.k[2], 640.0 - 300.0);
  EXPECT_DOUBLE_EQ(info.p[2], 640.0 - 300.0);
  EXPECT_DOUBLE_EQ(info.k[5], 240.0);  // vertical unchanged
}

TEST(ColorconvertCameraInfoTransform, LeavesIntrinsicsAlone)
{
  PipelineConfig c;
  c.action = "colorconvert";
  c.target_encoding = "mono8";
  c.source_width = 640;
  c.source_height = 480;

  PipelineFactory factory(cpu_platform(), c);
  auto transforms = factory.camera_info_transforms();
  ASSERT_EQ(transforms.size(), 1u);

  CameraInfo info = make_info(640, 480, 500.0, 500.0, 300.0, 240.0);
  const CameraInfo before = info;
  apply(transforms, info, c);

  EXPECT_EQ(info.width, before.width);
  EXPECT_EQ(info.height, before.height);
  EXPECT_EQ(info.k, before.k);
  EXPECT_EQ(info.p, before.p);
}

TEST(ChainedTransforms, CropThenResizeComposes)
{
  PipelineConfig c;
  c.action = "crop,resize";
  c.source_width = 1920;
  c.source_height = 1080;
  c.crop_x = 100;
  c.crop_y = 50;
  c.crop_width = 800;
  c.crop_height = 600;
  c.target_width = 400;
  c.target_height = 300;  // 50% downscale after crop

  PipelineFactory factory(cpu_platform(), c);
  auto transforms = factory.camera_info_transforms();
  ASSERT_EQ(transforms.size(), 2u);

  CameraInfo info = make_info(1920, 1080, 1500.0, 1500.0, 960.0, 540.0);
  apply(transforms, info, c);

  // After crop: width=800, height=600, cx=860, cy=490
  // After resize (800,600 → 400,300; sx=sy=0.5): width=400, height=300,
  // cx=430, cy=245, fx=750, fy=750
  EXPECT_EQ(info.width, 400u);
  EXPECT_EQ(info.height, 300u);
  EXPECT_DOUBLE_EQ(info.k[0], 750.0);
  EXPECT_DOUBLE_EQ(info.k[4], 750.0);
  EXPECT_DOUBLE_EQ(info.k[2], 430.0);
  EXPECT_DOUBLE_EQ(info.k[5], 245.0);
}

TEST(PipelineFactoryBuild, SingleResizeOnCpu)
{
  PipelineConfig c;
  c.action = "resize";
  c.source_width = 1920;
  c.source_height = 1080;
  c.target_width = 640;
  c.target_height = 480;

  PipelineFactory factory(cpu_platform(), c);
  const std::string pipeline = factory.build();

  EXPECT_NE(pipeline.find("appsrc"),      std::string::npos);
  EXPECT_NE(pipeline.find("videoscale"),  std::string::npos);
  EXPECT_NE(pipeline.find("appsink"),     std::string::npos);
  EXPECT_NE(pipeline.find("width=640"),   std::string::npos);
  EXPECT_NE(pipeline.find("height=480"),  std::string::npos);
}

int main(int argc, char ** argv)
{
  // Needed for gst_element_factory_find() calls inside fragment builders.
  gst_init(&argc, &argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
