cmake_minimum_required(VERSION 3.8)
project(prism_image_proc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

# ---------------------------------------------------------------------------
# Library: prism_core — hardware detection + pipeline factory
# ---------------------------------------------------------------------------
add_library(prism_core SHARED
  src/hardware_detector.cpp
  src/pipeline_factory.cpp
)

target_include_directories(prism_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GST_INCLUDE_DIRS}
)

target_link_directories(prism_core PUBLIC
  ${GST_LIBRARY_DIRS}
)

# Link GStreamer via ament_target_dependencies to avoid mixing keyword/plain
# target_link_libraries signatures (ament uses plain form internally).
target_link_libraries(prism_core
  ${GST_LIBRARIES}
)

ament_target_dependencies(prism_core
  rclcpp
  sensor_msgs
)

# ---------------------------------------------------------------------------
# Component library: prism_image_proc_component
# ---------------------------------------------------------------------------
add_library(prism_image_proc_component SHARED
  src/image_proc_node.cpp
  src/resize_node.cpp
  src/crop_node.cpp
  src/color_convert_node.cpp
  src/rectify_node.cpp
)

# rectify_node uses cv::initUndistortRectifyMap and
# cv::getOptimalNewCameraMatrix from opencv_calib3d.
find_package(OpenCV REQUIRED COMPONENTS calib3d imgproc core)

target_link_libraries(prism_image_proc_component
  prism_core
  opencv_calib3d
  opencv_imgproc
  opencv_core
)

ament_target_dependencies(prism_image_proc_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

rclcpp_components_register_nodes(prism_image_proc_component
  "prism::ImageProcNode"
  "prism::ResizeNode"
  "prism::CropNode"
  "prism::ColorConvertNode"
  "prism::RectifyNode"
)

# ---------------------------------------------------------------------------
# Component library: synthetic_4k_pub_component
# ---------------------------------------------------------------------------
add_library(synthetic_4k_pub_component SHARED
  src/synthetic_4k_pub_node.cpp
)

target_include_directories(synthetic_4k_pub_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(synthetic_4k_pub_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
)

rclcpp_components_register_nodes(synthetic_4k_pub_component
  "prism::Synthetic4kPubNode"
)

# ---------------------------------------------------------------------------
# Component library: media_streamer_component
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS videoio)

add_library(media_streamer_component SHARED
  src/media_streamer_node.cpp
)

target_include_directories(media_streamer_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(media_streamer_component
  opencv_videoio
)

ament_target_dependencies(media_streamer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
)

rclcpp_components_register_nodes(media_streamer_component
  "prism::MediaStreamerNode"
)

# ---------------------------------------------------------------------------
# Standalone executable
# ---------------------------------------------------------------------------
add_executable(prism_image_proc
  src/main.cpp
)

target_link_libraries(prism_image_proc
  prism_image_proc_component
)

ament_target_dependencies(prism_image_proc
  rclcpp
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS prism_core prism_image_proc_component synthetic_4k_pub_component media_streamer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS prism_image_proc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/synthetic_4k_pub.py
  scripts/latency_tracker.py
  scripts/cpu_monitor.py
  scripts/visualize_demo.py
  bench/helpers/cv_bridge_subscriber_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/dashboard.html
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pipeline_factory test/test_pipeline_factory.cpp)
  target_link_libraries(test_pipeline_factory prism_core)
  target_include_directories(test_pipeline_factory PRIVATE
    ${GST_INCLUDE_DIRS}
  )
  target_link_directories(test_pipeline_factory PRIVATE
    ${GST_LIBRARY_DIRS}
  )
  target_link_libraries(test_pipeline_factory ${GST_LIBRARIES})
  ament_target_dependencies(test_pipeline_factory sensor_msgs)
endif()

ament_package()
