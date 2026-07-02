cmake_minimum_required(VERSION 3.8)
project(prism_image_proc)

# Pin the language standard for reproducible builds across toolchains. The
# sources use C++17 features ([[fallthrough]], std::clamp, structured init).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
)

target_link_libraries(prism_image_proc_component
  prism_core
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
# prism_core is the reusable library (hardware detection + pipeline factory)
# downstream packages may link against, so export it as an imported target.
install(TARGETS prism_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# The component plugin libraries are loaded at runtime by the component
# container; they are not link targets for downstream packages.
install(TARGETS prism_image_proc_component synthetic_4k_pub_component media_streamer_component
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

# Make prism_core, its headers, and its transitive deps available to downstream
# packages that find_package(prism_image_proc).
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(prism_core)
ament_export_dependencies(rclcpp sensor_msgs)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/latency_tracker.py
  bench/helpers/cv_bridge_subscriber_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude the third-party-style bench/scripts Python from the lint sweep
  # (tracked separately); lint still covers the package's own C++ and configs.
  set(ament_cmake_copyright_FOUND TRUE)
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
  ament_target_dependencies(test_pipeline_factory rclcpp sensor_msgs)

  # Integration test: drive a running ImageProcNode (direct mode) end-to-end.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_direct_mode_integration.py
    TIMEOUT 120
  )
endif()

ament_package()
