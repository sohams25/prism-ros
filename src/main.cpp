#include <rclcpp/rclcpp.hpp>

#include "prism_image_proc/image_proc_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<prism::ImageProcNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
