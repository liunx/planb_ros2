#include "planb/camera_node.hpp"

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create and add camera node
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(true);
  CameraNode node(options);
  node.loop();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
