#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "planb/behaviortree_action.hpp"

#define DEFAULT_BT_XML "./demo.xml"

void bind_actions(BT::BehaviorTreeFactory &factory)
{
  static BehaviorTreeAction action = BehaviorTreeAction();
  factory.registerSimpleAction("CheckCameraIdle", std::bind(&BehaviorTreeAction::check_camera_idle, &action));
  factory.registerSimpleAction("SetCameraIdle", std::bind(&BehaviorTreeAction::set_camera_idle, &action));
  factory.registerSimpleAction("CheckCameraRunning", std::bind(&BehaviorTreeAction::check_camera_running, &action));
  factory.registerSimpleAction("SetCameraRunning", std::bind(&BehaviorTreeAction::set_camera_running, &action));
}

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  // Init ROS
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("brain");
  nh->declare_parameter("bt_xml", rclcpp::ParameterValue(std::string(DEFAULT_BT_XML)));
  std::string bt_xml;
  nh->get_parameter("bt_xml", bt_xml);
  // Init behavior tree
  BT::BehaviorTreeFactory factory;
  bind_actions(factory);
  auto tree = factory.createTreeFromFile(bt_xml);
  BT::StdCoutLogger logger_cout(tree);
  BT::printTreeRecursively(tree.rootNode());
  while (rclcpp::ok()) {
    tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
