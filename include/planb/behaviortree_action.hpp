#ifndef BEHAVIORTREE_ACTION_HPP
#define BEHAVIORTREE_ACTION_HPP
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

class BehaviorTreeAction
{
public:
    BehaviorTreeAction();
    ~BehaviorTreeAction();
    // for camera
    BT::NodeStatus check_camera_idle();
    BT::NodeStatus set_camera_idle();
    BT::NodeStatus check_camera_running();
    BT::NodeStatus set_camera_running();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_hardware_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_camera_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_camera_status_;
};

#endif
