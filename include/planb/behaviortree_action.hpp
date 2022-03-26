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

    BT::NodeStatus fetch_data();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_hardware_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_camera_status_;
};

#endif
