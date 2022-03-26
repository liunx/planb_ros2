#include "planb/behaviortree_action.hpp"

BehaviorTreeAction::BehaviorTreeAction()
{
    auto do_nothing = [](std_msgs::msg::String::UniquePtr) { assert(false); };
    node_ = rclcpp::Node::make_shared("behaviortree_action");
    sub_camera_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/camera/status", 10, do_nothing);
    pub_hardware_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/hardware/cmd", 1);
}

BehaviorTreeAction::~BehaviorTreeAction()
{
}

BT::NodeStatus BehaviorTreeAction::fetch_data()
{
    return BT::NodeStatus::SUCCESS;
}