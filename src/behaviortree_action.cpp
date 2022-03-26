#include "planb/behaviortree_action.hpp"

BehaviorTreeAction::BehaviorTreeAction()
{
    auto do_nothing = [](std_msgs::msg::String::UniquePtr) { assert(false); };
    node_ = rclcpp::Node::make_shared("behaviortree_action");
    sub_camera_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/camera/status", 1, do_nothing);
    pub_camera_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/camera/cmd", 1);
    pub_hardware_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/hardware/cmd", 1);
}

BehaviorTreeAction::~BehaviorTreeAction()
{
}

BT::NodeStatus BehaviorTreeAction::check_camera_idle()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_camera_status_->take(msg, msg_info)) {
        if (msg.data == "Idle")
            stat = BT::NodeStatus::SUCCESS;
        else
            stat = BT::NodeStatus::FAILURE;
    }
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_camera_idle()
{
    std_msgs::msg::String msg;
    msg.data = "Idle";
    pub_camera_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::check_camera_running()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_camera_status_->take(msg, msg_info)) {
        if (msg.data == "Running")
            stat = BT::NodeStatus::SUCCESS;
        else
            stat = BT::NodeStatus::FAILURE;
    }
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_camera_running()
{
    std_msgs::msg::String msg;
    msg.data = "Running";
    pub_camera_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}