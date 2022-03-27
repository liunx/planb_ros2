#include "planb/behaviortree_action.hpp"

BehaviorTreeAction::BehaviorTreeAction()
    : hardware_status_("Unkown"),
      camera_status_("Unkown"),
      aruco_status_("Unkown"),
      tracker_status_("Unkown")
{
    auto do_nothing = [](std_msgs::msg::String::UniquePtr) { assert(false); };
    auto do_nothing2 = [](planb_ros2::msg::Aruco::UniquePtr) { assert(false); };
    node_ = rclcpp::Node::make_shared("behaviortree_action");
    // hardware
    pub_hardware_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/hardware/cmd", 1);
    // camera
    sub_camera_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/camera/status", 1, do_nothing);
    pub_camera_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/camera/cmd", 1);
    // aruco
    sub_aruco_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/aruco/status", 1, do_nothing);
    sub_aruco_markers_ = node_->create_subscription<planb_ros2::msg::Aruco>("/planb/aruco/markers", 1, do_nothing2);
    pub_aruco_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/aruco/cmd", 1);
    // tracker
    sub_tracker_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/tracker/status", 1, do_nothing);
    pub_tracker_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/tracker/cmd", 1);
}

BehaviorTreeAction::~BehaviorTreeAction()
{
}

// camera
BT::NodeStatus BehaviorTreeAction::check_camera_idle()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_camera_status_->take(msg, msg_info)) {
        camera_status_ = msg.data;
    }
    if (camera_status_ == "Idle")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
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
        camera_status_ = msg.data;
    }
    if (camera_status_ == "Running")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_camera_running()
{
    std_msgs::msg::String msg;
    msg.data = "Running";
    pub_camera_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

// aruco
BT::NodeStatus BehaviorTreeAction::check_aruco_idle()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_aruco_status_->take(msg, msg_info)) {
        aruco_status_ = msg.data;
    }
    if (aruco_status_ == "Idle")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_aruco_idle()
{
    std_msgs::msg::String msg;
    msg.data = "Idle";
    pub_aruco_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::check_aruco_running()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_aruco_status_->take(msg, msg_info)) {
        aruco_status_ = msg.data;
    }
    if (aruco_status_ == "Running")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_aruco_running()
{
    std_msgs::msg::String msg;
    msg.data = "Running";
    pub_aruco_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::detect_aruco_markers()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    planb_ros2::msg::Aruco msg;
    rclcpp::MessageInfo msg_info;
    if (sub_aruco_markers_->take(msg, msg_info)) {
        if (msg.data.size() > 0)
            stat = BT::NodeStatus::SUCCESS;
    }
    return stat;
}

// tracker
BT::NodeStatus BehaviorTreeAction::check_tracker_idle()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_tracker_status_->take(msg, msg_info)) {
        tracker_status_ = msg.data;
    }
    if (tracker_status_ == "Idle")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_tracker_idle()
{
    std_msgs::msg::String msg;
    msg.data = "Idle";
    pub_tracker_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::check_tracker_running()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_tracker_status_->take(msg, msg_info)) {
        tracker_status_ = msg.data;
    }
    if (tracker_status_ == "Running")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_tracker_running()
{
    std_msgs::msg::String msg;
    msg.data = "Running";
    pub_tracker_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

// hardware
BT::NodeStatus BehaviorTreeAction::reset_hardware()
{
    std_msgs::msg::String msg;
    msg.data = "Reset";
    pub_hardware_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}