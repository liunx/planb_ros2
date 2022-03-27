#ifndef BEHAVIORTREE_ACTION_HPP
#define BEHAVIORTREE_ACTION_HPP
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "planb_ros2/msg/aruco.hpp"

class BehaviorTreeAction
{
public:
    BehaviorTreeAction();
    ~BehaviorTreeAction();
    // camera
    BT::NodeStatus check_camera_idle();
    BT::NodeStatus set_camera_idle();
    BT::NodeStatus check_camera_running();
    BT::NodeStatus set_camera_running();
    // aruco
    BT::NodeStatus check_aruco_idle();
    BT::NodeStatus set_aruco_idle();
    BT::NodeStatus check_aruco_running();
    BT::NodeStatus set_aruco_running();
    BT::NodeStatus detect_aruco_markers();
    // tracker
    BT::NodeStatus check_tracker_idle();
    BT::NodeStatus set_tracker_idle();
    BT::NodeStatus check_tracker_running();
    BT::NodeStatus set_tracker_running();
    // hardware
    BT::NodeStatus reset_hardware();

private:
    rclcpp::Node::SharedPtr node_;
    // hardware
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_hardware_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_hardware_status_;
    std::string hardware_status_;
    // camera
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_camera_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_camera_status_;
    std::string camera_status_;
    // aruco
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_aruco_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_aruco_status_;
    rclcpp::Subscription<planb_ros2::msg::Aruco>::SharedPtr sub_aruco_markers_;
    std::string aruco_status_;
    // tracker
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_tracker_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_tracker_status_;
    std::string tracker_status_;
};

#endif
