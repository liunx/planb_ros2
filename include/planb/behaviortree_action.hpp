#ifndef BEHAVIORTREE_ACTION_HPP
#define BEHAVIORTREE_ACTION_HPP
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "planb_ros2/msg/aruco.hpp"
#include "planb_ros2/msg/box.hpp"

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
    BT::NodeStatus check_tracker_running();
    BT::NodeStatus tracking_target();
    // hardware
    BT::NodeStatus init_hardware();
    BT::NodeStatus reset_hardware();
    BT::NodeStatus check_hardware_ready();
    // others
    BT::NodeStatus wait();
    BT::NodeStatus follow_target();
    BT::NodeStatus stop_follow();

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
    planb_ros2::msg::Aruco aruco_msg_;
    // tracker
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_tracker_status_;
    rclcpp::Publisher<planb_ros2::msg::Box>::SharedPtr pub_tracker_input_;
    rclcpp::Subscription<planb_ros2::msg::Box>::SharedPtr sub_tracker_data_;
    std::string tracker_status_;
    // others
    bool flag_follow_stopped_;
};

class ArucoAction : public BT::SyncActionNode
{
public:
    ArucoAction(const std::string &name, const BT::NodeConfiguration &config);
    BT::NodeStatus get_status();
    BT::NodeStatus get_markers();
    BT::NodeStatus turn_on();
    BT::NodeStatus turn_off();

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("ids"),
                BT::OutputPort<std::string>("status")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    std::string name_;
    std::string status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::Subscription<planb_ros2::msg::Aruco>::SharedPtr sub_markers_;
};

class TrackerAction : public BT::SyncActionNode
{
public:
    ArucoAction(const std::string &name, const BT::NodeConfiguration &config);
    BT::NodeStatus get_status();
    BT::NodeStatus get_markers();
    BT::NodeStatus turn_on();
    BT::NodeStatus turn_off();

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("ids"),
                BT::OutputPort<std::string>("status")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    std::string name_;
    std::string status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::Subscription<planb_ros2::msg::Aruco>::SharedPtr sub_markers_;
};

#endif
