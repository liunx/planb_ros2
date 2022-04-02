#ifndef BEHAVIORTREE_ACTION_HPP
#define BEHAVIORTREE_ACTION_HPP
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "planb_ros2/msg/aruco_marker.hpp"
#include "planb_ros2/msg/coord_info.hpp"
#include "planb_ros2/msg/cmd.hpp"

class ArucoAction : public BT::SyncActionNode
{
public:
    ArucoAction(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr &node);
    BT::NodeStatus get_status();
    BT::NodeStatus get_data();
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
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<planb_ros2::msg::Cmd>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::Subscription<planb_ros2::msg::ArucoMarker>::SharedPtr sub_data_;
};

class TrackerAction : public BT::SyncActionNode
{
public:
    TrackerAction(const std::string &name,
                  const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr &node);
    BT::NodeStatus get_status();
    BT::NodeStatus turn_on();
    BT::NodeStatus turn_off();
    BT::NodeStatus track_aruco();

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("id"),
                BT::OutputPort<std::string>("status")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    std::string name_;
    std::string status_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<planb_ros2::msg::Cmd>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
};

class CameraAction : public BT::SyncActionNode
{
public:
    CameraAction(const std::string &name,
                 const BT::NodeConfiguration &config,
                 rclcpp::Node::SharedPtr &node);
    BT::NodeStatus get_status();
    BT::NodeStatus turn_on();
    BT::NodeStatus turn_off();

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("status")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    std::string name_;
    std::string status_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<planb_ros2::msg::Cmd>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
};

class HardwareAction : public BT::SyncActionNode
{
public:
    HardwareAction(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr &node);
    BT::NodeStatus get_status();
    BT::NodeStatus turn_on();
    BT::NodeStatus turn_off();

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("status")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    std::string name_;
    std::string status_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<planb_ros2::msg::Cmd>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
};

class WaitAction : public BT::SyncActionNode
{
public:
    WaitAction(const std::string &name,
               const BT::NodeConfiguration &config,
               rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::uint32_t>("msec")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
};

class LogicAction : public BT::SyncActionNode
{
public:
    LogicAction(const std::string &name,
                const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("ids"),
                BT::InputPort<std::string>("id"),
                BT::InputPort<std::string>("value1"),
                BT::InputPort<std::string>("value2")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    BT::NodeStatus check_equal();
    BT::NodeStatus find_id();
    std::string name_;
};

class LogAction : public BT::SyncActionNode
{
public:
    LogAction(const std::string &name,
                const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("text")};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    BT::NodeStatus info();
    BT::NodeStatus warn();
    BT::NodeStatus error();
    std::string name_;
};

#endif
