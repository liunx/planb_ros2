#ifndef HARDWARE_HPP
#define HARDWARE_HPP
#include <mraa.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <planb_interfaces/msg/robot.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "planb_interfaces/msg/cmd.hpp"
#include "planb_interfaces/msg/operate.hpp"

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode(const rclcpp::NodeOptions &options);
    ~HardwareNode() override;

private:
    void serial_init(std::string &dev_path, const int baud);
    void publish_status(const std::string &status);
    void cmd_callback(const planb_interfaces::msg::Cmd &msg);
    void operate_callback(const planb_interfaces::msg::Operate &msg);
    void twist_callback(const geometry_msgs::msg::Twist &msg);
    void robot_callback(const planb_interfaces::msg::Robot &msg);
    void turn_on();
    void turn_off();
    void tx_data();
    std::vector<uint8_t> calc_angles(float angle, bool direct_left);
    std::vector<uint8_t> calc_accel(float angle, uint8_t accel, bool direct_left);
    void normal_mode(const planb_interfaces::msg::Robot &msg);
    void accel_circle(const int accel);
    void circle_mode(const planb_interfaces::msg::Robot &msg);
    void accel_arckerman(const int accel);
    void arckerman_mode(const planb_interfaces::msg::Robot &msg);

private:
    const float d1 = 9.0;
    const float d2 = 10.0;
    const float d3 = 11.0;
    const float d4 = 12.0;
    int32_t angle_ = 90;
    int32_t accel_ = 0;
    uint8_t control_data_[16] = {90, 90, 90, 90};
    std::string status_;
    std::shared_ptr<mraa::Uart> uart_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<planb_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
    rclcpp::Subscription<planb_interfaces::msg::Operate>::SharedPtr sub_operate_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Subscription<planb_interfaces::msg::Robot>::SharedPtr sub_robot_;
};

#endif
