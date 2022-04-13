#ifndef HARDWARE_HPP
#define HARDWARE_HPP
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "planb_ros2/msg/cmd.hpp"
#include "planb_ros2/msg/operate.hpp"

#define MIN_ANGLE_INDEX 0
#define MAX_ANGLE_INDEX 12
#define MIN_SPEED_INDEX 0
#define MAX_SPEED_INDEX 8

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode(const rclcpp::NodeOptions &options);
    ~HardwareNode() override;

private:
    int serial_init(const char *dev, const int baud);
    void publish_status(const std::string &status);
    void cmd_callback(const planb_ros2::msg::Cmd &msg);
    void operate_callback(const planb_ros2::msg::Operate &msg);
    void twist_callback(const geometry_msgs::msg::Twist &msg);
    void turn_on();
    void turn_off();
    void tx_data();
    std::string status_;
    int serial_fd_;
    std::string dev_;
    int baud_;
    uint8_t cmd_[4];
    int speed_;
    int angle_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<planb_ros2::msg::Cmd>::SharedPtr sub_cmd_;
    rclcpp::Subscription<planb_ros2::msg::Operate>::SharedPtr sub_operate_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
};

#endif
