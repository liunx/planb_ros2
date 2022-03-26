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

#define MIN_ANGLE_INDEX 0
#define MAX_ANGLE_INDEX 12
#define MIN_POWER_INDEX 0
#define MAX_POWER_INDEX 8

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode(const rclcpp::NodeOptions &options);
    ~HardwareNode() override;

private:
    int serial_init(const char *dev, const int baud);
    void publish_status();
    void input_callback(const std_msgs::msg::String &msg);
    void reset();
    void forward();
    void backward();
    void turn_left();
    void turn_right();
    void stop();
    void tx_data();
    std::string status_;
    int serial_fd_;
    std::string dev_;
    int baud_;
    uint8_t cmd_[4];
    uint8_t power_;
    uint8_t angle_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_input_;
};

#endif
