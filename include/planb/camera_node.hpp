#ifndef CAMERA_HPP
#define CAMERA_HPP
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "planb_ros2/msg/cmd.hpp"
#include "planb_ros2/srv/camera_info.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode(const rclcpp::NodeOptions &options);
    ~CameraNode() override;

private:
    void camera_on();
    void camera_off();
    void publish_status(const std::string &status);
    void timer_callback();
    void cmd_callback(const planb_ros2::msg::Cmd &msg);
    cv::VideoCapture cam_;
    uint32_t device_id_, width_, height_, msec_;
    std::string status_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_stream_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<planb_ros2::msg::Cmd>::SharedPtr sub_cmd_;
    rclcpp::Service<planb_ros2::srv::CameraInfo>::SharedPtr srv_;
};

#endif
