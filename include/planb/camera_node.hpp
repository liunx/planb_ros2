#ifndef CAMERA_HPP
#define CAMERA_HPP
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode(const rclcpp::NodeOptions &options);
    ~CameraNode() override;
    void loop();
    void idle();
    void running();

private:
    void publish_status();
    cv::VideoCapture cam_;
    uint32_t device_id_, width_, height_;
    std::string mode_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_stream_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
};

#endif
