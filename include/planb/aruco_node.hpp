#ifndef ARUCO_HPP
#define ARUCO_HPP
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/aruco.hpp>
#include "planb_ros2/msg/aruco_marker.hpp"
#include "planb_ros2/msg/cmd.hpp"

class ArucoNode : public rclcpp::Node
{
public:
    ArucoNode(const rclcpp::NodeOptions &options);
    ~ArucoNode() override;

private:
    void stream_on();
    void stream_off();
    void publish_status(const std::string &status);
    void stream_callback(const sensor_msgs::msg::Image::UniquePtr msg);
    void cmd_callback(const planb_ros2::msg::Cmd &msg);
    std::string status_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<planb_ros2::msg::ArucoMarker>::SharedPtr pub_data_;
    rclcpp::Subscription<planb_ros2::msg::Cmd>::SharedPtr sub_cmd_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

#endif
