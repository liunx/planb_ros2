#ifndef ARUCO_HPP
#define ARUCO_HPP
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/aruco.hpp>
#include "planb_ros2/msg/aruco.hpp"

class ArucoNode : public rclcpp::Node
{
public:
    ArucoNode(const rclcpp::NodeOptions &options);
    ~ArucoNode() override;

private:
    void update_status();
    void stream_callback(const sensor_msgs::msg::Image::UniquePtr msg);
    void cmd_callback(const std_msgs::msg::String &msg);
    std::string status_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<planb_ros2::msg::Aruco>::SharedPtr pub_aruco_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

#endif
