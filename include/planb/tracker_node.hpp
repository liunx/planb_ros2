#ifndef TRACKER_HPP
#define TRACKER_HPP
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "planb_ros2/msg/aruco_marker.hpp"
#include "planb_ros2/msg/coord_info.hpp"
#include "planb_ros2/msg/cmd.hpp"

class TrackerNode : public rclcpp::Node
{
public:
    TrackerNode(const rclcpp::NodeOptions &options);
    ~TrackerNode() override;

private:
    void publish_status(const std::string &status);
    void stream_on();
    void stream_off();
    void stream_callback(const sensor_msgs::msg::Image::UniquePtr msg);
    void cmd_callback(const planb_ros2::msg::Cmd &msg);
    void aruco_data_callback(const planb_ros2::msg::ArucoMarker &msg);
    std::string status_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<planb_ros2::msg::Cmd>::SharedPtr sub_cmd_;
    rclcpp::Publisher<planb_ros2::msg::CoordInfo>::SharedPtr pub_data_;
    rclcpp::Subscription<planb_ros2::msg::ArucoMarker>::SharedPtr sub_aruco_data_;
    cv::Ptr<cv::Tracker> tracker_;
    uint8_t track_type_;
    uint8_t marker_id_;
    cv::Rect2d bbox_;
    bool flag_init_;
    bool flag_tracking_;
};

#endif
