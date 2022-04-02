#include <thread>
#include "planb/aruco_node.hpp"
#include "planb_ros2/msg/bounding_box.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

ArucoNode::ArucoNode(const rclcpp::NodeOptions &options)
    : Node("aruco", options),
      status_("OFF")
{
    auto dict_id = this->declare_parameter("dict_id", 0);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));
    detector_params_ = cv::aruco::DetectorParameters::create();

    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/aruco/status", 1);
    pub_data_  = this->create_publisher<planb_ros2::msg::ArucoMarker>("/planb/aruco/data", 1);
    auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr) { assert(false); };
    // In OFF mode, streaming nothing
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>("/planb/camera/null", 10, do_nothing);
    sub_cmd_ = this->create_subscription<planb_ros2::msg::Cmd>(
        "/planb/aruco/cmd",
        1,
        std::bind(&ArucoNode::cmd_callback, this, _1));
}

ArucoNode::~ArucoNode()
{
}

void ArucoNode::publish_status(const std::string &status)
{
    std_msgs::msg::String msg;
    status_ = status;
    msg.data = status;
    pub_status_->publish(std::move(msg));
}

void ArucoNode::stream_callback(const sensor_msgs::msg::Image::UniquePtr msg)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    cv::Mat frame(
        msg->height, msg->width, planb::encoding2mat_type(msg->encoding),
        const_cast<unsigned char *>(msg->data.data()), msg->step);
    if (frame.empty())
        return;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);
    if (ids.size() > 0) {
        planb_ros2::msg::ArucoMarker _msg;
        _msg.header.stamp = this->get_clock()->now();
        for (size_t i{0}; i < ids.size(); i++) {
            planb_ros2::msg::BoundingBox bbox;
            auto _id = ids.at(i);
            auto corner = corners.at(i);
            std::vector<float> xs = {corner.at(0).x, corner.at(1).x, corner.at(2).x, corner.at(3).x};
            std::vector<float> ys = {corner.at(0).y, corner.at(1).y, corner.at(2).y, corner.at(3).y};
            auto result_x = std::minmax_element(std::begin(xs), std::end(xs));
            auto result_y = std::minmax_element(std::begin(ys), std::end(ys));
            bbox.id = _id;
            bbox.x = *result_x.first;
            bbox.width = *result_x.second - *result_x.first;
            bbox.y = *result_y.first;
            bbox.height = *result_y.second - *result_y.first;
            _msg.bboxes.push_back(bbox);
        }
        pub_data_->publish(std::move(_msg));
    }
}

void ArucoNode::stream_on()
{
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/planb/camera/stream",
        10,
        std::bind(&ArucoNode::stream_callback, this, _1));
}

void ArucoNode::stream_off()
{
    auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr) { assert(false); };
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>("/planb/camera/null", 10, do_nothing);
}


void ArucoNode::cmd_callback(const planb_ros2::msg::Cmd &msg)
{
    if (msg.cmd == planb::CMD_TURN_ON)
    {
        publish_status("ON");
        stream_on();
    }
    else if (msg.cmd == planb::CMD_TURN_OFF)
    {
        publish_status("OFF");
        stream_off();
    }
}
