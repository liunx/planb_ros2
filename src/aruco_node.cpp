#include <thread>
#include "planb/aruco_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

ArucoNode::ArucoNode(const rclcpp::NodeOptions &options)
    : Node("aruco", options),
      status_("Idle")
{
    auto dict_id = this->declare_parameter("dict_id", 0);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));
    detector_params_ = cv::aruco::DetectorParameters::create();

    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/aruco/status", 1);
    pub_aruco_  = this->create_publisher<planb_ros2::msg::Aruco>("/planb/aruco/aruco", 1);
    auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr) { assert(false); };
    // In Idle mode, streaming nothing
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>("/planb/camera/null", 10, do_nothing);
    sub_cmd_ = this->create_subscription<std_msgs::msg::String>(
        "/planb/aruco/cmd",
        1,
        std::bind(&ArucoNode::cmd_callback, this, _1));
}

ArucoNode::~ArucoNode()
{
}

void ArucoNode::update_status()
{
    std_msgs::msg::String msg;
    msg.data = status_;
    pub_status_->publish(std::move(msg));
}

void ArucoNode::stream_callback(const sensor_msgs::msg::Image::UniquePtr msg)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2i>> corners, rejected;

    cv::Mat frame(
        msg->height, msg->width, planb::encoding2mat_type(msg->encoding),
        const_cast<unsigned char *>(msg->data.data()), msg->step);
    if (frame.empty())
        return;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);
    if (ids.size() > 0) {
        planb_ros2::msg::Aruco _msg;
        for (size_t i{0}; i < ids.size(); i++) {
            planb_ros2::msg::Box box;
            auto _id = ids.at(i);
            auto corner = corners.at(i);
            std::vector<int> xs = {corner.at(0).x, corner.at(1).x, corner.at(2).x, corner.at(3).x};
            std::vector<int> ys = {corner.at(0).y, corner.at(1).y, corner.at(2).y, corner.at(3).y};
            auto result_x = std::minmax_element(std::begin(xs), std::end(xs));
            auto result_y = std::minmax_element(std::begin(ys), std::end(ys));
            box.id = _id;
            box.x = *result_x.first;
            box.width = *result_x.second - *result_x.first;
            box.y = *result_y.first;
            box.height = *result_y.second - *result_y.first;
            _msg.data.push_back(box);
        }
        pub_aruco_->publish(std::move(_msg));
    }
}

void ArucoNode::cmd_callback(const std_msgs::msg::String &msg)
{
    if (msg.data == "Run")
    {
        if (status_ != "Run")
        {
            sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/planb/camera/stream",
                10,
                std::bind(&ArucoNode::stream_callback, this, _1));
            status_ = "Run";
            update_status();
        }
    }
    else if (msg.data == "Idle")
    {
        if (status_ != "Idle") {
            auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr) { assert(false); };
            sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>("/planb/camera/null", 10, do_nothing);
            status_ = "Idle";
            update_status();
        }
    }
}
