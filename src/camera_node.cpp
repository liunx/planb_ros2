#include <thread>
#include <chrono>
#include "planb/camera_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

CameraNode::CameraNode(const rclcpp::NodeOptions &options)
    : Node("camera", options),
      status_("OFF")
{
    width_ = this->declare_parameter("width", 640);
    height_ = this->declare_parameter("height", 480);
    device_id_ = this->declare_parameter("device_id", 0);
    msec_ = this->declare_parameter("msec", 30);

    pub_stream_  = this->create_publisher<sensor_msgs::msg::Image>("/planb/camera/stream", 1);
    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/camera/status", 1);

    sub_cmd_ = this->create_subscription<planb_ros2::msg::Cmd>(
        "/planb/camera/cmd",
        1,
        std::bind(&CameraNode::cmd_callback, this, _1));

    auto handle_service =
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<planb_ros2::srv::CameraInfo::Request> request,
               std::shared_ptr<planb_ros2::srv::CameraInfo::Response> response) -> void
    {
        (void)request_header;
        (void)request;
        response->width = width_;
        response->height = height_;
    };
    srv_ = create_service<planb_ros2::srv::CameraInfo>("camera_info", handle_service);
}

CameraNode::~CameraNode()
{
}

void CameraNode::camera_on()
{
    if (status_ == "ON") {
        publish_status("ON");
        return;
    }

    cam_.open(device_id_);
    if (!cam_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d!", device_id_);
        publish_status("FAULT");
        return;
    }

    cam_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    publish_status("ON");
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(msec_),
        std::bind(&CameraNode::timer_callback, this));
}

void CameraNode::camera_off()
{
    if (status_ == "OFF")
    {
        publish_status("OFF");
        return;
    }

    timer_->cancel();
    cam_.release();
    publish_status("OFF");
}

void CameraNode::publish_status(const std::string &status)
{
    std_msgs::msg::String msg;
    msg.data = status;
    status_ = status;
    pub_status_->publish(std::move(msg));
}

void CameraNode::timer_callback()
{
    cv::Mat frame;
    cam_ >> frame;
    if (frame.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Frame is empty!");
    }
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->get_clock()->now();
    planb::convert_frame_to_message(frame, *msg);
    pub_stream_->publish(std::move(msg));
}

void CameraNode::cmd_callback(const planb_ros2::msg::Cmd &msg)
{
    if (msg.cmd == planb::CMD_TURN_ON)
    {
        camera_on();
    }
    else if (msg.cmd == planb::CMD_TURN_OFF)
    {
        camera_off();
    }
}
