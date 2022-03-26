#include <thread>
#include "planb/camera_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;

CameraNode::CameraNode(const rclcpp::NodeOptions &options)
    : Node("camera", options),
      mode_("Idle")
{
    width_ = this->declare_parameter("width", 640);
    height_ = this->declare_parameter("height", 480);
    device_id_ = this->declare_parameter("device_id", 0);
    auto qos = rclcpp::QoS(10);
    pub_stream_  = this->create_publisher<sensor_msgs::msg::Image>("/planb/camera/stream", qos);
    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/camera/status", 1);
    auto do_nothing = [](std_msgs::msg::String::UniquePtr) { assert(false); };
    sub_cmd_ = this->create_subscription<std_msgs::msg::String>("/planb/camera/cmd", 1, do_nothing);
}

CameraNode::~CameraNode()
{
}

void CameraNode::publish_status()
{
    std_msgs::msg::String msg;
    msg.data = mode_;
    pub_status_->publish(std::move(msg));
}

void CameraNode::loop()
{
    while (rclcpp::ok()) {
        if (mode_ == "Idle") {
            publish_status();
            idle();
        }
        else if (mode_ == "Running") {
            publish_status();
            running();
        }
        else {
            mode_ = "Idle";
        }
    }
}

void CameraNode::idle()
{
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    rclcpp::WaitSet wait_set({{{sub_cmd_}}}, {}, {});
    RCLCPP_INFO(this->get_logger(), "In Idle mode...");
    while (rclcpp::ok()) {
        const auto wait_result = wait_set.wait(3s);
        if (wait_result.kind() == rclcpp::WaitResultKind::Ready)
        {
            if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U])
            {
                if (sub_cmd_->take(msg, msg_info))
                {
                    if (msg.data != "Idle")
                    {
                        mode_ = msg.data;
                        break;
                    }
                }
            }
        }
    }
}

void CameraNode::running()
{
    cam_.open(device_id_);
    if (!cam_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d!", device_id_);
        mode_ = "Idle";
        return;
    }
    cam_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cv::Mat frame;
    RCLCPP_INFO(this->get_logger(), "In Running mode...");

    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    while (rclcpp::ok())
    {
        if (sub_cmd_->take(msg, msg_info)) {
            if (msg.data != "Running") {
                mode_ = msg.data;
                break;
            }
        }
        cam_ >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Frame is empty, breaking!");
            mode_ = "Idle";
            break;
        }
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->get_clock()->now();
        planb::convert_frame_to_message(frame, *msg);
        pub_stream_->publish(std::move(msg));
    }
    cam_.release();
}