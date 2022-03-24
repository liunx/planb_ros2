#include "planb/vision_node.hpp"

#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace planb
{
  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type)
    {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
    }
  }

  void convert_frame_to_message(const cv::Mat &frame, sensor_msgs::msg::Image &msg)
  {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = "vision_frame";
  }

  VisionNode::VisionNode(const rclcpp::NodeOptions &options)
      : Node("vision_node", options)
  {
    width_ = this->declare_parameter("width", 640);
    height_ = this->declare_parameter("height", 480);
    device_id_ = this->declare_parameter("device_id", 0);

    // Initialize OpenCV video capture stream.
    cap_.open(device_id_);

    // Set the width and height based on command line arguments.
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    if (!cap_.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    timer_ = this->create_wall_timer(
        30ms,
        std::bind(&planb::VisionNode::timer_callback, this));

    auto qos = rclcpp::QoS(10);
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("topic_vision", qos);
  }

  VisionNode::~VisionNode()
  {
    cap_.release();
  }

  void VisionNode::timer_callback()
  {
    cv::Mat frame;
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->get_clock()->now();
    cap_ >> frame;
    if (frame.empty())
      return;
    convert_frame_to_message(frame, *msg);
    pub_->publish(std::move(msg));
  }

} // namespace planb

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(planb::VisionNode)