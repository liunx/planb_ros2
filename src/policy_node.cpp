#include "planb/policy_node.hpp"

#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace planb
{
  int encoding2mat_type(const std::string &encoding)
  {
    if (encoding == "mono8")
    {
      return CV_8UC1;
    }
    else if (encoding == "bgr8")
    {
      return CV_8UC3;
    }
    else if (encoding == "mono16")
    {
      return CV_16SC1;
    }
    else if (encoding == "rgba8")
    {
      return CV_8UC4;
    }
    else if (encoding == "bgra8")
    {
      return CV_8UC4;
    }
    else if (encoding == "32FC1")
    {
      return CV_32FC1;
    }
    else if (encoding == "rgb8")
    {
      return CV_8UC3;
    }
    else
    {
      throw std::runtime_error("Unsupported encoding type");
    }
  }

  PolicyNode::PolicyNode(const rclcpp::NodeOptions &options)
      : Node("policy_node", options)
  {
    timer_ = this->create_wall_timer(
        30ms,
        std::bind(&planb::PolicyNode::timer_callback, this));

    sub_vision_ = this->create_subscription<sensor_msgs::msg::Image>(
        "topic_vision", 1,
        [this](sensor_msgs::msg::Image::UniquePtr msg)
        {
          cv::Mat frame(
              msg->height, msg->width, encoding2mat_type(msg->encoding),
              const_cast<unsigned char *>(msg->data.data()), msg->step);

          RCLCPP_INFO(this->get_logger(),
                      "Receiving Image Size: %dx%d - Timestamp: %u.%u sec ",
                      msg->width, msg->height,
                      msg->header.stamp.sec, msg->header.stamp.nanosec);

          if (msg->encoding == "rgb8")
          {
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
          }
        });
  }

  PolicyNode::~PolicyNode()
  {
  }

  void PolicyNode::timer_callback()
  {
    RCLCPP_INFO(get_logger(), "timer callback!");
  }

} // namespace planb

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(planb::PolicyNode)