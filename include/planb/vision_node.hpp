#ifndef VISION_NODE_HPP
#define VISION_NODE_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace planb
{

  class VisionNode : public rclcpp::Node
  {
  public:
    explicit VisionNode(const rclcpp::NodeOptions &options);

    ~VisionNode() override;

  private:
    void timer_callback();
    cv::VideoCapture cap_;
    uint32_t device_id_, width_, height_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  };

} // namespace planb

#endif // VISION_NODE_HPP
