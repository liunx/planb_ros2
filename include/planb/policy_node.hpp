#ifndef POLICY_NODE_HPP
#define POLICY_NODE_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace planb
{

  class PolicyNode : public rclcpp::Node
  {
  public:
    explicit PolicyNode(const rclcpp::NodeOptions &options);

    ~PolicyNode() override;

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_vision_;
  };

} // namespace planb

#endif // POLICY_NODE_HPP
