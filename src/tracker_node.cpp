#include "planb/tracker_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

TrackerNode::TrackerNode(const rclcpp::NodeOptions &options)
    : Node("tracker", options),
      status_("Idle"),
      flag_init_(false)
{
    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/tracker/status", 1);
    pub_data_  = this->create_publisher<planb_ros2::msg::Box>("/planb/tracker/data", 1);
    auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr) { assert(false); };
    // In Idle mode, streaming nothing
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>("/planb/camera/null", 10, do_nothing);
    sub_input_ = this->create_subscription<planb_ros2::msg::Box>(
        "/planb/tracker/input",
        1,
        std::bind(&TrackerNode::input_callback, this, _1));
}

TrackerNode::~TrackerNode()
{
}

void TrackerNode::publish_status()
{
    std_msgs::msg::String msg;
    msg.data = status_;
    pub_status_->publish(std::move(msg));
}

void TrackerNode::stream_on()
{
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/planb/camera/stream",
        10,
        std::bind(&TrackerNode::stream_callback, this, _1));
}

void TrackerNode::stream_off()
{
    auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr) { assert(false); };
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>("/planb/camera/null", 10, do_nothing);
}


void TrackerNode::stream_callback(const sensor_msgs::msg::Image::UniquePtr msg)
{
    cv::Mat frame(
        msg->height, msg->width, planb::encoding2mat_type(msg->encoding),
        const_cast<unsigned char *>(msg->data.data()), msg->step);
    if (frame.empty())
        return;
    if (flag_init_ == false) {
        tracker_ = cv::TrackerMOSSE::create();
        tracker_->init(frame, bbox_);
        flag_init_ = true;
        return;
    }

    if (tracker_->update(frame, bbox_)) {
        planb_ros2::msg::Box msg;
        msg.frame_width = frame.cols;
        msg.frame_height = frame.rows;
        msg.x = bbox_.x;
        msg.y = bbox_.y;
        msg.width = bbox_.width;
        msg.height = bbox_.height;
        pub_data_->publish(std::move(msg));
    }
    else {
        flag_init_ = false;
        stream_off();
        status_ = "Idle";
        publish_status();
    }
}

void TrackerNode::input_callback(const planb_ros2::msg::Box &msg)
{
    if (status_ == "Running") {
        publish_status();
        return;
    }
    status_ = "Running";
    bbox_ = cv::Rect2d(msg.x, msg.y, msg.width, msg.height);
    stream_on();
}
