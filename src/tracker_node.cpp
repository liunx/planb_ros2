#include "planb/tracker_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

TrackerNode::TrackerNode(const rclcpp::NodeOptions &options)
    : Node("tracker", options),
      status_("OFF"),
      track_type_(0),
      marker_id_(0),
      flag_init_(true),
      flag_tracking_(false)
{
    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/tracker/status", 1);
    pub_data_  = this->create_publisher<planb_ros2::msg::CoordInfo>("/planb/tracker/data", 1);
    sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/planb/camera/null",
        10,
        [](sensor_msgs::msg::Image::UniquePtr)
        { assert(false); });

    sub_aruco_data_ = this->create_subscription<planb_ros2::msg::ArucoMarker>(
        "/planb/aruco/data",
        1,
        std::bind(&TrackerNode::aruco_data_callback, this, _1));

    sub_cmd_ = this->create_subscription<planb_ros2::msg::Cmd>(
        "/planb/tracker/cmd",
        1,
        std::bind(&TrackerNode::cmd_callback, this, _1));
}

TrackerNode::~TrackerNode()
{
}

void TrackerNode::publish_status(const std::string &status)
{
    status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
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
    if (!flag_tracking_)
        return;
    cv::Mat frame(
        msg->height, msg->width, planb::encoding2mat_type(msg->encoding),
        const_cast<unsigned char *>(msg->data.data()), msg->step);
    if (frame.empty())
        return;
    if (flag_init_) {
        tracker_ = cv::TrackerMOSSE::create();
        tracker_->init(frame, bbox_);
        flag_init_ = false;
        return;
    }

    if (tracker_->update(frame, bbox_)) {
        planb_ros2::msg::CoordInfo msg;
        msg.header.stamp = this->get_clock()->now();
        msg.type = planb::TRACK_TYPE_ARUCO;
        msg.bbox.x = bbox_.x;
        msg.bbox.y = bbox_.y;
        msg.bbox.width = bbox_.width;
        msg.bbox.height = bbox_.height;
        pub_data_->publish(std::move(msg));
    }
    else {
        flag_init_ = false;
        flag_tracking_ = false;
    }
}

void TrackerNode::aruco_data_callback(const planb_ros2::msg::ArucoMarker &msg)
{
    if (track_type_ != planb::CMD_TRACK_ARUCO)
        return;
    if (flag_tracking_)
        return;
    // find matched id
    for (auto marker : msg.bboxes)
    {
        if (marker.id == marker_id_)
        {
            bbox_ = cv::Rect2d(marker.x, marker.y, marker.width, marker.height);
            flag_init_ = true;
            flag_tracking_ = true;
            break;
        }
    }
}

void TrackerNode::cmd_callback(const planb_ros2::msg::Cmd &msg)
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
    else if (msg.cmd == planb::CMD_TRACK_ARUCO)
    {
        track_type_ = planb::TRACK_TYPE_ARUCO;
        RCLCPP_INFO(this->get_logger(), "tracking id: %d", msg.id);
        marker_id_ = msg.id;
    }
}
