#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include "planb_common/common.hpp"
#include "planb_interfaces/msg/aruco_marker.hpp"
#include "planb_interfaces/msg/coord_info.hpp"

using namespace std::chrono_literals;

int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("monitor");
    auto camera_stream = node->create_subscription<sensor_msgs::msg::Image>(
        "/planb/camera/stream",
        1,
        [](sensor_msgs::msg::Image::UniquePtr)
        { assert(false); });
    auto aruco_data = node->create_subscription<planb_interfaces::msg::ArucoMarker>(
        "/planb/aruco/data",
        1,
        [](planb_interfaces::msg::ArucoMarker::UniquePtr)
        { assert(false); });
    auto tracker_data = node->create_subscription<planb_interfaces::msg::CoordInfo>(
        "/planb/tracker/data",
        1,
        [](planb_interfaces::msg::CoordInfo::UniquePtr)
        { assert(false); });

    rclcpp::WaitSet wait_set({{{camera_stream}, {aruco_data}, {tracker_data}}}, {}, {});
    sensor_msgs::msg::Image image_msg;
    planb_interfaces::msg::ArucoMarker aruco_msg;
    planb_interfaces::msg::CoordInfo tracker_msg;
    rclcpp::MessageInfo msg_info;
    cv::namedWindow("monitor", cv::WINDOW_NORMAL);
    bool flag_tracking = false;
    while (rclcpp::ok()) {
        auto wait_result = wait_set.wait(3s);
        // timeout
        if (wait_result.kind() != rclcpp::WaitResultKind::Ready)
            continue;
        if (aruco_data->take(aruco_msg, msg_info))
        {
        }
        if (tracker_data->take(tracker_msg, msg_info))
        {
            flag_tracking = true;
        }
        if (camera_stream->take(image_msg, msg_info))
        {
            cv::Mat frame(
                image_msg.height, image_msg.width, planb::encoding2mat_type(image_msg.encoding),
                const_cast<unsigned char *>(image_msg.data.data()), image_msg.step);
            if (frame.empty())
                continue;
            if (aruco_msg.bboxes.size() > 0) {
                for (auto marker : aruco_msg.bboxes) {
                    auto bbox = cv::Rect2i(marker.x, marker.y, marker.width, marker.height);
                    cv::rectangle(frame, bbox, cv::Scalar(255, 0, 255), 2, 1);
                }
                // cleanup the data
                aruco_msg.bboxes.clear();
            }
            if (flag_tracking) {
                auto bbox = cv::Rect2i(tracker_msg.bbox.x,
                                       tracker_msg.bbox.y,
                                       tracker_msg.bbox.width,
                                       tracker_msg.bbox.height);
                cv::rectangle(frame, bbox, cv::Scalar(255, 255, 0), 2, 1);
                flag_tracking = false;
            }
            cv::imshow("monitor", frame);
            cv::waitKey(1);
        }
    }
    rclcpp::shutdown();
    return 0;
}