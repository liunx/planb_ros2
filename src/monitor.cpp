#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include "planb/common.hpp"
#include "planb_ros2/msg/aruco.hpp"

using namespace std::chrono_literals;

int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("monitor");
    auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr) { assert(false); };
    auto do_nothing2 = [](planb_ros2::msg::Aruco::UniquePtr) { assert(false); };
    auto camera_stream = node->create_subscription<sensor_msgs::msg::Image>("/planb/camera/stream", 1, do_nothing);
    auto aruco_markers = node->create_subscription<planb_ros2::msg::Aruco>("/planb/aruco/markers", 1, do_nothing2);

    rclcpp::WaitSet wait_set({{{camera_stream}, {aruco_markers}}}, {}, {});
    sensor_msgs::msg::Image image_msg;
    planb_ros2::msg::Aruco markers_msg;
    rclcpp::MessageInfo msg_info;
    cv::namedWindow("monitor", cv::WINDOW_NORMAL);
    while (rclcpp::ok()) {
        auto wait_result = wait_set.wait(3s);
        // timeout
        if (wait_result.kind() != rclcpp::WaitResultKind::Ready)
            continue;
        if (aruco_markers->take(markers_msg, msg_info))
        {
        }
        if (camera_stream->take(image_msg, msg_info))
        {
            cv::Mat frame(
                image_msg.height, image_msg.width, planb::encoding2mat_type(image_msg.encoding),
                const_cast<unsigned char *>(image_msg.data.data()), image_msg.step);
            if (frame.empty())
                continue;
            if (markers_msg.data.size() > 0) {
                for (auto marker : markers_msg.data) {
                    auto bbox = cv::Rect2i(marker.x, marker.y, marker.width, marker.height);
                    cv::rectangle(frame, bbox, cv::Scalar(255, 0, 255), 2, 1);
                }
                // cleanup the data
                markers_msg.data.clear();
            }
            cv::imshow("monitor", frame);
            cv::waitKey(1);
        }
    }
    rclcpp::shutdown();
    return 0;
}