#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace planb {
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
        msg.header.frame_id = "camera_frame";
    }
}