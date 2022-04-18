#include <string>
#include <memory>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace planb {
    enum {
        RET_SUCCESS = 0,
        RET_ERROR = 1
    };

    enum {
        KEY_BS = 0x08,
        KEY_ESC = 0x1B,
        KEY_CR = 0x0D,
        KEY_SPACE = 0x20,
        KEY_DEL = 0x7F
    };

    enum {
        CMD_TURN_OFF = 0,
        CMD_TURN_ON = 1,
        CMD_TRACK_ARUCO = 2
    };

    enum {
        TRACK_TYPE_UNKNOW = 0,
        TRACK_TYPE_ARUCO = 2,
    };

    struct CoordInfo {
        rclcpp::Time stamp;
        float_t x;
        float_t y;
        uint32_t width;
        uint32_t height;
    };

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

    template <typename... Args>
    std::string string_format(const std::string &format, Args... args)
    {
        size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
        if (size <= 0)
        {
            throw std::runtime_error("Error during formatting.");
        }
        std::unique_ptr<char[]> buf(new char[size]);
        snprintf(buf.get(), size, format.c_str(), args...);
        return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
    }
}