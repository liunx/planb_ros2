#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "planb/common.hpp"
#include <list>

#define WIN_CANVAS "canvas output"

using namespace std::chrono_literals;

class Checkboard: public rclcpp::Node
{
public:
    Checkboard(const rclcpp::NodeOptions &options)
        : Node("checkboard_capture", options)
    {
        width_ = this->declare_parameter("width", 640);
        height_ = this->declare_parameter("height", 240);
        output_path_ = this->declare_parameter("output_path", "./data");
        auto rect_width = width_ / 2;
        rect_L_ = cv::Rect(0, 0, rect_width, height_);
        rect_R_ = cv::Rect(rect_width, 0, rect_width, height_);

        sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/planb/camera/stream",
            1,
            [](sensor_msgs::msg::Image::UniquePtr)
            { assert(false); });
    }

    ~Checkboard() {}

    void run()
    {
        rclcpp::WaitSet wait_set({{{sub_stream_}}}, {}, {});
        sensor_msgs::msg::Image image_msg;
        rclcpp::MessageInfo msg_info;

        // canvas
        cv::namedWindow(WIN_CANVAS, cv::WINDOW_NORMAL);
        cv::resizeWindow(WIN_CANVAS, width_, height_);

        uint32_t count  = 0;

        while (rclcpp::ok())
        {
            auto wait_result = wait_set.wait(3s);
            // timeout
            if (wait_result.kind() != rclcpp::WaitResultKind::Ready)
                continue;
            if (sub_stream_->take(image_msg, msg_info))
            {
                cv::Mat frame(image_msg.height,
                              image_msg.width,
                              planb::encoding2mat_type(image_msg.encoding),
                              const_cast<unsigned char *>(image_msg.data.data()),
                              image_msg.step);
                if (frame.empty())
                    continue;

                cv::Mat frameL, frameR;
                frameL = frame(rect_L_);
                frameR = frame(rect_R_);

                cv::Mat frameL_gray, frameR_gray;
                // Converting images to grayscale
                cv::cvtColor(frameL, frameL_gray, cv::COLOR_BGR2GRAY);
                cv::cvtColor(frameR, frameR_gray, cv::COLOR_BGR2GRAY);

                std::vector<cv::Point2f> cornersL, cornersR;
                cv::Size board = cv::Size(6, 9);
                bool foundL = cv::findChessboardCorners(
                    frameL_gray,
                    board,
                    cornersL,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

                bool foundR = cv::findChessboardCorners(
                    frameR_gray,
                    board,
                    cornersR,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

                auto key = cv::waitKey(1);

                if (foundL && foundR)
                {
                    if (key == planb::KEY_SPACE)
                    {
                        cv::imwrite(output_path_ + "/stereoL/img_" + planb::string_format("%02d", count) + ".png", frameL);
                        cv::imwrite(output_path_ + "/stereoR/img_" + planb::string_format("%02d", count) + ".png", frameR);
                        RCLCPP_INFO(this->get_logger(), "%s/stereoR/img_%02d.png", output_path_.c_str(), count);
                        count++;
                    }

                    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
                    // refining pixel coordinates for given 2d points.
                    cv::cornerSubPix(frameL_gray, cornersL, cv::Size(11, 11), cv::Size(-1, -1), criteria);
                    cv::cornerSubPix(frameR_gray, cornersR, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                    // Displaying the detected corner points on the checker board
                    cv::drawChessboardCorners(frameL, board, cornersL, foundL);
                    cv::drawChessboardCorners(frameR, board, cornersR, foundR);
                }

                cv::imshow(WIN_CANVAS, frame);
            }
        }
        cv::destroyAllWindows();
    }

private:
    std::string output_path_;
    uint32_t width_, height_;
    cv::Rect rect_L_, rect_R_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;
};

int main(int argc, char **argv)
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Init ROS
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options{};
    options.use_intra_process_comms(true);
    auto node = std::make_shared<Checkboard>(options);
    node->run();

    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}