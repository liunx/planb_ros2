#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "planb_common/common.hpp"
#include <list>

#define WIN_CANVAS "canvas output"

using namespace std::chrono_literals;

class Movie3d: public rclcpp::Node
{
public:
    Movie3d(const rclcpp::NodeOptions &options)
        : Node("movie3d", options)
    {
        width_ = this->declare_parameter("width", 640);
        height_ = this->declare_parameter("height", 240);
        auto rect_width = width_ / 2;
        rect_L_ = cv::Rect(0, 0, rect_width, height_);
        rect_R_ = cv::Rect(rect_width, 0, rect_width, height_);

        auto maps = this->declare_parameter("maps", "./data/stereo_rectify_maps.xml");
        cv::FileStorage cv_file2 = cv::FileStorage(maps, cv::FileStorage::READ);
        cv_file2["Left_Stereo_Map_x"] >> left_stereo_map1_;
        cv_file2["Left_Stereo_Map_y"] >> left_stereo_map2_;
        cv_file2["Right_Stereo_Map_x"] >> right_stereo_map1_;
        cv_file2["Right_Stereo_Map_y"] >> right_stereo_map2_;
        cv_file2.release();

        sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/planb/camera/stream",
            1,
            [](sensor_msgs::msg::Image::UniquePtr)
            { assert(false); });
    }

    ~Movie3d() {}

    void run()
    {
        rclcpp::WaitSet wait_set({{{sub_stream_}}}, {}, {});
        sensor_msgs::msg::Image image_msg;
        rclcpp::MessageInfo msg_info;

        // canvas
        cv::namedWindow(WIN_CANVAS, cv::WINDOW_NORMAL);
        cv::resizeWindow(WIN_CANVAS, width_ / 2, height_);

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

                    cv::Mat left_nice, right_nice;

                    cv::remap(frameL,
                              left_nice,
                              left_stereo_map1_,
                              left_stereo_map2_,
                              cv::INTER_LANCZOS4,
                              cv::BORDER_CONSTANT,
                              0);

                    cv::remap(frameR,
                              right_nice,
                              right_stereo_map1_,
                              right_stereo_map2_,
                              cv::INTER_LANCZOS4,
                              cv::BORDER_CONSTANT,
                              0);

                    cv::Mat left_nice_split[3], right_nice_split[3];

                    std::vector<cv::Mat> anaglyph_channels;

                    cv::split(left_nice, left_nice_split);
                    cv::split(right_nice, right_nice_split);

                    anaglyph_channels.push_back(left_nice_split[0]);
                    anaglyph_channels.push_back(left_nice_split[1]);
                    anaglyph_channels.push_back(right_nice_split[2]);

                    cv::Mat output_canvas;

                    cv::merge(anaglyph_channels, output_canvas);

                    cv::imshow(WIN_CANVAS, output_canvas);
                    cv::waitKey(1);
            }
        }
        cv::destroyAllWindows();
    }

private:
    std::string data_path_;
    uint32_t width_, height_;
    cv::Rect rect_L_, rect_R_;
    cv::Mat left_stereo_map1_, left_stereo_map2_;
    cv::Mat right_stereo_map1_, right_stereo_map2_;
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
    auto node = std::make_shared<Movie3d>(options);
    node->run();

    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}