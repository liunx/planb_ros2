#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "planb/common.hpp"
#include <list>

#define WIN_CANVAS "canvas output"
#define WIN_DISPARITY "disparity"

using namespace std::chrono_literals;

// initialize values for StereoSGBM parameters
int numDisparities = 8;
int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 5;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
float M = 0.0;

void mouse_event(int evt, int x, int y, int flags, void *param);
static void on_trackbar1(int, void *);
static void on_trackbar2(int, void *);
static void on_trackbar3(int, void *);
static void on_trackbar4(int, void *);
static void on_trackbar5(int, void *);
static void on_trackbar6(int, void *);
static void on_trackbar7(int, void *);
static void on_trackbar8(int, void *);
static void on_trackbar9(int, void *);
static void on_trackbar10(int, void *);
static void on_trackbar11(int, void *);

class Calibrate: public rclcpp::Node
{
public:
    Calibrate(const rclcpp::NodeOptions &options)
        : Node("stereo_calibrate", options)
    {
        width_ = this->declare_parameter("width", 640);
        height_ = this->declare_parameter("height", 240);
        auto rect_width = width_ / 2;
        rect_L_ = cv::Rect(0, 0, rect_width, height_);
        rect_R_ = cv::Rect(rect_width, 0, rect_width, height_);

        params_file_ = this->declare_parameter("params", "./config/depth_estimation_params.xml");
        auto maps = this->declare_parameter("maps", "./config/stereo_rectify_maps.xml");
        // Creating an object of StereoBM algorithm
        stereo_ = cv::StereoBM::create();
        // Reading the mapping values for stereo image rectification
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

    ~Calibrate() {}

    void run()
    {
        rclcpp::WaitSet wait_set({{{sub_stream_}}}, {}, {});
        sensor_msgs::msg::Image image_msg;
        rclcpp::MessageInfo msg_info;

        // canvas
        cv::namedWindow(WIN_CANVAS, cv::WINDOW_NORMAL);
        //cv::resizeWindow(WIN_CANVAS, width_ / 2, height_);
        // disparity
        cv::namedWindow(WIN_DISPARITY, cv::WINDOW_NORMAL);
        cv::resizeWindow(WIN_DISPARITY, width_ / 2, height_);
        cv::setMouseCallback(WIN_DISPARITY, mouse_event, this);
        // Creating trackbars to dynamically update the StereoBM parameters
        cv::createTrackbar("numDisparities", WIN_DISPARITY, &numDisparities, 18, on_trackbar1, this);
        cv::createTrackbar("blockSize", WIN_DISPARITY, &blockSize, 50, on_trackbar2, this);
        cv::createTrackbar("preFilterType", WIN_DISPARITY, &preFilterType, 1, on_trackbar3, this);
        cv::createTrackbar("preFilterSize", WIN_DISPARITY, &preFilterSize, 25, on_trackbar4, this);
        cv::createTrackbar("preFilterCap", WIN_DISPARITY, &preFilterCap, 62, on_trackbar5, this);
        cv::createTrackbar("textureThreshold", WIN_DISPARITY, &textureThreshold, 100, on_trackbar6, this);
        cv::createTrackbar("uniquenessRatio", WIN_DISPARITY, &uniquenessRatio, 100, on_trackbar7, this);
        cv::createTrackbar("speckleRange", WIN_DISPARITY, &speckleRange, 100, on_trackbar8, this);
        cv::createTrackbar("speckleWindowSize", WIN_DISPARITY, &speckleWindowSize, 25, on_trackbar9, this);
        cv::createTrackbar("disp12MaxDiff", WIN_DISPARITY, &disp12MaxDiff, 25, on_trackbar10, this);
        cv::createTrackbar("minDisparity", WIN_DISPARITY, &minDisparity, 25, on_trackbar11, this);

        cv::Mat disp, output_canvas;
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

                cv::Mat imgL, imgR, output_canvas;
                frame.copyTo(output_canvas);
                imgL = frame(rect_L_);
                imgR = frame(rect_R_);

                cv::Mat imgL_gray, imgR_gray;
                // Converting images to grayscale
                cv::cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
                cv::cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);

                // Initialize matrix for rectified stero images
                cv::Mat left_nice, right_nice;

                // Applying stereo image rectification on the left image
                cv::remap(imgL_gray,
                          left_nice,
                          left_stereo_map1_,
                          left_stereo_map2_,
                          cv::INTER_LANCZOS4,
                          cv::BORDER_CONSTANT,
                          0);
                // Applying stereo image rectification on the right image
                cv::remap(imgR_gray,
                          right_nice,
                          right_stereo_map1_,
                          right_stereo_map2_,
                          cv::INTER_LANCZOS4,
                          cv::BORDER_CONSTANT,
                          0);

                // Calculating disparith using the StereoBM algorithm
                stereo_->compute(left_nice, right_nice, disp);

                cv::Mat disparity;
                disp.convertTo(disparity, CV_32F, 1.0);
                // Scaling down the disparity values and normalizing them
                disparity_ = (disparity / (float)16.0 - (float)minDisparity) / ((float)numDisparities);

                auto key = cv::waitKey(1);
                if (key == planb::KEY_ESC)
                    points_.clear();
                else if (key == 's')
                    save_params();

                for (auto point : points_)
                {
                    cv::circle(disparity_, point, 5, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
                    cv::circle(output_canvas, point, 5, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
                }

                cv::imshow(WIN_DISPARITY, disparity_);
                cv::imshow(WIN_CANVAS, output_canvas);
            }
        }
        cv::destroyAllWindows();
    }

    void on_mouse_event(int x, int y)
    {
        float depth_val = disparity_.at<float>(y, x);
        if (depth_val > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Depth: %.2f", depth_val);
        }
        points_.push_back(cv::Point2i(x, y));
    }

    void save_params()
    {
        RCLCPP_INFO(this->get_logger(), "Save params to %s", params_file_.c_str());
        cv::FileStorage cv_file = cv::FileStorage(params_file_, cv::FileStorage::WRITE);
        cv_file.write("numDisparities", numDisparities);
        cv_file.write("blockSize", blockSize);
        cv_file.write("preFilterType", preFilterType);
        cv_file.write("preFilterSize", preFilterSize);
        cv_file.write("preFilterCap", preFilterCap);
        cv_file.write("textureThreshold", textureThreshold);
        cv_file.write("uniquenessRatio", uniquenessRatio);
        cv_file.write("speckleRange", speckleRange);
        cv_file.write("speckleWindowSize", speckleWindowSize);
        cv_file.write("disp12MaxDiff", disp12MaxDiff);
        cv_file.write("minDisparity", minDisparity);
        cv_file.write("M", M);
        cv_file.release();
    }

    void calculate_depth()
    {
    }

public:
    cv::Ptr<cv::StereoBM> stereo_;

private:
    std::string params_file_;
    uint32_t width_, height_;
    cv::Rect rect_L_, rect_R_;
    cv::Mat disparity_;
    cv::Mat left_stereo_map1_, left_stereo_map2_;
    cv::Mat right_stereo_map1_, right_stereo_map2_;
    std::vector<cv::Point2i> points_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;
};

void mouse_event(int evt, int x, int y, int, void *param)
{
    Calibrate *_this = (Calibrate *)param;
    if (evt == cv::EVENT_LBUTTONDOWN)
    {
        _this->on_mouse_event(x, y);
    }
}

static void on_trackbar1(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setNumDisparities(numDisparities * 16);
    numDisparities = numDisparities * 16;
}
static void on_trackbar2(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setBlockSize(blockSize * 2 + 5);
    blockSize = blockSize * 2 + 5;
}

static void on_trackbar3(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setPreFilterType(preFilterType);
}

static void on_trackbar4(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setPreFilterSize(preFilterSize * 2 + 5);
    preFilterSize = preFilterSize * 2 + 5;
}

static void on_trackbar5(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setPreFilterCap(preFilterCap);
}

static void on_trackbar6(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setTextureThreshold(textureThreshold);
}

static void on_trackbar7(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setUniquenessRatio(uniquenessRatio);
}

static void on_trackbar8(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setSpeckleRange(speckleRange);
}

static void on_trackbar9(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setSpeckleWindowSize(speckleWindowSize * 2);
    speckleWindowSize = speckleWindowSize * 2;
}

static void on_trackbar10(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setDisp12MaxDiff(disp12MaxDiff);
}

static void on_trackbar11(int, void *data)
{
    Calibrate *_this = (Calibrate *)data;
    _this->stereo_->setMinDisparity(minDisparity);
}

int main(int argc, char **argv)
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Init ROS
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options{};
    options.use_intra_process_comms(true);
    auto node = std::make_shared<Calibrate>(options);
    node->run();

    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}