#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "planb/common.hpp"
#include "planb_ros2/msg/cmd.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

// These parameters can vary according to the setup
float max_depth = 400.0;   // maximum distance the setup can measure (in cm)
float min_depth = 50.0;    // minimum distance the setup can measure (in cm)
float depth_thresh = 45.0; // Threshold for SAFE distance (in cm)

// initialize values for StereoSGBM parameters
int numDisparities = 8;
int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
float M = 0.0;

class StereoCV : public rclcpp::Node
{
public:
    StereoCV(const rclcpp::NodeOptions &options)
        : Node("stereoCV", options),
          status_("OFF")
    {
        uint32_t width = this->declare_parameter("width", 640);
        uint32_t height = this->declare_parameter("height", 240);
        std::string params = this->declare_parameter("params", "./config/depth_estimation_params.xml");
        std::string maps = this->declare_parameter("maps", "./config/stereo_rectify_maps.xml");

        auto rect_width = width / 2;
        rect_L_ = cv::Rect(0, 0, rect_width, height);
        rect_R_ = cv::Rect(rect_width, 0, rect_width, height);
        // Creating an object of StereoBM algorithm
        stereo_ = cv::StereoBM::create();

        // Reading the stored the StereoBM parameters
        cv::FileStorage cv_file = cv::FileStorage(params, cv::FileStorage::READ);
        cv_file["numDisparities"] >> numDisparities;
        cv_file["blockSize"] >> blockSize;
        cv_file["preFilterType"] >> preFilterType;
        cv_file["preFilterSize"] >> preFilterSize;
        cv_file["preFilterCap"] >> preFilterCap;
        cv_file["minDisparity"] >> minDisparity;
        cv_file["textureThreshold"] >> textureThreshold;
        cv_file["uniquenessRatio"] >> uniquenessRatio;
        cv_file["speckleRange"] >> speckleRange;
        cv_file["speckleWindowSize"] >> speckleWindowSize;
        cv_file["disp12MaxDiff"] >> disp12MaxDiff;
        cv_file["M"] >> M;

        // updating the parameter values of the StereoBM algorithm
        stereo_->setNumDisparities(numDisparities);
        stereo_->setBlockSize(blockSize);
        stereo_->setPreFilterType(preFilterType);
        stereo_->setPreFilterSize(preFilterSize);
        stereo_->setPreFilterCap(preFilterCap);
        stereo_->setTextureThreshold(textureThreshold);
        stereo_->setUniquenessRatio(uniquenessRatio);
        stereo_->setSpeckleRange(speckleRange);
        stereo_->setSpeckleWindowSize(speckleWindowSize);
        stereo_->setDisp12MaxDiff(disp12MaxDiff);
        stereo_->setMinDisparity(minDisparity);

        // Initialize variables to store the maps for stereo rectification

        // Reading the mapping values for stereo image rectification
        cv::FileStorage cv_file2 = cv::FileStorage(maps, cv::FileStorage::READ);
        cv_file2["Left_Stereo_Map_x"] >> left_stereo_map1_;
        cv_file2["Left_Stereo_Map_y"] >> left_stereo_map2_;
        cv_file2["Right_Stereo_Map_x"] >> right_stereo_map1_;
        cv_file2["Right_Stereo_Map_y"] >> right_stereo_map2_;
        cv_file2.release();

        pub_status_ = this->create_publisher<std_msgs::msg::String>("/planb/stereoCV/status", 1);

        sub_cmd_ = this->create_subscription<planb_ros2::msg::Cmd>(
            "/planb/stereoCV/cmd",
            1,
            std::bind(&StereoCV::cmd_callback, this, _1));

        stream_on();
    }

    ~StereoCV()
    {
    }

private:
    void publish_status(const std::string &status)
    {
        std_msgs::msg::String msg;
        status_ = status;
        msg.data = status;
        pub_status_->publish(std::move(msg));
    }
    void stream_on()
    {
        sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/planb/camera/stream",
            10,
            std::bind(&StereoCV::stream_callback, this, _1));
    }

    void stream_off()
    {
        auto do_nothing = [](sensor_msgs::msg::Image::UniquePtr)
        { assert(false); };
        sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>("/planb/camera/null", 10, do_nothing);
    }

    // function to sort contours from largest to smallest
    static bool compare_contour_areas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
    {
        double i = fabs(cv::contourArea(cv::Mat(contour1)));
        double j = fabs(cv::contourArea(cv::Mat(contour2)));
        return (i > j);
    }

    void obstacle_avoid(cv::Mat &depth_map, cv::Mat &disparity, cv::Mat output_canvas)
    {
        cv::Mat mask, mean, stddev, mask2;

        // Mask to segment regions with depth less than safe distance
        cv::inRange(depth_map, 10, depth_thresh, mask);
        double s = (cv::sum(mask)[0]) / 255.0;
        double img_area = double(mask.rows * mask.cols);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        // Check if a significantly large obstacle is present and filter out smaller noisy regions
        if (s > 0.08 * img_area)
        {
            // finding contours in the generated mask
            cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            // sorting contours from largest to smallest
            std::sort(contours.begin(), contours.end(), compare_contour_areas);

            // extracting the largest contour
            std::vector<cv::Point> cnt = contours[0];

            // Check if detected contour is significantly large (to avoid multiple tiny regions)
            double cnt_area = fabs(cv::contourArea(cv::Mat(cnt)));
            if (cnt_area > 0.01 * img_area)
            {
                cv::Rect box;

                // Finding the bounding rectangle for the largest contour
                box = cv::boundingRect(cnt);

                // finding average depth of region represented by the largest contour
                mask2 = mask * 0;
                cv::drawContours(mask2, contours, 0, (255), -1);
                cv::drawContours(disparity, contours, 0, (255), -1);
                cv::rectangle(output_canvas, box, cv::Scalar(255, 0, 255), 2, 1);

                // Calculating the average depth of the object closer than the safe distance
                cv::meanStdDev(depth_map, mean, stddev, mask2);

#if 1
                // Printing the warning text with object distance
                char text[10];
                std::sprintf(text, "%.2f cm", mean.at<double>(0, 0));
                cv::putText(output_canvas, "WARNING!", cv::Point2f(box.x + 5, box.y - 40), 1, 2, cv::Scalar(0, 0, 255), 2, 2);
                cv::putText(output_canvas, "Object at", cv::Point2f(box.x + 5, box.y), 1, 2, cv::Scalar(0, 0, 255), 2, 2);
                cv::putText(output_canvas, text, cv::Point2f(box.x + 5, box.y + 40), 1, 2, cv::Scalar(0, 0, 255), 2, 2);
#endif
            }
        }
        else
        {
            // Printing SAFE if no obstacle is closer than the safe distance
            cv::putText(output_canvas, "SAFE!", cv::Point2f(200, 200), 1, 2, cv::Scalar(0, 255, 0), 2, 2);
        }

        // Displaying the output of the obstacle avoidance system
        cv::imshow("disparity", disparity);
        cv::imshow("output_canvas", output_canvas);
    }

    void stream_callback(const sensor_msgs::msg::Image::UniquePtr msg)
    {
        cv::Mat disp, disparity, depth_map, output_canvas;
        cv::Mat imgL, imgR, imgL_gray, imgR_gray;

        cv::Mat frame(
            msg->height, msg->width, planb::encoding2mat_type(msg->encoding),
            const_cast<unsigned char *>(msg->data.data()), msg->step);
        if (frame.empty())
            return;
        imgL = frame(rect_L_);
        imgL.copyTo(output_canvas);
        imgR = frame(rect_R_);
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
        // NOTE: compute returns a 16bit signed single channel image,
        // CV_16S containing a disparity map scaled by 16. Hence it
        // is essential to convert it to CV_32F and scale it down 16 times.

        // Converting disparity values to CV_32F from CV_16S
        disp.convertTo(disparity, CV_32F, 1.0);

        // Scaling down the disparity values and normalizing them
        disparity = (disparity / 16.0f - (float)minDisparity) / ((float)numDisparities);
        depth_map = (float)M / disparity;

        obstacle_avoid(depth_map, disparity, output_canvas);

        cv::waitKey(1);
    }

    void cmd_callback(const planb_ros2::msg::Cmd &msg)
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
    }

    std::string status_;
    cv::Rect rect_L_, rect_R_;
    cv::Mat left_stereo_map1_, left_stereo_map2_;
    cv::Mat right_stereo_map1_, right_stereo_map2_;
    cv::Ptr<cv::StereoBM> stereo_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<planb_ros2::msg::Cmd>::SharedPtr sub_cmd_;
};

int main(int argc, char **argv)
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Init ROS
    rclcpp::init(argc, argv);

    // Create single-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create and add camera node
    rclcpp::NodeOptions options{};
    options.use_intra_process_comms(true);
    auto node = std::make_shared<StereoCV>(options);
    executor.add_node(node);

    // Spin until rclcpp::ok() returns false
    executor.spin();

    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}