#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "planb/common.hpp"
#include <list>

#define WIN_NAME "calibrate tool"

using namespace std::chrono_literals;

struct Line
{
    cv::Point2i p1;
    cv::Point2i p2;
};

void mouse_event(int evt, int x, int y, int flags, void *param);

class CalibrateTool: public rclcpp::Node
{
public:
    CalibrateTool(const rclcpp::NodeOptions &options)
        : Node("calibrate_tool", options),
          flag_print_menu_(false),
          menu_("main")
    {
        width_ = this->declare_parameter("width", 640);
        height_ = this->declare_parameter("height", 240);
        side_ = this->declare_parameter("side", "L");

        sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/planb/camera/stream",
            1,
            [](sensor_msgs::msg::Image::UniquePtr)
            { assert(false); });
    }

    ~CalibrateTool() {}

    void push_back(cv::Point2i point)
    {
        if (points_.size() >= 2)
        {
            points_.pop_front();
        }
        points_.push_back(point);
    }

    void run()
    {
        rclcpp::WaitSet wait_set({{{sub_stream_}}}, {}, {});
        sensor_msgs::msg::Image image_msg;
        rclcpp::MessageInfo msg_info;

        cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
        cv::resizeWindow(WIN_NAME, width_ / 2, height_);
        cv::setMouseCallback(WIN_NAME, mouse_event, this);

        auto rect_width = width_ / 2;
        auto rect_L = cv::Rect(0, 0, rect_width, height_);
        auto rect_R = cv::Rect(rect_width, 0, rect_width, height_);
        cv::Mat disp;

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
                if (side_ == "L")
                    disp = frame(rect_L);
                else
                    disp = frame(rect_R);

                auto key = cv::waitKey(1);
                if (menu_ == "main")
                    menu_main(key, disp);
                else if (menu_ == "draw_line")
                    menu_draw_line(key, disp);

                cv::imshow(WIN_NAME, disp);
            }
        }
        cv::destroyAllWindows();
    }

private:
    int linear_solving()
    {
        if (points_.size() < 2)
            return planb::RET_ERROR;
        Eigen::Matrix2f A;
        Eigen::Vector2f b;
        auto p1 = points_.front();
        points_.pop_front();
        auto p2 = points_.front();
        points_.pop_front();

        A << p1.x, 1, p2.x, 1;
        b << p1.y, p2.y;

        Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);

        auto x1 = -x[1] / x[0];
        auto x2 = (height_ - x[1]) / x[0];

        lines_.push_back({cv::Point2f(x1, 0), cv::Point2f(x2, height_)});

        RCLCPP_INFO(this->get_logger(), "%.2fx + %.2f = y", x[0], x[1]);

        return planb::RET_SUCCESS;
    }

    void menu_draw_line(const int key, cv::Mat &disp)
    {
        if (!flag_print_menu_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[draw line]\n\r--> (b)back (h)elp");
            flag_print_menu_ = true;
        }

        if (key == 'b')
        {
            menu_ = "main";
            flag_print_menu_ = false;
            points_.clear();
        }
        else if (key == 'h')
        {
            flag_print_menu_ = false;
        }
        else if (key == planb::KEY_SPACE)
        {
            linear_solving();
        }
        else if (key == planb::KEY_ESC)
        {
            lines_.clear();
        }

        for (auto point : points_)
        {
            cv::circle(disp, point, 2, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
        }

        for (auto line : lines_)
        {
            cv::line(disp, line.p1, line.p2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        }
    }

    void menu_main(const int key, cv::Mat &disp)
    {
        if (!flag_print_menu_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[main menu]\n\r--> (l)ine (h)elp");
            flag_print_menu_ = true;
        }

        if (key == 'l')
        {
            menu_ = "draw_line";
            flag_print_menu_ = false;
        }
        else if (key == 'h')
        {
            flag_print_menu_ = false;
        }
    }

private:
    bool flag_print_menu_;
    uint32_t width_, height_;
    std::string side_;
    std::string menu_;
    Eigen::Vector2f x_;
    std::list<cv::Point2i> points_;
    std::vector<Line> lines_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;
};

void mouse_event(int evt, int x, int y, int flags, void *param)
{
    CalibrateTool *cls = (CalibrateTool *)param;
    if (evt == cv::EVENT_LBUTTONDOWN)
    {
        cls->push_back(cv::Point2i(x, y));
    }
}

int main(int argc, char **argv)
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Init ROS
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options{};
    options.use_intra_process_comms(true);
    auto node = std::make_shared<CalibrateTool>(options);
    node->run();

    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}