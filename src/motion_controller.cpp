#include "rclcpp/rclcpp.hpp"
#include "planb/common.hpp"
#include "std_msgs/msg/string.hpp"
#include "planb_ros2/msg/coord_info.hpp"
#include "planb_ros2/msg/operate.hpp"

#define COORD_PADDING 6
#define COORD_Y_PADDING 3

using namespace std::chrono_literals;
using std::placeholders::_1;

class MotionController : public rclcpp::Node
{
public:
    MotionController(const rclcpp::NodeOptions &options)
        : Node("motion_controller", options),
          steering_(6),
          accel_(1),
          coords_({})
    {
        width_ = this->declare_parameter("width", 640);
        height_ = this->declare_parameter("height", 480);
        pub_operate_ = this->create_publisher<planb_ros2::msg::Operate>("/planb/hardware/operate", 1);
        sub_coordinfo_ = this->create_subscription<planb_ros2::msg::CoordInfo>(
            "/planb/tracker/data",
            1,
            std::bind(&MotionController::coordinfo_callback, this, _1));
        timer_ = this->create_wall_timer(100ms, std::bind(&MotionController::timer_callback, this));
    }

    ~MotionController()
    {
    }

private:
    void publish_operate(const uint8_t steering, const uint8_t accel)
    {
        if (steering == steering_ && accel == accel_)
            return;

        steering_ = steering;
        accel_ = accel;

        planb_ros2::msg::Operate msg;
        msg.steering = steering;
        msg.accel = accel;
        pub_operate_->publish(std::move(msg));
    }

    void timer_callback()
    {
        if (coords_.size() < 2)
            return;
        auto len = coords_.size();
        auto c1 = coords_[len-1];
        auto c1_x_center = c1.x + c1.width / 2;
        auto c1_y_center = c1.y + c1.height / 2;
        auto c2 = coords_[len-2];

        uint8_t _accel = accel_;
        uint8_t _steering = steering_;

        if (c1_y_center < height_ * 0.3f) { // Up
            // c1 <== c2
            if (c2.y - c1.y > COORD_Y_PADDING) {
                RCLCPP_INFO(this->get_logger(), "Go Up!");
                _accel = 1;
            }
        }
        else if (c1_y_center > height_ * 0.7f) { // Down
            // c2 ==> c1
            if (c1.y - c2.y > COORD_Y_PADDING) {
                RCLCPP_INFO(this->get_logger(), "Go Down!");
                _accel = 1;
            }
        }
        else {
            _accel = 3;
        }

        if (c1_x_center < width_ * 0.2f) {
            // c1 <== c2
            if (c2.x - c1.x > COORD_PADDING) { // Left
                RCLCPP_INFO(this->get_logger(), "Turn Left!");
                _steering = 4;
            }
        }
        else if (c1_x_center > width_ * 0.8f) { // Right
            // c2 ==> c1
            if (c1.x - c2.x > COORD_PADDING) {
                RCLCPP_INFO(this->get_logger(), "Turn Right!");
                _steering = 8;
            }
        }
        else {
            _steering = 6;
        }
        publish_operate(_steering, _accel);

        coords_.clear();
    }

    void coordinfo_callback(const planb_ros2::msg::CoordInfo &msg)
    {
        if (!coords_.empty())
        {
            auto last = coords_.back();
            auto msg_stamp = rclcpp::Time(msg.header.stamp);
            auto diff = (msg_stamp - last.stamp).seconds();
            double ratio = double(last.width * last.height) / double(msg.bbox.width * msg.bbox.height);
            // check if msg outdated
            if (diff > 1.0f)
            {
                coords_.clear();
            }
            // check bounding box is simular of 80%
            else if (ratio < 0.8f || ratio > 1.25f)
            {
                coords_.clear();
            }
        }
        planb::CoordInfo coord;
        coord.stamp = msg.header.stamp;
        coord.x = msg.bbox.x;
        coord.y = msg.bbox.y;
        coord.width = msg.bbox.width;
        coord.height = msg.bbox.height;
        coords_.push_back(coord);
    }

    uint8_t steering_, accel_;
    uint32_t width_, height_;
    std::string status_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<planb::CoordInfo> coords_;
    rclcpp::Subscription<planb_ros2::msg::CoordInfo>::SharedPtr sub_coordinfo_;
    rclcpp::Publisher<planb_ros2::msg::Operate>::SharedPtr pub_operate_;
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
    auto node = std::make_shared<MotionController>(options);
    executor.add_node(node);

    // Spin until rclcpp::ok() returns false
    executor.spin();

    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}