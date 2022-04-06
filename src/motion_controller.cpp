#include "rclcpp/rclcpp.hpp"
#include "planb/common.hpp"
#include "std_msgs/msg/string.hpp"
#include "planb_ros2/msg/coord_info.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MotionController : public rclcpp::Node
{
public:
    MotionController(const rclcpp::NodeOptions &options)
        : Node("motion_controller", options),
          coords_({})
    {
        sub_coordinfo_ = this->create_subscription<planb_ros2::msg::CoordInfo>(
            "/planb/tracker/data",
            1,
            std::bind(&MotionController::coordinfo_callback, this, _1));
    }

    ~MotionController()
    {
    }

private:
    void coordinfo_callback(const planb_ros2::msg::CoordInfo &msg)
    {
        if (!coords_.empty())
        {
            auto last = coords_.back();
            auto msg_stamp = rclcpp::Time(msg.header.stamp);
            auto diff = msg_stamp - last.stamp;
            RCLCPP_INFO(this->get_logger(), "Diff: [%f]", diff.seconds());
        }
        planb::CoordInfo coord;
        coord.stamp = this->get_clock()->now();
        coord.x = msg.bbox.x;
        coord.y = msg.bbox.y;
        coord.width = msg.bbox.width;
        coord.height = msg.bbox.height;
        coords_.push_back(coord);
        RCLCPP_INFO(this->get_logger(), "[X: %d, Y: %d]", coord.x, coord.y);
    }

    uint32_t width_, height_;
    std::string status_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::list<planb::CoordInfo> coords_;
    rclcpp::Subscription<planb_ros2::msg::CoordInfo>::SharedPtr sub_coordinfo_;
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