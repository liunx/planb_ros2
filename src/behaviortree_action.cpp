#include "planb/behaviortree_action.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;

BehaviorTreeAction::BehaviorTreeAction()
    : hardware_status_("Unkown"),
      camera_status_("Unkown"),
      aruco_status_("Unkown"),
      tracker_status_("Unkown"),
      flag_follow_stopped_(false)
{
    auto do_nothing = [](std_msgs::msg::String::UniquePtr) { assert(false); };
    auto do_nothing2 = [](planb_ros2::msg::Aruco::UniquePtr) { assert(false); };
    auto do_nothing3 = [](planb_ros2::msg::Box::UniquePtr) { assert(false); };
    node_ = rclcpp::Node::make_shared("behaviortree_action");
    // hardware
    pub_hardware_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/hardware/cmd", 1);
    sub_hardware_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/hardware/status", 1, do_nothing);
    // camera
    sub_camera_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/camera/status", 1, do_nothing);
    pub_camera_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/camera/cmd", 1);
    // aruco
    sub_aruco_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/aruco/status", 1, do_nothing);
    sub_aruco_markers_ = node_->create_subscription<planb_ros2::msg::Aruco>("/planb/aruco/markers", 1, do_nothing2);
    pub_aruco_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/aruco/cmd", 1);
    // tracker
    sub_tracker_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/tracker/status", 1, do_nothing);
    sub_tracker_data_ = node_->create_subscription<std_msgs::msg::String>("/planb/tracker/data", 1, do_nothing3);
    pub_tracker_input_ = node_->create_publisher<planb_ros2::msg::Box>("/planb/tracker/input", 1);
}

BehaviorTreeAction::~BehaviorTreeAction()
{
}

// camera
BT::NodeStatus BehaviorTreeAction::check_camera_idle()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_camera_status_->take(msg, msg_info)) {
        camera_status_ = msg.data;
    }
    if (camera_status_ == "Idle")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_camera_idle()
{
    std_msgs::msg::String msg;
    msg.data = "Idle";
    pub_camera_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::check_camera_running()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_camera_status_->take(msg, msg_info)) {
        camera_status_ = msg.data;
    }
    if (camera_status_ == "Running")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_camera_running()
{
    std_msgs::msg::String msg;
    msg.data = "Running";
    pub_camera_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

// aruco
BT::NodeStatus BehaviorTreeAction::check_aruco_idle()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_aruco_status_->take(msg, msg_info)) {
        aruco_status_ = msg.data;
    }
    if (aruco_status_ == "Idle")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_aruco_idle()
{
    std_msgs::msg::String msg;
    msg.data = "Idle";
    pub_aruco_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::check_aruco_running()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_aruco_status_->take(msg, msg_info)) {
        aruco_status_ = msg.data;
    }
    if (aruco_status_ == "Running")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::set_aruco_running()
{
    std_msgs::msg::String msg;
    msg.data = "Running";
    pub_aruco_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::detect_aruco_markers()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    rclcpp::MessageInfo msg_info;
    if (sub_aruco_markers_->take(aruco_msg_, msg_info)) {
        if (aruco_msg_.data.size() > 0)
            stat = BT::NodeStatus::SUCCESS;
    }
    return stat;
}

// tracker
BT::NodeStatus BehaviorTreeAction::check_tracker_idle()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_tracker_status_->take(msg, msg_info)) {
        tracker_status_ = msg.data;
    }
    if (tracker_status_ == "Idle")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::check_tracker_running()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_tracker_status_->take(msg, msg_info)) {
        tracker_status_ = msg.data;
    }
    if (tracker_status_ == "Running")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

BT::NodeStatus BehaviorTreeAction::tracking_target()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    for (auto marker : aruco_msg_.data)
    {
        if (marker.id == 0) {
            planb_ros2::msg::Box msg;
            msg.id = marker.id;
            msg.x = marker.x;
            msg.y = marker.y;
            msg.width = marker.width;
            msg.height = marker.height;
            pub_tracker_input_->publish(std::move(msg));
            stat = BT::NodeStatus::SUCCESS;
            break;
        }
    }
    aruco_msg_.data.clear();
    return stat;
}

// hardware
BT::NodeStatus BehaviorTreeAction::init_hardware()
{
    std_msgs::msg::String msg;
    msg.data = "Init";
    pub_hardware_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::reset_hardware()
{
    std_msgs::msg::String msg;
    msg.data = "Reset";
    pub_hardware_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::check_hardware_ready()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_hardware_status_->take(msg, msg_info)) {
        hardware_status_ = msg.data;
    }
    if (hardware_status_ == "Ready")
        stat = BT::NodeStatus::SUCCESS;
    else
        stat = BT::NodeStatus::FAILURE;
    return stat;
}

// others
BT::NodeStatus BehaviorTreeAction::wait()
{
    rclcpp::WaitSet wait_set({{{sub_camera_status_},
                               {sub_aruco_markers_},
                               {sub_aruco_status_},
                               {sub_hardware_status_},
                               {sub_tracker_status_}}},
                             {}, {});
    auto wait_result = wait_set.wait(300ms);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BehaviorTreeAction::follow_target()
{
    flag_follow_stopped_ = false;
    uint32_t padding = 30;
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    planb_ros2::msg::Box msg;
    rclcpp::MessageInfo msg_info;
    std_msgs::msg::String pub_msg;
    if (sub_tracker_data_->take(msg, msg_info)) {
        stat = BT::NodeStatus::SUCCESS;
        auto bbox_area = uint32_t(msg.width * msg.height);
        auto frame_area = uint32_t(msg.frame_width * msg.frame_height);
        auto center_x = uint32_t(msg.frame_width / 2);
        // too close, won't moving
        if ((bbox_area / frame_area) > 0.5f)
        {
            pub_msg.data = "Stop";
        }
        else if (center_x > (msg.x + msg.width) + padding)
            pub_msg.data = "Left";
        else if (center_x < msg.x - padding)
            pub_msg.data = "Right";
        else
            pub_msg.data = "Center";
        pub_hardware_cmd_->publish(std::move(pub_msg));
    }
    return stat;
}

BT::NodeStatus BehaviorTreeAction::stop_follow()
{
    BT::NodeStatus stat = BT::NodeStatus::SUCCESS;
    if (flag_follow_stopped_)
        return stat;
    std_msgs::msg::String msg;
    msg.data = "Stop";
    pub_hardware_cmd_->publish(std::move(msg));
    return stat;
}

// class ArucoAction  {
ArucoAction::ArucoAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config),
      name_(name),
      status_("Unknown")
{
    auto node_ = rclcpp::Node::make_shared("aruco_action");
    auto do_nothing = [](std_msgs::msg::String::UniquePtr)
    { assert(false); };
    auto do_nothing2 = [](planb_ros2::msg::Aruco::UniquePtr)
    { assert(false); };
    sub_status_ = node_->create_subscription<std_msgs::msg::String>("/planb/aruco/status", 1, do_nothing);
    sub_markers_ = node_->create_subscription<planb_ros2::msg::Aruco>("/planb/aruco/markers", 1, do_nothing2);
    pub_cmd_ = node_->create_publisher<std_msgs::msg::String>("/planb/aruco/cmd", 1);
}

BT::NodeStatus ArucoAction::get_status()
{
    BT::NodeStatus stat = BT::NodeStatus::SUCCESS;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    if (sub_status_->take(msg, msg_info))
    {
        stat = BT::NodeStatus::SUCCESS;
        status_ = msg.data;
    }
    setOutput("status", status_);
    return stat;
}

BT::NodeStatus ArucoAction::turn_on()
{
    std_msgs::msg::String msg;
    msg.data = "Running";
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ArucoAction::turn_off()
{
    std_msgs::msg::String msg;
    msg.data = "Idle";
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ArucoAction::get_markers()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    rclcpp::MessageInfo msg_info;
    planb_ros2::msg::Aruco msg_markers;
    if (sub_markers_->take(msg_markers, msg_info)) {
        std::string ids = "";
        for (auto marker : msg_markers.data)
        {
            ids += planb::string_format("%d,", marker.id);
        }
        setOutput("ids", ids);
        stat = BT::NodeStatus::SUCCESS;
    }
    return stat;
}

BT::NodeStatus ArucoAction::tick()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    if (name_ == "GetArucoStatus")
        stat = get_status();
    else if (name_ == "TurnArucoOn")
        stat = turn_on();
    else if (name_ == "TurnArucoOff")
        stat = turn_off();
    else if (name_ == "GetArucoMarkers")
        stat = get_markers();
    else
        throw BT::RuntimeError("Unsupported action:", name_);
    return stat;
}

// } class ArucoAction