#include "planb/behaviortree_action.hpp"
#include "planb/common.hpp"
#include <thread>

using namespace std::chrono_literals;

// class ArucoAction  {
ArucoAction::ArucoAction(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config),
      name_(name),
      status_("UNKNOWN"),
      node_(node)
{
    pub_cmd_ = node_->create_publisher<planb_ros2::msg::Cmd>("/planb/aruco/cmd", 1);
    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/planb/aruco/status",
        1,
        [](std_msgs::msg::String::UniquePtr)
        { assert(false); });

    sub_data_ = node_->create_subscription<planb_ros2::msg::ArucoMarker>(
        "/planb/aruco/data",
        1,
        [](planb_ros2::msg::ArucoMarker::UniquePtr)
        { assert(false); });
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
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_ON;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ArucoAction::turn_off()
{
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_OFF;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ArucoAction::get_data()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    rclcpp::MessageInfo msg_info;
    planb_ros2::msg::ArucoMarker msg_data;
    if (sub_data_->take(msg_data, msg_info)) {
        std::string ids = "";
        for (auto marker : msg_data.bboxes)
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
    else if (name_ == "GetArucoID")
        stat = get_data();
    else
        throw BT::RuntimeError("Unsupported action:", name_);
    return stat;
}

// } class ArucoAction

// class TrackerAction  {
TrackerAction::TrackerAction(const std::string &name,
                             const BT::NodeConfiguration &config,
                             rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config),
      name_(name),
      status_("UNKNOWN"),
      node_(node)
{
    pub_cmd_ = node_->create_publisher<planb_ros2::msg::Cmd>("/planb/tracker/cmd", 1);
    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/planb/tracker/status",
        1,
        [](std_msgs::msg::String::UniquePtr)
        { assert(false); });
}

BT::NodeStatus TrackerAction::get_status()
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

BT::NodeStatus TrackerAction::turn_on()
{
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_ON;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TrackerAction::turn_off()
{
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_OFF;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TrackerAction::track_aruco()
{
    auto res = getInput<int>("id");
    if (!res) {
        throw BT::RuntimeError("error reading port [id]:", res.error());
        return BT::NodeStatus::FAILURE;
    }
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TRACK_ARUCO;
    msg.id = res.value();
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TrackerAction::tick()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    if (name_ == "GetTrackerStatus")
        stat = get_status();
    else if (name_ == "TurnTrackerOn")
        stat = turn_on();
    else if (name_ == "TurnTrackerOff")
        stat = turn_off();
    else if (name_ == "TrackArucoID")
        stat = track_aruco();
    else
        throw BT::RuntimeError("Unsupported action:", name_);
    return stat;
}

// } class TrackerAction

// class CameraAction  {
CameraAction::CameraAction(const std::string &name,
                           const BT::NodeConfiguration &config,
                           rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config),
      name_(name),
      status_("UNKNOWN"),
      node_(node)
{
    pub_cmd_ = node_->create_publisher<planb_ros2::msg::Cmd>("/planb/camera/cmd", 1);
    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/planb/camera/status",
        1,
        [](std_msgs::msg::String::UniquePtr)
        { assert(false); });
}

BT::NodeStatus CameraAction::get_status()
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

BT::NodeStatus CameraAction::turn_on()
{
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_ON;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CameraAction::turn_off()
{
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_OFF;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CameraAction::tick()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    if (name_ == "GetCameraStatus")
        stat = get_status();
    else if (name_ == "TurnCameraOn")
        stat = turn_on();
    else if (name_ == "TurnCameraOff")
        stat = turn_off();
    else
        throw BT::RuntimeError("Unsupported action:", name_);
    return stat;
}

// } class CameraAction

// class HardwareAction  {
HardwareAction::HardwareAction(const std::string &name,
                               const BT::NodeConfiguration &config,
                               rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config),
      name_(name),
      status_("UNKNOWN"),
      node_(node)
{
    pub_cmd_ = node_->create_publisher<planb_ros2::msg::Cmd>("/planb/hardware/cmd", 1);
    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/planb/hardware/status",
        1,
        [](std_msgs::msg::String::UniquePtr)
        { assert(false); });
}

BT::NodeStatus HardwareAction::get_status()
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

BT::NodeStatus HardwareAction::turn_on()
{
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_ON;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus HardwareAction::turn_off()
{
    planb_ros2::msg::Cmd msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.cmd = planb::CMD_TURN_OFF;
    pub_cmd_->publish(std::move(msg));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus HardwareAction::tick()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    if (name_ == "GetHardwareStatus")
        stat = get_status();
    else if (name_ == "TurnHardwareOn")
        stat = turn_on();
    else if (name_ == "TurnHardwareOff")
        stat = turn_off();
    else
        throw BT::RuntimeError("Unsupported action:", name_);
    return stat;
}

// } class HardwareAction

// class WaitAction  {
WaitAction::WaitAction(const std::string &name,
                       const BT::NodeConfiguration &config,
                       rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config),
    node_(node)
{
}

BT::NodeStatus WaitAction::tick()
{
    auto res = getInput<std::uint32_t>("msec");
    if (!res) {
        throw BT::RuntimeError("error reading port [msec]:", res.error());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(res.value()));
    return BT::NodeStatus::SUCCESS;
}

// } class WaitAction

// class LogicAction  {
LogicAction::LogicAction(const std::string &name,
                         const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config),
      name_(name)
{
}

BT::NodeStatus LogicAction::check_equal()
{
    auto res1 = getInput<std::string>("value1");
    if (!res1) {
        throw BT::RuntimeError("error reading port [value1]:", res1.error());
    }
    auto res2 = getInput<std::string>("value2");
    if (!res2) {
        throw BT::RuntimeError("error reading port [value2]:", res2.error());
    }
    if (res1.value() == res2.value())
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LogicAction::find_id()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    auto res1 = getInput<std::string>("ids");
    if (!res1) {
        throw BT::RuntimeError("error reading port [ids]:", res1.error());
    }
    auto res2 = getInput<std::string>("id");
    if (!res2) {
        throw BT::RuntimeError("error reading port [id]:", res2.error());
    }
    auto _ids = BT::splitString(res1.value(), ',');
    if (_ids.size() > 0)
    {
        for (auto _id : _ids)
        {
            if (_id == res2.value())
            {
                stat = BT::NodeStatus::SUCCESS;
                break;
            }
        }
    }
    return stat;
}

BT::NodeStatus LogicAction::tick()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    if (name_ == "CheckEqual")
        stat = check_equal();
    else if (name_ == "FindID")
        stat = find_id();
    else
        throw BT::RuntimeError("Unsupported action:", name_);
    return stat;
}

// } class LogicAction

// class LogAction  {
LogAction::LogAction(const std::string &name,
                     const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config),
      name_(name)
{
}

BT::NodeStatus LogAction::info()
{
    auto res = getInput<std::string>("text");
    if (!res) {
        throw BT::RuntimeError("error reading port [text]:", res.error());
    }
    std::cout << "[INFO] " << res.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LogAction::warn()
{
    auto res = getInput<std::string>("text");
    if (!res) {
        throw BT::RuntimeError("error reading port [text]:", res.error());
    }
    std::cout << "[WARN] " << res.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LogAction::error()
{
    auto res = getInput<std::string>("text");
    if (!res) {
        throw BT::RuntimeError("error reading port [text]:", res.error());
    }
    std::cout << "[ERROR] " << res.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LogAction::tick()
{
    BT::NodeStatus stat = BT::NodeStatus::FAILURE;
    if (name_ == "Info")
        stat = info();
    else if (name_ == "Warn")
        stat = warn();
    else if (name_ == "Error")
        stat = error();
    else
        throw BT::RuntimeError("Unsupported action:", name_);
    return stat;
}

// } class LogAction