#include <thread>
#include <mraa.hpp>
#include "planb/hardware_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

HardwareNode::HardwareNode(const rclcpp::NodeOptions &options)
    : Node("hardware", options), status_("OFF")
{
    std::string dev_path = this->declare_parameter("dev_path", "/dev/ttyS5");
    int baud = this->declare_parameter("baud", 115200);
    serial_init(dev_path, baud);

    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/hardware/status", 1);
    sub_cmd_ = this->create_subscription<planb_ros2::msg::Cmd>(
        "/planb/hardware/cmd",
        1,
        std::bind(&HardwareNode::cmd_callback, this, _1));
    sub_operate_ = this->create_subscription<planb_ros2::msg::Operate>(
        "/planb/hardware/operate",
        1,
        std::bind(&HardwareNode::operate_callback, this, _1));
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        1,
        std::bind(&HardwareNode::twist_callback, this, _1));

    sub_robot_ = this->create_subscription<planb_interfaces::msg::Robot>(
        "/planb/hardware/robot",
        1,
        std::bind(&HardwareNode::robot_callback, this, _1));
}

HardwareNode::~HardwareNode()
{
    uint8_t cmd[16] = {0xFF, 0xFF, 0xF0, 0xF0};
    uart_->write((char *)cmd, 16);
    uart_->close();
}

void HardwareNode::serial_init(std::string &dev_path, const int baud)
{
    uart_ = std::make_shared<mraa::Uart>(dev_path);
    if (uart_->setBaudRate(baud) != mraa::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Error setting parity on UART");
        return;
    }

    if (uart_->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Error setting parity on UART");
        return;
    }

    if (uart_->setFlowcontrol(false, false) != mraa::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Error setting flow control UART");
        return;
    }

    uint8_t cmd[16] = {0xFF, 0xFF, 0xF1, 0xF1};

    uart_->write((char *)cmd, 16);
}

void HardwareNode::publish_status(const std::string &status)
{
    std_msgs::msg::String msg;
    msg.data = status;
    status_ = status;
    pub_status_->publish(std::move(msg));
}

void HardwareNode::turn_on()
{
    publish_status("ON");
}

void HardwareNode::turn_off()
{
    publish_status("OFF");
}

void HardwareNode::cmd_callback(const planb_ros2::msg::Cmd &msg)
{
    switch (msg.cmd)
    {
    case planb::CMD_TURN_ON:
        turn_on();
        break;
    case planb::CMD_TURN_OFF:
        turn_off();
        break;
    default:
        break;
    }
}

void HardwareNode::twist_callback(const geometry_msgs::msg::Twist &msg)
{
    // angle
    if (msg.angular.z == 0)
        angle_ = 90;
    else
        angle_ += msg.angular.z;

    if (angle_ < 0)
        angle_ = 0;
    else if (angle_ > 180)
        angle_ = 180;

    // accel
    if (msg.linear.x == 0)
        accel_ = 0;
    else
        accel_ += msg.linear.x;

    if (accel_ < -100)
        accel_ = -100;
    else if (accel_ > 100)
        accel_ = 100;
}

void HardwareNode::operate_callback(const planb_ros2::msg::Operate &msg)
{
}

void HardwareNode::normal_mode(const planb_interfaces::msg::Robot &msg)
{
    // servos
    uint8_t angle = (uint8_t)(90 + msg.servo.angle);
    if (msg.servo.left_front > 0)
        control_data_[0] = angle;
    if (msg.servo.left_tail > 0)
        control_data_[1] = angle;
    if (msg.servo.right_front > 0)
        control_data_[2] = angle;
    if (msg.servo.right_tail > 0)
        control_data_[3] = angle;

    // motors
    uint8_t motors[6] = {
        msg.motor.left_front,
        msg.motor.left_middle,
        msg.motor.left_tail,
        msg.motor.right_front,
        msg.motor.right_middle,
        msg.motor.right_tail};

    uint8_t accel = (uint8_t)abs(msg.motor.accel);

    for (int i = 0; i < 6; i++)
    {
        if (motors[i] <= 0)
            continue;

        if (msg.motor.accel < 0)
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = accel;
        }
        else if (msg.motor.accel > 0)
        {
            control_data_[4 + 2 * i] = accel;
            control_data_[4 + 2 * i + 1] = 0;
        }
        else
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = 0;
        }
    }

    tx_data();
}

void HardwareNode::accel_circle(const int accel)
{
    // motors
    uint8_t _accel = (uint8_t)abs(accel);

    for (int i = 0; i < 3; i++)
    {
        if (accel < 0)
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = _accel;
        }
        else if (accel > 0)
        {
            control_data_[4 + 2 * i] = _accel;
            control_data_[4 + 2 * i + 1] = 0;
        }
        else
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = 0;
        }
    }

    for (int i = 3; i < 6; i++)
    {
        if (accel < 0)
        {
            control_data_[4 + 2 * i] = _accel;
            control_data_[4 + 2 * i + 1] = 0;
        }
        else if (accel > 0)
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = _accel;
        }
        else
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = 0;
        }
    }
}

void HardwareNode::circle_mode(const planb_interfaces::msg::Robot &msg)
{
    // servos
    uint8_t angle = (uint8_t)std::round(std::atan(d1 / d2) * 180 / PI);
    control_data_[0] = 90 + angle;
    control_data_[1] = 90 - angle;
    control_data_[2] = 90 - angle;
    control_data_[3] = 90 + angle;
    // motors
    if (msg.motor.accel > 0)
        accel_circle(100);
    else if (msg.motor.accel < 0)
        accel_circle(-100);
    tx_data();
    std::this_thread::sleep_for(30ms);
    accel_circle(msg.motor.accel);
    tx_data();
}

void HardwareNode::accel_arckerman(const int accel)
{
    // motors
    uint8_t _accel = (uint8_t)abs(accel);
    std::vector<uint8_t> accels;
    if (accel < 0)
        accels = calc_accel((float)angle_, _accel, true);
    else
        accels = calc_accel((float)angle_, _accel, false);

    for (int i = 0; i < 6; i++)
    {
        if (accel < 0)
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = accels[i];
        }
        else if (accel > 0)
        {
            control_data_[4 + 2 * i] = accels[i];
            control_data_[4 + 2 * i + 1] = 0;
        }
        else
        {
            control_data_[4 + 2 * i] = 0;
            control_data_[4 + 2 * i + 1] = 0;
        }
    }
}

void HardwareNode::arckerman_mode(const planb_interfaces::msg::Robot &msg)
{
    // servos
    std::vector<uint8_t> angles;
    uint8_t angle = (uint8_t)abs(msg.servo.angle);

    if (msg.servo.angle < 0)
        angles = calc_angles(angle, true);
    else
        angles = calc_angles(angle, false);

    control_data_[0] = angles[0];
    control_data_[1] = angles[1];
    control_data_[2] = angles[2];
    control_data_[3] = angles[3];

    // motors
    if (msg.motor.accel > 0)
        accel_arckerman(100);
    else if (msg.motor.accel < 0)
        accel_arckerman(-100);
    tx_data();
    std::this_thread::sleep_for(30ms);
    accel_arckerman(msg.motor.accel);
    tx_data();
}

void HardwareNode::robot_callback(const planb_interfaces::msg::Robot &msg)
{
    if (msg.mode == 0)
    {
        normal_mode(msg);
    }
    else if (msg.mode == 1)
    {
        circle_mode(msg);
    }
    else if (msg.mode == 2)
    {
        arckerman_mode(msg);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Uknown mode: %d!", msg.mode);
    }
}

std::vector<uint8_t> HardwareNode::calc_angles(float angle, bool direct_left)
{
    if (angle == 0.0)
        return {90, 90, 90, 90};

    float radius = std::round(d1 + d3 / std::tan(angle * PI / 180));
    std::vector<uint8_t> angles;
    float a1 = std::round(std::atan(d3 / (d1 + radius)) * 180 / PI);
    float a2 = std::round(std::atan(d2 / (d1 + radius)) * 180 / PI);
    float a3 = std::round(std::atan(d3 / (radius - d1)) * 180 / PI);
    float a4 = std::round(std::atan(d2 / (radius - d1)) * 180 / PI);
    if (direct_left)
        return {(uint8_t)(90.0 - a1), (uint8_t)(90.0 + a2), (uint8_t)(90.0 - a3), (uint8_t)(90.0 + a4)};
    else
        return {(uint8_t)(90.0 + a1), (uint8_t)(90.0 - a2), (uint8_t)(90.0 + a3), (uint8_t)(90.0 - a4)};
}

std::vector<uint8_t> HardwareNode::calc_accel(float angle, uint8_t accel, bool direct_left)
{
    if (angle == 0.0)
        return {accel, accel, accel, accel, accel, accel};

    float radius = std::round(d1 + d3 / std::tan(angle * PI / 180));
    std::vector<uint8_t> accels;
    uint8_t v1 = std::round(accel * std::sqrt(std::pow(d3, 2.0) + std::pow(d1 + radius, 2.0)) / (radius + d4));
    uint8_t v2 = accel;
    uint8_t v3 = std::round(accel * std::sqrt(std::pow(d2, 2.0) + std::pow(d1 + radius, 2.0)) / (radius + d4));
    uint8_t v4 = std::round(accel * std::sqrt(std::pow(d3, 2.0) + std::pow(radius - d1, 2.0)) / (radius + d4));
    uint8_t v5 = std::round(accel * (radius - d4) / (radius + d4));
    uint8_t v6 = std::round(accel * std::sqrt(std::pow(d2, 2.0) + std::pow(radius - d1, 2.0)) / (radius + d4));

    if (direct_left)
        accels = {v4, v5, v6, v1, v2, v3};
    else
        accels = {v1, v2, v3, v4, v5, v6};

    float _max = *std::max_element(accels.begin(), accels.end());
    if (_max > 100.0)
    {
        float rate = 100.0 / _max;
        for (int i = 0; i < 6; i++)
        {
            accels[i] = std::round(accels[i] * rate);
        }
    }

    return accels;
}

void HardwareNode::tx_data()
{
    uart_->write((char *)control_data_, 16);
}