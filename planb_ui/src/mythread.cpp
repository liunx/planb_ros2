#include <thread>
#include "mythread.h"

using namespace std::chrono_literals;

MyThread::MyThread(QObject *parent, std::shared_ptr<rclcpp::Node> node)
    : QThread(parent)
    , node_(node)
{
    pub_robot_ = node->create_publisher<planb_interfaces::msg::Robot>("/planb/hardware/robot", 1);
}

void MyThread::run()
{
    RCLCPP_INFO(node_->get_logger(), "Running...");
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(1s);
    }
}

void MyThread::slot_robot(Robot robot)
{
    planb_interfaces::msg::Robot msg;

    msg.mode = robot.mode;
    msg.servo.angle = robot.servo.angle;
    msg.motor.accel = robot.motor.accel;
    msg.servo.left_front = robot.servo.left_front;
    msg.servo.left_tail = robot.servo.left_tail;
    msg.servo.right_front = robot.servo.right_front;
    msg.servo.right_tail = robot.servo.right_tail;
    msg.motor.left_front = robot.motor.left_front;
    msg.motor.left_middle = robot.motor.left_middle;
    msg.motor.left_tail = robot.motor.left_tail;
    msg.motor.right_front = robot.motor.right_front;
    msg.motor.right_middle = robot.motor.right_middle;
    msg.motor.right_tail = robot.motor.right_tail;

    pub_robot_->publish(std::move(msg));
}
