#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <planb_interfaces/msg/robot.hpp>
#include <QThread>
#include "common.h"

class MyThread : public QThread
{
    Q_OBJECT
public:
    MyThread(QObject *parent, std::shared_ptr<rclcpp::Node> node);

signals:

private slots:
    void slot_robot(Robot robot);

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<planb_interfaces::msg::Robot>::SharedPtr pub_robot_;
};

#endif // MYTHREAD_H
