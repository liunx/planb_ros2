#include <rclcpp/rclcpp.hpp>
#include "widget.h"
#include "mythread.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("ui");

    QApplication app(argc, argv);
    Widget w(nullptr);
    MyThread mthread(nullptr, node);
    w.connect(&w,
              SIGNAL(signal_robot(Robot)),
              &mthread,
              SLOT(slot_robot(Robot)));

    mthread.connect(&mthread,
              SIGNAL(signal_close()),
              &w,
              SLOT(on_close()));

    mthread.start();
    w.show();
    app.exec();

    rclcpp::shutdown();
    return 0;
}
