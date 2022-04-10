#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    Eigen::Vector3d v1(0, 1, 1);
    Eigen::Vector3d v2(1, 1, 2);
    Eigen::Vector3d v3(2, 1, 3);
    std::cout << v1.cross(v2) << std::endl;
    std::cout << v2.cross(v1) << std::endl;
    rclcpp::shutdown();
    return 0;
}