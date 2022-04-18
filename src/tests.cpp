#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

void demo01()
{
    Eigen::Vector3d v1(0, 1, 1);
    Eigen::Vector3d v2(1, 1, 2);
    Eigen::Vector3d v3(2, 1, 3);
    std::cout << v1.cross(v2) << std::endl;
    std::cout << v2.cross(v1) << std::endl;
}

void demo02()
{
    Eigen::Matrix2f A;
    Eigen::Vector2f b;
    A << 1, 1, 2, 1;
    b << 4, 5;
    std::cout << "Here is the matrix A:\n"
              << A << std::endl;
    std::cout << "Here is the vector b:\n"
              << b << std::endl;
    Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);
    std::cout << "The solution is:\n"
              << x << std::endl;
}

int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    demo01();
    rclcpp::shutdown();
    return 0;
}