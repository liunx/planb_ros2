#include <iostream>
#include <thread>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <mraa.hpp>

using namespace std::chrono_literals;

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

void demo03()
{
    const char *dev_path = "/dev/ttyUSB0";
    mraa::Uart uart = mraa::Uart(dev_path);

    if (uart.setBaudRate(115200) != mraa::SUCCESS)
    {
        std::cerr << "Error setting parity on UART" << std::endl;
    }

    if (uart.setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS)
    {
        std::cerr << "Error setting parity on UART" << std::endl;
    }

    if (uart.setFlowcontrol(false, false) != mraa::SUCCESS)
    {
        std::cerr << "Error setting flow control UART" << std::endl;
    }

    uint8_t cmd_start[16] = {0xFF, 0xFF, 0xF1, 0xF0};
    uint8_t cmd_stop[16] = {0xFF, 0xFF, 0xF0, 0xF0};
    uint8_t buf[2] = {};

    uart.write((char *)cmd_start, 16);

    for (int i = 0; i < 2; i++)
    {
        if (uart.dataAvailable(10))
        {
            uart.read((char *)buf, 2);
            printf("[%x:%x]\n", buf[0], buf[1]);
        }
    }
    std::this_thread::sleep_for(1s);
    uart.write((char *)cmd_stop, 16);

    uart.close();

}

#define PI 3.141592654

float d1 = 20.0;
float d2 = 30.0;
float d3 = 30.0;
float d4 = 20.0;

std::vector<uint8_t> calc_steering_angles(float angle, bool direct_left)
{
    float radius = std::round(d1 + d3 / std::tan(angle * PI / 180));
    std::vector<uint8_t> angles;
    if (radius == 0.0)
        return {0, 0, 0, 0};
    float a1 = std::round(std::atan(d3 / (d1 + radius)) * 180 / PI);
    float a2 = std::round(std::atan(d2 / (d1 + radius)) * 180 / PI);
    float a3 = std::round(std::atan(d3 / (radius - d1)) * 180 / PI);
    float a4 = std::round(std::atan(d2 / (radius - d1)) * 180 / PI);
    if (direct_left)
        return {(uint8_t)(90 - a1), (uint8_t)(90 + a2), (uint8_t)(90 - a3), (uint8_t)(90 + a4)};
    else
        return {(uint8_t)(90 + a1), (uint8_t)(90 - a2), (uint8_t)(90 + a3), (uint8_t)(90 - a4)};
}

void demo04()
{
    for (int i = 0; i <= 90; i++)
    {
        auto data = calc_steering_angles(i, true);
        printf("{\n");
        for (auto a : data)
        {
            printf("%d,", a);
        }
        printf("}\n");
    }
}

int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    demo04();
    rclcpp::shutdown();
    return 0;
}