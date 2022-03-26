#include "planb/hardware_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

HardwareNode::HardwareNode(const rclcpp::NodeOptions &options)
    : Node("hardware", options),
      status_("Idle"), power_(1), angle_(6)
{
    cmd_[0] = 0x06;
    cmd_[1] = 0x01;
    cmd_[2] = 0xFF;
    cmd_[3] = 0xFF;
    dev_ = this->declare_parameter("dev", "/dev/ttyS5");
    baud_ = this->declare_parameter("baud", 9600);

    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/hardware/status", 1);
    sub_input_ = this->create_subscription<std_msgs::msg::String>(
        "/planb/hardware/input",
        1,
        std::bind(&HardwareNode::input_callback, this, _1));
}

HardwareNode::~HardwareNode()
{
    close(serial_fd_);
}

int HardwareNode::serial_init(const char *dev, const int baud)
{
    struct termios options;
    speed_t myBaud;
    int status;

    switch (baud) {
    case 9600:
        myBaud = B9600;
        break;
    case 115200:
        myBaud = B115200;
        break;
    default:
        return -1;
    }

    if ((serial_fd_ = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
        return -1;

    fcntl(serial_fd_, F_SETFL, O_RDWR);
    // Get and modify current options:
    tcgetattr(serial_fd_, &options);
    cfmakeraw(&options);
    cfsetispeed(&options, myBaud);
    cfsetospeed(&options, myBaud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 100; // Ten seconds (100 deciseconds)
    tcsetattr(serial_fd_, TCSANOW, &options);
    ioctl(serial_fd_, TIOCMGET, &status);
    status |= TIOCM_DTR;
    status |= TIOCM_RTS;
    ioctl(serial_fd_, TIOCMSET, &status);
    usleep(10000); // 10mS

    return 0;
}

void HardwareNode::tx_data()
{
    write(serial_fd_, cmd_, sizeof(cmd_));
}

void HardwareNode::forward()
{
    if (cmd_[1] <= MAX_POWER_INDEX)
    {
        cmd_[1] += 1;
        tx_data();
    }
}

void HardwareNode::backward()
{
    if (cmd_[1] > MIN_POWER_INDEX)
    {
        cmd_[1] -= 1;
        tx_data();
    }
}

void HardwareNode::turn_left()
{
    if (cmd_[0] > MIN_ANGLE_INDEX)
    {
        cmd_[0] -= 1;
        tx_data();
    }
}

void HardwareNode::turn_right()
{
    if (cmd_[0] <= MAX_ANGLE_INDEX)
    {
        cmd_[0] += 1;
        tx_data();
    }
}

void HardwareNode::reset()
{
    if (angle_ != 6 || power_ != 1)
    {
        cmd_[0] = 6;
        cmd_[1] = 1;
        tx_data();
        angle_ = 6;
        power_ = 1;
    }
}

void HardwareNode::stop()
{
    if (power_ != 1)
    {
        cmd_[1] = 1;
        tx_data();
        power_ = 1;
    }
}

void HardwareNode::publish_status()
{
    std_msgs::msg::String msg;
    msg.data = status_;
    pub_status_->publish(std::move(msg));
}

void HardwareNode::input_callback(const std_msgs::msg::String &msg)
{
    if (msg.data == "Init") {
        if (serial_init(dev_.c_str(), baud_) == 0) {
            status_ = "Ready";
        }
        else {
            status_ = "Fault";
        }
        publish_status();
        return;
    }

    if (msg.data == "Reset") {
        reset();
    }
    else if (msg.data == "Stop") {
        stop();
    }
    else if (msg.data == "Forward") {
        forward();
    }
    else if (msg.data == "Backward") {
        backward();
    }
    else if (msg.data == "Left") {
        turn_left();
    }
    else if (msg.data == "Right") {
        turn_right();
    }
}
