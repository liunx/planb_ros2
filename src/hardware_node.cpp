#include "planb/hardware_node.hpp"
#include "planb/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

HardwareNode::HardwareNode(const rclcpp::NodeOptions &options)
    : Node("hardware", options),
      status_("OFF"), power_(1), angle_(6)
{
    cmd_[0] = 0x06;
    cmd_[1] = 0x01;
    cmd_[2] = 0xFF;
    cmd_[3] = 0xFF;
    dev_ = this->declare_parameter("dev", "/dev/ttyS5");
    baud_ = this->declare_parameter("baud", 9600);

    pub_status_  = this->create_publisher<std_msgs::msg::String>("/planb/hardware/status", 1);
    sub_cmd_ = this->create_subscription<planb_ros2::msg::Cmd>(
        "/planb/hardware/cmd",
        1,
        std::bind(&HardwareNode::cmd_callback, this, _1));
    sub_operate_ = this->create_subscription<planb_ros2::msg::Operate>(
        "/planb/hardware/operate",
        1,
        std::bind(&HardwareNode::operate_callback, this, _1));
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

void HardwareNode::publish_status(const std::string &status)
{
    std_msgs::msg::String msg;
    msg.data = status;
    status_ = status;
    pub_status_->publish(std::move(msg));
}

void HardwareNode::turn_on()
{
    std::string status;
    if (serial_init(dev_.c_str(), baud_) == 0)
    {
        status = "ON";
    }
    else
    {
        status = "Fault";
    }
    publish_status(status);
}

void HardwareNode::turn_off()
{
    close(serial_fd_);
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

void HardwareNode::operate_callback(const planb_ros2::msg::Operate &msg)
{
    if (msg.steering <= MIN_ANGLE_INDEX)
        cmd_[0] = MIN_ANGLE_INDEX;
    else if (msg.steering > MAX_ANGLE_INDEX)
        cmd_[0] = MAX_ANGLE_INDEX;
    else
        cmd_[0] = msg.steering;

    if (msg.accel <= MIN_POWER_INDEX)
        cmd_[0] = MIN_POWER_INDEX;
    else if (msg.accel > MAX_POWER_INDEX)
        cmd_[0] = MAX_POWER_INDEX;
    else
        cmd_[0] = msg.accel;

    tx_data();
}
