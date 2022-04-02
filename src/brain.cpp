#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "planb/behaviortree_action.hpp"

#define DEFAULT_BT_XML "./demo.xml"
using namespace std::chrono_literals;

void bind_actions(BT::BehaviorTreeFactory &factory, rclcpp::Node::SharedPtr &node)
{
    // aruco
    {
        auto builder = [&node](const std::string &name,
                               const BT::NodeConfiguration &config)
        {
            return std::make_unique<ArucoAction>(name, config, node);
        };
        factory.registerBuilder<ArucoAction>("GetArucoStatus", builder);
        factory.registerBuilder<ArucoAction>("GetArucoID", builder);
        factory.registerBuilder<ArucoAction>("TurnArucoOn", builder);
        factory.registerBuilder<ArucoAction>("TurnArucoOff", builder);
    }
    // tracker
    {
        auto builder = [&node](const std::string &name,
                               const BT::NodeConfiguration &config)
        {
            return std::make_unique<TrackerAction>(name, config, node);
        };
        factory.registerBuilder<TrackerAction>("GetTrackerStatus", builder);
        factory.registerBuilder<TrackerAction>("TrackArucoID", builder);
        factory.registerBuilder<TrackerAction>("TurnTrackerOn", builder);
        factory.registerBuilder<TrackerAction>("TurnTrackerOff", builder);
    }
    // camera
    {
        auto builder = [&node](const std::string &name,
                               const BT::NodeConfiguration &config)
        {
            return std::make_unique<CameraAction>(name, config, node);
        };
        factory.registerBuilder<CameraAction>("GetCameraStatus", builder);
        factory.registerBuilder<CameraAction>("TurnCameraOn", builder);
        factory.registerBuilder<CameraAction>("TurnCameraOff", builder);
    }
    // hardware 
    {
        auto builder = [&node](const std::string &name,
                               const BT::NodeConfiguration &config)
        {
            return std::make_unique<HardwareAction>(name, config, node);
        };
        factory.registerBuilder<HardwareAction>("GetHardwareStatus", builder);
        factory.registerBuilder<HardwareAction>("TurnHardwareOn", builder);
        factory.registerBuilder<HardwareAction>("TurnHardwareOff", builder);
    }
    // wait
    {
        auto builder = [&node](const std::string &name,
                               const BT::NodeConfiguration &config)
        {
            return std::make_unique<WaitAction>(name, config, node);
        };
        factory.registerBuilder<WaitAction>("Wait", builder);
    }
    // logic
    factory.registerNodeType<LogicAction>("CheckEqual");
    factory.registerNodeType<LogicAction>("FindID");
    // logger
    factory.registerNodeType<LogAction>("Info");
    factory.registerNodeType<LogAction>("Warn");
    factory.registerNodeType<LogAction>("Error");
}

int main(int argc, char **argv)
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Init ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("brain");
    node->declare_parameter("bt_xml", rclcpp::ParameterValue(std::string(DEFAULT_BT_XML)));
    std::string bt_xml;
    node->get_parameter("bt_xml", bt_xml);
    // Init behavior tree
    BT::BehaviorTreeFactory factory;
    bind_actions(factory, node);
    auto tree = factory.createTreeFromFile(bt_xml);
    //BT::StdCoutLogger logger_cout(tree);
    BT::printTreeRecursively(tree.rootNode());
    while (rclcpp::ok())
    {
        tree.tickRoot();
        std::this_thread::sleep_for(30ms);
    }
    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}
