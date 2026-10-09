// main.cpp - Entry point: starts the right-wall-following ROS node.

#include "turtlebot3_gazebo/wall_follower_node.hpp"

#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Keep publishing available after the default context receives Ctrl+C.
    rclcpp::InitOptions ContextOptions;
    ContextOptions.shutdown_on_signal = false;
    ContextOptions.auto_initialize_logging(false);  // Default context already owns logging.
    auto ControlContext = std::make_shared<rclcpp::Context>();
    ControlContext->init(argc, argv, ContextOptions);
    rclcpp::NodeOptions NodeOptions;
    NodeOptions.context(ControlContext);
    auto Node = std::make_shared<CWallFollowerNode>(NodeOptions);
    Node->Run();

    ControlContext->shutdown("Controller stopped");
    rclcpp::shutdown();

    return 0;
}
