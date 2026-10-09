// main.cpp - Entry point: starts the right-wall-following ROS node.

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turtlebot3Drive>());
    rclcpp::shutdown();

    return 0;
}
