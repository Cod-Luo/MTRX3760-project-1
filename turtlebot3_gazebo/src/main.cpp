// main.cpp - Entry point: starts the right-wall-following ROS node.

#include "turtlebot3_gazebo/wall_follower_node.hpp"

#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CWallFollowerNode>());
    rclcpp::shutdown();

    return 0;
}
