// velocity_publisher.hpp - Sends drive commands to the robot's wheels on cmd_vel.

#ifndef TURTLEBOT3_GAZEBO_VELOCITY_PUBLISHER_HPP
#define TURTLEBOT3_GAZEBO_VELOCITY_PUBLISHER_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

// Publishes timestamped velocity commands for the simulator and physical robot.
class CVelocityPublisher
{
    public:
        // Creates a TwistStamped publisher on aNode.
        explicit CVelocityPublisher(rclcpp::Node& aNode);

        // Sends one command: metres/second forward, radians/second turning left.
        void Publish(double aLinear, double aAngular);

    private:
        rclcpp::Node& mNode;  // Knows the node for timestamps; does not own it.

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mStampedPublisher;

        static const std::string Topic;
        static const std::string FrameId;  // Robot body frame the velocity applies to.
        static const int QueueDepth;
};

#endif
