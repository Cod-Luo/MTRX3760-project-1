// velocity_publisher.hpp - Sends drive commands to the robot's wheels on cmd_vel.

#ifndef TURTLEBOT3_GAZEBO_VELOCITY_PUBLISHER_HPP
#define TURTLEBOT3_GAZEBO_VELOCITY_PUBLISHER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

// Hides which velocity message type the robot expects. ROS 2 Jazzy TurtleBots
// listen for TwistStamped by default; older setups use plain Twist.
class CVelocityPublisher
{
    public:
        // Creates a publisher of the selected message type on aNode.
        CVelocityPublisher(rclcpp::Node& aNode, bool aUseStamped);

        // Sends one command: metres/second forward, radians/second turning left.
        void Publish(double aLinear, double aAngular);

    private:
        rclcpp::Node& mNode;  // Knows the node for timestamps; does not own it.

        const bool mUseStamped;  // Must match the receiver's cmd_vel message type.

        // Only the publisher matching mUseStamped is created.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mTwistPublisher;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mStampedPublisher;

        static const std::string Topic;
        static const std::string FrameId;  // Robot body frame the velocity applies to.
        static const int QueueDepth;
};

#endif
