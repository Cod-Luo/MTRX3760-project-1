// velocity_publisher.cpp - Builds and publishes TwistStamped commands.

#include "turtlebot3_gazebo/velocity_publisher.hpp"

#include <chrono>

const std::string CVelocityPublisher::Topic = "cmd_vel";
const std::string CVelocityPublisher::FrameId = "base_link";
const int CVelocityPublisher::QueueDepth = 10;
const int CVelocityPublisher::StopWaitMilliseconds = 500;

// Create the timestamped velocity publisher used by the simulator and robot.
CVelocityPublisher::CVelocityPublisher(rclcpp::Node& aNode)
    : mNode(aNode)
{
    mStampedPublisher =
        mNode.create_publisher<geometry_msgs::msg::TwistStamped>(
            Topic, QueueDepth);
}

// Convert a command into a timestamped ROS velocity message and send it.
void CVelocityPublisher::Publish(double aLinear, double aAngular)
{
    geometry_msgs::msg::TwistStamped Stamped;
    Stamped.header.stamp = mNode.now();
    Stamped.header.frame_id = FrameId;
    Stamped.twist.linear.x = aLinear;
    Stamped.twist.angular.z = aAngular;

    mStampedPublisher->publish(Stamped);
}

// Acknowledgement is a delivery check, not proof that the physical wheels stopped.
void CVelocityPublisher::Stop()
{
    Publish(0.0, 0.0);
    if (!mStampedPublisher->wait_for_all_acked(
        std::chrono::milliseconds(StopWaitMilliseconds)))
    {
        RCLCPP_WARN(mNode.get_logger(), "Timed out waiting for stop acknowledgement");
    }
}
