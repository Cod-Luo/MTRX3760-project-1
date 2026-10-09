// velocity_publisher.cpp - Builds and publishes Twist or TwistStamped commands.

#include "turtlebot3_gazebo/velocity_publisher.hpp"

const std::string CVelocityPublisher::Topic = "cmd_vel";
const std::string CVelocityPublisher::FrameId = "base_link";
const int CVelocityPublisher::QueueDepth = 10;

// Create only the velocity publisher selected for this robot setup.
CVelocityPublisher::CVelocityPublisher(rclcpp::Node& aNode, bool aUseStamped)
    : mNode(aNode),
      mUseStamped(aUseStamped)
{
    if (mUseStamped)
    {
        mStampedPublisher =
            mNode.create_publisher<geometry_msgs::msg::TwistStamped>(
                Topic, QueueDepth);
    }
    else
    {
        mTwistPublisher =
            mNode.create_publisher<geometry_msgs::msg::Twist>(
                Topic, QueueDepth);
    }
}

// Convert a command into the selected ROS velocity message and send it.
void CVelocityPublisher::Publish(double aLinear, double aAngular)
{
    geometry_msgs::msg::Twist Velocity;
    Velocity.linear.x = aLinear;
    Velocity.angular.z = aAngular;

    if (mUseStamped)
    {
        geometry_msgs::msg::TwistStamped Stamped;
        Stamped.header.stamp = mNode.now();
        Stamped.header.frame_id = FrameId;
        Stamped.twist = Velocity;

        mStampedPublisher->publish(Stamped);
    }
    else
    {
        mTwistPublisher->publish(Velocity);
    }
}
