// path_recorder.cpp - Odometry sampling and path publishing for RViz.

#include "turtlebot3_gazebo/path_recorder.hpp"

#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>

const double CPathRecorder::SamplePeriodSeconds = 0.2;  // Up to 5 poses per second.
const std::string CPathRecorder::OdometryTopic = "odom";
const std::string CPathRecorder::PathTopic = "wall_follower/path";

// Connect to the odometry input and the path output.
CPathRecorder::CPathRecorder(rclcpp::Node& aNode)
    : mNode(aNode),
      mLastSample(0, 0, aNode.get_clock()->get_clock_type())
{
    // Each message contains the complete path, so retain the latest message.
    mPathPublisher = mNode.create_publisher<nav_msgs::msg::Path>(
        PathTopic,
        rclcpp::QoS(1).reliable().transient_local());

    mOdometrySubscriber = mNode.create_subscription<nav_msgs::msg::Odometry>(
        OdometryTopic,
        rclcpp::SensorDataQoS(),
        std::bind(
            &CPathRecorder::OdometryCallback,
            this,
            std::placeholders::_1));
}

// Record an odometry history for the RViz Path display.
void CPathRecorder::OdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr aMessage)
{
    const rclcpp::Time SampleTime(
        aMessage->header.stamp,
        mNode.get_clock()->get_clock_type());

    // Start a new trajectory when time goes backwards or the frame changes.
    if (SampleTime < mLastSample
        || mPath.header.frame_id != aMessage->header.frame_id)
    {
        mPath.poses.clear();
    }

    if (mPath.poses.empty()
        || (SampleTime - mLastSample).seconds() >= SamplePeriodSeconds)
    {
        geometry_msgs::msg::PoseStamped Pose;
        Pose.header = aMessage->header;
        Pose.pose = aMessage->pose.pose;

        mPath.header = aMessage->header;
        mPath.poses.push_back(Pose);
        mPathPublisher->publish(mPath);

        mLastSample = SampleTime;
    }
}
