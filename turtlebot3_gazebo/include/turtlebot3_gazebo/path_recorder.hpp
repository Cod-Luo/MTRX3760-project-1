// path_recorder.hpp - Records the robot's odometry trajectory for display in RViz.

#ifndef TURTLEBOT3_GAZEBO_PATH_RECORDER_HPP
#define TURTLEBOT3_GAZEBO_PATH_RECORDER_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

// Listens to odometry and republishes the poses so far as one path message,
// so RViz can draw the route the robot has driven. It never affects control.
class CPathRecorder
{
    public:
        // Creates the odometry subscription and the path publisher on aNode.
        explicit CPathRecorder(rclcpp::Node& aNode);

        // The subscription callback points at this object, so it must not be copied.
        CPathRecorder(const CPathRecorder&) = delete;
        CPathRecorder& operator=(const CPathRecorder&) = delete;

    private:
        // Samples the path at a fixed rate and restarts it after a time or frame reset.
        void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr aMessage);

        rclcpp::Node& mNode;  // Knows the node for its clock; does not own it.

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPublisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdometrySubscriber;

        nav_msgs::msg::Path mPath;  // Poses recorded in the current run.
        rclcpp::Time mLastSample;   // Timestamp of the last recorded pose.

        static const double SamplePeriodSeconds;  // Minimum time between poses.
        static const std::string OdometryTopic;
        static const std::string PathTopic;
};

#endif
