// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

// turtlebot3_drive.hpp - ROS interface and trajectory recording for wall following.

#ifndef TURTLEBOT3_GAZEBO_TURTLEBOT3_DRIVE_HPP
#define TURTLEBOT3_GAZEBO_TURTLEBOT3_DRIVE_HPP

#include "turtlebot3_gazebo/wall_follower.hpp"

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Connects the wall-following controller to ROS and records an odometry trajectory.
class Turtlebot3Drive : public rclcpp::Node
{
    public:
        Turtlebot3Drive();

    private:
        // Reads startup parameters; settings are fixed for this node instance.
        WallFollower::Settings ReadSettings();

        // Updates laser sectors and records receipt and measurement times.
        void ScanCallback(
            const sensor_msgs::msg::LaserScan::SharedPtr aMessage);

        // Samples the path at up to 5 Hz and clears it after a time or frame reset.
        void OdometryCallback(
            const nav_msgs::msg::Odometry::SharedPtr aMessage);

        // Calculates and publishes commands using the latest usable scan.
        void Update();

        // Publishes the velocity message type selected at startup.
        void PublishCommand(const WallFollower::Command& aCommand);

        WallFollower mWallFollower;

        // Must match the receiver's cmd_vel message type.
        bool mUseStampedVelocity;

        // Prevents movement before the first scan arrives.
        bool mHaveScan = false;

        // Receipt time uses a steady clock; measurement time uses the ROS clock.
        std::chrono::steady_clock::time_point mLastScan;
        rclcpp::Time mLastScanStamp;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
            mVelocityPublisher;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
            mStampedPublisher;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
            mPathPublisher;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
            mScanSubscriber;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
            mOdometrySubscriber;

        rclcpp::TimerBase::SharedPtr mUpdateTimer;

        // Accumulated odometry poses from the current recorded run.
        nav_msgs::msg::Path mPath;

        // Timestamp of the last recorded pose.
        rclcpp::Time mLastPathSample;
};

#endif