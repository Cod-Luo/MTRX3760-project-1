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

// ROS interface for MTRX3760 Project 1 right-wall following.
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

class Turtlebot3Drive : public rclcpp::Node
{
    public:
        Turtlebot3Drive();

    private:
        WallFollower::Settings ReadSettings();
        void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr aMessage);
        void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr aMessage);
        void Update();
        void PublishCommand(const WallFollower::Command& aCommand);

        WallFollower mWallFollower;
        bool mUseStampedVelocity;
        bool mHaveScan = false;
        std::chrono::steady_clock::time_point mLastScan;
        rclcpp::Time mLastScanStamp;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelocityPublisher;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mStampedPublisher;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPublisher;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mScanSubscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdometrySubscriber;
        rclcpp::TimerBase::SharedPtr mUpdateTimer;
        nav_msgs::msg::Path mPath;
        rclcpp::Time mLastPathSample;
};

#endif
