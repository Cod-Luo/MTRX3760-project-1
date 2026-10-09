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

// wall_follower_node.hpp - ROS node that connects the wall follower to the robot.

#ifndef TURTLEBOT3_GAZEBO_WALL_FOLLOWER_NODE_HPP
#define TURTLEBOT3_GAZEBO_WALL_FOLLOWER_NODE_HPP

#include "turtlebot3_gazebo/path_recorder.hpp"
#include "turtlebot3_gazebo/wall_follower.hpp"

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Connects the wall-following controller to ROS. Top-level owner of the program's parts.
class CWallFollowerNode : public rclcpp::Node
{
    public:
        CWallFollowerNode();

    private:
        // Reads startup parameters; settings are fixed for this node instance.
        CWallFollower::Settings ReadSettings();

        // Updates laser sectors and records receipt and measurement times.
        void ScanCallback(
            const sensor_msgs::msg::LaserScan::SharedPtr aMessage);

        // Calculates and publishes commands using the latest usable scan.
        void Update();

        // Publishes the velocity message type selected at startup.
        void PublishCommand(const CWallFollower::Command& aCommand);

        CWallFollower mWallFollower;

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

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
            mScanSubscriber;

        rclcpp::TimerBase::SharedPtr mUpdateTimer;

        // Draws the driven route in RViz; independent of control.
        CPathRecorder mPathRecorder;
};

#endif