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
#include "turtlebot3_gazebo/scan_reader.hpp"
#include "turtlebot3_gazebo/velocity_publisher.hpp"
#include "turtlebot3_gazebo/wall_follower.hpp"

#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>

// Connects the wall-following controller to ROS. Top-level owner of the program's parts.
class CWallFollowerNode : public rclcpp::Node
{
    public:
        CWallFollowerNode();

        // Removes the shutdown hook so it cannot run after this node is gone.
        ~CWallFollowerNode();

    private:
        // Reads startup parameters; settings are fixed for this node instance.
        CWallFollower::Settings ReadSettings();

        // Updates the scan reader and records receipt and measurement times.
        void ScanCallback(
            const sensor_msgs::msg::LaserScan::SharedPtr aMessage);

        // Calculates and publishes commands using the latest usable scan.
        void Update();

        // Runs just before ROS shuts down (e.g. Ctrl+C) and sends one last stop.
        // Gazebo, and possibly the real robot, keep repeating the last command,
        // so without this the robot could keep driving after the program exits.
        void StopBeforeShutdown();

        CScanReader mScanReader;      // Turns laser scans into wall distances.
        CWallFollower mWallFollower;  // Turns wall distances into drive commands.

        // Sends the controller's commands to the wheels.
        CVelocityPublisher mVelocityPublisher;

        // Prevents movement before the first scan arrives.
        bool mHaveScan = false;

        // Receipt time uses a steady clock; measurement time uses the ROS clock.
        std::chrono::steady_clock::time_point mLastScanReceipt;
        rclcpp::Time mLastScanStamp;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
            mScanSubscriber;

        rclcpp::TimerBase::SharedPtr mUpdateTimer;

        // Draws the driven route in RViz; independent of control.
        CPathRecorder mPathRecorder;

        // The shutdown hook runs on a different thread from Update(), so this
        // lock makes sure no drive command can be sent after the final stop.
        std::mutex mPublishMutex;
        bool mStopped = false;  // Guarded by mPublishMutex.
        rclcpp::PreShutdownCallbackHandle mStopHook;

        static const std::string NodeName;
        static const std::string ScanTopic;
        static const double UpdatePeriodSeconds;  // Time between drive commands.
};

#endif