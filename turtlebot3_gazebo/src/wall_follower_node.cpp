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

// wall_follower_node.cpp - ROS communication and controller updates.

#include "turtlebot3_gazebo/wall_follower_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <rclcpp/create_timer.hpp>

const std::string CWallFollowerNode::NodeName = "turtlebot3_drive_node";
const std::string CWallFollowerNode::ScanTopic = "scan";
const double CWallFollowerNode::UpdatePeriodSeconds = 0.05;  // 20 Hz.

// Configure the controller and connect its inputs and outputs to ROS.
CWallFollowerNode::CWallFollowerNode()
    : Node(NodeName),
      mWallFollower(ReadSettings()),
      mVelocityPublisher(
          *this,
          declare_parameter<bool>("use_stamped_velocity", true)),
      mLastScanReceipt(std::chrono::steady_clock::now()),
      mLastScanStamp(0, 0, get_clock()->get_clock_type()),
      mPathRecorder(*this)
{
    if (!mWallFollower.HasValidSettings())
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid wall-follower settings; motion disabled");
    }

    mScanSubscriber = create_subscription<sensor_msgs::msg::LaserScan>(
        ScanTopic,
        rclcpp::SensorDataQoS(),
        std::bind(
            &CWallFollowerNode::ScanCallback,
            this,
            std::placeholders::_1));

    // Uses the ROS clock, so the rate also holds in faster-than-real-time simulation.
    mUpdateTimer = rclcpp::create_timer(
        this,
        get_clock(),
        rclcpp::Duration::from_seconds(UpdatePeriodSeconds),
        std::bind(&CWallFollowerNode::Update, this));

    RCLCPP_INFO(
        get_logger(),
        "Right-wall follower ready; waiting for laser data");
}

// Read parameters once when constructing the controller.
CWallFollower::Settings CWallFollowerNode::ReadSettings()
{
    CWallFollower::Settings Settings;

    Settings.WallDistance = declare_parameter<double>(
        "wall_distance", Settings.WallDistance);

    Settings.ForwardSpeed = declare_parameter<double>(
        "forward_speed", Settings.ForwardSpeed);

    Settings.TurnSpeed = declare_parameter<double>(
        "turn_speed", Settings.TurnSpeed);

    Settings.FrontStopDistance = declare_parameter<double>(
        "front_stop_distance", Settings.FrontStopDistance);

    Settings.FrontResumeDistance = declare_parameter<double>(
        "front_resume_distance", Settings.FrontResumeDistance);

    Settings.WallLostDistance = declare_parameter<double>(
        "wall_lost_distance", Settings.WallLostDistance);

    Settings.DistanceGain = declare_parameter<double>(
        "distance_gain", Settings.DistanceGain);

    Settings.HeadingGain = declare_parameter<double>(
        "heading_gain", Settings.HeadingGain);

    return Settings;
}

// Pass laser data to the scan reader and record its timing information.
void CWallFollowerNode::ScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr aMessage)
{
    mScanReader.Update(
        aMessage->ranges,
        aMessage->angle_min,
        aMessage->angle_increment,
        aMessage->range_min,
        aMessage->range_max);

    mLastScanReceipt = std::chrono::steady_clock::now();

    mLastScanStamp = rclcpp::Time(
        aMessage->header.stamp,
        get_clock()->get_clock_type());

    mHaveScan = true;
}

// Publish a stop unless a received scan is recent enough for the controller.
void CWallFollowerNode::Update()
{
    CWallFollower::Command Command;

    if (mHaveScan)
    {
        const double ReceiptAge = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - mLastScanReceipt).count();

        const double StampAge = (now() - mLastScanStamp).seconds();

        if (StampAge >= 0.0)
        {
            Command = mWallFollower.CalculateCommand(
                mScanReader,
                std::max(ReceiptAge, StampAge));
        }
    }

    mVelocityPublisher.Publish(Command.Linear, Command.Angular);
}