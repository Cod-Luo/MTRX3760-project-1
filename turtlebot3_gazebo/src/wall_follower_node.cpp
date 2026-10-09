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
#include <rclcpp/create_timer.hpp>

const std::string CWallFollowerNode::NodeName = "turtlebot3_drive_node";
const std::string CWallFollowerNode::ScanTopic = "scan";
const double CWallFollowerNode::UpdatePeriodSeconds = 0.05;  // 20 Hz.
const double CWallFollowerNode::PollFrequency = 100.0;

// Configure the controller and connect its inputs and outputs to ROS.
CWallFollowerNode::CWallFollowerNode(const rclcpp::NodeOptions& aOptions)
    : Node(NodeName, aOptions),
      mWallFollower(ReadSettings()),
      mVelocityPublisher(*this),
      mLastScanReceipt(std::chrono::steady_clock::now()),
      mLastScanStamp(0, 0, get_clock()->get_clock_type()),
      mPathRecorder(*this)
{
    if (!mWallFollower.HasValidSettings())
    {
        ReportInputStatus(CWallFollower::InvalidSettings);
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

// The default ROS context handles Ctrl+C/SIGTERM. This node has its own context,
// kept alive by main until this single-threaded loop has sent the final stop.
void CWallFollowerNode::Run()
{
    rclcpp::ExecutorOptions Options;
    Options.context = get_node_base_interface()->get_context();
    rclcpp::executors::SingleThreadedExecutor Executor(Options);
    Executor.add_node(get_node_base_interface());
    rclcpp::WallRate PollRate(PollFrequency);

    while (rclcpp::ok() && rclcpp::ok(Options.context))
    {
        Executor.spin_some();
        PollRate.sleep();
    }

    // No callbacks execute after this point, so a drive cannot follow this stop.
    mUpdateTimer->cancel();
    if (rclcpp::ok(Options.context))
    {
        mVelocityPublisher.Stop();
        RCLCPP_INFO(get_logger(), "Shutting down: stop command sent");
    }
    Executor.remove_node(get_node_base_interface());
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

// Fresh unusable scans permit slow recovery; missing or stale input stops.
void CWallFollowerNode::Update()
{
    CWallFollower::Command Command;

    if (mHaveScan)
    {
        const double ReceiptAge = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - mLastScanReceipt).count();

        const double StampAge = (now() - mLastScanStamp).seconds();

        double ScanAge = -1.0;
        if (StampAge >= 0.0)
        {
            ScanAge = std::max(ReceiptAge, StampAge);
        }
        ReportInputStatus(mWallFollower.CheckInput(mScanReader, ScanAge));
        Command = mWallFollower.CalculateCommand(mScanReader, ScanAge);
    }

    mVelocityPublisher.Publish(Command.Linear, Command.Angular);
}

// Keep safety checks enabled in release builds and make each failure diagnosable.
void CWallFollowerNode::ReportInputStatus(CWallFollower::DriveStatus aStatus)
{
    if (!mHaveInputStatus || aStatus != mInputStatus)
    {
        switch (aStatus)
        {
            case CWallFollower::Ready:
                RCLCPP_INFO(get_logger(), "Usable laser data received; motion enabled");
                break;
            case CWallFollower::InvalidSettings:
                RCLCPP_ERROR(get_logger(), "Invalid wall-follower settings; motion disabled");
                break;
            case CWallFollower::InvalidScan:
                RCLCPP_WARN(
                    get_logger(),
                    "Invalid laser readings or metadata; creeping forward at up to 0.01 m/s");
                break;
            case CWallFollower::InvalidScanTime:
                RCLCPP_WARN(get_logger(), "Invalid laser timestamp or scan age; stopping");
                break;
            case CWallFollower::StaleScan:
                RCLCPP_WARN(get_logger(), "Laser data is stale; stopping");
                break;
        }
        mInputStatus = aStatus;
        mHaveInputStatus = true;
    }
}
