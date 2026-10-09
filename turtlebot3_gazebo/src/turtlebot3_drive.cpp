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

// turtlebot3_drive.cpp - ROS communication, controller updates and path recording.

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <algorithm>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/create_timer.hpp>

// Configure the controller and connect its inputs and outputs to ROS.
Turtlebot3Drive::Turtlebot3Drive()
    : Node("turtlebot3_drive_node"),
      mWallFollower(ReadSettings()),
      mUseStampedVelocity(
          declare_parameter<bool>("use_stamped_velocity", true)),
      mLastScan(std::chrono::steady_clock::now()),
      mLastScanStamp(0, 0, get_clock()->get_clock_type()),
      mLastPathSample(0, 0, get_clock()->get_clock_type())
{
    if (!mWallFollower.HasValidSettings())
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid wall-follower settings; motion disabled");
    }

    // Create only the velocity publisher selected for this robot setup.
    if (mUseStampedVelocity)
    {
        mStampedPublisher =
            create_publisher<geometry_msgs::msg::TwistStamped>(
                "cmd_vel", 10);
    }
    else
    {
        mVelocityPublisher =
            create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel", 10);
    }

    // Each message contains the complete path, so retain the latest message.
    mPathPublisher = create_publisher<nav_msgs::msg::Path>(
        "wall_follower/path",
        rclcpp::QoS(1).reliable().transient_local());

    mScanSubscriber = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(
            &Turtlebot3Drive::ScanCallback,
            this,
            std::placeholders::_1));

    mOdometrySubscriber = create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        rclcpp::SensorDataQoS(),
        std::bind(
            &Turtlebot3Drive::OdometryCallback,
            this,
            std::placeholders::_1));

    // Run at 20 Hz according to the ROS clock, including simulation time.
    mUpdateTimer = rclcpp::create_timer(
        this,
        get_clock(),
        rclcpp::Duration::from_seconds(0.05),
        std::bind(&Turtlebot3Drive::Update, this));

    RCLCPP_INFO(
        get_logger(),
        "Right-wall follower ready; waiting for laser data");
}

// Read parameters once when constructing the controller.
WallFollower::Settings Turtlebot3Drive::ReadSettings()
{
    WallFollower::Settings Settings;

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

// Forward laser data to the controller and record its timing information.
void Turtlebot3Drive::ScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr aMessage)
{
    mWallFollower.UpdateScan(
        aMessage->ranges,
        aMessage->angle_min,
        aMessage->angle_increment,
        aMessage->range_min,
        aMessage->range_max);

    mLastScan = std::chrono::steady_clock::now();

    mLastScanStamp = rclcpp::Time(
        aMessage->header.stamp,
        get_clock()->get_clock_type());

    mHaveScan = true;
}

// Record an odometry history for the RViz Path display.
void Turtlebot3Drive::OdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr aMessage)
{
    const rclcpp::Time SampleTime(
        aMessage->header.stamp,
        get_clock()->get_clock_type());

    // Start a new trajectory when time goes backwards or the frame changes.
    if (SampleTime < mLastPathSample
        || mPath.header.frame_id != aMessage->header.frame_id)
    {
        mPath.poses.clear();
    }

    // Record poses at up to five samples per simulation second.
    if (mPath.poses.empty()
        || (SampleTime - mLastPathSample).seconds() >= 0.2)
    {
        geometry_msgs::msg::PoseStamped Pose;
        Pose.header = aMessage->header;
        Pose.pose = aMessage->pose.pose;

        mPath.header = aMessage->header;
        mPath.poses.push_back(Pose);
        mPathPublisher->publish(mPath);

        mLastPathSample = SampleTime;
    }
}

// Convert the controller's command into the selected ROS velocity message.
void Turtlebot3Drive::PublishCommand(
    const WallFollower::Command& aCommand)
{
    geometry_msgs::msg::Twist Velocity;
    Velocity.linear.x = aCommand.Linear;
    Velocity.angular.z = aCommand.Angular;

    if (mUseStampedVelocity)
    {
        geometry_msgs::msg::TwistStamped Stamped;
        Stamped.header.stamp = now();
        Stamped.header.frame_id = "base_link";
        Stamped.twist = Velocity;

        mStampedPublisher->publish(Stamped);
    }
    else
    {
        mVelocityPublisher->publish(Velocity);
    }
}

// Publish a stop unless a received scan is recent enough for the controller.
void Turtlebot3Drive::Update()
{
    WallFollower::Command Command;

    if (mHaveScan)
    {
        const double ReceiptAge = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - mLastScan).count();

        const double StampAge = (now() - mLastScanStamp).seconds();

        if (StampAge >= 0.0)
        {
            Command = mWallFollower.CalculateCommand(
                std::max(ReceiptAge, StampAge));
        }
    }

    PublishCommand(Command);
}