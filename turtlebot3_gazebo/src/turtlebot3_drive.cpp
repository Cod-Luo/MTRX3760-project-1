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

// Modified for MTRX3760 Project 1: steering and trajectory recording.
#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <algorithm>
#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/create_timer.hpp>

Turtlebot3Drive::Turtlebot3Drive()
    : Node("turtlebot3_drive_node"),
      mWallFollower(ReadSettings()),
      mUseStampedVelocity(declare_parameter<bool>("use_stamped_velocity", true)),
      mLastScan(std::chrono::steady_clock::now()),
      mLastScanStamp(0, 0, get_clock()->get_clock_type()),
      mLastPathSample(0, 0, get_clock()->get_clock_type())
{
    if (mUseStampedVelocity)
    {
        mStampedPublisher = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    }
    else
    {
        mVelocityPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    mPathPublisher = create_publisher<nav_msgs::msg::Path>("wall_follower/path",
        rclcpp::QoS(1).reliable().transient_local());
    mScanSubscriber = create_subscription<sensor_msgs::msg::LaserScan>("scan",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr aMessage)
        {
            ScanCallback(aMessage);
        });
    mOdometrySubscriber = create_subscription<nav_msgs::msg::Odometry>("odom",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr aMessage)
        {
            OdometryCallback(aMessage);
        });
    // Keep 20 Hz in simulation seconds, also when Gazebo runs faster than real time.
    mUpdateTimer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.05),
        [this]() { Update(); });
    RCLCPP_INFO(get_logger(), "Right-wall follower ready; waiting for laser data");
}

WallFollower::Settings Turtlebot3Drive::ReadSettings()
{
    WallFollower::Settings Settings;
    Settings.WallDistance = declare_parameter<double>("wall_distance", Settings.WallDistance);
    Settings.ForwardSpeed = declare_parameter<double>("forward_speed", Settings.ForwardSpeed);
    Settings.TurnSpeed = declare_parameter<double>("turn_speed", Settings.TurnSpeed);
    Settings.FrontStopDistance = declare_parameter<double>("front_stop_distance", Settings.FrontStopDistance);
    Settings.FrontResumeDistance = declare_parameter<double>("front_resume_distance", Settings.FrontResumeDistance);
    Settings.WallLostDistance = declare_parameter<double>("wall_lost_distance", Settings.WallLostDistance);
    Settings.DistanceGain = declare_parameter<double>("distance_gain", Settings.DistanceGain);
    Settings.HeadingGain = declare_parameter<double>("heading_gain", Settings.HeadingGain);
    return Settings;
}

void Turtlebot3Drive::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr aMessage)
{
    mWallFollower.UpdateScan(aMessage->ranges, aMessage->angle_min,
        aMessage->angle_increment, aMessage->range_min, aMessage->range_max);
    mLastScan = std::chrono::steady_clock::now();
    mLastScanStamp = rclcpp::Time(aMessage->header.stamp, get_clock()->get_clock_type());
    mHaveScan = true;
}

void Turtlebot3Drive::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr aMessage)
{
    const rclcpp::Time SampleTime(aMessage->header.stamp, get_clock()->get_clock_type());

    if (mPath.poses.empty() || SampleTime < mLastPathSample
        || (SampleTime - mLastPathSample).seconds() >= 0.2)
    {
        if (mPath.header.frame_id != aMessage->header.frame_id)
        {
            mPath.poses.clear();
        }

        geometry_msgs::msg::PoseStamped Pose;
        Pose.header = aMessage->header;
        Pose.pose = aMessage->pose.pose;
        mPath.header = aMessage->header;
        mPath.poses.push_back(Pose);
        mPathPublisher->publish(mPath);
        mLastPathSample = SampleTime;
    }
}

void Turtlebot3Drive::PublishCommand(const WallFollower::Command& aCommand)
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
            Command = mWallFollower.CalculateCommand(std::max(ReceiptAge, StampAge));
        }
    }

    PublishCommand(Command);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int Result = 0;

    try
    {
        rclcpp::spin(std::make_shared<Turtlebot3Drive>());
    }
    catch (const std::exception& Error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("turtlebot3_drive"), "%s", Error.what());
        Result = 1;
    }

    rclcpp::shutdown();
    return Result;
}
