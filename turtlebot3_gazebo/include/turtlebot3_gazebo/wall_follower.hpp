// wall_follower.hpp - Decides how to drive to follow a wall on the right.

#ifndef TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP
#define TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP

#include "turtlebot3_gazebo/scan_reader.hpp"

// Converts front and right wall distances into velocity commands for following
// a wall on the right. Knows nothing about ROS; CWallFollowerNode connects it.
//
// Rules, highest priority first:
//   1. Front blocked        -> turn left on the spot
//   2. Right wall lost      -> curve right to find it again
//   3. Otherwise            -> steer to hold the target distance from the wall
class CWallFollower
{
    public:
        // Input validation result, also used by the ROS node's diagnostics.
        enum DriveStatus
        {
            Ready,
            InvalidSettings,
            InvalidScan,
            InvalidScanTime,
            StaleScan
        };

        // Distances are metres; speeds are metres/second and radians/second.
        struct Settings
        {
            double WallDistance = 0.35;         // Target gap to the right wall.
            double ForwardSpeed = 0.15;         // Top driving speed.
            double TurnSpeed = 0.65;            // Top turning speed.
            double FrontStopDistance = 0.40;    // Start turning left below this.
            double FrontResumeDistance = 0.50;  // Stop turning left above this.
            double WallLostDistance = 0.90;     // Right wall counts as gone above this.
            double DistanceGain = 2.0;          // Steering per metre of distance error.
            double HeadingGain = 1.2;           // Steering per radian of wall angle.

            // True when every value is finite and the values make sense together.
            bool IsValid() const;
        };

        // Velocity requested by the controller; defaults to stopping.
        struct Command
        {
            double Linear = 0.0;   // Forward, metres/second.
            double Angular = 0.0;  // Positive turns left, radians/second.
        };

        // Stores the settings; invalid settings prevent all movement.
        explicit CWallFollower(const Settings& aSettings);

        // Reports whether the supplied settings are usable.
        bool HasValidSettings() const;

        // Explains why motion is permitted or disabled; does not change steering state.
        DriveStatus CheckInput(const CScanReader& aScan, double aScanAgeSeconds) const;

        // Fresh partial scans retain corner steering; missing or stale input stops.
        Command CalculateCommand(const CScanReader& aScan, double aScanAgeSeconds);

    private:
        // Use available sectors instead of cancelling a turn on partial scans.
        Command RecoverFromPartialScan(const CScanReader& aScan);

        // Uses separate stop and resume distances so the robot does not flick
        // between turning and driving when the front distance hovers near one value.
        void UpdateFrontBlocked(double aFrontDistance);

        // Rule 1: a wall is ahead, so turn left on the spot.
        Command TurnLeftInPlace() const;

        // Rule 2: the right wall has ended, so curve right around the corner.
        Command CurveRightToFindWall() const;

        // Rule 3: steer to hold the target distance and stay parallel to the wall.
        Command FollowWall(double aRightDistance, double aFrontRightDistance) const;

        const Settings mSettings;  // Fixed after construction.

        // Keeps a blocked-front turn going until the resume distance is reached.
        bool mFrontBlocked = false;

        static const double ScanTimeout;          // Maximum scan age, seconds.
        static const double CornerTurnFraction;   // Share of TurnSpeed used when curving.
        static const double MaxSteeringSlowdown;  // Speed lost at full steering.
        static const double InvalidScanSpeed;     // Maximum partial-scan driving, m/s.
};

#endif
