// wall_follower.hpp - Decides how to drive to follow a wall on the right.

#ifndef TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP
#define TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP

#include "turtlebot3_gazebo/scan_reader.hpp"

// Converts front and right wall distances into velocity commands for following
// a wall on the right. Knows nothing about ROS; CWallFollowerNode connects it.
class CWallFollower
{
    public:
        // Distances are metres; speeds are metres/second and radians/second.
        struct Settings
        {
            double WallDistance = 0.35;
            double ForwardSpeed = 0.15;
            double TurnSpeed = 0.65;
            double FrontStopDistance = 0.40;
            double FrontResumeDistance = 0.50;
            double WallLostDistance = 0.90;
            double DistanceGain = 2.0;
            double HeadingGain = 1.2;
        };

        // Velocity requested by the controller; defaults to stopping.
        struct Command
        {
            double Linear = 0.0;
            double Angular = 0.0;
        };

        // Stores and validates settings; invalid settings prevent movement.
        explicit CWallFollower(const Settings& aSettings);

        // Reports whether the supplied settings are usable.
        bool HasValidSettings() const;

        // Returns a stop command for missing, invalid or stale scans.
        Command CalculateCommand(const CScanReader& aScan, double aScanAgeSeconds);

    private:
        const Settings mSettings;  // Fixed after construction.
        bool mSettingsValid;       // False disables all movement.

        // Keeps a blocked-front turn active until the resume distance is reached.
        bool mTurningLeft = false;

        static const double ScanTimeout;  // Maximum scan age in seconds.
};

#endif