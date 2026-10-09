// wall_follower.hpp - Interface for sensor-based right-wall following.

#ifndef TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP
#define TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP

#include <vector>

// Converts laser sectors into velocity commands for following a wall on the right.
// ROS communication is handled separately by CWallFollowerNode.
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

        // Updates distances using the scan's angular and range information.
        void UpdateScan(
            const std::vector<float>& aRanges,
            double aAngleMin,
            double aAngleIncrement,
            double aRangeMin,
            double aRangeMax);

        // Returns a stop command for missing, invalid or stale scans.
        Command CalculateCommand(double aScanAgeSeconds);

    private:
        // A sector is usable only when it contains a valid laser return.
        struct Sector
        {
            double Distance = 0.0;
            bool Valid = false;
        };

        // Selects the minimum or median usable distance within a laser sector.
        static Sector ReadSector(
            const std::vector<float>& aRanges,
            double aAngleMin,
            double aAngleIncrement,
            double aRangeMin,
            double aRangeMax,
            double aCentre,
            double aHalfWidth,
            bool aUseMinimum);

        const Settings mSettings;  // Fixed after construction.
        bool mSettingsValid;       // False disables all movement.

        Sector mFront;
        Sector mRight;
        Sector mFrontRight;

        // Keeps a blocked-front turn active until the resume distance is reached.
        bool mTurningLeft = false;

        static const double Pi;
        static const double ScanTimeout;  // Maximum scan age in seconds.
};

#endif