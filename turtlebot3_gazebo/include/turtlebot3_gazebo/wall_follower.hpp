// wall_follower.hpp - Sensor-based right-wall steering, independent of ROS.
#ifndef TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP
#define TURTLEBOT3_GAZEBO_WALL_FOLLOWER_HPP

#include <vector>

class WallFollower
{
    public:
        // Distances: metres. Velocities: metres/second and radians/second.
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

        struct Command
        {
            double Linear = 0.0;
            double Angular = 0.0;
        };

        explicit WallFollower(const Settings& aSettings);
        // Uses scan angles instead of assuming one sample per degree.
        void UpdateScan(const std::vector<float>& aRanges, double aAngleMin,
            double aAngleIncrement, double aRangeMin, double aRangeMax);
        // Missing, invalid or stale scans produce a stop command.
        Command CalculateCommand(double aScanAgeSeconds);

    private:
        struct Sector
        {
            double Distance = 0.0;
            bool Valid = false;
        };

        static Sector ReadSector(const std::vector<float>& aRanges,
            double aAngleMin, double aAngleIncrement, double aRangeMin,
            double aRangeMax, double aCentre, double aHalfWidth, bool aUseMinimum);
        const Settings mSettings;
        Sector mFront;
        Sector mRight;
        Sector mFrontRight;
        bool mTurningLeft = false;
        static constexpr double Pi = 3.14159265358979323846;
        static constexpr double ScanTimeout = 0.5;
};

#endif
