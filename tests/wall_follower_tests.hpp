// wall_follower_tests.hpp - ROS-independent development regression checks.
#ifndef WALL_FOLLOWER_TESTS_HPP
#define WALL_FOLLOWER_TESTS_HPP

#include "turtlebot3_gazebo/wall_follower.hpp"

#include <string>
#include <vector>

// Encapsulates development checks; main is the only free function.
class CWallFollowerTests
{
    public:
        // Runs every check and returns a process exit status.
        int Run() const;

    private:
        struct Segment
        {
            double X1;
            double Y1;
            double X2;
            double Y2;
        };
        struct SensorProfile
        {
            std::string Name;
            int ScanEverySteps;
            double OffsetX;
            bool Noisy;
        };

        static bool Require(bool aCondition, const std::string& aMessage);
        static std::vector<float> Scan(double aRight, double aFront = 3.5, int aSamples = 360);
        static void Feed(CScanReader& aReader, const std::vector<float>& aRanges);
        static void InvalidateSector(std::vector<float>& aRanges, int aFirst, int aLast);
        // Test groups deliberately remain small and independently report failures.
        bool CheckSteering() const;
        bool CheckScanValidation() const;
        bool CheckInputStatus() const;
        bool CheckPersistentRecovery() const;
        bool CheckMissingDiagonalRecovery() const;
        bool CheckPartialScanCornerRecovery() const;
        bool CheckRecoverySpeedLimit() const;
        static double RayDistance(double aX, double aY, double aAngle, const Segment& aWall);
        static double Clearance(double aX, double aY, const Segment& aWall);
        bool CheckMaze(const SensorProfile& aProfile) const;

        static const double Pi;
        static const double StepSeconds;
        static const double MinimumRange;
        static const double MaximumRange;
        static const double WallHalfThickness;
        static const double MinimumSafeClearance;
        static const int LaserSamples;
        static const int MaximumSteps;
};


#endif
