// scan_reader.hpp - Turns a raw laser scan into front and right wall distances.

#ifndef TURTLEBOT3_GAZEBO_SCAN_READER_HPP
#define TURTLEBOT3_GAZEBO_SCAN_READER_HPP

#include <vector>

// Reads the three directions the wall follower needs from a full laser scan.
// Readings are found by angle, not by index, so lidars with a different
// number of readings per turn (simulator vs real LD19) give the same result.
// Knows nothing about ROS, so it can be tested on its own.
class CScanReader
{
    public:
        // Replaces the stored distances using one scan. Angles are radians,
        // anticlockwise from straight ahead; ranges are metres.
        void Update(
            const std::vector<float>& aRanges,
            double aAngleMin,
            double aAngleIncrement,
            double aRangeMin,
            double aRangeMax);

        // True when the latest scan gave a usable reading in every direction.
        bool HasValidReadings() const;

        // Partial scans can still identify a front obstacle or the right wall.
        bool HasValidFrontReading() const;
        bool HasValidRightReading() const;
        bool HasValidFrontRightReading() const;

        // Distinguish missing input from a received scan with unusable readings.
        bool HasReceivedScan() const;

        // Distances in metres from the latest scan.
        double FrontDistance() const;
        double RightDistance() const;
        double FrontRightDistance() const;

    private:
        // Describes how the scan's readings map to angles and valid ranges.
        struct ScanInfo
        {
            double AngleMin;
            double AngleIncrement;
            double RangeMin;
            double RangeMax;
        };

        // Which slice of the scan to look at, and how to summarise it.
        struct SectorShape
        {
            double Centre;     // Radians, 0 is straight ahead, positive is left.
            double HalfWidth;  // Radians either side of the centre.
            bool UseMinimum;   // Nearest return if true, otherwise the median.
        };

        // A sector is usable only when it contains a valid laser return.
        struct Sector
        {
            double Distance = 0.0;
            bool Valid = false;
        };

        // Rejects scans whose angle or range description cannot be trusted.
        static bool IsUsableScan(
            const std::vector<float>& aRanges,
            const ScanInfo& aInfo);

        // Selects the minimum or median usable distance within one sector.
        static Sector ReadSector(
            const std::vector<float>& aRanges,
            const ScanInfo& aInfo,
            const SectorShape& aShape);

        Sector mFront;
        Sector mRight;
        Sector mFrontRight;
        bool mHaveScan = false;

        static const double Pi;
        static const SectorShape FrontShape;       // Nearest obstacle ahead.
        static const SectorShape RightShape;       // The wall beside the robot.
        static const SectorShape FrontRightShape;  // The wall slightly ahead.
};

#endif
