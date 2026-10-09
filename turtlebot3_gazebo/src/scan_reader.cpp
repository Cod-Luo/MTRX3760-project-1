// scan_reader.cpp - Angle-based sector extraction from laser scans.

#include "turtlebot3_gazebo/scan_reader.hpp"

#include <algorithm>
#include <cmath>

const double CScanReader::Pi = 3.14159265358979323846;

// Front: 0 degrees, half-width 20 degrees, nearest return so no obstacle is missed.
const CScanReader::SectorShape CScanReader::FrontShape = {0.0, Pi / 9.0, true};

// Right: -90 degrees, half-width 5 degrees, median to ignore single noisy returns.
const CScanReader::SectorShape CScanReader::RightShape = {-Pi / 2.0, Pi / 36.0, false};

// Front-right: -45 degrees, half-width 5 degrees, median.
const CScanReader::SectorShape CScanReader::FrontRightShape = {-Pi / 4.0, Pi / 36.0, false};

// Recalculate all three sectors; any sector without a usable return becomes invalid.
void CScanReader::Update(
    const std::vector<float>& aRanges,
    double aAngleMin,
    double aAngleIncrement,
    double aRangeMin,
    double aRangeMax)
{
    const ScanInfo Info = {aAngleMin, aAngleIncrement, aRangeMin, aRangeMax};

    mFront = Sector{};
    mRight = Sector{};
    mFrontRight = Sector{};
    mHaveScan = true;

    if (IsUsableScan(aRanges, Info))
    {
        mFront = ReadSector(aRanges, Info, FrontShape);
        mRight = ReadSector(aRanges, Info, RightShape);
        mFrontRight = ReadSector(aRanges, Info, FrontRightShape);
    }
}

bool CScanReader::HasValidReadings() const
{
    return mFront.Valid && mRight.Valid && mFrontRight.Valid;
}

bool CScanReader::HasReceivedScan() const
{
    return mHaveScan;
}

bool CScanReader::HasValidFrontReading() const
{
    return mFront.Valid;
}

bool CScanReader::HasValidRightReading() const
{
    return mRight.Valid;
}

bool CScanReader::HasValidFrontRightReading() const
{
    return mFrontRight.Valid;
}

double CScanReader::FrontDistance() const
{
    return mFront.Distance;
}

double CScanReader::RightDistance() const
{
    return mRight.Distance;
}

double CScanReader::FrontRightDistance() const
{
    return mFrontRight.Distance;
}

bool CScanReader::IsUsableScan(
    const std::vector<float>& aRanges,
    const ScanInfo& aInfo)
{
    return !aRanges.empty()
        && std::isfinite(aInfo.AngleMin)
        && std::isfinite(aInfo.AngleIncrement)
        && aInfo.AngleIncrement != 0.0
        && std::isfinite(aInfo.RangeMin)
        && std::isfinite(aInfo.RangeMax)
        && aInfo.RangeMin >= 0.0
        && aInfo.RangeMax > aInfo.RangeMin;
}

CScanReader::Sector CScanReader::ReadSector(
    const std::vector<float>& aRanges,
    const ScanInfo& aInfo,
    const SectorShape& aShape)
{
    std::vector<double> ValidRanges;

    for (std::size_t Index = 0; Index < aRanges.size(); ++Index)
    {
        const double Angle =
            aInfo.AngleMin + static_cast<double>(Index) * aInfo.AngleIncrement;

        // Wrap the difference so equivalent angles lie in the same sector.
        const double Difference =
            std::remainder(Angle - aShape.Centre, 2.0 * Pi);

        const double Range = static_cast<double>(aRanges[Index]);

        if (std::abs(Difference) <= aShape.HalfWidth)
        {
            if (std::isinf(Range) && Range > 0.0)
            {
                // Positive infinity means no return within the sensor's range.
                ValidRanges.push_back(aInfo.RangeMax);
            }
            else if (
                std::isfinite(Range)
                && Range >= aInfo.RangeMin
                && Range <= aInfo.RangeMax)
            {
                ValidRanges.push_back(Range);
            }
        }
    }

    Sector Result;

    if (!ValidRanges.empty())
    {
        std::sort(ValidRanges.begin(), ValidRanges.end());
        Result.Valid = true;

        if (aShape.UseMinimum)
        {
            Result.Distance = ValidRanges.front();
        }
        else
        {
            Result.Distance = ValidRanges[ValidRanges.size() / 2];
        }
    }

    return Result;
}
