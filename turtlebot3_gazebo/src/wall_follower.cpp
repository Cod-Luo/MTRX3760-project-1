// wall_follower.cpp - Blocked passages, wall loss and wall-distance correction.
#include "turtlebot3_gazebo/wall_follower.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

WallFollower::WallFollower(const Settings& aSettings)
    : mSettings(aSettings)
{
    const bool Valid =
        std::isfinite(mSettings.WallDistance)
        && std::isfinite(mSettings.ForwardSpeed)
        && std::isfinite(mSettings.TurnSpeed)
        && std::isfinite(mSettings.FrontStopDistance)
        && std::isfinite(mSettings.FrontResumeDistance)
        && std::isfinite(mSettings.WallLostDistance)
        && std::isfinite(mSettings.DistanceGain)
        && std::isfinite(mSettings.HeadingGain)
        && mSettings.WallDistance > 0.0
        && mSettings.ForwardSpeed > 0.0
        && mSettings.TurnSpeed > 0.0
        && mSettings.FrontStopDistance > 0.0
        && mSettings.FrontResumeDistance > mSettings.FrontStopDistance
        && mSettings.WallLostDistance > mSettings.WallDistance
        && mSettings.DistanceGain > 0.0
        && mSettings.HeadingGain >= 0.0;

    if (!Valid)
    {
        throw std::invalid_argument("Invalid wall-follower settings");
    }
}

WallFollower::Sector WallFollower::ReadSector
(
    const std::vector<float>& aRanges,
    double aAngleMin,
    double aAngleIncrement,
    double aRangeMin,
    double aRangeMax,
    double aCentre,
    double aHalfWidth,
    bool aUseMinimum
)
{
    std::vector<double> ValidRanges;

    for (std::size_t Index = 0; Index < aRanges.size(); ++Index)
    {
        const double Angle = aAngleMin + static_cast<double>(Index) * aAngleIncrement;
        const double Difference = std::remainder(Angle - aCentre, 2.0 * Pi);
        const double Range = static_cast<double>(aRanges[Index]);

        if (std::abs(Difference) <= aHalfWidth)
        {
            if (std::isinf(Range) && Range > 0.0)
            {
                // Positive infinity means no return within the sensor's range.
                ValidRanges.push_back(aRangeMax);
            }
            else if (std::isfinite(Range) && Range >= aRangeMin && Range <= aRangeMax)
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
        Result.Distance = aUseMinimum ? ValidRanges.front()
            : ValidRanges[ValidRanges.size() / 2];
    }

    return Result;
}

void WallFollower::UpdateScan
(
    const std::vector<float>& aRanges,
    double aAngleMin,
    double aAngleIncrement,
    double aRangeMin,
    double aRangeMax
)
{
    mFront = Sector{};
    mRight = Sector{};
    mFrontRight = Sector{};

    const bool ValidMetadata = !aRanges.empty()
        && std::isfinite(aAngleMin)
        && std::isfinite(aAngleIncrement) && aAngleIncrement != 0.0
        && std::isfinite(aRangeMin) && std::isfinite(aRangeMax)
        && aRangeMin >= 0.0 && aRangeMax > aRangeMin;

    if (ValidMetadata)
    {
        mFront = ReadSector(aRanges, aAngleMin, aAngleIncrement,
            aRangeMin, aRangeMax, 0.0, Pi / 9.0, true);
        mRight = ReadSector(aRanges, aAngleMin, aAngleIncrement,
            aRangeMin, aRangeMax, -Pi / 2.0, Pi / 36.0, false);
        mFrontRight = ReadSector(aRanges, aAngleMin, aAngleIncrement,
            aRangeMin, aRangeMax, -Pi / 4.0, Pi / 36.0, false);
    }
}

WallFollower::Command WallFollower::CalculateCommand(double aScanAgeSeconds)
{
    Command Result;
    const bool ScanUsable = mFront.Valid && mRight.Valid && mFrontRight.Valid
        && std::isfinite(aScanAgeSeconds)
        && aScanAgeSeconds >= 0.0 && aScanAgeSeconds <= ScanTimeout;

    if (ScanUsable)
    {
        // Hysteresis prevents switching rapidly between turning and driving.
        mTurningLeft = mFront.Distance < mSettings.FrontStopDistance
            || (mTurningLeft && mFront.Distance < mSettings.FrontResumeDistance);

        if (mTurningLeft)
        {
            Result.Angular = mSettings.TurnSpeed;
        }
        else if (mRight.Distance > mSettings.WallLostDistance)
        {
            // A curved right turn reacquires the wall at an outside corner.
            Result.Linear = mSettings.ForwardSpeed;
            Result.Angular = -mSettings.TurnSpeed * 0.5;
        }
        else
        {
            double WallAngle = 0.0;

            if (mFrontRight.Distance < mSettings.WallLostDistance)
            {
                const double DiagonalComponent = mFrontRight.Distance / std::sqrt(2.0);
                WallAngle = std::atan2(DiagonalComponent - mRight.Distance,
                    DiagonalComponent);
            }

            // Positive angular velocity turns left in ROS coordinates.
            const double DistanceError = mSettings.WallDistance - mRight.Distance;
            Result.Angular = std::clamp(
                mSettings.DistanceGain * DistanceError - mSettings.HeadingGain * WallAngle,
                -mSettings.TurnSpeed, mSettings.TurnSpeed);
            const double SpeedFraction = 1.0 - 0.5 * std::abs(Result.Angular)
                / mSettings.TurnSpeed;
            Result.Linear = mSettings.ForwardSpeed * SpeedFraction;
        }
    }

    return Result;
}
