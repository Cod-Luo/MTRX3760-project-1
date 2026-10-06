// wall_follower.cpp - Implementation of right-wall steering and scan processing.

#include "turtlebot3_gazebo/wall_follower.hpp"

#include <algorithm>
#include <cmath>

const double WallFollower::Pi = 3.14159265358979323846;
const double WallFollower::ScanTimeout = 0.5;

// Invalid settings disable movement without using exception handling.
WallFollower::WallFollower(const Settings& aSettings)
    : mSettings(aSettings)
{
    mSettingsValid =
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
}

bool WallFollower::HasValidSettings() const
{
    return mSettingsValid;
}

// Collect usable returns inside the sector, then select its minimum or median.
WallFollower::Sector WallFollower::ReadSector(
    const std::vector<float>& aRanges,
    double aAngleMin,
    double aAngleIncrement,
    double aRangeMin,
    double aRangeMax,
    double aCentre,
    double aHalfWidth,
    bool aUseMinimum)
{
    std::vector<double> ValidRanges;

    for (std::size_t Index = 0; Index < aRanges.size(); ++Index)
    {
        const double Angle =
            aAngleMin + static_cast<double>(Index) * aAngleIncrement;

        // Wrap the difference so equivalent angles lie in the same sector.
        const double Difference =
            std::remainder(Angle - aCentre, 2.0 * Pi);

        const double Range = static_cast<double>(aRanges[Index]);

        if (std::abs(Difference) <= aHalfWidth)
        {
            if (std::isinf(Range) && Range > 0.0)
            {
                // Positive infinity means no return within the sensor's range.
                ValidRanges.push_back(aRangeMax);
            }
            else if (
                std::isfinite(Range)
                && Range >= aRangeMin
                && Range <= aRangeMax)
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

        if (aUseMinimum)
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

// Replace old sectors so an invalid scan cannot reuse previous distances.
void WallFollower::UpdateScan(
    const std::vector<float>& aRanges,
    double aAngleMin,
    double aAngleIncrement,
    double aRangeMin,
    double aRangeMax)
{
    mFront = Sector{};
    mRight = Sector{};
    mFrontRight = Sector{};

    const bool ValidMetadata =
        !aRanges.empty()
        && std::isfinite(aAngleMin)
        && std::isfinite(aAngleIncrement)
        && aAngleIncrement != 0.0
        && std::isfinite(aRangeMin)
        && std::isfinite(aRangeMax)
        && aRangeMin >= 0.0
        && aRangeMax > aRangeMin;

    if (ValidMetadata)
    {
        // Front: 0 degrees, with a half-width of 20 degrees.
        mFront = ReadSector(
            aRanges,
            aAngleMin,
            aAngleIncrement,
            aRangeMin,
            aRangeMax,
            0.0,
            Pi / 9.0,
            true);

        // Right: -90 degrees, with a half-width of 5 degrees.
        mRight = ReadSector(
            aRanges,
            aAngleMin,
            aAngleIncrement,
            aRangeMin,
            aRangeMax,
            -Pi / 2.0,
            Pi / 36.0,
            false);

        // Front-right: -45 degrees, with a half-width of 5 degrees.
        mFrontRight = ReadSector(
            aRanges,
            aAngleMin,
            aAngleIncrement,
            aRangeMin,
            aRangeMax,
            -Pi / 4.0,
            Pi / 36.0,
            false);
    }
}

// Prioritise front clearance, then wall reacquisition, then normal wall tracking.
WallFollower::Command WallFollower::CalculateCommand(double aScanAgeSeconds)
{
    Command Result;

    const bool ScanUsable =
        mSettingsValid
        && mFront.Valid
        && mRight.Valid
        && mFrontRight.Valid
        && std::isfinite(aScanAgeSeconds)
        && aScanAgeSeconds >= 0.0
        && aScanAgeSeconds <= ScanTimeout;

    if (ScanUsable)
    {
        // Separate stop and resume distances prevent rapid behaviour switching.
        mTurningLeft =
            mFront.Distance < mSettings.FrontStopDistance
            || (
                mTurningLeft
                && mFront.Distance < mSettings.FrontResumeDistance);

        if (mTurningLeft)
        {
            // Turn left on the spot until the front becomes clear.
            Result.Angular = mSettings.TurnSpeed;
        }
        else if (mRight.Distance > mSettings.WallLostDistance)
        {
            // Curve right to reacquire the wall around an outside corner.
            Result.Linear = mSettings.ForwardSpeed;
            Result.Angular = -mSettings.TurnSpeed * 0.5;
        }
        else
        {
            double WallAngle = 0.0;

            if (mFrontRight.Distance < mSettings.WallLostDistance)
            {
                // Estimate wall alignment from the right and diagonal returns.
                const double DiagonalComponent =
                    mFrontRight.Distance / std::sqrt(2.0);

                WallAngle = std::atan2(
                    DiagonalComponent - mRight.Distance,
                    DiagonalComponent);
            }

            const double DistanceError =
                mSettings.WallDistance - mRight.Distance;

            // Positive angular velocity turns left in ROS coordinates.
            Result.Angular =
                mSettings.DistanceGain * DistanceError
                - mSettings.HeadingGain * WallAngle;

            // Limit steering to the configured maximum turn speed.
            if (Result.Angular > mSettings.TurnSpeed)
            {
                Result.Angular = mSettings.TurnSpeed;
            }
            else if (Result.Angular < -mSettings.TurnSpeed)
            {
                Result.Angular = -mSettings.TurnSpeed;
            }

            // Reduce forward speed while making stronger steering corrections.
            const double SpeedFraction =
                1.0 - 0.5 * std::abs(Result.Angular) / mSettings.TurnSpeed;

            Result.Linear = mSettings.ForwardSpeed * SpeedFraction;
        }
    }

    return Result;
}