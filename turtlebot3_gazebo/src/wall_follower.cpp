// wall_follower.cpp - Right-wall steering decisions.

#include "turtlebot3_gazebo/wall_follower.hpp"

#include <algorithm>
#include <cmath>

const double CWallFollower::ScanTimeout = 0.5;

// Invalid settings disable movement without using exception handling.
CWallFollower::CWallFollower(const Settings& aSettings)
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

bool CWallFollower::HasValidSettings() const
{
    return mSettingsValid;
}

// Collect usable returns inside the sector, then select its minimum or median.
// Replace old sectors so an invalid scan cannot reuse previous distances.
// Prioritise front clearance, then wall reacquisition, then normal wall tracking.
CWallFollower::Command CWallFollower::CalculateCommand(
    const CScanReader& aScan,
    double aScanAgeSeconds)
{
    Command Result;

    const bool ScanUsable =
        mSettingsValid
        && aScan.HasValidReadings()
        && std::isfinite(aScanAgeSeconds)
        && aScanAgeSeconds >= 0.0
        && aScanAgeSeconds <= ScanTimeout;

    if (ScanUsable)
    {
        const double Front = aScan.FrontDistance();
        const double Right = aScan.RightDistance();
        const double FrontRight = aScan.FrontRightDistance();

        // Separate stop and resume distances prevent rapid behaviour switching.
        mTurningLeft =
            Front < mSettings.FrontStopDistance
            || (
                mTurningLeft
                && Front < mSettings.FrontResumeDistance);

        if (mTurningLeft)
        {
            // Turn left on the spot until the front becomes clear.
            Result.Angular = mSettings.TurnSpeed;
        }
        else if (Right > mSettings.WallLostDistance)
        {
            // Curve right to reacquire the wall around an outside corner.
            Result.Linear = mSettings.ForwardSpeed;
            Result.Angular = -mSettings.TurnSpeed * 0.5;
        }
        else
        {
            double WallAngle = 0.0;

            if (FrontRight < mSettings.WallLostDistance)
            {
                // Estimate wall alignment from the right and diagonal returns.
                const double DiagonalComponent =
                    FrontRight / std::sqrt(2.0);

                WallAngle = std::atan2(
                    DiagonalComponent - Right,
                    DiagonalComponent);
            }

            const double DistanceError =
                mSettings.WallDistance - Right;

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