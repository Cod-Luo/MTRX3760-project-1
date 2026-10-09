// wall_follower.cpp - Right-wall steering decisions.

#include "turtlebot3_gazebo/wall_follower.hpp"

#include <algorithm>
#include <cmath>

const double CWallFollower::ScanTimeout = 0.5;
const double CWallFollower::CornerTurnFraction = 0.5;
const double CWallFollower::MaxSteeringSlowdown = 0.5;
const double CWallFollower::InvalidScanSpeed = 0.01;

bool CWallFollower::Settings::IsValid() const
{
    return std::isfinite(WallDistance)
        && std::isfinite(ForwardSpeed)
        && std::isfinite(TurnSpeed)
        && std::isfinite(FrontStopDistance)
        && std::isfinite(FrontResumeDistance)
        && std::isfinite(WallLostDistance)
        && std::isfinite(DistanceGain)
        && std::isfinite(HeadingGain)
        && WallDistance > 0.0
        && ForwardSpeed > 0.0
        && TurnSpeed > 0.0
        && FrontStopDistance > 0.0
        && FrontResumeDistance > FrontStopDistance
        && WallLostDistance > WallDistance
        && DistanceGain > 0.0
        && HeadingGain >= 0.0;
}

// Invalid settings disable movement without using exception handling.
CWallFollower::CWallFollower(const Settings& aSettings)
    : mSettings(aSettings)
{
}

bool CWallFollower::HasValidSettings() const
{
    return mSettings.IsValid();
}

// Pick one of the three rules, in priority order.
CWallFollower::Command CWallFollower::CalculateCommand(
    const CScanReader& aScan,
    double aScanAgeSeconds)
{
    Command Result;  // Missing input, stale data or invalid settings still stop.
    const DriveStatus Status = CheckInput(aScan, aScanAgeSeconds);

    if (Status == Ready)
    {
        UpdateFrontBlocked(aScan.FrontDistance());

        if (mFrontBlocked)
        {
            Result = TurnLeftInPlace();
        }
        else if (aScan.RightDistance() > mSettings.WallLostDistance)
        {
            Result = CurveRightToFindWall();
        }
        else
        {
            Result = FollowWall(aScan.RightDistance(), aScan.FrontRightDistance());
        }
    }
    else if (Status == InvalidScan && aScan.HasReceivedScan())
    {
        Result = RecoverFromPartialScan(aScan);
    }

    return Result;
}

CWallFollower::Command CWallFollower::RecoverFromPartialScan(const CScanReader& aScan)
{
    if (aScan.HasValidFrontReading())
    {
        UpdateFrontBlocked(aScan.FrontDistance());
    }

    // Missing side readings must not cancel a known blocked-front turn.
    if (mFrontBlocked)
    {
        return TurnLeftInPlace();
    }

    Command Result;
    Result.Angular = -mSettings.TurnSpeed * CornerTurnFraction;

    // If the front is unknown, rotate to obtain a different view without advancing.
    if (aScan.HasValidFrontReading())
    {
        Result.Linear = std::min(InvalidScanSpeed, mSettings.ForwardSpeed);

        if (aScan.HasValidRightReading()
            && aScan.RightDistance() <= mSettings.WallLostDistance)
        {
            // Without a diagonal return, use distance control without a heading estimate.
            const double Diagonal = aScan.HasValidFrontRightReading()
                ? aScan.FrontRightDistance() : mSettings.WallLostDistance;
            Result = FollowWall(aScan.RightDistance(), Diagonal);
            Result.Linear = std::min(Result.Linear, InvalidScanSpeed);
        }
    }

    // No recovery timer: the next usable scan immediately restores normal control.
    return Result;
}

CWallFollower::DriveStatus CWallFollower::CheckInput(
    const CScanReader& aScan, double aScanAgeSeconds) const
{
    DriveStatus Result = Ready;
    if (!mSettings.IsValid())
    {
        Result = InvalidSettings;
    }
    else if (!std::isfinite(aScanAgeSeconds) || aScanAgeSeconds < 0.0)
    {
        Result = InvalidScanTime;
    }
    else if (aScanAgeSeconds > ScanTimeout)
    {
        Result = StaleScan;
    }
    else if (!aScan.HasValidReadings())
    {
        Result = InvalidScan;
    }
    return Result;
}

void CWallFollower::UpdateFrontBlocked(double aFrontDistance)
{
    mFrontBlocked =
        aFrontDistance < mSettings.FrontStopDistance
        || (mFrontBlocked && aFrontDistance < mSettings.FrontResumeDistance);
}

CWallFollower::Command CWallFollower::TurnLeftInPlace() const
{
    Command Result;
    Result.Angular = mSettings.TurnSpeed;  // Positive turns left in ROS.
    return Result;
}

CWallFollower::Command CWallFollower::CurveRightToFindWall() const
{
    Command Result;
    Result.Linear = mSettings.ForwardSpeed;
    Result.Angular = -mSettings.TurnSpeed * CornerTurnFraction;
    return Result;
}

CWallFollower::Command CWallFollower::FollowWall(
    double aRightDistance,
    double aFrontRightDistance) const
{
    double WallAngle = 0.0;

    if (aFrontRightDistance < mSettings.WallLostDistance)
    {
        // Estimate wall alignment from the right and diagonal returns.
        const double DiagonalComponent = aFrontRightDistance / std::sqrt(2.0);

        WallAngle = std::atan2(
            DiagonalComponent - aRightDistance,
            DiagonalComponent);
    }

    const double DistanceError = mSettings.WallDistance - aRightDistance;

    Command Result;
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

    // Slow down while making stronger steering corrections.
    const double SpeedFraction =
        1.0 - MaxSteeringSlowdown * std::abs(Result.Angular) / mSettings.TurnSpeed;

    Result.Linear = mSettings.ForwardSpeed * SpeedFraction;
    return Result;
}
