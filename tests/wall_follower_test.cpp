// wall_follower_test.cpp - Steering checks and a closed-loop ray-cast maze test.
// This idealised model is a development test, not Gazebo/A1 evidence.
#include "wall_follower_tests.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>


const double CWallFollowerTests::Pi = 3.14159265358979323846;
const double CWallFollowerTests::StepSeconds = 0.05;  // 20 Hz control.
const double CWallFollowerTests::MinimumRange = 0.12;
const double CWallFollowerTests::MaximumRange = 3.5;
const double CWallFollowerTests::WallHalfThickness = 0.05;
const double CWallFollowerTests::MinimumSafeClearance = 0.18;
const int CWallFollowerTests::LaserSamples = 360;
const int CWallFollowerTests::MaximumSteps = 18000;  // 900 simulated seconds.

// Report a failed check without exception handling.
bool CWallFollowerTests::Require(bool aCondition, const std::string& aMessage)
{
    if (!aCondition)
    {
        std::cerr << "FAIL: " << aMessage << '\n';
    }

    return aCondition;
}

std::vector<float> CWallFollowerTests::Scan(double aRight, double aFront, int aSamples)
{
    std::vector<float> Ranges(static_cast<std::size_t>(aSamples), static_cast<float>(MaximumRange));

    for (int Index = 0; Index < aSamples; ++Index)
    {
        const double Angle = -Pi + 2.0 * Pi * static_cast<double>(Index) / aSamples;

        if (Angle < 0.0)
        {
            Ranges[static_cast<std::size_t>(Index)] = static_cast<float>(
                std::min(MaximumRange, aRight / -std::sin(Angle)));
        }

        if (std::abs(Angle) < Pi / 8.0)
        {
            Ranges[static_cast<std::size_t>(Index)] = static_cast<float>(aFront);
        }
    }

    return Ranges;
}

void CWallFollowerTests::Feed(CScanReader& aReader, const std::vector<float>& aRanges)
{
    aReader.Update(aRanges, -Pi, 2.0 * Pi / static_cast<double>(aRanges.size()),
        MinimumRange, MaximumRange);
}

bool CWallFollowerTests::CheckSteering() const
{
    bool Result = true;
    CWallFollower Follower(CWallFollower::Settings{});
    CScanReader Reader;
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Linear == 0.0, "Motion before first scan") && Result;
    Feed(Reader, Scan(0.35));
    auto Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear > 0.0 && std::abs(Command.Angular) < 0.03,
        "Aligned wall should allow forward motion") && Result;
    Feed(Reader, Scan(0.20));
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Angular > 0.0, "Too close should turn left") && Result;
    Feed(Reader, Scan(0.60));
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Angular < 0.0, "Too far should turn right") && Result;
    Feed(Reader, Scan(0.35, 0.25));
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular > 0.0, "Blocked front should turn in place") && Result;
    Feed(Reader, Scan(0.35, 0.45));
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Linear == 0.0, "Corner hysteresis failed") && Result;
    Feed(Reader, Scan(0.35, 0.60));
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Linear > 0.0, "Corner turn should release") && Result;
    Feed(Reader, Scan(3.5));
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Angular < 0.0, "Lost wall should turn right") && Result;
    Command = Follower.CalculateCommand(Reader, 0.51);
    Result = Require(Command.Linear == 0.0 && Command.Angular == 0.0, "Stale scan should stop") && Result;
    if (Result)
    {
        std::cout << "PASS: steering, front hysteresis and wall loss\n";
    }
    return Result;
}

bool CWallFollowerTests::CheckScanValidation() const
{
    bool Result = true;
    CWallFollower Follower(CWallFollower::Settings{});
    CScanReader Reader;
    Feed(Reader, std::vector<float>(LaserSamples, std::numeric_limits<float>::quiet_NaN()));
    auto Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular < 0.0,
        "Fresh NaN scan should search right without advancing") && Result;
    Feed(Reader, std::vector<float>(LaserSamples, -std::numeric_limits<float>::infinity()));
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular < 0.0,
        "An unusable negative-infinity scan should rotate to search") && Result;
    Feed(Reader, std::vector<float>(LaserSamples, std::numeric_limits<float>::infinity()));
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Angular < 0.0, "Positive infinity should mean clear space") && Result;
    Feed(Reader, Scan(0.35, 3.5, 720));
    Result = Require(std::abs(Follower.CalculateCommand(Reader, 0.0).Angular) < 0.03, "720-sample scan failed") && Result;
    auto Wrapped = Scan(0.20);
    std::rotate(Wrapped.begin(), Wrapped.begin() + LaserSamples / 2, Wrapped.end());
    Reader.Update(Wrapped, 0.0, 2.0 * Pi / LaserSamples, MinimumRange, MaximumRange);
    Result = Require(Follower.CalculateCommand(Reader, 0.0).Angular > 0.0, "0-to-2pi angle wrapping failed") && Result;
    Reader.Update(Wrapped, 0.0, 0.0, MinimumRange, MaximumRange);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular < 0.0,
        "Fresh invalid metadata should rotate without advancing") && Result;
    if (Result)
    {
        std::cout << "PASS: range validation, metadata, resolutions and angle wrapping\n";
    }
    return Result;
}

bool CWallFollowerTests::CheckInputStatus() const
{
    bool Result = true;
    CWallFollower Follower(CWallFollower::Settings{});
    CScanReader Reader;
    CWallFollower::Command Command;
    CWallFollower::Settings Settings;
    Settings.WallDistance = -1.0;
    CWallFollower Invalid(Settings);
    Feed(Reader, Scan(0.35));
    Command = Invalid.CalculateCommand(Reader, 0.0);
    Result = Require(!Invalid.HasValidSettings(), "Invalid settings were accepted") && Result;
    Result = Require(Command.Linear == 0.0 && Command.Angular == 0.0,
        "Invalid settings should disable motion") && Result;
    Feed(Reader, Scan(0.35));
    Result = Require(Follower.CheckInput(Reader, 0.5) == CWallFollower::Ready,
        "Scan at the timeout boundary should be usable") && Result;
    Result = Require(Follower.CheckInput(Reader, 0.51) == CWallFollower::StaleScan,
        "Stale scan status should explain the stop") && Result;
    const std::vector<double> InvalidAges = {
        -0.01, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()
    };
    for (const auto Age : InvalidAges)
    {
        Command = Follower.CalculateCommand(Reader, Age);
        Result = Require(Follower.CheckInput(Reader, Age) == CWallFollower::InvalidScanTime
            && Command.Linear == 0.0 && Command.Angular == 0.0,
            "Invalid scan age must report the reason and stop") && Result;
    }
    Result = Require(Invalid.CheckInput(Reader, 0.0) == CWallFollower::InvalidSettings,
        "Invalid settings status should explain the stop") && Result;
    Reader.Update({}, 0.0, 0.0, MinimumRange, MaximumRange);
    Result = Require(Follower.CheckInput(Reader, 0.0) == CWallFollower::InvalidScan,
        "Invalid scan status should explain the stop") && Result;
    if (Result)
    {
        std::cout << "PASS: validation status, timeout boundary and invalid scan ages\n";
    }
    return Result;
}

bool CWallFollowerTests::CheckInvalidScanRecovery() const
{
    bool Result = true;
    CWallFollower Follower(CWallFollower::Settings{});
    CScanReader Reader;
    auto Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular == 0.0,
        "Recovery must not start before receiving a scan") && Result;

    const std::vector<float> Invalid(LaserSamples, std::numeric_limits<float>::quiet_NaN());
    for (int Step = 0; Step < 300; ++Step)
    {
        Feed(Reader, Invalid);
        Command = Follower.CalculateCommand(Reader, 0.0);
        Result = Require(Command.Linear == 0.0 && Command.Angular < 0.0,
            "Repeated fresh invalid scans must keep searching without a recovery time limit") && Result;
    }

    const std::vector<double> StopAges = {
        0.51, -0.01, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()
    };
    for (const auto Age : StopAges)
    {
        Command = Follower.CalculateCommand(Reader, Age);
        Result = Require(Command.Linear == 0.0 && Command.Angular == 0.0,
            "Unusable scans must not bypass stale or invalid-time stops") && Result;
    }

    auto Partial = Scan(0.35);
    for (int Index = 128; Index <= 142; ++Index)
    {
        Partial[static_cast<std::size_t>(Index)] = std::numeric_limits<float>::quiet_NaN();
    }
    Feed(Reader, Partial);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.01 && std::abs(Command.Angular) < 0.03,
        "A missing diagonal should still use the valid right-wall distance") && Result;

    Partial = Scan(0.20);
    for (int Index = 128; Index <= 142; ++Index)
    {
        Partial[static_cast<std::size_t>(Index)] = std::numeric_limits<float>::quiet_NaN();
    }
    Feed(Reader, Partial);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.01 && Command.Angular > 0.0,
        "A missing diagonal must not discard right-wall distance corrections") && Result;

    Partial = Scan(3.5);
    for (int Index = 128; Index <= 142; ++Index)
    {
        Partial[static_cast<std::size_t>(Index)] = std::numeric_limits<float>::quiet_NaN();
    }
    Feed(Reader, Partial);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.01 && Command.Angular < 0.0,
        "Losing the wall with an invalid diagonal must still turn right") && Result;

    Partial = Scan(0.35, 0.25);
    for (int Index = 80; Index <= 100; ++Index)
    {
        Partial[static_cast<std::size_t>(Index)] = std::numeric_limits<float>::quiet_NaN();
    }
    Feed(Reader, Partial);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Reader.HasValidFrontReading() && !Reader.HasValidRightReading()
        && Command.Linear == 0.0 && Command.Angular > 0.0,
        "At a T-junction, a missing right sector must not cancel a blocked-front turn") && Result;

    Feed(Reader, Invalid);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular > 0.0,
        "An unknown front must preserve a previously blocked-front turn") && Result;

    Partial = Scan(0.35, 0.45);
    for (int Index = 80; Index <= 100; ++Index)
    {
        Partial[static_cast<std::size_t>(Index)] = std::numeric_limits<float>::quiet_NaN();
    }
    Feed(Reader, Partial);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular > 0.0,
        "Partial scans must preserve blocked-front hysteresis") && Result;

    Partial = Scan(0.35, 0.60);
    for (int Index = 80; Index <= 100; ++Index)
    {
        Partial[static_cast<std::size_t>(Index)] = std::numeric_limits<float>::quiet_NaN();
    }
    Feed(Reader, Partial);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.01 && Command.Angular < 0.0,
        "Once the front clears, a missing right wall must trigger a rightward search") && Result;

    Feed(Reader, Invalid);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular < 0.0,
        "With no blocked-front state, an unknown front must search without advancing") && Result;

    Feed(Reader, Scan(0.35));
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear > 0.01 && std::abs(Command.Angular) < 0.03,
        "A valid scan must restore normal wall following immediately") && Result;

    CWallFollower::Settings SlowSettings;
    SlowSettings.ForwardSpeed = 0.005;
    CWallFollower SlowFollower(SlowSettings);
    Feed(Reader, Partial);
    Command = SlowFollower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.005,
        "Recovery must not exceed a lower configured forward speed") && Result;
    Reader.Update({}, 0.0, 0.0, MinimumRange, MaximumRange);
    Command = Follower.CalculateCommand(Reader, 0.0);
    Result = Require(Command.Linear == 0.0 && Command.Angular < 0.0,
        "An empty received scan must search without advancing") && Result;

    if (Result)
    {
        std::cout << "PASS: partial-scan corner turns, persistent search and return to normal control\n";
    }
    return Result;
}


double CWallFollowerTests::RayDistance(double aX, double aY, double aAngle, const Segment& aWall)
{
    const double Dx = std::cos(aAngle);
    const double Dy = std::sin(aAngle);
    const double Wx = aWall.X2 - aWall.X1;
    const double Wy = aWall.Y2 - aWall.Y1;
    const double Denominator = Dx * Wy - Dy * Wx;
    double Distance = MaximumRange;

    if (std::abs(Denominator) > 1e-9)
    {
        const double Ox = aWall.X1 - aX;
        const double Oy = aWall.Y1 - aY;
        const double T = (Ox * Wy - Oy * Wx) / Denominator;
        const double U = (Ox * Dy - Oy * Dx) / Denominator;

        if (T >= 0.0 && U >= 0.0 && U <= 1.0)
        {
            Distance = std::min(Distance, T);
        }
    }

    return Distance;
}

double CWallFollowerTests::Clearance(double aX, double aY, const Segment& aWall)
{
    const double Wx = aWall.X2 - aWall.X1;
    const double Wy = aWall.Y2 - aWall.Y1;
    const double U = std::min(1.0, std::max(0.0,
        ((aX - aWall.X1) * Wx + (aY - aWall.Y1) * Wy) / (Wx * Wx + Wy * Wy)));
    return std::hypot(aX - aWall.X1 - U * Wx, aY - aWall.Y1 - U * Wy);
}

bool CWallFollowerTests::CheckMaze(const SensorProfile& aProfile) const
{
    bool Result = true;
    // Same wall centrelines as the supplied Gazebo maze. No route is given to the controller.
    const std::vector<Segment> Walls = {
        {-3.0, -3.0, 3.0, -3.0}, {-3.0, -3.0, -3.0, 3.0},
        {-3.0, 3.0, 3.0, 3.0}, {3.0, -3.0, 3.0, 1.6},
        {3.0, 2.6, 3.0, 3.0}, {-3.0, -1.0, 2.0, -1.0},
        {-2.0, 1.0, 3.0, 1.0}, {3.0, 1.6, 4.5, 1.6},
        {3.0, 2.6, 4.5, 2.6}
    };
    CWallFollower Follower(CWallFollower::Settings{});
    CScanReader Reader;
    double X = -2.4;
    double Y = -2.55;
    double Heading = 0.0;
    double MinimumClearance = MaximumRange;
    bool ReachedExit = false;
    std::mt19937 Random(42);
    std::normal_distribution<double> Noise(0.0, 0.01);

    for (int Step = 0; Step < MaximumSteps && !ReachedExit && Result; ++Step)
    {
        if (Step % aProfile.ScanEverySteps == 0)
        {
            std::vector<float> Ranges(LaserSamples, static_cast<float>(MaximumRange));
            const double SensorX = X + aProfile.OffsetX * std::cos(Heading);
            const double SensorY = Y + aProfile.OffsetX * std::sin(Heading);
            const double AngleMin = aProfile.Noisy ? 0.0 : -Pi;
            const double AngleIncrement = aProfile.Noisy ? 6.28 / (LaserSamples - 1) : 2.0 * Pi / LaserSamples;

            for (int Index = 0; Index < LaserSamples; ++Index)
            {
                const double Angle = Heading + AngleMin + AngleIncrement * Index;
                double Distance = MaximumRange;

                for (const auto& Wall : Walls)
                {
                    // Approximate 0.1 m wall thickness along the ray.
                    Distance = std::min(Distance, std::max(MinimumRange,
                        RayDistance(SensorX, SensorY, Angle, Wall) - WallHalfThickness));
                }

                if (aProfile.Noisy)
                {
                    Distance = std::min(MaximumRange, std::max(MinimumRange, Distance + Noise(Random)));
                }

                Ranges[static_cast<std::size_t>(Index)] = static_cast<float>(Distance);
            }

            Reader.Update(Ranges, AngleMin, AngleIncrement, MinimumRange, MaximumRange);
        }

        const auto Command = Follower.CalculateCommand(Reader, (Step % aProfile.ScanEverySteps) * StepSeconds);
        Heading += Command.Angular * StepSeconds;
        X += Command.Linear * std::cos(Heading) * StepSeconds;
        Y += Command.Linear * std::sin(Heading) * StepSeconds;

        for (const auto& Wall : Walls)
        {
            MinimumClearance = std::min(MinimumClearance, Clearance(X, Y, Wall) - WallHalfThickness);
        }

        if (MinimumClearance <= MinimumSafeClearance)
        {
            std::cerr << "Collision at step " << Step << ": " << X << ", " << Y
                << ", heading " << Heading << ", command " << Command.Linear
                << ", " << Command.Angular << '\n';
        }

        Result = Require(MinimumClearance > MinimumSafeClearance, "Idealised maze collision") && Result;
        ReachedExit = X > 4.0 && Y > 1.6 && Y < 2.6;
    }

    std::cout << "Maze final position: " << X << ", " << Y
        << "; minimum clearance: " << MinimumClearance << " m\n";
    Result = Require(ReachedExit, "Idealised maze exit was not reached within 900 simulated seconds") && Result;
    if (Result)
    {
        std::cout << "PASS: sensor-driven S-maze traversal (" << aProfile.Name
            << ", " << 1.0 / (StepSeconds * aProfile.ScanEverySteps) << " Hz"
            << ", sensor x = " << aProfile.OffsetX << " m; development test only)\n";
    }
    return Result;
}

// Retain the previous platform profile and add the selected Burger configuration.
int CWallFollowerTests::Run() const
{
    const std::vector<SensorProfile> Profiles = {
        {"idealised readings", 1, 0.0, false},
        {"Waffle Pi, seeded noise", 2, -0.064, true},  // 10 Hz laser.
        {"Burger camera, seeded noise", 4, -0.032, true}  // 5 Hz laser.
    };
    bool Passed = CheckSteering();
    Passed = CheckScanValidation() && Passed;
    Passed = CheckInputStatus() && Passed;
    Passed = CheckInvalidScanRecovery() && Passed;
    for (const auto& Profile : Profiles)
    {
        Passed = CheckMaze(Profile) && Passed;
    }
    int Result = 0;
    if (!Passed)
    {
        Result = 1;
    }
    return Result;
}

int main()
{
    CWallFollowerTests Tests;
    return Tests.Run();
}
