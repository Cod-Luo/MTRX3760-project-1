// wall_follower_test.cpp - Steering checks and a closed-loop ray-cast maze test.
// This idealised model is a development test, not Gazebo/A1 evidence.
#include "turtlebot3_gazebo/wall_follower.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    constexpr double Pi = 3.14159265358979323846;

    void Require(bool aCondition, const std::string& aMessage)
    {
        if (!aCondition)
        {
            throw std::runtime_error(aMessage);
        }
    }

    std::vector<float> Scan(double aRight, double aFront = 3.5, int aSamples = 360)
    {
        std::vector<float> Ranges(static_cast<std::size_t>(aSamples), 3.5f);

        for (int Index = 0; Index < aSamples; ++Index)
        {
            const double Angle = -Pi + 2.0 * Pi * static_cast<double>(Index) / aSamples;

            if (Angle < 0.0)
            {
                Ranges[static_cast<std::size_t>(Index)] = static_cast<float>(
                    std::min(3.5, aRight / -std::sin(Angle)));
            }

            if (std::abs(Angle) < Pi / 8.0)
            {
                Ranges[static_cast<std::size_t>(Index)] = static_cast<float>(aFront);
            }
        }

        return Ranges;
    }

    void Feed(WallFollower& aFollower, const std::vector<float>& aRanges)
    {
        aFollower.UpdateScan(aRanges, -Pi, 2.0 * Pi / static_cast<double>(aRanges.size()),
            0.12, 3.5);
    }

    void CheckSteering()
    {
        WallFollower Follower(WallFollower::Settings{});
        Require(Follower.CalculateCommand(0.0).Linear == 0.0, "Motion before first scan");
        Feed(Follower, Scan(0.35));
        auto Command = Follower.CalculateCommand(0.0);
        Require(Command.Linear > 0.0 && std::abs(Command.Angular) < 0.03,
            "Aligned wall should allow forward motion");
        Feed(Follower, Scan(0.20));
        Require(Follower.CalculateCommand(0.0).Angular > 0.0, "Too close should turn left");
        Feed(Follower, Scan(0.60));
        Require(Follower.CalculateCommand(0.0).Angular < 0.0, "Too far should turn right");
        Feed(Follower, Scan(0.35, 0.25));
        Command = Follower.CalculateCommand(0.0);
        Require(Command.Linear == 0.0 && Command.Angular > 0.0, "Blocked front should turn in place");
        Feed(Follower, Scan(0.35, 0.45));
        Require(Follower.CalculateCommand(0.0).Linear == 0.0, "Corner hysteresis failed");
        Feed(Follower, Scan(0.35, 0.60));
        Require(Follower.CalculateCommand(0.0).Linear > 0.0, "Corner turn should release");
        Feed(Follower, Scan(3.5));
        Require(Follower.CalculateCommand(0.0).Angular < 0.0, "Lost wall should turn right");
        Command = Follower.CalculateCommand(0.51);
        Require(Command.Linear == 0.0 && Command.Angular == 0.0, "Stale scan should stop");
        Feed(Follower, std::vector<float>(360, std::numeric_limits<float>::quiet_NaN()));
        Require(Follower.CalculateCommand(0.0).Linear == 0.0, "NaN scan should stop");
        Feed(Follower, std::vector<float>(360, -std::numeric_limits<float>::infinity()));
        Require(Follower.CalculateCommand(0.0).Linear == 0.0, "Negative infinity should stop");
        Feed(Follower, std::vector<float>(360, std::numeric_limits<float>::infinity()));
        Require(Follower.CalculateCommand(0.0).Angular < 0.0, "Positive infinity should mean clear space");
        Feed(Follower, Scan(0.35, 3.5, 720));
        Require(std::abs(Follower.CalculateCommand(0.0).Angular) < 0.03, "720-sample scan failed");
        auto Wrapped = Scan(0.20);
        std::rotate(Wrapped.begin(), Wrapped.begin() + 180, Wrapped.end());
        Follower.UpdateScan(Wrapped, 0.0, 2.0 * Pi / 360.0, 0.12, 3.5);
        Require(Follower.CalculateCommand(0.0).Angular > 0.0, "0-to-2pi angle wrapping failed");
        Follower.UpdateScan(Wrapped, 0.0, 0.0, 0.12, 3.5);
        Require(Follower.CalculateCommand(0.0).Linear == 0.0, "Invalid scan metadata should stop");
        WallFollower::Settings Settings;
        Settings.WallDistance = -1.0;
        bool Rejected = false;

        try
        {
            const WallFollower Invalid(Settings);
        }
        catch (const std::invalid_argument&)
        {
            Rejected = true;
        }

        Require(Rejected, "Invalid settings were accepted");
        std::cout << "PASS: steering, corners, sensor validation, timeout and angle wrapping\n";
    }

    struct Segment
    {
        double X1;
        double Y1;
        double X2;
        double Y2;
    };

    double RayDistance(double aX, double aY, double aAngle, const Segment& aWall)
    {
        const double Dx = std::cos(aAngle);
        const double Dy = std::sin(aAngle);
        const double Wx = aWall.X2 - aWall.X1;
        const double Wy = aWall.Y2 - aWall.Y1;
        const double Denominator = Dx * Wy - Dy * Wx;
        double Distance = 3.5;

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

    double Clearance(double aX, double aY, const Segment& aWall)
    {
        const double Wx = aWall.X2 - aWall.X1;
        const double Wy = aWall.Y2 - aWall.Y1;
        const double U = std::clamp(((aX - aWall.X1) * Wx + (aY - aWall.Y1) * Wy)
            / (Wx * Wx + Wy * Wy), 0.0, 1.0);
        return std::hypot(aX - aWall.X1 - U * Wx, aY - aWall.Y1 - U * Wy);
    }

    void CheckMaze(bool aModelSensor)
    {
        // Same wall centrelines as the supplied Gazebo maze. No route is given to the controller.
        const std::vector<Segment> Walls = {
            {-3.0, -3.0, 3.0, -3.0}, {-3.0, -3.0, -3.0, 3.0},
            {-3.0, 3.0, 3.0, 3.0}, {3.0, -3.0, 3.0, 1.6},
            {3.0, 2.6, 3.0, 3.0}, {-3.0, -1.0, 2.0, -1.0},
            {-2.0, 1.0, 3.0, 1.0}, {3.0, 1.6, 4.5, 1.6},
            {3.0, 2.6, 4.5, 2.6}
        };
        WallFollower Follower(WallFollower::Settings{});
        double X = -2.4;
        double Y = -2.55;
        double Heading = 0.0;
        double MinimumClearance = 3.5;
        constexpr double StepSeconds = 0.05;
        bool ReachedExit = false;
        std::mt19937 Random(42);
        std::normal_distribution<double> Noise(0.0, 0.01);

        for (int Step = 0; Step < 18000 && !ReachedExit; ++Step)
        {
            if (!aModelSensor || Step % 2 == 0)
            {
                std::vector<float> Ranges(360, 3.5f);
                const double SensorX = X - (aModelSensor ? 0.064 * std::cos(Heading) : 0.0);
                const double SensorY = Y - (aModelSensor ? 0.064 * std::sin(Heading) : 0.0);
                const double AngleMin = aModelSensor ? 0.0 : -Pi;
                const double AngleIncrement = aModelSensor ? 6.28 / 359.0 : 2.0 * Pi / 360.0;

                for (int Index = 0; Index < 360; ++Index)
                {
                    const double Angle = Heading + AngleMin + AngleIncrement * Index;
                    double Distance = 3.5;

                    for (const auto& Wall : Walls)
                    {
                        // Approximate 0.1 m wall thickness along the ray.
                        Distance = std::min(Distance, std::max(0.12,
                            RayDistance(SensorX, SensorY, Angle, Wall) - 0.05));
                    }

                    if (aModelSensor)
                    {
                        Distance = std::clamp(Distance + Noise(Random), 0.12, 3.5);
                    }

                    Ranges[static_cast<std::size_t>(Index)] = static_cast<float>(Distance);
                }

                Follower.UpdateScan(Ranges, AngleMin, AngleIncrement, 0.12, 3.5);
            }

            const auto Command = Follower.CalculateCommand(aModelSensor && Step % 2 != 0
                ? StepSeconds : 0.0);
            Heading += Command.Angular * StepSeconds;
            X += Command.Linear * std::cos(Heading) * StepSeconds;
            Y += Command.Linear * std::sin(Heading) * StepSeconds;

            for (const auto& Wall : Walls)
            {
                MinimumClearance = std::min(MinimumClearance, Clearance(X, Y, Wall) - 0.05);
            }

            if (MinimumClearance <= 0.18)
            {
                std::cerr << "Collision at step " << Step << ": " << X << ", " << Y
                    << ", heading " << Heading << ", command " << Command.Linear
                    << ", " << Command.Angular << '\n';
            }

            Require(MinimumClearance > 0.18, "Idealised maze collision");
            ReachedExit = X > 4.0 && Y > 1.6 && Y < 2.6;
        }

        std::cout << "Maze final position: " << X << ", " << Y
            << "; minimum clearance: " << MinimumClearance << " m\n";
        Require(ReachedExit, "Idealised maze exit was not reached within 900 simulated seconds");
        std::cout << "PASS: sensor-driven S-maze traversal ("
            << (aModelSensor ? "mounted laser, 10 Hz, seeded noise" : "idealised model")
            << "; development test only)\n";
    }
}

int main()
{
    int Result = 0;

    try
    {
        CheckSteering();
        CheckMaze(false);
        CheckMaze(true);
    }
    catch (const std::exception& Error)
    {
        std::cerr << "FAIL: " << Error.what() << '\n';
        Result = 1;
    }

    return Result;
}
