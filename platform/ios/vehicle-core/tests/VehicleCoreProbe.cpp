#include "Differentials.h"
#include "SimConstants.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace RoR;

namespace
{
bool NearlyEqual(float a, float b, float epsilon = 0.01f)
{
    return std::fabs(a - b) <= epsilon;
}

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

DifferentialData MakeData(float speed0, float speed1, float torque)
{
    DifferentialData data{};
    data.speed[0] = speed0;
    data.speed[1] = speed1;
    data.in_torque = torque;
    data.dt = PHYSICS_DT;
    return data;
}
}

int main()
{
    static_assert(PHYSICS_DT == 0.0005f, "RoR vehicle core must retain the 2 kHz physics timestep");

    {
        DifferentialData data = MakeData(10.0f, 10.0f, 1000.0f);
        Differential::CalcSeparateDiff(data);
        Require(NearlyEqual(data.out_torque[0], 500.0f), "split diff left torque");
        Require(NearlyEqual(data.out_torque[1], 500.0f), "split diff right torque");
    }

    {
        DifferentialData data = MakeData(12.0f, 6.0f, 1000.0f);
        Differential::CalcOpenDiff(data);
        Require(NearlyEqual(data.out_torque[0] + data.out_torque[1], 1000.0f), "open diff conserves input torque");
        Require(data.out_torque[0] > data.out_torque[1], "open diff responds to wheel-speed difference");
    }

    {
        DifferentialData data = MakeData(12.0f, 6.0f, 1000.0f);
        Differential::CalcViscousDiff(data);
        Require(NearlyEqual(data.out_torque[0] + data.out_torque[1], 1000.0f), "viscous diff conserves input torque");
        Require(data.out_torque[0] < data.out_torque[1], "viscous diff resists wheel-speed difference");
    }

    {
        DifferentialData data = MakeData(12.0f, 6.0f, 1000.0f);
        Differential::CalcLockedDiff(data);
        Require(NearlyEqual(data.out_torque[0] + data.out_torque[1], 1000.0f), "locked diff conserves input torque");
        Require(data.delta_rotation > 0.0f, "locked diff accumulates axle rotation delta");
    }

    {
        Differential diff;
        diff.AddDifferentialType(LOCKED_DIFF);
        diff.AddDifferentialType(VISCOUS_DIFF);
        Require(diff.GetActiveDiffType() == LOCKED_DIFF, "locked diff starts active");
        diff.ToggleDifferentialMode();
        Require(diff.GetActiveDiffType() == VISCOUS_DIFF, "diff mode toggles");
    }

    std::cout << "RoR vehicle-core probe passed at " << (1.0f / PHYSICS_DT) << " Hz physics cadence.\n";
    return EXIT_SUCCESS;
}
