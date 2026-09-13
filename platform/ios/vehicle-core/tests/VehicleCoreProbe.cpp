#include "BeamPhysics.h"
#include "Differentials.h"
#include "GroundFriction.h"
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

float Distance(const PhysicsVec3& a, const PhysicsVec3& b)
{
    const PhysicsVec3 d = a - b;
    return std::sqrt(d.squaredLength());
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

    {
        GroundFrictionParams asphalt{};
        asphalt.adhesion_velocity = 0.5f;
        asphalt.static_friction = 1.2f;
        asphalt.sliding_friction = 0.85f;
        asphalt.hydrodynamic_friction = 0.02f;
        asphalt.stribeck_velocity = 0.35f;
        asphalt.stribeck_alpha = 2.0f;

        const float static_low = CalcStaticFrictionScale(0.05f, asphalt);
        const float static_high = CalcStaticFrictionScale(0.40f, asphalt);
        const float sliding_low = CalcStribeckFrictionScale(0.40f, asphalt);
        const float sliding_high = CalcStribeckFrictionScale(8.0f, asphalt);

        Require(std::isfinite(static_low) && std::isfinite(static_high), "static friction remains finite");
        Require(std::isfinite(sliding_low) && std::isfinite(sliding_high), "Stribeck friction remains finite");
        Require(static_low < 0.0f && static_high < static_low, "static friction increasingly opposes slip");
        Require(sliding_low < 0.0f && sliding_high < 0.0f, "sliding friction opposes slip");
    }

    {
        NodeCoreState anchor{};
        anchor.position = PhysicsVec3(0.0f, 0.0f, 0.0f);
        anchor.mass = 50.0f;
        anchor.immovable = true;

        NodeCoreState moving{};
        moving.position = PhysicsVec3(1.25f, 0.0f, 0.0f);
        moving.mass = 50.0f;

        BeamCoreState beam{};
        beam.rest_length = 1.0f;
        beam.spring = 20000.0f;
        beam.damping = 1500.0f;

        ApplyBeamForce(anchor, moving, beam);
        Require(beam.stress < 0.0f, "stretched beam produces restoring tension");
        Require(NearlyEqual(anchor.force.x + moving.force.x, 0.0f, 0.05f), "beam forces are equal and opposite");

        anchor.force = PhysicsVec3();
        moving.force = PhysicsVec3();

        const float initial_error = std::fabs(Distance(anchor.position, moving.position) - beam.rest_length);
        for (int step = 0; step < 4000; ++step)
        {
            IntegrateNode(anchor, 0.0f, PHYSICS_DT);
            IntegrateNode(moving, 0.0f, PHYSICS_DT);
            ApplyBeamForce(anchor, moving, beam);
        }

        const float final_error = std::fabs(Distance(anchor.position, moving.position) - beam.rest_length);
        Require(std::isfinite(final_error), "node/beam integration remains finite");
        Require(final_error < initial_error * 0.15f, "damped beam returns toward rest length");
        Require(std::fabs(moving.velocity.x) < 0.2f, "damped beam settles instead of exploding");
    }

    std::cout << "RoR vehicle-core probe passed at " << (1.0f / PHYSICS_DT)
              << " Hz with differential, ground-friction, and node/beam dynamics checks.\n";
    return EXIT_SUCCESS;
}
