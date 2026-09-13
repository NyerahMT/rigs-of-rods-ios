#include "BeamPhysics.h"
#include "HydroPhysics.h"
#include "SimConstants.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace RoR;

namespace
{
constexpr float PI = 3.14159265358979323846f;

struct BeamLink
{
    NodeCoreState* a = nullptr;
    NodeCoreState* b = nullptr;
    BeamCoreState beam;
};

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

float Length(const PhysicsVec3& value)
{
    return std::sqrt(value.squaredLength());
}

float Distance(const PhysicsVec3& a, const PhysicsVec3& b)
{
    return Length(a - b);
}

BeamLink MakeBeam(NodeCoreState* a, NodeCoreState* b, float spring, float damping)
{
    BeamLink link{};
    link.a = a;
    link.b = b;
    link.beam.rest_length = Distance(a->position, b->position);
    link.beam.spring = spring;
    link.beam.damping = damping;
    return link;
}

float SteeringAngle(const NodeCoreState& axis0, const NodeCoreState& axis1)
{
    const PhysicsVec3 axis = axis1.position - axis0.position;
    return std::atan2(axis.x, axis.z);
}
}

int main()
{
    static_assert(PHYSICS_DT == 0.0005f, "steering probe requires RoR's 2 kHz physics step");

    // --- Structural steering linkage ---
    // Two wheel-axis nodes are constrained around a vertical kingpin. The rack
    // does not assign an angle; it changes a hydro beam length, which rotates
    // the entire carrier through the node/beam geometry.
    NodeCoreState kingpin_low{};
    kingpin_low.position = PhysicsVec3(0.0f, 0.0f, 0.0f);
    kingpin_low.immovable = true;

    NodeCoreState kingpin_high{};
    kingpin_high.position = PhysicsVec3(0.0f, 1.0f, 0.0f);
    kingpin_high.immovable = true;

    NodeCoreState rack_anchor{};
    rack_anchor.position = PhysicsVec3(0.80f, 0.50f, 0.0f);
    rack_anchor.immovable = true;

    NodeCoreState axis0{};
    axis0.position = PhysicsVec3(0.0f, 0.50f, 0.62f);
    axis0.mass = 12.0f;

    NodeCoreState axis1{};
    axis1.position = PhysicsVec3(0.0f, 0.50f, 1.02f);
    axis1.mass = 12.0f;

    std::vector<BeamLink> steering_beams;
    steering_beams.push_back(MakeBeam(&kingpin_low,  &axis0, 900000.0f, 9000.0f));
    steering_beams.push_back(MakeBeam(&kingpin_high, &axis0, 900000.0f, 9000.0f));
    steering_beams.push_back(MakeBeam(&kingpin_low,  &axis1, 900000.0f, 9000.0f));
    steering_beams.push_back(MakeBeam(&kingpin_high, &axis1, 900000.0f, 9000.0f));
    steering_beams.push_back(MakeBeam(&axis0, &axis1, 900000.0f, 9000.0f));

    BeamLink hydro = MakeBeam(&rack_anchor, &axis1, 300000.0f, 6000.0f);
    const float hydro_reference = hydro.beam.rest_length;
    const float initial_axis_length = Distance(axis0.position, axis1.position);

    float steering_state = 0.0f;
    float left_angle = 0.0f;

    for (int step = 0; step < 6000; ++step)
    {
        const float command = (step < 2600) ? 1.0f : -1.0f;
        steering_state = StepHydroSteeringState(
            steering_state, command, 12.0f, false, 1.0f, 1.0f, PHYSICS_DT);

        hydro.beam.rest_length = CalcHydroTargetLength(
            hydro_reference, steering_state, 0.18f, 0.25f, 0.25f);

        for (BeamLink& beam : steering_beams)
        {
            ApplyBeamForce(*beam.a, *beam.b, beam.beam);
        }
        ApplyBeamForce(*hydro.a, *hydro.b, hydro.beam);

        IntegrateNode(kingpin_low, 0.0f, PHYSICS_DT);
        IntegrateNode(kingpin_high, 0.0f, PHYSICS_DT);
        IntegrateNode(rack_anchor, 0.0f, PHYSICS_DT);
        IntegrateNode(axis0, 0.0f, PHYSICS_DT);
        IntegrateNode(axis1, 0.0f, PHYSICS_DT);

        if (step == 2599)
        {
            left_angle = SteeringAngle(axis0, axis1);
        }
    }

    const float right_angle = SteeringAngle(axis0, axis1);
    const float final_axis_length = Distance(axis0.position, axis1.position);

    Require(std::fabs(left_angle) > 3.0f * PI / 180.0f, "positive hydro command rotates the wheel carrier");
    Require(std::fabs(right_angle) > 3.0f * PI / 180.0f, "negative hydro command rotates the wheel carrier");
    Require(left_angle * right_angle < 0.0f, "hydro steering produces opposite left/right carrier angles");
    Require(std::fabs(final_axis_length - initial_axis_length) < 0.015f, "steering structure retains wheel-axis width");

    // --- Sprung front-corner load test ---
    // A wheel-carrier mass is supported by a RoR spring/damper beam. Give it a
    // vertical velocity impulse and verify damping settles the oscillation.
    NodeCoreState chassis_mount{};
    chassis_mount.position = PhysicsVec3(0.0f, 1.05f, 0.0f);
    chassis_mount.immovable = true;

    NodeCoreState suspension_hub{};
    suspension_hub.position = PhysicsVec3(0.0f, 0.55f, 0.0f);
    suspension_hub.velocity = PhysicsVec3(0.0f, 1.5f, 0.0f);
    suspension_hub.mass = 45.0f;
    suspension_hub.force = PhysicsVec3(0.0f, suspension_hub.mass * DEFAULT_GRAVITY, 0.0f);

    BeamLink shock = MakeBeam(&chassis_mount, &suspension_hub, 65000.0f, 6000.0f);
    const float shock_rest = shock.beam.rest_length;
    float peak_speed = 0.0f;

    for (int step = 0; step < 5000; ++step)
    {
        ApplyBeamForce(*shock.a, *shock.b, shock.beam);
        IntegrateNode(chassis_mount, DEFAULT_GRAVITY, PHYSICS_DT);
        IntegrateNode(suspension_hub, DEFAULT_GRAVITY, PHYSICS_DT);
        peak_speed = std::max(peak_speed, std::fabs(suspension_hub.velocity.y));
    }

    const float settled_speed = std::fabs(suspension_hub.velocity.y);
    const float settled_length = Distance(chassis_mount.position, suspension_hub.position);

    Require(peak_speed > 1.0f, "suspension receives the vertical disturbance");
    Require(settled_speed < 0.05f, "shock damping settles wheel-carrier motion");
    Require(std::fabs(settled_length - shock_rest) < 0.08f, "sprung corner remains near its design length under gravity");

    std::cout << "RoR steering/suspension probe passed: carrier "
              << (left_angle * 180.0f / PI) << " deg -> "
              << (right_angle * 180.0f / PI) << " deg, settled shock speed "
              << settled_speed << " m/s.\n";
    return EXIT_SUCCESS;
}
