#include "SimConstants.h"
#include "WheelPhysics.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace RoR;

namespace
{
constexpr int RAYS = 12;
constexpr int WHEEL_NODES = RAYS * 2;
constexpr float PI = 3.14159265358979323846f;

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool NearlyEqual(float a, float b, float epsilon = 0.02f)
{
    return std::fabs(a - b) <= epsilon;
}

PhysicsVec3 Normalized(const PhysicsVec3& value)
{
    const float length = std::sqrt(value.squaredLength());
    if (length <= 1.0e-8f)
    {
        return PhysicsVec3();
    }
    return value * (1.0f / length);
}
}

int main()
{
    NodeCoreState axis0{};
    NodeCoreState axis1{};
    axis0.position = PhysicsVec3(0.0f, 0.0f, -0.15f);
    axis1.position = PhysicsVec3(0.0f, 0.0f,  0.15f);
    axis0.mass = axis1.mass = 25.0f;

    std::array<NodeCoreState, WHEEL_NODES> tire_nodes{};
    std::vector<WheelNodeBinding> bindings;
    bindings.reserve(WHEEL_NODES);

    const PhysicsVec3 axis = Normalized(axis1.position - axis0.position);
    const float radius = 0.50f;
    const float target_tread_speed = 12.0f;

    for (int j = 0; j < WHEEL_NODES; ++j)
    {
        const int ray = j / 2;
        const bool side1 = (j % 2) != 0;
        const float angle = (2.0f * PI * static_cast<float>(ray)) / static_cast<float>(RAYS);
        NodeCoreState& inner = side1 ? axis1 : axis0;
        NodeCoreState& outer = tire_nodes[j];

        const PhysicsVec3 radial(
            radius * std::cos(angle),
            radius * std::sin(angle),
            0.0f);

        outer.position = inner.position + radial;
        outer.mass = 4.0f;

        const PhysicsVec3 tangent = Normalized(axis.cross(radial));
        outer.velocity = tangent * target_tread_speed;

        WheelNodeBinding binding{};
        binding.outer = &outer;
        binding.inner = &inner;
        bindings.push_back(binding);
    }

    WheelCoreState wheel{};
    wheel.radius = radius;
    wheel.rotational_mass = 96.0f;
    wheel.speed = target_tread_speed;
    wheel.average_speed = target_tread_speed;
    wheel.torque = 2400.0f;

    const WheelStepResult powered = StepWheelNodes(wheel, axis0, axis1, bindings, PHYSICS_DT);
    Require(NearlyEqual(powered.measured_speed, target_tread_speed), "wheel speed is measured back from tire-node motion");
    Require(NearlyEqual(powered.applied_torque, 2400.0f), "wheel applies accumulated drive torque");
    Require(NearlyEqual(wheel.torque, 0.0f), "wheel torque accumulator resets after physics step");

    for (int j = 0; j < WHEEL_NODES; ++j)
    {
        const NodeCoreState& inner = (j % 2) ? axis1 : axis0;
        const PhysicsVec3 radial = tire_nodes[j].position - inner.position;
        const PhysicsVec3 tangent = Normalized(axis.cross(radial));
        Require(tire_nodes[j].force.dot(tangent) > 0.0f, "drive torque pushes each tire node tangentially");
    }

    const float brake_torque = CalcWheelBrakeTorque(
        wheel.speed,
        wheel.average_speed,
        wheel.radius,
        wheel.rotational_mass,
        wheel.last_reaction_torque,
        1800.0f,
        PHYSICS_DT);

    Require(brake_torque < 0.0f, "brake torque opposes forward wheel rotation");
    Require(std::fabs(brake_torque) <= 1800.01f, "brake torque is limited by available brake force");

    NodeCoreState arm{};
    NodeCoreState attach{};
    arm.position = PhysicsVec3(1.0f, 0.0f, 0.0f);
    attach.position = PhysicsVec3(0.0f, 0.0f, 0.0f);

    const bool reacted = ApplyWheelReactionTorque(1000.0f, axis, arm, attach);
    Require(reacted, "wheel torque produces a chassis reaction couple");
    Require(NearlyEqual((arm.force + attach.force).squaredLength(), 0.0f, 0.001f), "reaction torque forces remain equal and opposite");
    Require(arm.force.squaredLength() > 1.0f, "reaction torque loads the suspension arm");

    WheelCoreState reversed = wheel;
    reversed.reverse_rotation = true;
    reversed.torque = 0.0f;
    const WheelStepResult reverse_measurement = StepWheelNodes(reversed, axis0, axis1, bindings, PHYSICS_DT);
    Require(NearlyEqual(reverse_measurement.measured_speed, -target_tread_speed), "backward wheel orientation reverses measured tread speed");

    std::cout << "RoR wheel probe passed: " << RAYS << " rays / " << WHEEL_NODES
              << " deformable tire nodes at " << target_tread_speed << " m/s.\n";
    return EXIT_SUCCESS;
}
