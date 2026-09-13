/*
    This source file is part of Rigs of Rods.

    Portable wheel-force helpers extracted from Actor::CalcWheels().
*/

#include "WheelPhysics.h"

#include <algorithm>
#include <cmath>

namespace RoR {
namespace
{
PhysicsVec3 Normalized(const PhysicsVec3& value)
{
    const float length_squared = value.squaredLength();
    if (length_squared <= 1.0e-12f)
    {
        return PhysicsVec3();
    }
    return value * (1.0f / std::sqrt(length_squared));
}

float Length(const PhysicsVec3& value)
{
    return std::sqrt(value.squaredLength());
}
}

float CalcWheelBrakeTorque(
    float wheel_speed,
    float average_speed,
    float radius,
    float rotational_mass,
    float last_reaction_torque,
    float available_brake_torque,
    float dt)
{
    if (available_brake_torque <= 0.0f || radius <= 0.0f || rotational_mass <= 0.0f || dt <= 0.0f)
    {
        return 0.0f;
    }

    // Same braking-force estimate used by Actor::CalcWheels().
    float force = -average_speed * radius * rotational_mass / dt;
    force -= last_reaction_torque;

    if (wheel_speed > 0.0f)
    {
        return std::max(-available_brake_torque, std::min(force, 0.0f));
    }
    return std::min(available_brake_torque, std::max(force, 0.0f));
}

WheelStepResult StepWheelNodes(
    WheelCoreState& wheel,
    const NodeCoreState& axis_node_0,
    const NodeCoreState& axis_node_1,
    const std::vector<WheelNodeBinding>& bindings,
    float dt)
{
    WheelStepResult result{};
    if (bindings.empty() || wheel.radius <= 0.0f || wheel.rotational_mass <= 0.0f || dt <= 0.0f)
    {
        return result;
    }

    const PhysicsVec3 axis = Normalized(axis_node_1.position - axis_node_0.position);
    if (axis.squaredLength() <= 1.0e-12f)
    {
        return result;
    }

    const float applied_torque = wheel.torque;
    const float torque_per_node = applied_torque / static_cast<float>(bindings.size());
    const float expected_wheel_speed_before_contact = wheel.speed +
        ((wheel.last_torque / wheel.radius) / wheel.rotational_mass) * dt;

    float measured_speed = 0.0f;
    int measured_nodes = 0;

    for (const WheelNodeBinding& binding : bindings)
    {
        if (binding.outer == nullptr || binding.inner == nullptr)
        {
            continue;
        }

        PhysicsVec3 radius = binding.outer->position - binding.inner->position;
        const float radius_length = Length(radius);
        if (radius_length <= 1.0e-8f)
        {
            continue;
        }

        const float inverse_radius_length = 1.0f / radius_length;
        if (wheel.reverse_rotation)
        {
            radius = -radius;
        }

        // Matches Actor::CalcWheels(): axis x radius gives tread direction,
        // and torque is converted to force by the wheel-node radius.
        const PhysicsVec3 direction = axis.cross(radius) * inverse_radius_length;
        binding.outer->force += direction * torque_per_node * inverse_radius_length;
        measured_speed += (binding.outer->velocity - binding.inner->velocity).dot(direction);
        ++measured_nodes;
    }

    if (measured_nodes > 0)
    {
        measured_speed /= static_cast<float>(measured_nodes);
        wheel.speed = measured_speed;
        wheel.net_rotation += (wheel.speed / wheel.radius) * dt;

        // RoR intentionally overestimates the average slightly for braking quality.
        wheel.average_speed = wheel.average_speed * 0.99f + wheel.speed * 0.1f;
        wheel.last_reaction_torque = wheel.rotational_mass *
            (wheel.speed - expected_wheel_speed_before_contact) / dt;
    }

    wheel.last_torque = applied_torque;
    wheel.torque = 0.0f;

    result.measured_speed = wheel.speed;
    result.applied_torque = applied_torque;
    return result;
}

bool ApplyWheelReactionTorque(
    float wheel_torque,
    const PhysicsVec3& wheel_axis,
    NodeCoreState& arm_node,
    NodeCoreState& near_attach_node)
{
    const PhysicsVec3 axis = Normalized(wheel_axis);
    if (axis.squaredLength() <= 1.0e-12f || std::fabs(wheel_torque) <= 0.01f)
    {
        return false;
    }

    const PhysicsVec3 reaction_arm = arm_node.position - near_attach_node.position;
    PhysicsVec3 projected_arm = reaction_arm - axis * reaction_arm.dot(axis);
    const PhysicsVec3 offset_vector = reaction_arm - projected_arm;
    const float offset = Length(offset_vector);
    const float arm_length = Length(projected_arm);

    // Same geometry rejection used by Actor::CalcWheels().
    if (arm_length <= 0.01f || offset * 2.0f >= arm_length)
    {
        return false;
    }

    projected_arm = projected_arm * (1.0f / arm_length);
    PhysicsVec3 counter_force = axis.cross(projected_arm);
    counter_force = counter_force *
        ((0.5f * wheel_torque / arm_length) * (1.0f - ((offset * 2.0f) / arm_length)));

    arm_node.force -= counter_force;
    near_attach_node.force += counter_force;
    return true;
}

} // namespace RoR
