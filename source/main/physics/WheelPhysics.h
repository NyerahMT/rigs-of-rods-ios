/*
    This source file is part of Rigs of Rods.

    Portable wheel-force helpers extracted from Actor::CalcWheels().
*/

#pragma once

#include "BeamPhysics.h"

#include <vector>

namespace RoR {

struct WheelCoreState
{
    float radius = 0.5f;
    float rotational_mass = 1.0f;
    float speed = 0.0f;              //!< Linear tread speed in m/s, matching wheel_t::wh_speed
    float average_speed = 0.0f;      //!< Smoothed speed used by RoR braking
    float torque = 0.0f;             //!< Torque accumulated for the current physics step
    float last_torque = 0.0f;        //!< Last internal torque (engine/brake/differential)
    float last_reaction_torque = 0.0f; //!< Last torque inferred from external tire forces
    float net_rotation = 0.0f;
    bool reverse_rotation = false;    //!< Equivalent to WheelPropulsion::BACKWARD orientation
};

struct WheelNodeBinding
{
    NodeCoreState* outer = nullptr;
    NodeCoreState* inner = nullptr;
};

struct WheelStepResult
{
    float measured_speed = 0.0f;
    float applied_torque = 0.0f;
};

/// RoR's braking estimate from CalcWheels(). The return value is the torque
/// increment to add to wheel.torque for this physics step.
float CalcWheelBrakeTorque(
    float wheel_speed,
    float average_speed,
    float radius,
    float rotational_mass,
    float last_reaction_torque,
    float available_brake_torque,
    float dt);

/// Distributes the wheel torque as tangential forces to deformable wheel nodes,
/// measures tread speed back from node velocities, and updates RoR's reaction
/// torque bookkeeping.
WheelStepResult StepWheelNodes(
    WheelCoreState& wheel,
    const NodeCoreState& axis_node_0,
    const NodeCoreState& axis_node_1,
    const std::vector<WheelNodeBinding>& bindings,
    float dt);

/// Applies RoR's chassis reaction-torque approximation to the reference arm.
bool ApplyWheelReactionTorque(
    float wheel_torque,
    const PhysicsVec3& wheel_axis,
    NodeCoreState& arm_node,
    NodeCoreState& near_attach_node);

} // namespace RoR
