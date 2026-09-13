/*
    This source file is part of Rigs of Rods.

    Portable solid-ground contact extracted from primitiveCollision().
*/

#include "GroundContact.h"

#include <cmath>

namespace RoR {

GroundContactResult CalcGroundContact(
    const NodeCoreState& node,
    const PhysicsVec3& normal,
    float penetration,
    const GroundContactParams& params,
    float dt)
{
    GroundContactResult result{};

    if (penetration < params.solid_ground_level || dt <= 0.0f || node.mass <= 0.0f)
    {
        return result;
    }

    result.contacted = true;

    const float normal_velocity = node.velocity.dot(normal);
    const float normal_force = node.force.dot(normal);

    float reaction = -normal_force;
    if (normal_velocity < 0.0f)
    {
        const float penetration_depth = params.solid_ground_level - penetration;
        reaction -= (0.8f * normal_velocity + 0.2f * penetration_depth / dt) * node.mass / dt;
    }

    if (reaction <= 0.0f)
    {
        return result;
    }

    const PhysicsVec3 slip_force = node.force - normal * normal_force;
    PhysicsVec3 slip = node.velocity - normal * normal_velocity;
    const float slip_squared = slip.squaredLength();
    const float slip_velocity = slip_squared > 0.0f ? std::sqrt(slip_squared) : 0.0f;
    result.slip_velocity = slip_velocity;

    if (slip_velocity > 1.0e-8f)
    {
        slip = slip * (1.0f / slip_velocity);
    }
    else
    {
        slip = PhysicsVec3();
    }

    const float moderated_reaction = reaction * params.ground_strength * params.node_friction;
    const float static_limit = params.friction.static_friction * moderated_reaction;

    if (slip_velocity < params.friction.adhesion_velocity &&
        moderated_reaction > 0.0f &&
        slip_force.squaredLength() <= static_limit * static_limit)
    {
        const float friction_force =
            CalcStaticFrictionScale(slip_velocity, params.friction) * moderated_reaction;
        result.force = normal * reaction + slip * friction_force - slip_force;
        result.static_friction = true;
    }
    else
    {
        const float friction_force =
            CalcStribeckFrictionScale(slip_velocity, params.friction) * moderated_reaction;
        result.force = normal * reaction + slip * friction_force;
    }

    return result;
}

bool ApplyFlatGroundContact(
    NodeCoreState& node,
    float ground_height,
    const GroundContactParams& params,
    float dt)
{
    if (node.position.y >= ground_height)
    {
        return false;
    }

    const float penetration = ground_height - node.position.y;
    const GroundContactResult result = CalcGroundContact(
        node, PhysicsVec3(0.0f, 1.0f, 0.0f), penetration, params, dt);

    node.force += result.force;
    return result.contacted;
}

} // namespace RoR
