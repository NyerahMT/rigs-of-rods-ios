/*
    This source file is part of Rigs of Rods.

    Portable solid-ground contact extracted from primitiveCollision().
*/

#pragma once

#include "BeamPhysics.h"
#include "GroundFriction.h"

namespace RoR {

struct GroundContactParams
{
    GroundFrictionParams friction{};
    float ground_strength = 1.0f;
    float node_friction = 1.0f;
    float solid_ground_level = 0.0f;
};

struct GroundContactResult
{
    PhysicsVec3 force;
    float slip_velocity = 0.0f;
    bool static_friction = false;
    bool contacted = false;
};

// Solid-ground portion of RoR::primitiveCollision().
GroundContactResult CalcGroundContact(
    const NodeCoreState& node,
    const PhysicsVec3& normal,
    float penetration,
    const GroundContactParams& params,
    float dt);

// Convenience for the first iOS chassis probe: an infinite horizontal plane.
bool ApplyFlatGroundContact(
    NodeCoreState& node,
    float ground_height,
    const GroundContactParams& params,
    float dt);

} // namespace RoR
