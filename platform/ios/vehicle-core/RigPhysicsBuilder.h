/*
    This source file is part of Rigs of Rods.

    iOS bring-up bridge from portable RigDef data to the renderer-free
    node/beam solver. This intentionally handles structural nodes/beams first;
    shocks, hydros, generated wheels and RoR mass distribution are separate
    bring-up stages rather than being approximated here.
*/

#pragma once

#include "BeamPhysics.h"
#include "PortableRigDef.h"

#include <cstddef>
#include <string>
#include <vector>

namespace RoR {
namespace IOSVehicleCore {

struct RigPhysicsNode
{
    std::string id;
    NodeCoreState state;
};

struct RigPhysicsBeam
{
    std::size_t node_a = 0;
    std::size_t node_b = 0;
    BeamCoreState state;
};

struct RigPhysicsModel
{
    std::vector<RigPhysicsNode> nodes;
    std::vector<RigPhysicsBeam> beams;
    std::vector<std::string> errors;
};

/// Builds the currently supported structural subset of a parsed RoR rig.
/// default_node_mass is temporary bring-up mass only; production RoR actor
/// spawning will replace it with native mass-distribution semantics.
RigPhysicsModel BuildStructuralModel(
    const PortableRigDef::Document& rig,
    float default_node_mass = 10.0f);

/// Advances the structural model by one physics tick using the same portable
/// beam-force and node-integration helpers exercised by the iPhone core tests.
void StepStructuralModel(RigPhysicsModel& model, float gravity, float dt);

} // namespace IOSVehicleCore
} // namespace RoR
