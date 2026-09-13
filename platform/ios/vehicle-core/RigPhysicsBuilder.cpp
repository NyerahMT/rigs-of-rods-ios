/*
    This source file is part of Rigs of Rods.
*/

#include "RigPhysicsBuilder.h"
#include "SimConstants.h"

#include <cmath>
#include <sstream>
#include <unordered_map>

namespace RoR {
namespace IOSVehicleCore {
namespace {

float Distance(const PhysicsVec3& a, const PhysicsVec3& b)
{
    const PhysicsVec3 d = a - b;
    return std::sqrt(d.squaredLength());
}

void AddError(RigPhysicsModel& model, const std::string& text)
{
    model.errors.push_back(text);
}

} // namespace

RigPhysicsModel BuildStructuralModel(
    const PortableRigDef::Document& rig,
    float default_node_mass)
{
    RigPhysicsModel model;
    if (default_node_mass <= 0.0f)
    {
        default_node_mass = 10.0f;
    }

    std::unordered_map<std::string, std::size_t> node_indices;
    model.nodes.reserve(rig.nodes.size());

    for (const PortableRigDef::Node& source_node : rig.nodes)
    {
        if (source_node.id.empty())
        {
            AddError(model, "encountered node with empty id");
            continue;
        }

        if (node_indices.find(source_node.id) != node_indices.end())
        {
            AddError(model, "duplicate node id: " + source_node.id);
            continue;
        }

        RigPhysicsNode node;
        node.id = source_node.id;
        node.state.position = PhysicsVec3(source_node.x, source_node.y, source_node.z);
        node.state.mass = default_node_mass;
        node.state.force = PhysicsVec3(0.0f, default_node_mass * DEFAULT_GRAVITY, 0.0f);

        const std::size_t index = model.nodes.size();
        model.nodes.push_back(node);
        node_indices[source_node.id] = index;
    }

    model.beams.reserve(rig.beams.size());
    for (const PortableRigDef::Beam& source_beam : rig.beams)
    {
        const auto a_it = node_indices.find(source_beam.node_a);
        const auto b_it = node_indices.find(source_beam.node_b);
        if (a_it == node_indices.end() || b_it == node_indices.end())
        {
            std::ostringstream message;
            message << "beam references missing node: "
                    << source_beam.node_a << " -> " << source_beam.node_b;
            AddError(model, message.str());
            continue;
        }

        if (a_it->second == b_it->second)
        {
            AddError(model, "beam connects node to itself: " + source_beam.node_a);
            continue;
        }

        const float rest_length = Distance(
            model.nodes[a_it->second].state.position,
            model.nodes[b_it->second].state.position);
        if (rest_length <= 1.0e-6f)
        {
            std::ostringstream message;
            message << "beam has zero rest length: "
                    << source_beam.node_a << " -> " << source_beam.node_b;
            AddError(model, message.str());
            continue;
        }

        RigPhysicsBeam beam;
        beam.node_a = a_it->second;
        beam.node_b = b_it->second;
        beam.state.rest_length = rest_length;
        beam.state.spring = source_beam.spring;
        beam.state.damping = source_beam.damping;
        model.beams.push_back(beam);
    }

    return model;
}

void StepStructuralModel(RigPhysicsModel& model, float gravity, float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    for (RigPhysicsBeam& beam : model.beams)
    {
        if (beam.node_a >= model.nodes.size() || beam.node_b >= model.nodes.size())
        {
            continue;
        }
        ApplyBeamForce(
            model.nodes[beam.node_a].state,
            model.nodes[beam.node_b].state,
            beam.state);
    }

    for (RigPhysicsNode& node : model.nodes)
    {
        IntegrateNode(node.state, gravity, dt);
    }
}

} // namespace IOSVehicleCore
} // namespace RoR
