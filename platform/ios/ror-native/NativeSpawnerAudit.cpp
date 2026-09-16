#include "NativeSpawnerAudit.h"

#include "RigDef_Parser.h"

#include <sstream>

namespace RoR {
namespace IOSNative {
namespace {

RigDef::DocumentPtr Parse(const std::string& text)
{
    if (text.empty()) return RigDef::DocumentPtr();
    RigDef::Parser parser;
    parser.Prepare();
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line))
        parser.ProcessRawLine(line.c_str());
    parser.Finalize();
    return parser.GetFile();
}

template <typename T>
void CountWheelFamily(const T& wheels, std::size_t nodes_per_ray,
                      std::size_t beams_per_ray, SpawnTopologySummary& out)
{
    for (const auto& wheel : wheels)
    {
        ++out.wheels;
        out.spawned_nodes += static_cast<std::size_t>(wheel.num_rays) * nodes_per_ray;
        out.spawned_beams += static_cast<std::size_t>(wheel.num_rays) *
            (beams_per_ray + (wheel.rigidity_node.IsValidAnyState() ? 1u : 0u));
    }
}

} // namespace

SpawnTopologySummary CalcSpawnTopology(const std::string& truck_text)
{
    SpawnTopologySummary out;
    RigDef::DocumentPtr doc = Parse(truck_text);
    if (!doc || !doc->root_module) return out;
    const auto& m = doc->root_module;
    out.ready = true;

    out.authored_nodes = m->nodes.size();
    out.authored_beams = m->beams.size();
    out.spawned_nodes = m->nodes.size();
    out.spawned_beams = m->beams.size();

    for (const RigDef::Node& node : m->nodes)
    {
        if ((node.options & RigDef::Node::OPTION_h_HOOK_POINT) != 0)
            ++out.spawned_beams;
    }

    out.spawned_beams += m->ties.size();
    out.spawned_beams += m->ropes.size();
    out.spawned_beams += m->hydros.size();
    out.spawned_beams += m->triggers.size();
    out.spawned_beams += m->animators.size();

    out.spawned_nodes += m->cinecam.size();
    out.spawned_beams += m->cinecam.size() * 8u;

    out.shocks = m->shocks.size() + m->shocks2.size() + m->shocks3.size();
    out.spawned_beams += out.shocks;
    out.spawned_beams += m->commands2.size();

    CountWheelFamily(m->wheels,      2u, 8u, out);
    CountWheelFamily(m->meshwheels,  2u, 8u, out);
    CountWheelFamily(m->meshwheels2, 2u, 8u, out);
    CountWheelFamily(m->wheels2, 4u, 24u, out);
    CountWheelFamily(m->flexbodywheels, 4u, 20u, out);

    out.unsupported_physics_sections += !m->ties.empty();
    out.unsupported_physics_sections += !m->ropes.empty();
    out.unsupported_physics_sections += !m->triggers.empty();
    out.unsupported_physics_sections += !m->animators.empty();
    out.unsupported_physics_sections += !m->cinecam.empty();
    out.unsupported_physics_sections += !m->commands2.empty();
    out.unsupported_physics_sections += !m->rotators.empty() || !m->rotators2.empty();
    out.unsupported_physics_sections += !m->flexbodywheels.empty();
    out.unsupported_physics_sections += !m->axles.empty() || !m->interaxles.empty();
    out.unsupported_physics_sections += !m->transfercase.empty();

    // Custom torquecurves are now executed with the same Ogre::SimpleSpline
    // coefficient path as desktop. Predefined named models still depend on the
    // external torque_models.cfg resource, so keep only those audit-visible.
    bool has_unresolved_named_torque_model = false;
    for (const RigDef::TorqueCurve& curve : m->torquecurve)
    {
        if (!curve.predefined_func_name.empty())
        {
            has_unresolved_named_torque_model = true;
            break;
        }
    }
    out.unsupported_physics_sections += has_unresolved_named_torque_model;

    out.unsupported_physics_sections += !m->wings.empty() || !m->fusedrag.empty() || !m->airbrakes.empty();

    return out;
}

} // namespace IOSNative
} // namespace RoR
