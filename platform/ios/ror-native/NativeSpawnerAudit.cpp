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

    // Keep these formulas in lockstep with upstream
    // ActorSpawner::CalcMemoryRequirements().
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

    // wheels/meshwheels/meshwheels2: BuildWheelObjectAndNodes + BuildWheelBeams
    // => 2 nodes/ray and 8 beams/ray, with one rigidity beam/ray when present.
    CountWheelFamily(m->wheels,      2u, 8u, out);
    CountWheelFamily(m->meshwheels,  2u, 8u, out);
    CountWheelFamily(m->meshwheels2, 2u, 8u, out);

    // wheels2: 4 nodes/ray, 10 rim + 14 tyre beams/ray, plus rigidity.
    CountWheelFamily(m->wheels2, 4u, 24u, out);

    // flexbodywheels: 4 nodes/ray, 8 rim + 10 tyre + 2 support beams/ray,
    // plus one rigidity beam/ray when authored.
    CountWheelFamily(m->flexbodywheels, 4u, 20u, out);

    // Keep a visible count of physics-bearing sections that the portable actor
    // still cannot claim as upstream-equivalent. This number should converge to
    // zero as the following parity workstreams land. SHOCK3 is intentionally
    // absent here: the portable core now preserves and executes its full
    // asymmetric velocity-split force law.
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
    out.unsupported_physics_sections += !m->torquecurve.empty();
    out.unsupported_physics_sections += !m->wings.empty() || !m->fusedrag.empty() || !m->airbrakes.empty();

    return out;
}

} // namespace IOSNative
} // namespace RoR
