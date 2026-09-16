#pragma once

#include <cstddef>
#include <string>

namespace RoR {
namespace IOSNative {

struct SpawnTopologySummary
{
    bool ready = false;
    std::size_t authored_nodes = 0;
    std::size_t authored_beams = 0;
    std::size_t spawned_nodes = 0;
    std::size_t spawned_beams = 0;
    std::size_t shocks = 0;
    std::size_t wheels = 0;
    std::size_t unsupported_physics_sections = 0;
};

// Mirrors the physical-count portion of upstream
// ActorSpawner::CalcMemoryRequirements(). This is deliberately calculated from
// the real RigDef document, so generated wheel topology and special node-created
// beams can be checked against the portable actor after construction.
SpawnTopologySummary CalcSpawnTopology(const std::string& truck_text);

} // namespace IOSNative
} // namespace RoR
