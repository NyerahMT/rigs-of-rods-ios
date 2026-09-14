#pragma once

#include <cstddef>
#include <string>

namespace RoR {
namespace IOSNative {

struct RigDefSummary
{
    bool ready = false;
    std::string name;
    std::size_t nodes = 0;
    std::size_t beams = 0;
    std::size_t wheels = 0;
    std::size_t engines = 0;
    std::size_t axles = 0;
    std::size_t shocks = 0;
    std::size_t commands = 0;
    std::size_t props = 0;
    std::size_t flexbodies = 0;
    std::size_t wings = 0;
};

// Runs the real upstream RigDef::Parser + SequentialImporter over raw .truck
// text. This is the same document model ActorSpawner consumes on desktop RoR.
RigDefSummary ParseRigDef(const std::string& truck_text);

} // namespace IOSNative
} // namespace RoR
