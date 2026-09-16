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

    // Exact ActorSpawner::CalcMemoryRequirements topology counts for the
    // supported physical sections. These include generated wheel nodes/beams,
    // not just authored RigDef lines.
    std::size_t spawned_nodes = 0;
    std::size_t spawned_beams = 0;
};

// Runs the real upstream RigDef::Parser + SequentialImporter over raw .truck
// text. This is the same document model ActorSpawner consumes on desktop RoR.
RigDefSummary ParseRigDef(const std::string& truck_text);

// Parse with upstream RigDef and serialize the physics sections understood by
// the portable iOS actor core into a deterministic canonical truck definition.
// This makes legacy/import/default inheritance semantics come from upstream
// rather than from the lightweight compatibility parser. Unsupported sections
// remain visible to the summary/audit and are ported subsystem-by-subsystem.
std::string CanonicalPhysicsRigDef(const std::string& truck_text);

// Transitional companion for the full SHOCK3 port. It emits the SHOCK3 section
// from the same upstream RigDef document so the app can splice it immediately
// before CanonicalPhysicsRigDef()'s final `end` marker without teaching the
// lightweight compatibility parser any legacy import semantics itself.
std::string CanonicalShock3Section(const std::string& truck_text);

} // namespace IOSNative
} // namespace RoR
