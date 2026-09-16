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

    std::size_t spawned_nodes = 0;
    std::size_t spawned_beams = 0;
};

RigDefSummary ParseRigDef(const std::string& truck_text);
std::string CanonicalPhysicsRigDef(const std::string& truck_text);
std::string CanonicalShock3Section(const std::string& truck_text);

// Emits the complete eleven-parameter RigDef::Engoption line from the upstream
// parsed document. The older canonical serializer only carried parameters 1-6;
// this companion preserves stall/idle RPM, idle mixtures and braking torque too.
std::string CanonicalEngOptionSection(const std::string& truck_text);

} // namespace IOSNative
} // namespace RoR
