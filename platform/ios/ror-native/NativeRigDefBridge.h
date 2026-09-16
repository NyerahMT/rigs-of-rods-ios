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
std::string CanonicalEngOptionSection(const std::string& truck_text);

// Emits either the upstream predefined torque-model name or every authored
// custom rpm,torque-multiplier sample from RigDef::TorqueCurve.
std::string CanonicalTorqueCurveSection(const std::string& truck_text);

} // namespace IOSNative
} // namespace RoR
