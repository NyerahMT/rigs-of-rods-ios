/*
    This source file is part of Rigs of Rods.

    Minimal renderer-free rig-definition model for the iOS vehicle-core bring-up.
    Field order follows RigDef_Parser and the documented .truck format; this is
    intentionally a compatibility layer, not a new vehicle format.
*/

#pragma once

#include <string>
#include <vector>

namespace RoR {
namespace PortableRigDef {

struct Globals
{
    bool present = false;
    float dry_mass = 0.0f;
    float load_mass = 0.0f;
    std::string material;
};

struct Node
{
    std::string id;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::string options;
};

struct Beam
{
    std::string node_a;
    std::string node_b;
    float spring = 9000000.0f;
    float damping = 12000.0f;
    float deform = 400000.0f;
    float strength = 100000.0f;
    std::string options;
};

struct Hydro
{
    std::string node_a;
    std::string node_b;
    float lengthening_factor = 0.0f;
    std::string options;
    float spring = 9000000.0f;
    float damping = 12000.0f;
};

struct Shock
{
    std::string node_a;
    std::string node_b;
    float spring = 0.0f;
    float damping = 0.0f;
    float short_bound = 0.0f;
    float long_bound = 0.0f;
    float precompression = 1.0f;
};

struct Wheel
{
    bool wheels2 = false;
    float rim_radius = 0.0f;
    float tire_radius = 0.0f;
    float width = 0.0f;
    int num_rays = 0;
    std::string axis_node_0;
    std::string axis_node_1;
    std::string rigidity_node;
    int braking = 0;
    int propulsion = 0;
    std::string reference_arm_node;
    float mass = 0.0f;
    float rim_spring = 0.0f;
    float rim_damping = 0.0f;
    float tire_spring = 0.0f;
    float tire_damping = 0.0f;
};

struct Engine
{
    bool present = false;
    float shift_down_rpm = 0.0f;
    float shift_up_rpm = 0.0f;
    float torque = 0.0f;
    float differential_ratio = 1.0f;
    std::vector<float> gear_ratios;
};

struct Brakes
{
    bool present = false;
    float service_force = 30000.0f;
    float parking_force = -1.0f;
};

struct Document
{
    std::string name;
    Globals globals;
    std::vector<Node> nodes;
    std::vector<Beam> beams;
    std::vector<Hydro> hydros;
    std::vector<Shock> shocks;
    std::vector<Wheel> wheels;
    Engine engine;
    Brakes brakes;
    std::vector<std::string> contacters;
    std::vector<std::string> warnings;
};

/// Parses the structural/driveline subset needed by the iOS vehicle-core.
/// Unknown sections are skipped so complete real RoR files can be fed to this
/// parser while compatibility is added incrementally.
Document Parse(const std::string& text);

} // namespace PortableRigDef
} // namespace RoR
