/*
    This source file is part of Rigs of Rods.

    Renderer-free RigDef compatibility model used by the iOS vehicle core.
    The goal is semantic parity with the upstream truck format, not a new format.
*/
#pragma once
#include <string>
#include <vector>
namespace RoR { namespace PortableRigDef {
struct Globals { bool present=false; float dry_mass=0,load_mass=0; std::string material; };
struct NodeDefaults {
    float load_weight=0.0f;
    float friction=1.0f;
    float volume=1.0f;
    float surface=1.0f;
    std::string options;
};
struct Node {
    std::string id;
    float x=0,y=0,z=0;
    std::string options;
    float load_weight=0.0f;
    float friction=1.0f;
    float volume=1.0f;
    float surface=1.0f;
    float minimass=50.0f;
    bool loaded_mass=false;
    bool override_mass=false;
};
struct Beam { std::string node_a,node_b; float spring=9000000,damping=12000,deform=400000,strength=100000; std::string options; };
struct Hydro { std::string node_a,node_b; float lengthening_factor=0; std::string options; float spring=9000000,damping=12000; };
struct Shock {
    std::string node_a,node_b;
    bool shock2=false;
    // Classic `shocks` values. For `shocks2`, spring/damping mirror the
    // compression-side base values for compatibility with older callers.
    float spring=0,damping=0,short_bound=0,long_bound=0,precompression=1;
    // BeamDefaults active on the line. Upstream stores these as shock_t::sbd_*
    // and uses them as the hard bump-stop target once SHOCK1/2 bounds are exceeded.
    float bump_spring=9000000,bump_damping=12000;
    // Full shocks2 asymmetric/progressive data.
    float spring_in=0,damp_in=0,progress_spring_in=0,progress_damp_in=0;
    float spring_out=0,damp_out=0,progress_spring_out=0,progress_damp_out=0;
    std::string options;
};
struct Wheel { bool wheels2=false; float rim_radius=0,tire_radius=0,width=0; int num_rays=0; std::string axis_node_0,axis_node_1,rigidity_node; int braking=0,propulsion=0; std::string reference_arm_node; float mass=0,rim_spring=0,rim_damping=0,tire_spring=0,tire_damping=0; };
struct Engine {
    bool present=false;
    float shift_down_rpm=0;
    float shift_up_rpm=0;
    float torque=0;
    float differential_ratio=1;
    float reverse_gear_ratio=0;
    float neutral_gear_ratio=1;
    std::vector<float> gear_ratios; // Forward gears only, matching RigDef::Engine::gear_ratios.
};
// Mirrors upstream RigDef::Engoption. Negative timing/force values mean
// "leave Engine defaults unchanged" in classic truck files.
struct EngOption {
    bool present=false;
    float inertia=10.0f;
    char type='t';
    float clutch_force=-1.0f;
    float shift_time=-1.0f;
    float clutch_time=-1.0f;
    float post_shift_time=-1.0f;
};
struct Brakes { bool present=false; float service_force=30000,parking_force=-1; };
struct Document {
    std::string name;
    Globals globals;
    NodeDefaults node_defaults;
    float minimass=50.0f;
    std::vector<Node> nodes;
    std::vector<Beam> beams;
    std::vector<Hydro> hydros;
    std::vector<Shock> shocks;
    std::vector<Wheel> wheels;
    Engine engine;
    EngOption engoption;
    Brakes brakes;
    std::vector<std::string> contacters;
    std::vector<std::string> warnings;
};
Document Parse(const std::string& text);
} }
