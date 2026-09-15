/*
    This source file is part of Rigs of Rods.

    Minimal renderer-free rig-definition model for the iOS vehicle-core bring-up.
    Field order follows RigDef_Parser and the documented .truck format; this is
    intentionally a compatibility layer, not a new vehicle format.
*/
#pragma once
#include <string>
#include <vector>
namespace RoR { namespace PortableRigDef {
struct Globals { bool present=false; float dry_mass=0,load_mass=0; std::string material; };
struct NodeDefaults { float load_weight=0.0f; float minimass=50.0f; bool load_weight_is_set=false; };
struct Node { std::string id; float x=0,y=0,z=0; std::string options; float load_weight=0.0f; float minimass=50.0f; bool load_weight_is_set=false; };
struct Beam { std::string node_a,node_b; float spring=9000000,damping=12000,deform=400000,strength=100000; std::string options; };
struct Hydro { std::string node_a,node_b; float lengthening_factor=0; std::string options; float spring=9000000,damping=12000; };
struct Shock { std::string node_a,node_b; float spring=0,damping=0,short_bound=0,long_bound=0,precompression=1; };
struct Wheel { bool wheels2=false; float rim_radius=0,tire_radius=0,width=0; int num_rays=0; std::string axis_node_0,axis_node_1,rigidity_node; int braking=0,propulsion=0; std::string reference_arm_node; float mass=0,rim_spring=0,rim_damping=0,tire_spring=0,tire_damping=0; };
struct Engine { bool present=false; float shift_down_rpm=0,shift_up_rpm=0,torque=0,differential_ratio=1; std::vector<float> gear_ratios; };
struct Brakes { bool present=false; float service_force=30000,parking_force=-1; };
struct Document { std::string name; Globals globals; NodeDefaults node_defaults; std::vector<Node> nodes; std::vector<Beam> beams; std::vector<Hydro> hydros; std::vector<Shock> shocks; std::vector<Wheel> wheels; Engine engine; Brakes brakes; std::vector<std::string> contacters; std::vector<std::string> warnings; };
Document Parse(const std::string& text);
} }
