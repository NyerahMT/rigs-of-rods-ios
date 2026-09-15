/* This source file is part of Rigs of Rods. */
#include "PortableRigDef.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <sstream>

namespace RoR {
namespace PortableRigDef {
namespace {

enum class Section
{
    None,
    Globals,
    Nodes,
    Beams,
    Hydros,
    Shocks,
    Shocks2,
    Wheels,
    Wheels2,
    MeshWheels2,
    Engine,
    Brakes,
    Contacters,
    Minimass,
    Unknown
};

struct BeamDefaults
{
    float spring = 9000000.0f;
    float damping = 12000.0f;
    float deform = 400000.0f;
    float strength = 100000.0f;
};

struct BeamScale
{
    float spring = 1.0f;
    float damping = 1.0f;
    float deform = 1.0f;
    float strength = 1.0f;
};

std::string Trim(const std::string& input)
{
    std::size_t first = 0;
    std::size_t last = input.size();
    while (first < last && std::isspace(static_cast<unsigned char>(input[first]))) ++first;
    while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1]))) --last;
    return input.substr(first, last - first);
}

std::string Lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string StripComments(const std::string& input)
{
    std::size_t cut = input.size();
    const std::size_t semicolon = input.find(';');
    const std::size_t slashslash = input.find("//");
    if (semicolon != std::string::npos) cut = std::min(cut, semicolon);
    if (slashslash != std::string::npos) cut = std::min(cut, slashslash);
    return Trim(input.substr(0, cut));
}

std::vector<std::string> Tokens(std::string line)
{
    for (char& c : line)
    {
        if (c == ',' || c == ':' || c == '|' || c == '\t') c = ' ';
    }
    std::istringstream stream(line);
    std::vector<std::string> out;
    std::string token;
    while (stream >> token) out.push_back(token);
    return out;
}

float F(const std::string& value, float fallback = 0.0f)
{
    char* end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    return end != value.c_str() ? parsed : fallback;
}

int I(const std::string& value, int fallback = 0)
{
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    return end != value.c_str() ? static_cast<int>(parsed) : fallback;
}

float Resettable(const std::string& value, float default_value)
{
    const float parsed = F(value, default_value);
    return parsed < 0.0f ? default_value : parsed;
}

Section SectionFromKeyword(const std::string& keyword)
{
    if (keyword == "globals") return Section::Globals;
    if (keyword == "nodes" || keyword == "nodes2") return Section::Nodes;
    if (keyword == "beams") return Section::Beams;
    if (keyword == "hydros") return Section::Hydros;
    if (keyword == "shocks") return Section::Shocks;
    if (keyword == "shocks2") return Section::Shocks2;
    if (keyword == "wheels") return Section::Wheels;
    if (keyword == "wheels2") return Section::Wheels2;
    if (keyword == "meshwheels2") return Section::MeshWheels2;
    if (keyword == "engine") return Section::Engine;
    if (keyword == "brakes") return Section::Brakes;
    if (keyword == "contacters") return Section::Contacters;
    if (keyword == "minimass") return Section::Minimass;
    return Section::Unknown;
}

bool StartsUnsupportedBlock(const std::string& keyword)
{
    static const char* const blocks[] = {
        "airbrakes", "animators", "assetpacks", "axles", "cab", "cameras", "camerarail",
        "cinecam", "collisionboxes", "commands", "commands2", "customdashboardinputs",
        "description", "engoption", "engturbo", "exhausts", "fixes", "flares", "flares2",
        "flares3", "flaregroups_no_import", "flexbodies", "flexbodywheels", "fusedrag",
        "guisettings", "help", "hooks", "interaxles", "lockgroups", "managedmaterials",
        "materialflarebindings", "meshwheels", "particles", "pistonprops", "props", "railgroups",
        "ropables", "ropes", "rotators", "rotators2", "screwprops", "scripts", "shocks3",
        "slidenodes", "soundsources", "soundsources2", "submesh", "texcoords", "ties",
        "torquecurve", "transfercase", "triggers", "turbojets", "turboprops", "turboprops2",
        "videocamera", "wheeldetachers", "wings"
    };
    for (const char* block : blocks)
    {
        if (keyword == block) return true;
    }
    return false;
}

bool IsIgnoredDirective(const std::string& keyword)
{
    // These are directives in upstream RigDef: they are processed immediately and do not
    // change the active block. Portable physics does not need their payload, but it must
    // not accidentally parse them as nodes/beams/shocks/etc.
    static const char* const directives[] = {
        "add_animation", "antilockbrakes", "author", "backmesh", "cruisecontrol",
        "default_skin", "detacher_group", "disabledefaultsounds", "enable_advanced_deformation",
        "extcamera", "fileformatversion", "fileinfo", "flexbody_camera_mode", "forset", "forvert",
        "forwardcommands", "guid", "hideinchooser", "hookgroup", "importcommands",
        "lockgroup_default_nolock", "nodecollision", "prop_camera_mode", "rescuer", "rigidifiers",
        "rollon", "section", "set_collision_range", "set_inertia_defaults",
        "set_managedmaterials_options", "set_skeleton_settings", "slidenode_connect_instantly",
        "speedlimiter", "submesh_groundmodel", "tractioncontrol"
    };
    for (const char* directive : directives)
    {
        if (keyword == directive) return true;
    }
    return false;
}

void Warn(Document& document, int line_number, const std::string& text)
{
    std::ostringstream stream;
    stream << "line " << line_number << ": " << text;
    document.warnings.push_back(stream.str());
}

bool Has(const std::string& text, char c)
{
    return text.find(c) != std::string::npos;
}

} // namespace

Document Parse(const std::string& text)
{
    Document document;
    BeamDefaults beam_defaults;
    BeamScale beam_scale;
    Section section = Section::None;
    float active_minimass = 50.0f;

    std::istringstream input(text);
    std::string raw_line;
    int line_number = 0;

    while (std::getline(input, raw_line))
    {
        ++line_number;
        const std::string line = StripComments(raw_line);
        if (line.empty()) continue;

        if (document.name.empty())
        {
            document.name = line;
            continue;
        }

        const std::vector<std::string> tokens = Tokens(line);
        if (tokens.empty()) continue;
        const std::string keyword = Lower(tokens[0]);

        if (keyword == "end") break;
        if (keyword == "end_description" || keyword == "end_comment")
        {
            section = Section::None;
            continue;
        }

        // Stateful RigDef directives. Like the upstream parser, these do not end the
        // current block, which matters for legacy files that interleave directives with beams.
        if (keyword == "set_beam_defaults_scale")
        {
            if (tokens.size() >= 5)
            {
                beam_scale.spring = F(tokens[1], 1.0f);
                beam_scale.damping = F(tokens[2], 1.0f);
                beam_scale.deform = F(tokens[3], 1.0f);
                beam_scale.strength = F(tokens[4], 1.0f);
            }
            else
            {
                Warn(document, line_number, "set_beam_defaults_scale requires four values");
            }
            continue;
        }

        if (keyword == "set_beam_defaults")
        {
            if (tokens.size() >= 5)
            {
                beam_defaults.spring = Resettable(tokens[1], 9000000.0f) * beam_scale.spring;
                beam_defaults.damping = Resettable(tokens[2], 12000.0f) * beam_scale.damping;
                beam_defaults.deform = Resettable(tokens[3], 400000.0f) * beam_scale.deform;
                beam_defaults.strength = Resettable(tokens[4], 100000.0f) * beam_scale.strength;
            }
            else
            {
                Warn(document, line_number, "set_beam_defaults requires four values");
            }
            continue;
        }

        if (keyword == "set_node_defaults")
        {
            if (tokens.size() >= 2)
            {
                document.node_defaults.load_weight = Resettable(tokens[1], 0.0f);
                if (tokens.size() >= 3) document.node_defaults.friction = Resettable(tokens[2], 1.0f);
                if (tokens.size() >= 4) document.node_defaults.volume = Resettable(tokens[3], 1.0f);
                if (tokens.size() >= 5) document.node_defaults.surface = Resettable(tokens[4], 1.0f);
                document.node_defaults.options = tokens.size() >= 6 ? tokens[5] : "";
            }
            else
            {
                Warn(document, line_number, "set_node_defaults requires load weight");
            }
            continue;
        }

        if (keyword == "set_default_minimass")
        {
            if (tokens.size() >= 2)
                active_minimass = Resettable(tokens[1], document.minimass);
            else
                Warn(document, line_number, "set_default_minimass requires a value");
            continue;
        }

        if (IsIgnoredDirective(keyword))
            continue;

        const Section new_section = SectionFromKeyword(keyword);
        if (new_section != Section::Unknown)
        {
            section = new_section;
            continue;
        }

        if (StartsUnsupportedBlock(keyword))
        {
            section = Section::Unknown;
            continue;
        }

        switch (section)
        {
            case Section::Globals:
                if (tokens.size() >= 2)
                {
                    document.globals.present = true;
                    document.globals.dry_mass = F(tokens[0]);
                    document.globals.load_mass = F(tokens[1]);
                    if (tokens.size() > 2) document.globals.material = tokens[2];
                }
                else
                {
                    Warn(document, line_number, "globals requires dry mass and load mass");
                }
                section = Section::None;
                break;

            case Section::Minimass:
                document.minimass = F(tokens[0], 50.0f);
                active_minimass = document.minimass;
                section = Section::None;
                break;

            case Section::Nodes:
                if (tokens.size() >= 4)
                {
                    Node node;
                    node.id = tokens[0];
                    node.x = F(tokens[1]);
                    node.y = F(tokens[2]);
                    node.z = F(tokens[3]);
                    node.options = document.node_defaults.options;
                    if (tokens.size() > 4) node.options += tokens[4];
                    node.load_weight = document.node_defaults.load_weight;
                    node.friction = document.node_defaults.friction;
                    node.volume = document.node_defaults.volume;
                    node.surface = document.node_defaults.surface;
                    node.minimass = active_minimass;
                    node.loaded_mass = node.load_weight > 0.0f || Has(node.options, 'l') || Has(node.options, 'L');
                    node.override_mass = node.load_weight > 0.0f;
                    if (tokens.size() > 5 && (Has(node.options, 'l') || Has(node.options, 'L')))
                    {
                        node.load_weight = F(tokens[5], node.load_weight);
                        node.override_mass = true;
                        node.loaded_mass = true;
                    }
                    document.nodes.push_back(node);
                }
                else
                {
                    Warn(document, line_number, "node requires id,x,y,z");
                }
                break;

            case Section::Beams:
                if (tokens.size() >= 2)
                {
                    Beam beam;
                    beam.node_a = tokens[0];
                    beam.node_b = tokens[1];
                    beam.spring = beam_defaults.spring;
                    beam.damping = beam_defaults.damping;
                    beam.deform = beam_defaults.deform;
                    beam.strength = beam_defaults.strength;
                    if (tokens.size() > 2) beam.options = tokens[2];
                    document.beams.push_back(beam);
                }
                break;

            case Section::Hydros:
                if (tokens.size() >= 3)
                {
                    Hydro hydro;
                    hydro.node_a = tokens[0];
                    hydro.node_b = tokens[1];
                    hydro.lengthening_factor = F(tokens[2]);
                    if (tokens.size() > 3) hydro.options = tokens[3];
                    hydro.spring = beam_defaults.spring;
                    hydro.damping = beam_defaults.damping;
                    document.hydros.push_back(hydro);
                }
                break;

            case Section::Shocks:
            case Section::Shocks2:
                if (tokens.size() >= 7)
                {
                    Shock shock;
                    shock.node_a = tokens[0];
                    shock.node_b = tokens[1];
                    shock.spring = F(tokens[2]);
                    shock.damping = F(tokens[3]);
                    if (section == Section::Shocks)
                    {
                        shock.short_bound = F(tokens[4]);
                        shock.long_bound = F(tokens[5]);
                        shock.precompression = F(tokens[6], 1.0f);
                    }
                    else if (tokens.size() >= 13)
                    {
                        shock.short_bound = F(tokens[10]);
                        shock.long_bound = F(tokens[11]);
                        shock.precompression = F(tokens[12], 1.0f);
                    }
                    document.shocks.push_back(shock);
                }
                break;

            case Section::Wheels:
                if (tokens.size() >= 12)
                {
                    Wheel wheel;
                    wheel.tire_radius = F(tokens[0]);
                    wheel.width = F(tokens[1]);
                    wheel.num_rays = I(tokens[2]);
                    wheel.axis_node_0 = tokens[3];
                    wheel.axis_node_1 = tokens[4];
                    wheel.rigidity_node = tokens[5];
                    wheel.braking = I(tokens[6]);
                    wheel.propulsion = I(tokens[7]);
                    wheel.reference_arm_node = tokens[8];
                    wheel.mass = F(tokens[9]);
                    wheel.tire_spring = F(tokens[10]);
                    wheel.tire_damping = F(tokens[11]);
                    document.wheels.push_back(wheel);
                }
                break;

            case Section::Wheels2:
                if (tokens.size() >= 15)
                {
                    Wheel wheel;
                    wheel.wheels2 = true;
                    wheel.rim_radius = F(tokens[0]);
                    wheel.tire_radius = F(tokens[1]);
                    wheel.width = F(tokens[2]);
                    wheel.num_rays = I(tokens[3]);
                    wheel.axis_node_0 = tokens[4];
                    wheel.axis_node_1 = tokens[5];
                    wheel.rigidity_node = tokens[6];
                    wheel.braking = I(tokens[7]);
                    wheel.propulsion = I(tokens[8]);
                    wheel.reference_arm_node = tokens[9];
                    wheel.mass = F(tokens[10]);
                    wheel.rim_spring = F(tokens[11]);
                    wheel.rim_damping = F(tokens[12]);
                    wheel.tire_spring = F(tokens[13]);
                    wheel.tire_damping = F(tokens[14]);
                    document.wheels.push_back(wheel);
                }
                break;

            case Section::MeshWheels2:
                // Unlike wheels2, upstream meshwheels2 uses the simple two-nodes-per-ray
                // wheel topology. The rim radius is visual metadata; physical nodes live at
                // tyre_radius. The active beam defaults stiffen only the ring reinforcement.
                if (tokens.size() >= 16)
                {
                    Wheel wheel;
                    wheel.wheels2 = false;
                    wheel.tire_radius = F(tokens[0]);
                    wheel.rim_radius = F(tokens[1]);
                    wheel.width = F(tokens[2]);
                    wheel.num_rays = I(tokens[3]);
                    wheel.axis_node_0 = tokens[4];
                    wheel.axis_node_1 = tokens[5];
                    wheel.rigidity_node = tokens[6];
                    wheel.braking = I(tokens[7]);
                    wheel.propulsion = I(tokens[8]);
                    wheel.reference_arm_node = tokens[9];
                    wheel.mass = F(tokens[10]);
                    wheel.rim_spring = beam_defaults.spring;
                    wheel.rim_damping = beam_defaults.damping;
                    wheel.tire_spring = F(tokens[11]);
                    wheel.tire_damping = F(tokens[12]);
                    document.wheels.push_back(wheel);
                }
                else
                {
                    Warn(document, line_number, "meshwheels2 requires 16 values");
                }
                break;

            case Section::Engine:
                if (tokens.size() >= 6)
                {
                    document.engine.present = true;
                    document.engine.shift_down_rpm = F(tokens[0]);
                    document.engine.shift_up_rpm = F(tokens[1]);
                    document.engine.torque = F(tokens[2]);
                    document.engine.differential_ratio = F(tokens[3], 1.0f);
                    document.engine.reverse_gear_ratio = F(tokens[4]);
                    document.engine.neutral_gear_ratio = F(tokens[5], 1.0f);
                    document.engine.gear_ratios.clear();
                    bool terminated = false;
                    for (std::size_t i = 6; i < tokens.size(); ++i)
                    {
                        const float ratio = F(tokens[i]);
                        if (ratio < 0.0f)
                        {
                            terminated = true;
                            break;
                        }
                        document.engine.gear_ratios.push_back(ratio);
                    }
                    if (!terminated)
                        Warn(document, line_number, "engine forward gears should end with -1 terminator");
                    if (document.engine.gear_ratios.empty())
                        Warn(document, line_number, "engine requires at least one forward gear");
                }
                else
                {
                    Warn(document, line_number, "engine requires shift RPMs, torque, differential, reverse and neutral ratios");
                }
                break;

            case Section::Brakes:
                if (!tokens.empty())
                {
                    document.brakes.present = true;
                    document.brakes.service_force = F(tokens[0], 30000.0f);
                    if (tokens.size() > 1) document.brakes.parking_force = F(tokens[1], -1.0f);
                }
                break;

            case Section::Contacters:
                document.contacters.push_back(tokens[0]);
                break;

            case Section::None:
            case Section::Unknown:
            default:
                break;
        }
    }

    return document;
}

} // namespace PortableRigDef
} // namespace RoR
