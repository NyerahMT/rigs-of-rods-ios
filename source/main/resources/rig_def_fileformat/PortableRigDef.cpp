/*
    This source file is part of Rigs of Rods.

    Portable structural subset of the native RigDef parser for the iOS bring-up.
    Token separators and field ordering intentionally follow RigDef_Parser.
*/

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
    Nodes,
    Beams,
    Hydros,
    Shocks,
    Shocks2,
    Wheels,
    Wheels2,
    Engine,
    Brakes,
    Unknown
};

struct BeamDefaults
{
    float spring = 9000000.0f;
    float damping = 12000.0f;
    float deform = 400000.0f;
    float strength = 100000.0f;
};

std::string Trim(const std::string& input)
{
    size_t first = 0;
    while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first])))
        ++first;

    size_t last = input.size();
    while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1])))
        --last;

    return input.substr(first, last - first);
}

std::string Lower(std::string input)
{
    std::transform(input.begin(), input.end(), input.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return input;
}

std::string StripComments(const std::string& input)
{
    size_t cut = input.size();
    const size_t semicolon = input.find(';');
    if (semicolon != std::string::npos)
        cut = std::min(cut, semicolon);

    const size_t slash = input.find("//");
    if (slash != std::string::npos)
        cut = std::min(cut, slash);

    return Trim(input.substr(0, cut));
}

std::vector<std::string> Tokens(const std::string& line)
{
    // RigDef_Parser treats whitespace, ':', '|' and ',' as separators.
    std::string normalized = line;
    for (char& c : normalized)
    {
        if (c == ',' || c == ':' || c == '|' || c == '\t')
            c = ' ';
    }

    std::vector<std::string> out;
    std::istringstream stream(normalized);
    std::string token;
    while (stream >> token)
        out.push_back(token);
    return out;
}

float ToFloat(const std::string& text, float fallback = 0.0f)
{
    char* end = nullptr;
    const float value = std::strtof(text.c_str(), &end);
    return (end != text.c_str()) ? value : fallback;
}

int ToInt(const std::string& text, int fallback = 0)
{
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    return (end != text.c_str()) ? static_cast<int>(value) : fallback;
}

Section SectionForKeyword(const std::string& keyword)
{
    if (keyword == "nodes" || keyword == "nodes2") return Section::Nodes;
    if (keyword == "beams") return Section::Beams;
    if (keyword == "hydros") return Section::Hydros;
    if (keyword == "shocks") return Section::Shocks;
    if (keyword == "shocks2") return Section::Shocks2;
    if (keyword == "wheels") return Section::Wheels;
    if (keyword == "wheels2") return Section::Wheels2;
    if (keyword == "engine") return Section::Engine;
    if (keyword == "brakes") return Section::Brakes;
    return Section::Unknown;
}

bool IsKnownSectionKeyword(const std::string& keyword)
{
    return SectionForKeyword(keyword) != Section::Unknown;
}

bool IsUnsupportedSectionKeyword(const std::string& keyword)
{
    // Keep this list broad enough that lines from unsupported real truck-file
    // blocks are not accidentally interpreted as data for the previous block.
    static const char* sections[] = {
        "airbrakes", "animators", "axles", "cameras", "camerarail", "cinecam",
        "commands", "commands2", "contacters", "exhausts", "flares", "flares2",
        "flexbodies", "flexbodywheels", "fixes", "hooks", "interaxles", "lockgroups",
        "managedmaterials", "meshwheels", "meshwheels2", "particles", "props",
        "railgroups", "ropables", "ropes", "rotators", "rotators2", "screwprops",
        "shocks3", "slidenodes", "soundsources", "soundsources2", "submesh",
        "ties", "torquecurve", "triggers", "turbojets", "turboprops", "wings"
    };
    for (const char* section : sections)
    {
        if (keyword == section)
            return true;
    }
    return false;
}

void Warn(Document& document, int line_number, const std::string& text)
{
    std::ostringstream message;
    message << "line " << line_number << ": " << text;
    document.warnings.push_back(message.str());
}

} // namespace

Document Parse(const std::string& text)
{
    Document document;
    BeamDefaults beam_defaults;
    Section section = Section::None;

    std::istringstream input(text);
    std::string raw_line;
    int line_number = 0;

    while (std::getline(input, raw_line))
    {
        ++line_number;
        const std::string line = StripComments(raw_line);
        if (line.empty())
            continue;

        if (document.name.empty())
        {
            document.name = line;
            continue;
        }

        const std::vector<std::string> tokens = Tokens(line);
        if (tokens.empty())
            continue;

        const std::string keyword = Lower(tokens.front());
        if (keyword == "end")
            break;

        // RoR defaults directive; field order mirrors RigDef_Parser.
        if (keyword == "set_beam_defaults")
        {
            if (tokens.size() >= 5)
            {
                const float spring = ToFloat(tokens[1], -1.0f);
                const float damping = ToFloat(tokens[2], -1.0f);
                const float deform = ToFloat(tokens[3], -1.0f);
                const float strength = ToFloat(tokens[4], -1.0f);
                beam_defaults.spring = spring < 0.0f ? 9000000.0f : spring;
                beam_defaults.damping = damping < 0.0f ? 12000.0f : damping;
                beam_defaults.deform = deform < 0.0f ? 400000.0f : deform;
                beam_defaults.strength = strength < 0.0f ? 100000.0f : strength;
            }
            else
            {
                Warn(document, line_number, "set_beam_defaults requires four values");
            }
            continue;
        }

        if (IsKnownSectionKeyword(keyword))
        {
            section = SectionForKeyword(keyword);
            continue;
        }
        if (IsUnsupportedSectionKeyword(keyword))
        {
            section = Section::Unknown;
            continue;
        }

        switch (section)
        {
        case Section::Nodes:
            if (tokens.size() < 4)
            {
                Warn(document, line_number, "node requires id,x,y,z");
                break;
            }
            {
                Node node;
                node.id = tokens[0];
                node.x = ToFloat(tokens[1]);
                node.y = ToFloat(tokens[2]);
                node.z = ToFloat(tokens[3]);
                if (tokens.size() > 4)
                    node.options = tokens[4];
                document.nodes.push_back(node);
            }
            break;

        case Section::Beams:
            if (tokens.size() < 2)
            {
                Warn(document, line_number, "beam requires two node references");
                break;
            }
            {
                Beam beam;
                beam.node_a = tokens[0];
                beam.node_b = tokens[1];
                beam.spring = beam_defaults.spring;
                beam.damping = beam_defaults.damping;
                beam.deform = beam_defaults.deform;
                beam.strength = beam_defaults.strength;
                if (tokens.size() > 2)
                    beam.options = tokens[2];
                document.beams.push_back(beam);
            }
            break;

        case Section::Hydros:
            if (tokens.size() < 3)
            {
                Warn(document, line_number, "hydro requires nodeA,nodeB,lengtheningFactor");
                break;
            }
            {
                Hydro hydro;
                hydro.node_a = tokens[0];
                hydro.node_b = tokens[1];
                hydro.lengthening_factor = ToFloat(tokens[2]);
                if (tokens.size() > 3)
                    hydro.options = tokens[3];
                hydro.spring = beam_defaults.spring;
                hydro.damping = beam_defaults.damping;
                document.hydros.push_back(hydro);
            }
            break;

        case Section::Shocks:
        case Section::Shocks2:
            if (tokens.size() < 7)
            {
                Warn(document, line_number, "shock requires node pair, rates and bounds");
                break;
            }
            {
                Shock shock;
                shock.node_a = tokens[0];
                shock.node_b = tokens[1];
                shock.spring = ToFloat(tokens[2]);
                shock.damping = ToFloat(tokens[3]);
                if (section == Section::Shocks)
                {
                    shock.short_bound = ToFloat(tokens[4]);
                    shock.long_bound = ToFloat(tokens[5]);
                    shock.precompression = ToFloat(tokens[6], 1.0f);
                }
                else if (tokens.size() >= 13)
                {
                    // shocks2: bounds/precompression live at 10/11/12.
                    shock.short_bound = ToFloat(tokens[10]);
                    shock.long_bound = ToFloat(tokens[11]);
                    shock.precompression = ToFloat(tokens[12], 1.0f);
                }
                document.shocks.push_back(shock);
            }
            break;

        case Section::Wheels:
            if (tokens.size() < 12)
            {
                Warn(document, line_number, "wheel requires 12 values");
                break;
            }
            {
                Wheel wheel;
                wheel.wheels2 = false;
                wheel.tire_radius = ToFloat(tokens[0]);
                wheel.rim_radius = 0.0f;
                wheel.width = ToFloat(tokens[1]);
                wheel.num_rays = ToInt(tokens[2]);
                wheel.axis_node_0 = tokens[3];
                wheel.axis_node_1 = tokens[4];
                wheel.rigidity_node = tokens[5];
                wheel.braking = ToInt(tokens[6]);
                wheel.propulsion = ToInt(tokens[7]);
                wheel.reference_arm_node = tokens[8];
                wheel.mass = ToFloat(tokens[9]);
                wheel.tire_spring = ToFloat(tokens[10]);
                wheel.tire_damping = ToFloat(tokens[11]);
                document.wheels.push_back(wheel);
            }
            break;

        case Section::Wheels2:
            if (tokens.size() < 15)
            {
                Warn(document, line_number, "wheels2 requires 15 values");
                break;
            }
            {
                Wheel wheel;
                wheel.wheels2 = true;
                wheel.rim_radius = ToFloat(tokens[0]);
                wheel.tire_radius = ToFloat(tokens[1]);
                wheel.width = ToFloat(tokens[2]);
                wheel.num_rays = ToInt(tokens[3]);
                wheel.axis_node_0 = tokens[4];
                wheel.axis_node_1 = tokens[5];
                wheel.rigidity_node = tokens[6];
                wheel.braking = ToInt(tokens[7]);
                wheel.propulsion = ToInt(tokens[8]);
                wheel.reference_arm_node = tokens[9];
                wheel.mass = ToFloat(tokens[10]);
                wheel.rim_spring = ToFloat(tokens[11]);
                wheel.rim_damping = ToFloat(tokens[12]);
                wheel.tire_spring = ToFloat(tokens[13]);
                wheel.tire_damping = ToFloat(tokens[14]);
                document.wheels.push_back(wheel);
            }
            break;

        case Section::Engine:
            if (tokens.size() < 4)
            {
                Warn(document, line_number, "engine requires RPMs, torque and differential ratio");
                break;
            }
            document.engine.present = true;
            document.engine.shift_down_rpm = ToFloat(tokens[0]);
            document.engine.shift_up_rpm = ToFloat(tokens[1]);
            document.engine.torque = ToFloat(tokens[2]);
            document.engine.differential_ratio = ToFloat(tokens[3], 1.0f);
            document.engine.gear_ratios.clear();
            for (size_t i = 4; i < tokens.size(); ++i)
                document.engine.gear_ratios.push_back(ToFloat(tokens[i]));
            break;

        case Section::Brakes:
            if (tokens.empty())
                break;
            document.brakes.present = true;
            document.brakes.service_force = ToFloat(tokens[0], 30000.0f);
            if (tokens.size() > 1)
                document.brakes.parking_force = ToFloat(tokens[1], -1.0f);
            break;

        case Section::None:
        case Section::Unknown:
            break;
        }
    }

    return document;
}

} // namespace PortableRigDef
} // namespace RoR
