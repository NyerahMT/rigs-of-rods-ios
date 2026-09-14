/*
    This source file is part of the Rigs of Rods iOS port bring-up.
*/

#include "AuthoredVisualGeometry.h"
#include "PortableRigDef.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_map>

namespace RoR {
namespace IOSOgre {
namespace {

std::string Trim(const std::string& input)
{
    std::size_t first = 0;
    while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first]))) ++first;
    std::size_t last = input.size();
    while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1]))) --last;
    return input.substr(first, last - first);
}

std::string Lower(std::string input)
{
    std::transform(input.begin(), input.end(), input.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return input;
}

std::string StripComments(const std::string& input)
{
    const std::size_t semicolon = input.find(';');
    return Trim(input.substr(0, semicolon == std::string::npos ? input.size() : semicolon));
}

std::vector<std::string> Tokens(const std::string& input)
{
    std::string normalized = input;
    for (char& c : normalized) if (c == ',' || c == ':' || c == '|' || c == '\t') c = ' ';
    std::istringstream stream(normalized);
    std::vector<std::string> out;
    std::string token;
    while (stream >> token) out.push_back(token);
    return out;
}

bool IsTopLevelSection(const std::string& lower)
{
    static const char* sections[] = {
        "animators", "axles", "beams", "brakes", "cab", "cinecam", "commands", "commands2",
        "contacters", "engine", "engoption", "exhausts", "flares", "flares2", "flexbodies",
        "fusedrag", "globals", "guisettings", "help", "hooks", "hydros", "lockgroups", "managedmaterials",
        "meshwheels", "meshwheels2", "nodes", "nodes2", "particles", "props", "railgroups", "rigidifiers",
        "ropables", "ropes", "rotators", "rotators2", "screwprops", "shocks", "shocks2", "slidenodes",
        "soundsources", "soundsources2", "submesh", "ties", "torquecurve", "triggers", "turboprops2",
        "videocamera", "wheels", "wheels2", "wings"
    };
    for (const char* section : sections) if (lower == section) return true;
    return false;
}

bool ParseFloat(const std::string& text, float& value)
{
    try
    {
        std::size_t used = 0;
        value = std::stof(text, &used);
        return used == text.size();
    }
    catch (...) { return false; }
}

} // namespace

AuthoredVisualGeometry ParseAuthoredVisualGeometry(const std::string& truck_text)
{
    AuthoredVisualGeometry out;
    const PortableRigDef::Document rig = PortableRigDef::Parse(truck_text);

    std::unordered_map<std::string, std::size_t> node_index;
    for (std::size_t i = 0; i < rig.nodes.size(); ++i) node_index[rig.nodes[i].id] = i;

    for (const PortableRigDef::Wheel& wheel : rig.wheels)
    {
        const auto a = node_index.find(wheel.axis_node_0);
        const auto b = node_index.find(wheel.axis_node_1);
        if (a == node_index.end() || b == node_index.end())
        {
            out.warnings.push_back("visual wheel references a missing authored axle node");
            continue;
        }
        WheelVisual visual;
        visual.axis0 = a->second;
        visual.axis1 = b->second;
        visual.radius = wheel.tire_radius;
        visual.width = wheel.width;
        out.wheels.push_back(visual);
    }

    enum class SubmeshMode { None, Texcoords, Cab };
    bool in_submesh = false;
    SubmeshMode mode = SubmeshMode::None;
    std::unordered_map<std::string, TextureCoord> texcoords;
    std::istringstream input(truck_text);
    std::string raw;
    while (std::getline(input, raw))
    {
        const std::string line = StripComments(raw);
        if (line.empty()) continue;
        const std::string lower = Lower(line);
        if (lower == "end") break;
        if (lower == "submesh")
        {
            in_submesh = true;
            mode = SubmeshMode::None;
            texcoords.clear();
            continue;
        }
        if (!in_submesh) continue;
        if (lower == "texcoords") { mode = SubmeshMode::Texcoords; continue; }
        if (lower == "cab") { mode = SubmeshMode::Cab; continue; }
        if (lower == "backmesh") { mode = SubmeshMode::None; continue; }
        if (IsTopLevelSection(lower))
        {
            in_submesh = false;
            mode = SubmeshMode::None;
            texcoords.clear();
            continue;
        }

        const std::vector<std::string> tokens = Tokens(line);
        if (mode == SubmeshMode::Texcoords)
        {
            if (tokens.size() >= 3)
            {
                try { texcoords[tokens[0]] = {std::stof(tokens[1]), std::stof(tokens[2])}; }
                catch (...) { out.warnings.push_back("invalid authored submesh texcoord"); }
            }
            continue;
        }
        if (mode != SubmeshMode::Cab || tokens.size() < 3) continue;

        const auto a = node_index.find(tokens[0]);
        const auto b = node_index.find(tokens[1]);
        const auto c = node_index.find(tokens[2]);
        if (a == node_index.end() || b == node_index.end() || c == node_index.end())
        {
            out.warnings.push_back("cab triangle references an unknown authored node");
            continue;
        }

        CabTriangle triangle;
        triangle.a = a->second;
        triangle.b = b->second;
        triangle.c = c->second;
        const auto ua = texcoords.find(tokens[0]);
        const auto ub = texcoords.find(tokens[1]);
        const auto uc = texcoords.find(tokens[2]);
        triangle.has_uv = ua != texcoords.end() && ub != texcoords.end() && uc != texcoords.end();
        if (triangle.has_uv)
        {
            triangle.uv_a = ua->second;
            triangle.uv_b = ub->second;
            triangle.uv_c = uc->second;
        }
        triangle.contact = tokens.size() > 3 && tokens[3].find('c') != std::string::npos;
        out.cab_triangles.push_back(triangle);
    }

    // Props are rigid OGRE meshes anchored to three deforming rig nodes. Parse
    // their authored attachment frame separately so the renderer can apply the
    // same live transform used by upstream GfxActor::UpdateProps().
    bool in_props = false;
    std::istringstream prop_input(truck_text);
    while (std::getline(prop_input, raw))
    {
        const std::string line = StripComments(raw);
        if (line.empty()) continue;
        const std::string lower = Lower(line);
        if (lower == "end") break;
        if (lower == "props") { in_props = true; continue; }
        if (!in_props) continue;
        if (IsTopLevelSection(lower)) { in_props = false; continue; }

        const std::vector<std::string> tokens = Tokens(line);
        if (tokens.size() < 10) continue;
        const auto ref = node_index.find(tokens[0]);
        const auto x = node_index.find(tokens[1]);
        const auto y = node_index.find(tokens[2]);
        if (ref == node_index.end() || x == node_index.end() || y == node_index.end())
        {
            out.warnings.push_back("prop references an unknown authored node");
            continue;
        }

        PropVisual prop;
        prop.node_ref = ref->second;
        prop.node_x = x->second;
        prop.node_y = y->second;
        if (!ParseFloat(tokens[3], prop.offset_x) || !ParseFloat(tokens[4], prop.offset_y) ||
            !ParseFloat(tokens[5], prop.offset_z) || !ParseFloat(tokens[6], prop.rot_x_degrees) ||
            !ParseFloat(tokens[7], prop.rot_y_degrees) || !ParseFloat(tokens[8], prop.rot_z_degrees))
        {
            out.warnings.push_back("invalid authored prop transform");
            continue;
        }
        prop.mesh_name = tokens[9];
        out.props.push_back(std::move(prop));
    }

    if (out.cab_triangles.empty()) out.warnings.push_back("authored vehicle contains no portable cab triangles");
    return out;
}

} // namespace IOSOgre
} // namespace RoR
