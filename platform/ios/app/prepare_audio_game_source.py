#!/usr/bin/env python3
"""Generate the iOS game-view source used by the current on-device RoR bring-up.

The checked-in renderer stays deliberately small. This guarded transform wires in
AVFoundation engine audio plus the current Foxbody compatibility path and flexbody
renderer. Every source edit has an exact anchor so upstream changes fail loudly.
"""

from pathlib import Path
import sys

if len(sys.argv) != 3:
    raise SystemExit("usage: prepare_audio_game_source.py <OgreGameApp.mm> <output.mm>")

src = Path(sys.argv[1])
out = Path(sys.argv[2])
text = src.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"game source patch anchor '{label}' expected once, found {count}")
    text = text.replace(old, new, 1)


replace_once(
    '#include "AuthoredVehicleRuntime.h"\n',
    '#include "AuthoredVehicleRuntime.h"\n#include "RoREngineAudio.h"\n#include "IOSFlexBody.h"\n',
    "audio/flexbody includes",
)

replace_once(
    '#include <cmath>\n',
    '#include <cmath>\n#include <cctype>\n#include <sstream>\n',
    "foxbody parser includes",
)

parent_path = '''std::string ParentPath(const std::string& path)
{
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? std::string() : path.substr(0, slash);
}
'''

foxbody_helper = r'''

std::string TrimCopy(const std::string& input)
{
    std::size_t first = 0;
    while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first]))) ++first;
    std::size_t last = input.size();
    while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1]))) --last;
    return input.substr(first, last - first);
}

std::string LowerCopy(std::string input)
{
    std::transform(input.begin(), input.end(), input.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return input;
}

std::vector<std::string> CommaFields(const std::string& input)
{
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (start <= input.size())
    {
        const std::size_t comma = input.find(',', start);
        const std::size_t end = comma == std::string::npos ? input.size() : comma;
        fields.push_back(TrimCopy(input.substr(start, end - start)));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return fields;
}

// The community Foxbody uses optional `section` modules for its wheel packages and
// classic `meshwheels`. Our portable physics runtime does not understand those two
// legacy features yet. Flatten one authored module and map only its structural wheel
// fields to wheels2; the visual mesh package itself remains untouched.
std::string PrepareFoxbodyTruckText(const std::string& source)
{
    const std::string selected_module = "saleen_wheels";
    std::istringstream input(source);
    std::ostringstream output;
    std::string raw;
    bool in_module = false;
    bool keep_module = false;
    bool in_meshwheels = false;
    bool selected_seen = false;
    int converted_wheels = 0;

    while (std::getline(input, raw))
    {
        const std::size_t semicolon = raw.find(';');
        const std::string code = TrimCopy(raw.substr(0, semicolon == std::string::npos ? raw.size() : semicolon));
        const std::string lower = LowerCopy(code);

        if (lower.rfind("section ", 0) == 0)
        {
            std::istringstream section(code);
            std::string token;
            std::string module;
            while (section >> token) module = token;
            in_module = true;
            keep_module = LowerCopy(module) == selected_module;
            selected_seen = selected_seen || keep_module;
            in_meshwheels = false;
            continue;
        }
        if (lower == "end_section")
        {
            in_module = false;
            keep_module = false;
            in_meshwheels = false;
            continue;
        }
        if (in_module && !keep_module) continue;
        if (lower.rfind("sectionconfig ", 0) == 0) continue;

        // These directives are valid RoR, but the current portable parser does
        // not implement their state. Leaving them inside nodes/beams would make
        // it mistake the directive text for structural data.
        if (lower.rfind("set_beam_defaults_scale", 0) == 0 ||
            lower.rfind("set_node_defaults", 0) == 0)
            continue;

        if (lower == "meshwheels")
        {
            output << "wheels2\n";
            in_meshwheels = true;
            continue;
        }

        if (in_meshwheels)
        {
            if (lower == "flexbodies")
            {
                in_meshwheels = false;
                output << raw << '\n';
                continue;
            }
            if (code.empty())
            {
                output << raw << '\n';
                continue;
            }

            const std::vector<std::string> f = CommaFields(code);
            const bool numeric = !f.empty() && !f[0].empty() &&
                (std::isdigit(static_cast<unsigned char>(f[0][0])) || f[0][0] == '-' || f[0][0] == '+' || f[0][0] == '.');
            if (numeric && f.size() >= 13)
            {
                // meshwheels: tyre,rim,width,rays,n0,n1,rigidity,braking,propulsion,arm,mass,spring,damping,...
                // wheels2:    rim,tyre,width,rays,n0,n1,rigidity,braking,propulsion,arm,mass,rimK,rimD,tyreK,tyreD
                output << f[1] << ", " << f[0] << ", " << f[2] << ", " << f[3] << ", "
                       << f[4] << ", " << f[5] << ", " << f[6] << ", " << f[7] << ", "
                       << f[8] << ", " << f[9] << ", " << f[10] << ", " << f[11] << ", "
                       << f[12] << ", " << f[11] << ", " << f[12] << '\n';
                ++converted_wheels;
                continue;
            }
            if (!code.empty() && std::isalpha(static_cast<unsigned char>(code[0])))
                in_meshwheels = false;
        }

        // These upstream keywords can otherwise inherit the previous portable
        // parser section (engine, brakes or contacters). `help` is an explicitly
        // unsupported block, so it safely neutralises their following payload.
        if (lower == "videocamera" || lower == "engoption" || lower == "fusedrag" ||
            lower.rfind("tractioncontrol", 0) == 0 || lower.rfind("cruisecontrol", 0) == 0)
        {
            output << "help\n";
            continue;
        }

        output << raw << '\n';
    }

    if (!selected_seen || converted_wheels != 4)
        return std::string();
    return output.str();
}
'''

replace_once(parent_path, parent_path + foxbody_helper, "foxbody compatibility helper")

replace_once(
    '        if (view) [view removeFromSuperview];\n        terrain.reset();\n',
    '        if (view) [view removeFromSuperview];\n        flexbody_instances.clear();\n        terrain.reset();\n',
    "flexbody teardown",
)

replace_once(
    '            UpdateBody(s, visual);\n            UpdateWheels(s, visual);\n            UpdateProps(s, visual);\n            UpdateCamera(s.telemetry);\n',
    '            UpdateBody(s, visual);\n            UpdateWheels(s, visual);\n            UpdateProps(s, visual);\n            UpdateFlexBodies(s, visual);\n            UpdateCamera(s.telemetry);\n',
    "flexbody frame update",
)

replace_once(
    '        const std::string terrain_path = ParentPath(content_path) + "/simple2-terrain";\n        auto& groups = Ogre::ResourceGroupManager::getSingleton();\n        groups.addResourceLocation(media_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);\n        groups.addResourceLocation(content_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);\n',
    '        const std::string terrain_path = ParentPath(content_path) + "/simple2-terrain";\n'
    '        const std::string daf_path = ParentPath(content_path) + "/dafsemi";\n'
    '        auto& groups = Ogre::ResourceGroupManager::getSingleton();\n'
    '        groups.addResourceLocation(media_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);\n'
    '        groups.addResourceLocation(content_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);\n'
    '        groups.addResourceLocation(daf_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);\n',
    "foxbody + DAF resource locations",
)

replace_once(
    '    void UpdateBody(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)\n    {\n        if (visual.cab_triangles.empty()) return;\n',
    '    void UpdateBody(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)\n    {\n'
    '        // Foxbody collision cab triangles are not the visible body. Its authored\n'
    '        // flexbody meshes below are the actual exterior/interior.\n'
    '        if (!visual.flexbodies.empty()) return;\n'
    '        if (visual.cab_triangles.empty()) return;\n',
    "hide cab shell for flexbody vehicles",
)

flexbody_methods = r'''
    void EnsureFlexBodies(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (flexbodies_built || !scene || s.nodes.empty()) return;
        flexbodies_built = true;

        std::vector<Ogre::Vector3> nodes;
        nodes.reserve(s.nodes.size());
        for (const RoR::PhysicsVec3& node : s.nodes) nodes.push_back(OgreVec(node));

        for (std::size_t i = 0; i < visual.flexbodies.size(); ++i)
        {
            std::unique_ptr<RoR::IOSOgre::IOSFlexBody> instance(new RoR::IOSOgre::IOSFlexBody(
                scene, visual.flexbodies[i], nodes, std::to_string(i)));
            if (instance->Ready())
            {
                flexbody_instances.push_back(std::move(instance));
            }
            else
            {
                Ogre::LogManager::getSingleton().logMessage(
                    "iOS flexbody skipped: " + visual.flexbodies[i].mesh_name + " - " + instance->Error());
            }
        }
    }

    void UpdateFlexBodies(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        EnsureFlexBodies(s, visual);
        if (flexbody_instances.empty()) return;

        std::vector<Ogre::Vector3> nodes;
        nodes.reserve(s.nodes.size());
        for (const RoR::PhysicsVec3& node : s.nodes) nodes.push_back(OgreVec(node));
        for (auto& instance : flexbody_instances) instance->Update(nodes, vehicle_world_offset);
    }

'''

replace_once(
    '    void UpdateCamera(const RoR::IOSVehicleCore::AuthoredVehicleTelemetry& t)\n',
    flexbody_methods + '    void UpdateCamera(const RoR::IOSVehicleCore::AuthoredVehicleTelemetry& t)\n',
    "flexbody renderer methods",
)

replace_once(
    '    std::unique_ptr<RoR::IOSOgre::RoRTerrainScene> terrain;\n    std::vector<PropInstance> prop_instances;\n',
    '    std::unique_ptr<RoR::IOSOgre::RoRTerrainScene> terrain;\n'
    '    std::vector<PropInstance> prop_instances;\n'
    '    std::vector<std::unique_ptr<RoR::IOSOgre::IOSFlexBody>> flexbody_instances;\n',
    "flexbody renderer storage",
)

replace_once(
    '    bool body_built=false, wheels_built=false, props_built=false, camera_started=false;\n',
    '    bool body_built=false, wheels_built=false, props_built=false, flexbodies_built=false, camera_started=false;\n',
    "flexbody renderer state",
)

replace_once(
    '    SimulationHost* _simulation; OgreRenderer* _renderer; UIView* _ogreView; CADisplayLink* _displayLink; UILabel* _speed; UILabel* _status;\n',
    '    SimulationHost* _simulation; OgreRenderer* _renderer; UIView* _ogreView; CADisplayLink* _displayLink; UILabel* _speed; UILabel* _status;\n'
    '    std::unique_ptr<RoR::IOSAudio::EngineAudio> _engineAudio;\n',
    "controller audio member",
)

replace_once(
    '    NSString* path=[[NSBundle mainBundle] pathForResource:@"b6b0UID-semi" ofType:@"truck" inDirectory:@"Content/dafsemi"];\n'
    '    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;\n'
    '    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));\n',
    '    NSString* path=[[NSBundle mainBundle] pathForResource:@"Foxbody" ofType:@"truck" inDirectory:@"Content/foxbody-mustang"];\n'
    '    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;\n'
    '    std::string foxbody=PrepareFoxbodyTruckText(std::string(text?text.UTF8String:""));\n'
    '    _simulation=new SimulationHost(foxbody);\n'
    '    NSString* audioPath=[[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"Content/foxbody-audio"];\n'
    '    _engineAudio.reset(new RoR::IOSAudio::EngineAudio(std::string(audioPath.UTF8String)));\n'
    '    if(_engineAudio->Ready()){_engineAudio->Start();_engineAudio->PlayStarter();}\n',
    "Foxbody vehicle + audio construction",
)

replace_once(
    '- (void)dealloc{[_displayLink invalidate];delete _renderer;delete _simulation;}\n',
    '- (void)dealloc{[_displayLink invalidate];_engineAudio.reset();delete _renderer;delete _simulation;}\n',
    "audio teardown",
)

replace_once(
    'NSString* content=[rootPath stringByAppendingPathComponent:@"Content/dafsemi"];',
    'NSString* content=[rootPath stringByAppendingPathComponent:@"Content/foxbody-mustang"];',
    "Foxbody renderer content path",
)

replace_once(
    '    Snapshot s=_simulation->GetSnapshot(); if(_renderer)_renderer->Draw(s,_simulation->Visual()); _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];\n',
    '    Snapshot s=_simulation->GetSnapshot(); if(_engineAudio&&_engineAudio->Ready())_engineAudio->UpdateFromVehicle(s.telemetry.driven_wheel_speed_mps,s.telemetry.throttle); if(_renderer)_renderer->Draw(s,_simulation->Visual()); _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];\n',
    "audio frame update",
)

replace_once(
    'else if(_renderer)_status.text=[NSString stringWithFormat:@"OGRE 14.6 • %s • %.0f FPS\\nROR PHYSICS 2,000 HZ • %llu STEPS",_renderer->TerrainStatus().c_str(),_fps,s.telemetry.physics_steps];\n',
    'else if(_renderer)_status.text=[NSString stringWithFormat:@"OGRE 14.6 • FOXBODY • %s • %.0f FPS\\nROR PHYSICS 2,000 HZ • %llu STEPS",_renderer->TerrainStatus().c_str(),_fps,s.telemetry.physics_steps];\n',
    "Foxbody status",
)

replace_once(
    '- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();if(_renderer)_renderer->ResetLook();}\n',
    '- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();if(_engineAudio&&_engineAudio->Ready())_engineAudio->PlayStarter();if(_renderer)_renderer->ResetLook();}\n',
    "starter on reset",
)

out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(text)
print(f"Generated Foxbody + audio game source: {out}")
