#!/usr/bin/env python3
"""Generate the iOS game-view source for the current on-device parity test.

The device build deliberately launches the same Gabester Bandit 400 GT used by
CI's authored-vehicle smoke probe. This removes the Foxbody compatibility shim as
a variable while we validate the portable RoR loader/physics path on real iOS.
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
    '#include "AuthoredVehicleRuntime.h"\n#include "IOSFlexBody.h"\n',
    "flexbody include",
)

replace_once(
    '#include <cmath>\n',
    '#include <cmath>\n#include <sstream>\n',
    "diagnostic stream include",
)

replace_once(
    '    std::string error;\n    std::vector<RoR::PhysicsVec3> nodes;\n',
    '    std::string error;\n    std::string diagnostic;\n    std::vector<RoR::PhysicsVec3> nodes;\n',
    "snapshot diagnostic field",
)

replace_once(
    '        if (!runtime.Errors().empty()) s.error = runtime.Errors().front();\n        s.nodes.reserve(runtime.NodeCount());\n',
    '''        if (!runtime.Errors().empty()) s.error = runtime.Errors().front();
        std::ostringstream diagnostic;
        diagnostic << runtime.VehicleName()
                   << " • N" << runtime.NodeCount()
                   << " B" << runtime.BeamPairs().size()
                   << " T" << runtime.TireNodeIndices().size()
                   << " • STEP " << s.telemetry.physics_steps;
        const auto& early = runtime.EarlyDiagnostics();
        if (!runtime.IsFinite() && !early.empty())
        {
            const auto& d = early.back();
            diagnostic << "\\nN" << d.worst_node
                       << " V=" << std::sqrt(d.velocity.squaredLength())
                       << " M=" << d.mass
                       << " • B" << d.worst_beam
                       << " " << d.beam_a << "-" << d.beam_b
                       << " L=" << d.beam_length << "/" << d.beam_rest_length
                       << " S=" << d.beam_stress;
        }
        s.diagnostic = diagnostic.str();
        s.nodes.reserve(runtime.NodeCount());
''',
    "publish runtime diagnostics",
)

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
    "Bandit + DAF resource locations",
)

replace_once(
    '    void UpdateBody(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)\n    {\n        if (visual.cab_triangles.empty()) return;\n',
    '    void UpdateBody(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)\n    {\n'
    '        // A flexbody is the visible shell when one is authored; do not also\n'
    '        // draw the collision cab as a second body over it.\n'
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
    '    NSString* path=[[NSBundle mainBundle] pathForResource:@"b6b0UID-semi" ofType:@"truck" inDirectory:@"Content/dafsemi"];\n'
    '    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;\n'
    '    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));\n',
    '    NSString* path=[[NSBundle mainBundle] pathForResource:@"Bandit" ofType:@"truck" inDirectory:@"Content/bandit"];\n'
    '    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;\n'
    '    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));\n',
    "Bandit vehicle construction",
)

replace_once(
    '_status.numberOfLines=2;',
    '_status.numberOfLines=5;',
    "expanded diagnostics label",
)

replace_once(
    'NSString* content=[rootPath stringByAppendingPathComponent:@"Content/dafsemi"];',
    'NSString* content=[rootPath stringByAppendingPathComponent:@"Content/bandit"];',
    "Bandit renderer content path",
)

replace_once(
    '    if(!s.ready)_status.text=[NSString stringWithFormat:@"VEHICLE LOAD FAILED\\n%s",s.error.c_str()]; else if(!s.finite)_status.text=@"PHYSICS STOPPED • TAP RESET"; else if(_renderer)_status.text=[NSString stringWithFormat:@"OGRE 14.6 • %s • %.0f FPS\\nROR PHYSICS 2,000 HZ • %llu STEPS",_renderer->TerrainStatus().c_str(),_fps,s.telemetry.physics_steps];\n',
    '    if(!s.ready)_status.text=[NSString stringWithFormat:@"VEHICLE LOAD FAILED\\n%s\\n%s",s.error.c_str(),s.diagnostic.c_str()]; else if(!s.finite)_status.text=[NSString stringWithFormat:@"PHYSICS STOPPED • TAP RESET\\n%s",s.diagnostic.c_str()]; else if(_renderer)_status.text=[NSString stringWithFormat:@"OGRE 14.6 • BANDIT 400 GT • %s • %.0f FPS\\nROR PHYSICS 2,000 HZ • %llu STEPS\\n%s",_renderer->TerrainStatus().c_str(),_fps,s.telemetry.physics_steps,s.diagnostic.c_str()];\n',
    "Bandit diagnostic status",
)

out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(text)
print(f"Generated Bandit 400 GT parity-test game source: {out}")
