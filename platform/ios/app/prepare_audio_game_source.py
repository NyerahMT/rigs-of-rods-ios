#!/usr/bin/env python3
"""Generate the iOS game-view source for the current on-device Bandit parity test.

The generated Objective-C++ shell deliberately launches Gabester's Bandit 400 GT,
uses the authored flexbody meshes/material names, and drives the AVFoundation engine
loop from the same drivetrain RPM that produces wheel torque.
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
    '#include "AuthoredVehicleRuntime.h"\n#include "IOSFlexBody.h"\n#include "RoREngineAudio.h"\n',
    "flexbody/audio includes",
)

replace_once(
    '#include <cmath>\n',
    '#include <cmath>\n#include <cctype>\n#include <sstream>\n',
    "parser/diagnostic includes",
)

managed_material_helpers = r'''
struct ManagedMaterialDef
{
    std::string name;
    std::string type;
    std::string diffuse;
};

std::string TrimRigText(const std::string& value)
{
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string LowerRigText(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::vector<std::string> RigTokens(std::string line)
{
    for (char& c : line) if (c == ',' || c == '\t') c = ' ';
    std::istringstream stream(line);
    std::vector<std::string> out;
    std::string token;
    while (stream >> token) out.push_back(token);
    return out;
}

bool IsTopLevelRigSection(const std::string& lower)
{
    static const char* sections[] = {
        "airbrakes", "animators", "axles", "beams", "brakes", "cab", "cinecam",
        "commands", "commands2", "contacters", "engine", "engoption", "exhausts",
        "flares", "flares2", "flexbodies", "globals", "guisettings", "hooks", "hydros",
        "managedmaterials", "meshwheels", "meshwheels2", "nodes", "nodes2", "props",
        "shocks", "shocks2", "slidenodes", "soundsources", "soundsources2", "submesh",
        "ties", "torquecurve", "triggers", "wheels", "wheels2", "wings"
    };
    for (const char* section : sections) if (lower == section) return true;
    return false;
}

std::vector<ManagedMaterialDef> ParseManagedMaterials(const std::string& truck_text)
{
    std::vector<ManagedMaterialDef> out;
    bool active = false;
    std::istringstream input(truck_text);
    std::string raw;
    while (std::getline(input, raw))
    {
        const std::size_t comment = raw.find(';');
        if (comment != std::string::npos) raw.resize(comment);
        const std::string line = TrimRigText(raw);
        if (line.empty()) continue;
        const std::string lower = LowerRigText(line);
        if (lower == "managedmaterials") { active = true; continue; }
        if (!active) continue;
        if (lower == "end" || IsTopLevelRigSection(lower)) break;

        const std::vector<std::string> tokens = RigTokens(line);
        if (tokens.size() < 3) continue;
        ManagedMaterialDef def;
        def.name = tokens[0];
        def.type = LowerRigText(tokens[1]);
        def.diffuse = tokens[2];
        out.push_back(std::move(def));
    }
    return out;
}

'''

replace_once(
    'struct Snapshot\n{\n',
    managed_material_helpers + 'struct Snapshot\n{\n',
    "managed material parser",
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
                   << " • STEP " << s.telemetry.physics_steps
                   << " • RPM " << static_cast<int>(s.telemetry.engine_rpm)
                   << " G" << s.telemetry.gear;
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
    '''    OgreRenderer(CGSize size, const std::string& media_path, const std::string& content_path,
                 const std::string& prop_mesh_path)
    {
        Initialise(size, media_path, content_path, prop_mesh_path);
    }
''',
    '''    OgreRenderer(CGSize size, const std::string& media_path, const std::string& content_path,
                 const std::string& prop_mesh_path, const std::string& truck_text)
    {
        Initialise(size, media_path, content_path, prop_mesh_path, truck_text);
    }
''',
    "renderer accepts truck text",
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
    '''    void Initialise(CGSize size, const std::string& media_path, const std::string& content_path,
                    const std::string& prop_mesh_path)
''',
    '''    void Initialise(CGSize size, const std::string& media_path, const std::string& content_path,
                    const std::string& prop_mesh_path, const std::string& truck_text)
''',
    "renderer initialise truck text",
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
    '        groups.initialiseAllResourceGroups();\n        CreateMaterials();\n',
    '        groups.initialiseAllResourceGroups();\n        CreateMaterials();\n        CreateAuthoredManagedMaterials(truck_text);\n',
    "create authored managed materials",
)

replace_once(
    '        tvp->load(); tfp->load(); efp->load();\n',
    '''        tvp->load(); tfp->load(); efp->load();

        Ogre::GpuProgramPtr fvp = programs.createProgram("RoRFlexVP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_VERTEX_PROGRAM);
        fvp->setSourceFile("RoRGame.metal"); fvp->setParameter("entry_point", "ror_flex_vp");
        Ogre::GpuProgramPtr ffp = programs.createProgram("RoRFlexFP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_FRAGMENT_PROGRAM);
        ffp->setSourceFile("RoRGame.metal"); ffp->setParameter("entry_point", "ror_flex_fp"); ffp->setParameter("shader_reflection_pair_hint", "RoRFlexVP");
        fvp->load(); ffp->load();
''',
    "flexbody Metal programs",
)

managed_material_method = r'''
    void CreateAuthoredManagedMaterials(const std::string& truck_text)
    {
        const std::vector<ManagedMaterialDef> definitions = ParseManagedMaterials(truck_text);
        auto& manager = Ogre::MaterialManager::getSingleton();
        std::size_t created = 0;
        for (const ManagedMaterialDef& def : definitions)
        {
            if (def.name.empty() || def.diffuse.empty() || def.diffuse == "-") continue;
            Ogre::MaterialPtr material = manager.getByName(
                def.name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            if (material.isNull())
                material = manager.create(def.name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            material->removeAllTechniques();
            Ogre::Pass* pass = material->createTechnique()->createPass();
            const bool transparent = def.type.find("transparent") != std::string::npos;
            pass->setLightingEnabled(false);
            pass->setCullingMode(Ogre::CULL_NONE);
            pass->setDepthCheckEnabled(true);
            pass->setDepthWriteEnabled(!transparent);
            if (transparent)
                pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
            pass->setVertexProgram("RoRFlexVP");
            pass->setFragmentProgram("RoRFlexFP");
            pass->getVertexProgramParameters()->setNamedAutoConstant(
                "mvpMtx", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
            Ogre::TextureUnitState* texture = pass->createTextureUnitState(def.diffuse);
            texture->setTextureFiltering(Ogre::TFO_ANISOTROPIC);
            texture->setTextureAnisotropy(8);
            material->load();
            ++created;
        }
        managed_material_status = std::to_string(created) + " managed materials";
    }

'''

replace_once(
    '    void CreateGround()\n',
    managed_material_method + '    void CreateGround()\n',
    "managed material renderer method",
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

        std::size_t failed = 0;
        std::string last_error;
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
                ++failed;
                last_error = instance->Error();
                Ogre::LogManager::getSingleton().logMessage(
                    "iOS flexbody skipped: " + visual.flexbodies[i].mesh_name + " - " + instance->Error());
            }
        }
        std::ostringstream status;
        status << flexbody_instances.size() << "/" << visual.flexbodies.size() << " flexbodies";
        if (failed) status << " • " << failed << " failed: " << last_error;
        flexbody_status = status.str();
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
    '    std::string terrain_status="NO TERRAIN";\n',
    '    std::string terrain_status="NO TERRAIN";\n'
    '    std::string managed_material_status="0 managed materials";\n'
    '    std::string flexbody_status="flexbodies pending";\n',
    "visual status storage",
)

replace_once(
    '    bool body_built=false, wheels_built=false, props_built=false, camera_started=false;\n',
    '    bool body_built=false, wheels_built=false, props_built=false, flexbodies_built=false, camera_started=false;\n',
    "flexbody renderer state",
)

replace_once(
    '    const std::string& TerrainStatus() const { return terrain_status; }\n',
    '    const std::string& TerrainStatus() const { return terrain_status; }\n'
    '    std::string VehicleVisualStatus() const { return managed_material_status + " • " + flexbody_status; }\n',
    "renderer visual status",
)

replace_once(
    '    SimulationHost* _simulation; OgreRenderer* _renderer; UIView* _ogreView; CADisplayLink* _displayLink; UILabel* _speed; UILabel* _status;\n',
    '    SimulationHost* _simulation; OgreRenderer* _renderer; RoR::IOSAudio::EngineAudio* _engineAudio; UIView* _ogreView; CADisplayLink* _displayLink; UILabel* _speed; UILabel* _status;\n'
    '    std::string _truckText; std::string _audioError;\n',
    "audio controller storage",
)

replace_once(
    '    NSString* path=[[NSBundle mainBundle] pathForResource:@"b6b0UID-semi" ofType:@"truck" inDirectory:@"Content/dafsemi"];\n'
    '    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;\n'
    '    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));\n',
    '''    NSString* path=[[NSBundle mainBundle] pathForResource:@"Bandit" ofType:@"truck" inDirectory:@"Content/bandit"];
    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;
    _truckText=std::string(text?text.UTF8String:"");
    _simulation=new SimulationHost(_truckText);
    NSString* rootPath=[[NSBundle mainBundle] resourcePath];
    NSString* audioPath=[rootPath stringByAppendingPathComponent:@"Content/bandit"];
    _engineAudio=new RoR::IOSAudio::EngineAudio(
        std::string(audioPath.UTF8String), "gavrilbandit.soundscript", _truckText);
    if(_engineAudio->Ready()) { _engineAudio->Start(); _engineAudio->PlayStarter(); }
    else _audioError=_engineAudio->Error();
''',
    "Bandit vehicle and audio construction",
)

replace_once(
    '_status.numberOfLines=2;',
    '_status.numberOfLines=7;',
    "expanded diagnostics label",
)

replace_once(
    '- (void)viewDidAppear:(BOOL)animated{[super viewDidAppear:animated];[self ensureRenderer];}\n',
    '- (void)viewDidAppear:(BOOL)animated{[super viewDidAppear:animated];[self ensureRenderer];if(_engineAudio)_engineAudio->Start();}\n',
    "resume engine audio",
)

replace_once(
    '- (void)dealloc{[_displayLink invalidate];delete _renderer;delete _simulation;}\n',
    '- (void)dealloc{[_displayLink invalidate];delete _engineAudio;delete _renderer;delete _simulation;}\n',
    "audio teardown",
)

replace_once(
    'NSString* content=[rootPath stringByAppendingPathComponent:@"Content/dafsemi"];',
    'NSString* content=[rootPath stringByAppendingPathComponent:@"Content/bandit"];',
    "Bandit renderer content path",
)

replace_once(
    'try{_renderer=new OgreRenderer(self.view.bounds.size,std::string(media.UTF8String),std::string(content.UTF8String),std::string(propMeshes.UTF8String));',
    'try{_renderer=new OgreRenderer(self.view.bounds.size,std::string(media.UTF8String),std::string(content.UTF8String),std::string(propMeshes.UTF8String),_truckText);',
    "renderer receives Bandit truck text",
)

replace_once(
    '    Snapshot s=_simulation->GetSnapshot(); if(_renderer)_renderer->Draw(s,_simulation->Visual()); _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];\n',
    '    Snapshot s=_simulation->GetSnapshot(); if(_renderer)_renderer->Draw(s,_simulation->Visual()); if(_engineAudio)_engineAudio->UpdateFromEngine(s.telemetry.engine_rpm,s.telemetry.throttle); _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];\n',
    "drive audio from drivetrain telemetry",
)

replace_once(
    '    if(!s.ready)_status.text=[NSString stringWithFormat:@"VEHICLE LOAD FAILED\\n%s",s.error.c_str()]; else if(!s.finite)_status.text=@"PHYSICS STOPPED • TAP RESET"; else if(_renderer)_status.text=[NSString stringWithFormat:@"OGRE 14.6 • %s • %.0f FPS\\nROR PHYSICS 2,000 HZ • %llu STEPS",_renderer->TerrainStatus().c_str(),_fps,s.telemetry.physics_steps];\n',
    '''    const char* audioStatus=_audioError.empty()?"AUDIO OK":_audioError.c_str();
    const std::string visualStatus=_renderer?_renderer->VehicleVisualStatus():"renderer pending";
    if(!s.ready)_status.text=[NSString stringWithFormat:@"VEHICLE LOAD FAILED\\n%s\\n%s",s.error.c_str(),s.diagnostic.c_str()];
    else if(!s.finite)_status.text=[NSString stringWithFormat:@"PHYSICS STOPPED • TAP RESET\\n%s",s.diagnostic.c_str()];
    else if(_renderer)_status.text=[NSString stringWithFormat:@"OGRE 14.6 • BANDIT 400 GT • %s • %.0f FPS\\nROR PHYSICS 2,000 HZ • %llu STEPS\\n%s\\n%s\\n%s",_renderer->TerrainStatus().c_str(),_fps,s.telemetry.physics_steps,s.diagnostic.c_str(),visualStatus.c_str(),audioStatus];
''',
    "Bandit diagnostic/audio status",
)

replace_once(
    '- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();if(_renderer)_renderer->ResetLook();}\n',
    '- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();if(_renderer)_renderer->ResetLook();if(_engineAudio)_engineAudio->PlayStarter();}\n',
    "starter on reset",
)

out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(text)
print(f"Generated Bandit 400 GT flexbody/material/audio parity-test source: {out}")
