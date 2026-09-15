#!/usr/bin/env python3
"""Patch the generated Bandit Objective-C++ shell with RoR-style meshwheel visuals.

The physics runtime generates the same alternating 2*num_rays tyre-node ring used
by ActorSpawner::BuildWheelObjectAndNodes(). This renderer consumes those live
nodes directly, reproduces FlexMeshWheel's five-band tyre surface, and attaches
the authored static rim mesh with the same side-dependent orientation as upstream.
"""

from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_meshwheel_renderer.py <generated-OgreGameApp.mm>")

p = Path(sys.argv[1])
s = p.read_text()


def one(old: str, new: str, label: str) -> None:
    global s
    if s.count(old) != 1:
        raise SystemExit(f"meshwheel renderer anchor '{label}' expected once, found {s.count(old)}")
    s = s.replace(old, new, 1)


def between(start: str, end: str, replacement: str, label: str) -> None:
    global s
    a = s.find(start)
    b = s.find(end, a + len(start)) if a >= 0 else -1
    if a < 0 or b < 0:
        raise SystemExit(f"meshwheel renderer range '{label}' drifted")
    s = s[:a] + replacement + "\n\n" + s[b:]


one(
    '#include "RoREngineAudio.h"\n',
    '#include "RoREngineAudio.h"\n#include "PortableRigDef.h"\n',
    "portable RigDef include")

wheel_defs = r'''
struct MeshWheelRenderDef
{
    std::size_t axis0 = 0;
    std::size_t axis1 = 0;
    std::size_t node_start = 0;
    int num_rays = 0;
    float tire_radius = 0.0f;
    float rim_radius = 0.0f;
    bool reverse_rim = false;
    std::string rim_mesh;
    std::string tire_material;
};

std::vector<MeshWheelRenderDef> ParseMeshWheelRenderDefs(const std::string& truck_text)
{
    const RoR::PortableRigDef::Document rig = RoR::PortableRigDef::Parse(truck_text);
    std::unordered_map<std::string, std::size_t> node_index;
    for (std::size_t i = 0; i < rig.nodes.size(); ++i)
        node_index[rig.nodes[i].id] = i;

    std::vector<MeshWheelRenderDef> out;
    std::size_t generated_start = rig.nodes.size();
    for (const RoR::PortableRigDef::Wheel& wheel : rig.wheels)
    {
        const std::size_t this_start = generated_start;
        generated_start += static_cast<std::size_t>(std::max(0, wheel.num_rays)) * (wheel.wheels2 ? 4u : 2u);

        if ((!wheel.meshwheel && !wheel.meshwheels2) || wheel.num_rays < 3 ||
            wheel.mesh_name.empty() || wheel.material_name.empty())
            continue;

        const auto a = node_index.find(wheel.axis_node_0);
        const auto b = node_index.find(wheel.axis_node_1);
        if (a == node_index.end() || b == node_index.end())
            continue;

        MeshWheelRenderDef def;
        def.axis0 = a->second;
        def.axis1 = b->second;
        // Match ActorSpawner::GetWheelAxisNodes() and the portable runtime.
        if (rig.nodes[def.axis0].z > rig.nodes[def.axis1].z)
            std::swap(def.axis0, def.axis1);
        def.node_start = this_start;
        def.num_rays = wheel.num_rays;
        def.tire_radius = wheel.tire_radius;
        def.rim_radius = wheel.rim_radius;
        def.reverse_rim = wheel.side != 'r'; // upstream: side != WheelSide::RIGHT
        def.rim_mesh = wheel.mesh_name;
        def.tire_material = wheel.material_name;
        out.push_back(std::move(def));
    }
    return out;
}

'''
one('struct Snapshot\n{\n', wheel_defs + 'struct Snapshot\n{\n', "meshwheel metadata parser")

# The prepared controller already receives truck_text. Preserve wheel metadata once
# rather than reparsing the truck every frame.
one(
'''    OgreRenderer(CGSize size, const std::string& media_path, const std::string& content_path,
                 const std::string& prop_mesh_path, const std::string& truck_text)
    {
        Initialise(size, media_path, content_path, prop_mesh_path, truck_text);
    }
''',
'''    OgreRenderer(CGSize size, const std::string& media_path, const std::string& content_path,
                 const std::string& prop_mesh_path, const std::string& truck_text)
        : mesh_wheel_defs(ParseMeshWheelRenderDefs(truck_text))
    {
        Initialise(size, media_path, content_path, prop_mesh_path, truck_text);
        meshwheel_status = mesh_wheel_defs.empty()
            ? "no authored meshwheels"
            : std::to_string(mesh_wheel_defs.size()) + " authored meshwheels";
    }
''',
"renderer wheel metadata initialization")

# ManualObject section matching the RoRFlexVP declaration: POSITION + TEXCOORD0.
one(
'''    static void VT(Ogre::ManualObject* o, const Ogre::Vector3& p, const Ogre::ColourValue& c,
                   const RoR::IOSOgre::TextureCoord& uv)
    {
        o->position(p); o->colour(c); o->textureCoord(uv.u, uv.v);
    }
''',
'''    static void VT(Ogre::ManualObject* o, const Ogre::Vector3& p, const Ogre::ColourValue& c,
                   const RoR::IOSOgre::TextureCoord& uv)
    {
        o->position(p); o->colour(c); o->textureCoord(uv.u, uv.v);
    }

    static void WheelVT(Ogre::ManualObject* o, const Ogre::Vector3& p, float u, float v)
    {
        o->position(p); o->textureCoord(u, v);
    }

    static void WheelQuad(Ogre::ManualObject* o,
                          const Ogre::Vector3& a, const Ogre::Vector3& b,
                          const Ogre::Vector3& c, const Ogre::Vector3& d,
                          float u0, float u1, float v0, float v1)
    {
        WheelVT(o, a, u0, v0); WheelVT(o, b, u0, v1); WheelVT(o, c, u1, v1);
        WheelVT(o, a, u0, v0); WheelVT(o, c, u1, v1); WheelVT(o, d, u1, v0);
    }
''',
"wheel textured vertex helpers")

# Instance state lives inside OgreRenderer so each authored wheel owns one static
# rim entity and one dynamic tyre ManualObject.
one(
'''    struct PropInstance
    {
        std::size_t visual_index = 0;
        Ogre::SceneNode* node = nullptr;
        Ogre::Entity* entity = nullptr;
    };
''',
'''    struct PropInstance
    {
        std::size_t visual_index = 0;
        Ogre::SceneNode* node = nullptr;
        Ogre::Entity* entity = nullptr;
    };

    struct MeshWheelInstance
    {
        std::size_t definition_index = 0;
        Ogre::SceneNode* rim_node = nullptr;
        Ogre::Entity* rim_entity = nullptr;
        Ogre::ManualObject* tire = nullptr;
        bool tire_built = false;
    };
''',
"wheel renderer instance")

wheel_methods = r'''    void EnsureMeshWheels(const Snapshot& s)
    {
        if (meshwheels_built || mesh_wheel_defs.empty() || !scene) return;
        meshwheels_built = true;
        std::size_t failed = 0;
        std::string last_error;

        for (std::size_t i = 0; i < mesh_wheel_defs.size(); ++i)
        {
            const MeshWheelRenderDef& def = mesh_wheel_defs[i];
            if (def.axis0 >= s.nodes.size() || def.axis1 >= s.nodes.size() ||
                def.node_start + static_cast<std::size_t>(def.num_rays * 2) > s.nodes.size())
            {
                ++failed;
                last_error = "generated tyre node range is outside runtime snapshot";
                continue;
            }

            try
            {
                const std::string suffix = std::to_string(i);
                Ogre::Entity* rim = scene->createEntity("RoRMeshWheelRimEntity" + suffix, def.rim_mesh);
                rim->setCastShadows(false);
                Ogre::SceneNode* rim_node = scene->getRootSceneNode()->createChildSceneNode("RoRMeshWheelRimNode" + suffix);
                rim_node->attachObject(rim);

                Ogre::ManualObject* tire = scene->createManualObject("RoRMeshWheelTire" + suffix);
                tire->setDynamic(true);
                scene->getRootSceneNode()->createChildSceneNode("RoRMeshWheelTireNode" + suffix)->attachObject(tire);

                MeshWheelInstance instance;
                instance.definition_index = i;
                instance.rim_node = rim_node;
                instance.rim_entity = rim;
                instance.tire = tire;
                mesh_wheel_instances.push_back(instance);
            }
            catch (const Ogre::Exception& e)
            {
                ++failed;
                last_error = e.getDescription();
                Ogre::LogManager::getSingleton().logMessage(
                    "iOS meshwheel skipped: " + def.rim_mesh + " - " + e.getFullDescription());
            }
        }

        std::ostringstream status;
        status << mesh_wheel_instances.size() << "/" << mesh_wheel_defs.size() << " meshwheels";
        if (failed) status << " • " << failed << " failed: " << last_error;
        meshwheel_status = status.str();
    }

    void UpdateMeshWheels(const Snapshot& s)
    {
        EnsureMeshWheels(s);
        static const float vband[6] = {0.00f, 0.23f, 0.27f, 0.73f, 0.77f, 1.00f};

        for (MeshWheelInstance& instance : mesh_wheel_instances)
        {
            if (instance.definition_index >= mesh_wheel_defs.size()) continue;
            const MeshWheelRenderDef& def = mesh_wheel_defs[instance.definition_index];
            if (def.axis0 >= s.nodes.size() || def.axis1 >= s.nodes.size()) continue;

            const Ogre::Vector3 p0 = WorldVec(s.nodes[def.axis0]);
            const Ogre::Vector3 p1 = WorldVec(s.nodes[def.axis1]);
            Ogre::Vector3 axis = p0 - p1;
            if (axis.squaredLength() < 1.0e-10f) continue;
            axis.normalise();

            // Upstream FlexMeshWheel::flexitPrepare(): orient the authored static
            // rim from the axle plus the first generated tyre ray.
            const std::size_t first = def.node_start;
            if (first >= s.nodes.size()) continue;
            const Ogre::Vector3 first_outer = WorldVec(s.nodes[first]);
            Ogre::Vector3 rim_axis = axis;
            if (def.reverse_rim) rim_axis = -rim_axis;
            Ogre::Vector3 rim_ray = first_outer - p0;
            Ogre::Vector3 rim_normal = rim_axis.crossProduct(rim_ray);
            if (rim_normal.squaredLength() > 1.0e-10f)
            {
                rim_normal.normalise();
                rim_ray = rim_axis.crossProduct(rim_normal);
                rim_ray.normalise();
                instance.rim_node->setPosition((p0 + p1) * 0.5f);
                instance.rim_node->setOrientation(Ogre::Quaternion(rim_axis, rim_normal, rim_ray));
                instance.rim_entity->setVisible(true);
            }
            else
            {
                instance.rim_entity->setVisible(false);
            }

            if (instance.tire_built) instance.tire->beginUpdate(0);
            else instance.tire->begin(def.tire_material, Ogre::RenderOperation::OT_TRIANGLE_LIST);

            // Reproduce FlexMeshWheel::updateVertices(), but emit world-space
            // vertices directly from the portable runtime's live tyre nodes.
            for (int ray = 0; ray < def.num_rays; ++ray)
            {
                const int next = (ray + 1) % def.num_rays;
                const std::size_t oi = def.node_start + static_cast<std::size_t>(ray * 2);
                const std::size_t ii = oi + 1;
                const std::size_t on = def.node_start + static_cast<std::size_t>(next * 2);
                const std::size_t in = on + 1;
                if (in >= s.nodes.size()) continue;

                const Ogre::Vector3 outer = WorldVec(s.nodes[oi]);
                const Ogre::Vector3 inner = WorldVec(s.nodes[ii]);
                const Ogre::Vector3 next_outer = WorldVec(s.nodes[on]);
                const Ogre::Vector3 next_inner = WorldVec(s.nodes[in]);

                auto bands = [&](const Ogre::Vector3& out_node, const Ogre::Vector3& in_node)
                {
                    std::array<Ogre::Vector3, 6> v;
                    Ogre::Vector3 out_ray = out_node - p0;
                    out_ray -= axis * out_ray.dotProduct(axis);
                    if (out_ray.squaredLength() > 1.0e-10f) out_ray.normalise();
                    Ogre::Vector3 in_ray = in_node - p1;
                    in_ray -= axis * in_ray.dotProduct(axis);
                    if (in_ray.squaredLength() > 1.0e-10f) in_ray.normalise();

                    v[0] = p0 + out_ray * def.rim_radius;
                    v[1] = out_node - 0.05f * (out_node - p0);
                    v[2] = out_node - 0.10f * (out_node - in_node);
                    v[3] = in_node - 0.10f * (in_node - out_node);
                    v[4] = in_node - 0.05f * (in_node - p1);
                    v[5] = p1 + in_ray * def.rim_radius;
                    return v;
                };

                const auto a = bands(outer, inner);
                const auto b = bands(next_outer, next_inner);
                const float u0 = static_cast<float>(ray) / static_cast<float>(def.num_rays);
                const float u1 = static_cast<float>(ray + 1) / static_cast<float>(def.num_rays);
                for (int band = 0; band < 5; ++band)
                    WheelQuad(instance.tire, a[band], a[band + 1], b[band + 1], b[band],
                              u0, u1, vband[band], vband[band + 1]);
            }
            instance.tire->end();
            instance.tire_built = true;
        }
    }

    void UpdateFallbackWheels(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (visual.wheels.empty()) return;
        if (wheels_built) wheels->beginUpdate(0); else wheels->begin("RoR/Game", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        const Ogre::ColourValue tread(0.025f,0.028f,0.032f,1), side(0.045f,0.048f,0.052f,1), hub(0.43f,0.44f,0.46f,1);
        for (const auto& w : visual.wheels)
        {
            if (w.axis0 >= s.nodes.size() || w.axis1 >= s.nodes.size()) continue;
            const Ogre::Vector3 p0 = WorldVec(s.nodes[w.axis0]), p1 = WorldVec(s.nodes[w.axis1]);
            Ogre::Vector3 axis = p1-p0; if (axis.isZeroLength()) continue; axis.normalise();
            Ogre::Vector3 r0 = Ogre::Vector3::UNIT_Y - axis * axis.dotProduct(Ogre::Vector3::UNIT_Y);
            if (r0.isZeroLength()) r0 = Ogre::Vector3::UNIT_X - axis * axis.dotProduct(Ogre::Vector3::UNIT_X);
            r0.normalise(); Ogre::Vector3 r1 = axis.crossProduct(r0).normalisedCopy();
            const float radius = std::max(0.05f, w.radius);
            for (int i=0; i<kWheelSegments; ++i)
            {
                const float a0 = Ogre::Math::TWO_PI*i/static_cast<float>(kWheelSegments), a1 = Ogre::Math::TWO_PI*(i+1)/static_cast<float>(kWheelSegments);
                const Ogre::Vector3 q0=(r0*std::cos(a0)+r1*std::sin(a0))*radius, q1=(r0*std::cos(a1)+r1*std::sin(a1))*radius;
                Quad(wheels,p0+q0,p1+q0,p1+q1,p0+q1,tread); Tri(wheels,p0,p0+q1,p0+q0,side); Tri(wheels,p1,p1+q0,p1+q1,side);
                Tri(wheels,p0,p0+q1*0.42f,p0+q0*0.42f,hub); Tri(wheels,p1,p1+q0*0.42f,p1+q1*0.42f,hub);
            }
        }
        wheels->end(); wheels_built = true;
    }

    void UpdateWheels(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (!mesh_wheel_defs.empty())
        {
            UpdateMeshWheels(s);
            return;
        }
        UpdateFallbackWheels(s, visual);
    }
'''
between(
    '    void UpdateWheels(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)\n',
    '    void EnsureProps(const RoR::IOSOgre::AuthoredVisualGeometry& visual)\n',
    wheel_methods,
    "replace placeholder wheel renderer")

# Include wheel status alongside managed materials/flexbodies in the existing HUD.
one(
    '    std::string VehicleVisualStatus() const { return managed_material_status + " • " + flexbody_status; }\n',
    '    std::string VehicleVisualStatus() const { return managed_material_status + " • " + flexbody_status + " • " + meshwheel_status; }\n',
    "meshwheel HUD status")

one(
'''    std::unique_ptr<RoR::IOSOgre::RoRTerrainScene> terrain;
    std::vector<PropInstance> prop_instances;
    std::vector<std::unique_ptr<RoR::IOSOgre::IOSFlexBody>> flexbody_instances;
''',
'''    std::unique_ptr<RoR::IOSOgre::RoRTerrainScene> terrain;
    std::vector<PropInstance> prop_instances;
    std::vector<std::unique_ptr<RoR::IOSOgre::IOSFlexBody>> flexbody_instances;
    std::vector<MeshWheelRenderDef> mesh_wheel_defs;
    std::vector<MeshWheelInstance> mesh_wheel_instances;
''',
"meshwheel renderer storage")

one(
    '    bool body_built=false, wheels_built=false, props_built=false, camera_started=false;\n',
    '    bool body_built=false, wheels_built=false, props_built=false, flexbodies_built=false, meshwheels_built=false, camera_started=false;\n',
    "meshwheel renderer flags")

one(
    '    std::string flexbody_status="flexbodies pending";\n',
    '    std::string flexbody_status="flexbodies pending";\n    std::string meshwheel_status="meshwheels pending";\n',
    "meshwheel status storage")

# std::array and unordered_map are now used directly by the generated wheel path.
one('#include <atomic>\n', '#include <array>\n#include <atomic>\n', "array include")
one('#include <vector>\n', '#include <unordered_map>\n#include <vector>\n', "unordered_map include")

p.write_text(s)
print('applied live-node FlexMeshWheel-style tyres and authored static rim meshes')
