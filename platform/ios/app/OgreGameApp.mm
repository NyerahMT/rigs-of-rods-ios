#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>

#include "AuthoredVehicleRuntime.h"
#include "AuthoredVisualGeometry.h"
#include "SimConstants.h"

#include "Ogre.h"
#include "OgreEntity.h"
#include "OgreGpuProgramManager.h"
#include "OgreManualObject.h"
#include "OgreMaterialManager.h"
#include "OgreMetalPlugin.h"
#include "OgreResourceGroupManager.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr float kMetersToMph = 2.23693629f;
constexpr int kRenderFps = 60;
constexpr int kWheelSegments = 18;
constexpr int kMaxSimStepsPerBatch = 120;
constexpr double kMaxSimCatchup = 0.05;

Ogre::Vector3 OgreVec(const RoR::PhysicsVec3& v) { return {v.x, v.y, v.z}; }
float Clamp01(float x) { return std::max(0.0f, std::min(1.0f, x)); }
float WrapAngle(float x) { return std::atan2(std::sin(x), std::cos(x)); }

struct Snapshot
{
    bool ready = false;
    bool finite = true;
    std::string error;
    std::vector<RoR::PhysicsVec3> nodes;
    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;
};

class SimulationHost
{
public:
    explicit SimulationHost(const std::string& text)
        : runtime(text), visual(RoR::IOSOgre::ParseAuthoredVisualGeometry(text))
    {
        Publish();
        running.store(true);
        thread = std::thread([this] { Loop(); });
    }

    ~SimulationHost()
    {
        running.store(false);
        if (thread.joinable()) thread.join();
    }

    void SetControls(float steer, float throttle, float brake, bool handbrake)
    {
        steering.store(std::max(-1.0f, std::min(1.0f, steer)));
        gas.store(Clamp01(throttle));
        service_brake.store(Clamp01(brake));
        parking_brake.store(handbrake);
    }

    void Reset() { reset_requested.store(true); }

    Snapshot GetSnapshot() const
    {
        std::lock_guard<std::mutex> guard(snapshot_mutex);
        return snapshot;
    }

    const RoR::IOSOgre::AuthoredVisualGeometry& Visual() const { return visual; }

private:
    void Publish()
    {
        Snapshot s;
        s.ready = runtime.Ready();
        s.finite = runtime.IsFinite();
        s.telemetry = runtime.Telemetry();
        if (!runtime.Errors().empty()) s.error = runtime.Errors().front();
        s.nodes.reserve(runtime.NodeCount());
        for (std::size_t i = 0; i < runtime.NodeCount(); ++i) s.nodes.push_back(runtime.Node(i).position);
        std::lock_guard<std::mutex> guard(snapshot_mutex);
        snapshot = std::move(s);
    }

    void Loop()
    {
        using Clock = std::chrono::steady_clock;
        auto previous = Clock::now();
        double accumulator = 0.0;
        while (running.load())
        {
            const auto now = Clock::now();
            accumulator += std::min(std::chrono::duration<double>(now - previous).count(), kMaxSimCatchup);
            previous = now;
            if (reset_requested.exchange(false))
            {
                runtime.Reset();
                accumulator = 0.0;
            }
            runtime.SetControls(steering.load(), gas.load(), service_brake.load(), parking_brake.load());
            int steps = 0;
            while (accumulator >= PHYSICS_DT && steps < kMaxSimStepsPerBatch)
            {
                runtime.Step(PHYSICS_DT);
                accumulator -= PHYSICS_DT;
                ++steps;
            }
            if (steps == kMaxSimStepsPerBatch) accumulator = 0.0;
            if (steps > 0) Publish();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    RoR::IOSVehicleCore::AuthoredVehicleRuntime runtime;
    RoR::IOSOgre::AuthoredVisualGeometry visual;
    std::atomic<bool> running{false};
    std::atomic<bool> reset_requested{false};
    std::atomic<float> steering{0.0f};
    std::atomic<float> gas{0.0f};
    std::atomic<float> service_brake{0.0f};
    std::atomic<bool> parking_brake{false};
    std::thread thread;
    mutable std::mutex snapshot_mutex;
    Snapshot snapshot;
};

class OgreRenderer
{
public:
    OgreRenderer(CGSize size, const std::string& media_path, const std::string& content_path,
                 const std::string& prop_mesh_path)
    {
        Initialise(size, media_path, content_path, prop_mesh_path);
    }

    ~OgreRenderer()
    {
        if (view) [view removeFromSuperview];
        delete root;
        root = nullptr;
        delete metal_plugin;
        metal_plugin = nullptr;
        view = nil;
    }

    UIView* View() const { return view; }

    void Resize(CGSize size)
    {
        if (!window || size.width < 2 || size.height < 2) return;
        view.frame = CGRectMake(0, 0, size.width, size.height);
        window->resize(static_cast<unsigned int>(size.width), static_cast<unsigned int>(size.height));
        camera->setAspectRatio(static_cast<Ogre::Real>(size.width / size.height));
    }

    void AddLookDelta(float dx_points, float dy_points)
    {
        // Finger drag orbits the chase camera around the vehicle. Keep the yaw
        // unbounded/wrapped so the player can look fully around the truck, while
        // clamping pitch before the camera can flip through the ground.
        look_yaw = WrapAngle(look_yaw - dx_points * 0.0060f);
        look_pitch = std::max(-0.52f, std::min(0.78f, look_pitch - dy_points * 0.0045f));
    }

    void ResetLook()
    {
        look_yaw = 0.0f;
        look_pitch = 0.0f;
    }

    void Draw(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (!root) return;
        if (s.ready && s.finite && !s.nodes.empty())
        {
            UpdateBody(s, visual);
            UpdateWheels(s, visual);
            UpdateProps(s, visual);
            UpdateCamera(s.telemetry);
        }
        root->renderOneFrame();
    }

private:
    struct PropInstance
    {
        std::size_t visual_index = 0;
        Ogre::SceneNode* node = nullptr;
        Ogre::Entity* entity = nullptr;
    };

    static void V(Ogre::ManualObject* o, const Ogre::Vector3& p, const Ogre::ColourValue& c)
    {
        o->position(p); o->colour(c);
    }

    static void VT(Ogre::ManualObject* o, const Ogre::Vector3& p, const Ogre::ColourValue& c,
                   const RoR::IOSOgre::TextureCoord& uv)
    {
        o->position(p); o->colour(c); o->textureCoord(uv.u, uv.v);
    }

    static void Tri(Ogre::ManualObject* o, const Ogre::Vector3& a, const Ogre::Vector3& b,
                    const Ogre::Vector3& c, const Ogre::ColourValue& colour)
    {
        V(o, a, colour); V(o, b, colour); V(o, c, colour);
    }

    static void Quad(Ogre::ManualObject* o, const Ogre::Vector3& a, const Ogre::Vector3& b,
                     const Ogre::Vector3& c, const Ogre::Vector3& d, const Ogre::ColourValue& colour)
    {
        Tri(o, a, b, c, colour); Tri(o, a, c, d, colour);
    }

    void Initialise(CGSize size, const std::string& media_path, const std::string& content_path,
                    const std::string& prop_mesh_path)
    {
        NSString* log = [NSTemporaryDirectory() stringByAppendingPathComponent:@"RoROgre.log"];
        root = new Ogre::Root("", "", log.UTF8String);
        metal_plugin = new Ogre::MetalPlugin();
        root->installPlugin(metal_plugin);
        Ogre::RenderSystem* metal = root->getRenderSystemByName("Metal Rendering Subsystem");
        if (!metal) OGRE_EXCEPT(Ogre::Exception::ERR_RENDERINGAPI_ERROR, "Metal RenderSystem missing", "OgreRenderer");
        root->setRenderSystem(metal);
        root->initialise(false);

        auto& groups = Ogre::ResourceGroupManager::getSingleton();
        groups.addResourceLocation(media_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        groups.addResourceLocation(content_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        groups.addResourceLocation(prop_mesh_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        const unsigned int w = std::max(2u, static_cast<unsigned int>(size.width));
        const unsigned int h = std::max(2u, static_cast<unsigned int>(size.height));
        window = root->createRenderWindow("RoR iOS OGRE", w, h, false, nullptr);
        void* raw_view = nullptr;
        window->getCustomAttribute("UIView", &raw_view);
        if (!raw_view) OGRE_EXCEPT(Ogre::Exception::ERR_RENDERINGAPI_ERROR, "Metal window returned no UIView", "OgreRenderer");
        view = (__bridge_transfer UIView*)raw_view;
        view.frame = CGRectMake(0, 0, size.width, size.height);
        view.autoresizingMask = UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
        view.userInteractionEnabled = NO;

        groups.initialiseAllResourceGroups();
        CreateMaterials();
        scene = root->createSceneManager("DefaultSceneManager", "GameScene");
        camera = scene->createCamera("ChaseCamera");
        camera_node = scene->getRootSceneNode()->createChildSceneNode("ChaseCameraNode");
        camera_node->attachObject(camera);
        camera_node->setFixedYawAxis(true, Ogre::Vector3::UNIT_Y);
        camera->setNearClipDistance(0.08f);
        camera->setFarClipDistance(1200.0f);
        camera->setFOVy(Ogre::Degree(58.0f));
        camera->setAspectRatio(static_cast<Ogre::Real>(size.width / std::max<CGFloat>(1.0, size.height)));
        Ogre::Viewport* viewport = window->addViewport(camera);
        viewport->setBackgroundColour(Ogre::ColourValue(0.40f, 0.60f, 0.79f, 1.0f));

        CreateGround();
        body = scene->createManualObject("RoRAuthoredCab");
        body->setDynamic(true);
        scene->getRootSceneNode()->createChildSceneNode()->attachObject(body);
        wheels = scene->createManualObject("RoRWheels");
        wheels->setDynamic(true);
        scene->getRootSceneNode()->createChildSceneNode()->attachObject(wheels);
    }

    void CreateMaterials()
    {
        auto& programs = Ogre::GpuProgramManager::getSingleton();
        Ogre::GpuProgramPtr vp = programs.createProgram("RoRGameVP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_VERTEX_PROGRAM);
        vp->setSourceFile("RoRGame.metal"); vp->setParameter("entry_point", "ror_game_vp");
        Ogre::GpuProgramPtr fp = programs.createProgram("RoRGameFP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_FRAGMENT_PROGRAM);
        fp->setSourceFile("RoRGame.metal"); fp->setParameter("entry_point", "ror_game_fp"); fp->setParameter("shader_reflection_pair_hint", "RoRGameVP");
        vp->load(); fp->load();

        Ogre::GpuProgramPtr pvp = programs.createProgram("RoRPropVP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_VERTEX_PROGRAM);
        pvp->setSourceFile("RoRGame.metal"); pvp->setParameter("entry_point", "ror_prop_vp");
        Ogre::GpuProgramPtr pfp = programs.createProgram("RoRPropFP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_FRAGMENT_PROGRAM);
        pfp->setSourceFile("RoRGame.metal"); pfp->setParameter("entry_point", "ror_prop_fp"); pfp->setParameter("shader_reflection_pair_hint", "RoRPropVP");
        pvp->load(); pfp->load();

        Ogre::GpuProgramPtr tvp = programs.createProgram("RoRVehicleVP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_VERTEX_PROGRAM);
        tvp->setSourceFile("RoRGame.metal"); tvp->setParameter("entry_point", "ror_vehicle_vp");
        Ogre::GpuProgramPtr tfp = programs.createProgram("RoRVehicleFP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_FRAGMENT_PROGRAM);
        tfp->setSourceFile("RoRGame.metal"); tfp->setParameter("entry_point", "ror_vehicle_fp"); tfp->setParameter("shader_reflection_pair_hint", "RoRVehicleVP");
        Ogre::GpuProgramPtr efp = programs.createProgram("RoRVehicleEmissiveFP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_FRAGMENT_PROGRAM);
        efp->setSourceFile("RoRGame.metal"); efp->setParameter("entry_point", "ror_vehicle_emissive_fp"); efp->setParameter("shader_reflection_pair_hint", "RoRVehicleVP");
        tvp->load(); tfp->load(); efp->load();

        Ogre::MaterialPtr flat = Ogre::MaterialManager::getSingleton().create("RoR/Game", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        flat->removeAllTechniques();
        Ogre::Pass* pass = flat->createTechnique()->createPass();
        pass->setLightingEnabled(false); pass->setCullingMode(Ogre::CULL_NONE); pass->setDepthCheckEnabled(true); pass->setDepthWriteEnabled(true);
        pass->setVertexProgram("RoRGameVP"); pass->setFragmentProgram("RoRGameFP");
        pass->getVertexProgramParameters()->setNamedAutoConstant("mvpMtx", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
        flat->load();

        Ogre::MaterialPtr prop_material = Ogre::MaterialManager::getSingleton().create("RoR/Prop", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        prop_material->removeAllTechniques();
        Ogre::Pass* prop_pass = prop_material->createTechnique()->createPass();
        prop_pass->setLightingEnabled(false); prop_pass->setCullingMode(Ogre::CULL_NONE); prop_pass->setDepthCheckEnabled(true); prop_pass->setDepthWriteEnabled(true);
        prop_pass->setVertexProgram("RoRPropVP"); prop_pass->setFragmentProgram("RoRPropFP");
        prop_pass->getVertexProgramParameters()->setNamedAutoConstant("mvpMtx", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
        prop_material->load();

        Ogre::MaterialPtr truck = Ogre::MaterialManager::getSingleton().create("RoR/DAFOfficial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        truck->removeAllTechniques();
        Ogre::Technique* truck_technique = truck->createTechnique();

        Ogre::Pass* truck_pass = truck_technique->createPass();
        truck_pass->setLightingEnabled(false); truck_pass->setCullingMode(Ogre::CULL_NONE); truck_pass->setDepthCheckEnabled(true); truck_pass->setDepthWriteEnabled(true);
        truck_pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        truck_pass->setAlphaRejectSettings(Ogre::CMPF_GREATER, 128);
        truck_pass->setVertexProgram("RoRVehicleVP"); truck_pass->setFragmentProgram("RoRVehicleFP");
        truck_pass->getVertexProgramParameters()->setNamedAutoConstant("mvpMtx", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
        Ogre::TextureUnitState* texture = truck_pass->createTextureUnitState("b6b0UID-semi.dds");
        texture->setTextureFiltering(Ogre::TFO_ANISOTROPIC);
        texture->setTextureAnisotropy(8);

        Ogre::Pass* emissive_pass = truck_technique->createPass();
        emissive_pass->setLightingEnabled(false); emissive_pass->setCullingMode(Ogre::CULL_NONE); emissive_pass->setDepthCheckEnabled(true); emissive_pass->setDepthWriteEnabled(false);
        emissive_pass->setSceneBlending(Ogre::SBT_ADD);
        emissive_pass->setVertexProgram("RoRVehicleVP"); emissive_pass->setFragmentProgram("RoRVehicleEmissiveFP");
        emissive_pass->getVertexProgramParameters()->setNamedAutoConstant("mvpMtx", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
        Ogre::TextureUnitState* emissive = emissive_pass->createTextureUnitState("b6b0UID-ampliroll_emissive.dds");
        emissive->setTextureFiltering(Ogre::TFO_ANISOTROPIC);
        emissive->setTextureAnisotropy(8);
        truck->load();
    }

    void CreateGround()
    {
        Ogre::ManualObject* pad = scene->createManualObject("AsphaltPad");
        pad->begin("RoR/Game", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        const Ogre::ColourValue asphalt(0.105f, 0.115f, 0.125f, 1.0f);
        Quad(pad, {-300,-0.025f,-300}, {300,-0.025f,-300}, {300,-0.025f,300}, {-300,-0.025f,300}, asphalt);
        const Ogre::ColourValue grid(0.18f, 0.19f, 0.20f, 1.0f);
        for (int i = -20; i <= 20; ++i)
        {
            const float p = i * 10.0f;
            Quad(pad, {-200,-0.018f,p-0.025f}, {200,-0.018f,p-0.025f}, {200,-0.018f,p+0.025f}, {-200,-0.018f,p+0.025f}, grid);
            Quad(pad, {p-0.025f,-0.018f,-200}, {p+0.025f,-0.018f,-200}, {p+0.025f,-0.018f,200}, {p-0.025f,-0.018f,200}, grid);
        }
        pad->end();
        scene->getRootSceneNode()->createChildSceneNode()->attachObject(pad);
    }

    void UpdateBody(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (visual.cab_triangles.empty()) return;
        if (body_built) body->beginUpdate(0); else body->begin("RoR/DAFOfficial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        const Ogre::Vector3 sun = Ogre::Vector3(-0.45f, 0.82f, 0.35f).normalisedCopy();
        for (const auto& t : visual.cab_triangles)
        {
            if (t.a >= s.nodes.size() || t.b >= s.nodes.size() || t.c >= s.nodes.size() || !t.has_uv) continue;
            const Ogre::Vector3 a = OgreVec(s.nodes[t.a]);
            const Ogre::Vector3 b = OgreVec(s.nodes[t.b]);
            const Ogre::Vector3 c = OgreVec(s.nodes[t.c]);
            Ogre::Vector3 n = (b-a).crossProduct(c-a);
            if (!n.isZeroLength()) n.normalise();
            const float light = 0.58f + 0.42f * std::max(0.0f, n.dotProduct(sun));
            const float contact = t.contact ? 0.93f : 1.0f;
            const Ogre::ColourValue tint(light*contact, light*contact, light*contact, 1.0f);
            VT(body, a, tint, t.uv_a); VT(body, b, tint, t.uv_b); VT(body, c, tint, t.uv_c);
        }
        body->end(); body_built = true;
    }

    void UpdateWheels(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (visual.wheels.empty()) return;
        if (wheels_built) wheels->beginUpdate(0); else wheels->begin("RoR/Game", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        const Ogre::ColourValue tread(0.025f,0.028f,0.032f,1), side(0.045f,0.048f,0.052f,1), hub(0.43f,0.44f,0.46f,1);
        for (const auto& w : visual.wheels)
        {
            if (w.axis0 >= s.nodes.size() || w.axis1 >= s.nodes.size()) continue;
            const Ogre::Vector3 p0 = OgreVec(s.nodes[w.axis0]), p1 = OgreVec(s.nodes[w.axis1]);
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

    void EnsureProps(const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (props_built || !scene) return;
        props_built = true;
        for (std::size_t i = 0; i < visual.props.size(); ++i)
        {
            const auto& prop = visual.props[i];
            try
            {
                const std::string suffix = std::to_string(i);
                Ogre::Entity* entity = scene->createEntity("RoRPropEntity" + suffix, prop.mesh_name);
                entity->setMaterialName("RoR/Prop");
                entity->setCastShadows(false);
                Ogre::SceneNode* node = scene->getRootSceneNode()->createChildSceneNode("RoRPropNode" + suffix);
                node->attachObject(entity);
                prop_instances.push_back({i, node, entity});
            }
            catch (const Ogre::Exception& e)
            {
                Ogre::LogManager::getSingleton().logMessage(
                    "iOS prop skipped: " + prop.mesh_name + " - " + e.getFullDescription());
            }
        }
    }

    void UpdateProps(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        EnsureProps(visual);
        for (PropInstance& instance : prop_instances)
        {
            if (instance.visual_index >= visual.props.size()) continue;
            const auto& prop = visual.props[instance.visual_index];
            if (prop.node_ref >= s.nodes.size() || prop.node_x >= s.nodes.size() || prop.node_y >= s.nodes.size())
            {
                instance.entity->setVisible(false);
                continue;
            }

            const Ogre::Vector3 ref = OgreVec(s.nodes[prop.node_ref]);
            const Ogre::Vector3 diff_x = OgreVec(s.nodes[prop.node_x]) - ref;
            const Ogre::Vector3 diff_y = OgreVec(s.nodes[prop.node_y]) - ref;
            if (diff_x.squaredLength() < 1.0e-8f || diff_y.squaredLength() < 1.0e-8f)
            {
                instance.entity->setVisible(false);
                continue;
            }

            // Upstream GfxActor::UpdateProps() uses diffY x diffX for the prop
            // normal, then constructs a full orthonormal attachment frame. The
            // previous iOS code used diffX.getRotationTo(diffY), which rotates
            // around the mesh origin as the chassis flexes and caused the props
            // to visibly swivel through themselves.
            Ogre::Vector3 normal = diff_y.crossProduct(diff_x);
            if (normal.squaredLength() < 1.0e-8f)
            {
                instance.entity->setVisible(false);
                continue;
            }
            normal.normalise();

            const Ogre::Vector3 position = ref
                + prop.offset_x * diff_x
                + prop.offset_y * diff_y
                + prop.offset_z * normal;

            const Ogre::Vector3 ref_x = diff_x.normalisedCopy();
            Ogre::Vector3 ref_y = ref_x.crossProduct(normal);
            if (ref_y.squaredLength() < 1.0e-8f)
            {
                instance.entity->setVisible(false);
                continue;
            }
            ref_y.normalise();

            // ActorSpawner builds pp_rot as Z * Y * X; preserve that exact RoR
            // Euler order before applying it to the live node-derived basis.
            const Ogre::Quaternion authored_rotation =
                Ogre::Quaternion(Ogre::Degree(prop.rot_z_degrees), Ogre::Vector3::UNIT_Z)
                * Ogre::Quaternion(Ogre::Degree(prop.rot_y_degrees), Ogre::Vector3::UNIT_Y)
                * Ogre::Quaternion(Ogre::Degree(prop.rot_x_degrees), Ogre::Vector3::UNIT_X);
            const Ogre::Quaternion orientation = Ogre::Quaternion(ref_x, normal, ref_y) * authored_rotation;

            instance.node->setPosition(position);
            instance.node->setOrientation(orientation);
            instance.entity->setVisible(true);
        }
    }

    void UpdateCamera(const RoR::IOSVehicleCore::AuthoredVehicleTelemetry& t)
    {
        const Ogre::Vector3 target_center = OgreVec(t.center);
        if (!camera_started) { camera_center=target_center; camera_heading=t.heading_radians; camera_started=true; }
        camera_center += (target_center-camera_center)*0.14f;
        camera_heading = WrapAngle(camera_heading + WrapAngle(t.heading_radians-camera_heading)*0.12f);

        const Ogre::Vector3 vehicle_forward(std::cos(camera_heading),0,std::sin(camera_heading));
        const float chase=10.5f+std::min(t.speed_mps,35.0f)*0.055f;
        const float orbit_heading = camera_heading + look_yaw;
        const Ogre::Vector3 orbit_forward(std::cos(orbit_heading),0,std::sin(orbit_heading));
        const float horizontal = chase * std::cos(look_pitch);
        const float vertical = 4.4f + chase * std::sin(look_pitch);
        const Ogre::Vector3 look_target = camera_center + vehicle_forward*2.2f + Ogre::Vector3(0,1.05f,0);

        camera_node->setPosition(camera_center-orbit_forward*horizontal+Ogre::Vector3(0,vertical,0));
        camera_node->lookAt(look_target,Ogre::Node::TS_PARENT);
    }

    Ogre::Root* root=nullptr; Ogre::MetalPlugin* metal_plugin=nullptr; Ogre::RenderWindow* window=nullptr; Ogre::SceneManager* scene=nullptr;
    Ogre::Camera* camera=nullptr; Ogre::SceneNode* camera_node=nullptr; Ogre::ManualObject* body=nullptr; Ogre::ManualObject* wheels=nullptr;
    std::vector<PropInstance> prop_instances;
    __strong UIView* view=nil;
    bool body_built=false, wheels_built=false, props_built=false, camera_started=false;
    Ogre::Vector3 camera_center=Ogre::Vector3::ZERO;
    float camera_heading=0.0f;
    float look_yaw=0.0f;
    float look_pitch=0.0f;
};
} // namespace

@interface RoRGameViewController : UIViewController<UIGestureRecognizerDelegate>
@end

@implementation RoRGameViewController
{
    SimulationHost* _simulation; OgreRenderer* _renderer; UIView* _ogreView; CADisplayLink* _displayLink; UILabel* _speed; UILabel* _status;
    UIPanGestureRecognizer* _lookPan;
    BOOL _left,_right,_gas,_brake,_handbrake; CFTimeInterval _lastFrame; float _fps;
}

- (UIButton*)button:(NSString*)title
{
    UIButton* b=[UIButton buttonWithType:UIButtonTypeSystem]; b.translatesAutoresizingMaskIntoConstraints=NO; [b setTitle:title forState:UIControlStateNormal];
    [b setTitleColor:UIColor.whiteColor forState:UIControlStateNormal]; b.titleLabel.font=[UIFont systemFontOfSize:16 weight:UIFontWeightBold];
    b.backgroundColor=[UIColor colorWithWhite:0.04 alpha:0.62]; b.layer.cornerRadius=19; b.layer.borderWidth=1; b.layer.borderColor=[UIColor colorWithWhite:1 alpha:0.13].CGColor; return b;
}

- (void)viewDidLoad
{
    [super viewDidLoad]; self.view.backgroundColor=UIColor.blackColor;
    NSString* path=[[NSBundle mainBundle] pathForResource:@"b6b0UID-semi" ofType:@"truck" inDirectory:@"Content/dafsemi"];
    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;
    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));

    _speed=[[UILabel alloc] init]; _speed.translatesAutoresizingMaskIntoConstraints=NO; _speed.textColor=UIColor.whiteColor; _speed.font=[UIFont monospacedDigitSystemFontOfSize:25 weight:UIFontWeightBold]; _speed.layer.shadowColor=UIColor.blackColor.CGColor; _speed.layer.shadowOpacity=.8; _speed.layer.shadowRadius=4; [self.view addSubview:_speed];
    _status=[[UILabel alloc] init]; _status.translatesAutoresizingMaskIntoConstraints=NO; _status.textColor=[UIColor colorWithWhite:1 alpha:.72]; _status.font=[UIFont monospacedSystemFontOfSize:10.5 weight:UIFontWeightSemibold]; _status.numberOfLines=2; [self.view addSubview:_status];

    UIButton *left=[self button:@"◀"],*right=[self button:@"▶"],*brake=[self button:@"BRAKE"],*hb=[self button:@"HB"],*gas=[self button:@"GAS"],*reset=[self button:@"RESET"];
    gas.backgroundColor=[UIColor colorWithRed:.03 green:.30 blue:.11 alpha:.68]; brake.backgroundColor=[UIColor colorWithRed:.34 green:.04 blue:.04 alpha:.68]; hb.backgroundColor=[UIColor colorWithRed:.33 green:.15 blue:.02 alpha:.68]; reset.titleLabel.font=[UIFont systemFontOfSize:11 weight:UIFontWeightBold];
    [left addTarget:self action:@selector(leftDown:) forControlEvents:UIControlEventTouchDown]; [left addTarget:self action:@selector(leftUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [right addTarget:self action:@selector(rightDown:) forControlEvents:UIControlEventTouchDown]; [right addTarget:self action:@selector(rightUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [gas addTarget:self action:@selector(gasDown:) forControlEvents:UIControlEventTouchDown]; [gas addTarget:self action:@selector(gasUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [brake addTarget:self action:@selector(brakeDown:) forControlEvents:UIControlEventTouchDown]; [brake addTarget:self action:@selector(brakeUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [hb addTarget:self action:@selector(hbDown:) forControlEvents:UIControlEventTouchDown]; [hb addTarget:self action:@selector(hbUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel]; [reset addTarget:self action:@selector(reset:) forControlEvents:UIControlEventTouchUpInside];

    UIStackView* steer=[[UIStackView alloc] initWithArrangedSubviews:@[left,right]]; steer.translatesAutoresizingMaskIntoConstraints=NO; steer.spacing=12; steer.distribution=UIStackViewDistributionFillEqually; [self.view addSubview:steer];
    UIStackView* pedals=[[UIStackView alloc] initWithArrangedSubviews:@[brake,hb,gas]]; pedals.translatesAutoresizingMaskIntoConstraints=NO; pedals.spacing=10; pedals.distribution=UIStackViewDistributionFillEqually; [self.view addSubview:pedals]; [self.view addSubview:reset];
    [NSLayoutConstraint activateConstraints:@[[_speed.leadingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.leadingAnchor constant:18],[_speed.topAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.topAnchor constant:14],[_status.leadingAnchor constraintEqualToAnchor:_speed.leadingAnchor],[_status.topAnchor constraintEqualToAnchor:_speed.bottomAnchor constant:2],[steer.leadingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.leadingAnchor constant:18],[steer.bottomAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.bottomAnchor constant:-16],[steer.widthAnchor constraintEqualToConstant:176],[steer.heightAnchor constraintEqualToConstant:72],[pedals.trailingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.trailingAnchor constant:-18],[pedals.bottomAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.bottomAnchor constant:-16],[pedals.widthAnchor constraintEqualToConstant:260],[pedals.heightAnchor constraintEqualToConstant:72],[reset.trailingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.trailingAnchor constant:-16],[reset.topAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.topAnchor constant:12],[reset.widthAnchor constraintEqualToConstant:64],[reset.heightAnchor constraintEqualToConstant:34]]];

    // Drag anywhere in the open center of the scene to orbit the camera. The
    // recognizer deliberately ignores UIControls so steering/throttle touches
    // remain independent and can still be held while the other thumb looks.
    _lookPan=[[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(lookPan:)];
    _lookPan.maximumNumberOfTouches=1;
    _lookPan.cancelsTouchesInView=NO;
    _lookPan.delegate=self;
    [self.view addGestureRecognizer:_lookPan];

    _displayLink=[CADisplayLink displayLinkWithTarget:self selector:@selector(frame:)]; if(@available(iOS 15.0,*)) _displayLink.preferredFrameRateRange=CAFrameRateRangeMake(kRenderFps,kRenderFps,kRenderFps); else _displayLink.preferredFramesPerSecond=kRenderFps; [_displayLink addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer*)gesture shouldReceiveTouch:(UITouch*)touch
{
    (void)gesture;
    UIView* v=touch.view;
    while(v){if([v isKindOfClass:UIControl.class])return NO;v=v.superview;}
    const CGPoint p=[touch locationInView:self.view];
    const CGFloat h=MAX(1.0,self.view.bounds.size.height);
    // Reserve the top HUD strip and the bottom control strip. The broad middle
    // of the Metal view is the dedicated look surface.
    return p.y>h*0.14 && p.y<h*0.82;
}

- (void)lookPan:(UIPanGestureRecognizer*)pan
{
    const CGPoint delta=[pan translationInView:self.view];
    [pan setTranslation:CGPointZero inView:self.view];
    if(_renderer)_renderer->AddLookDelta((float)delta.x,(float)delta.y);
}

- (void)viewDidAppear:(BOOL)animated{[super viewDidAppear:animated];[self ensureRenderer];}
- (void)viewDidLayoutSubviews{[super viewDidLayoutSubviews];if(_renderer){_ogreView.frame=self.view.bounds;_renderer->Resize(self.view.bounds.size);}}
- (void)dealloc{[_displayLink invalidate];delete _renderer;delete _simulation;}

- (void)ensureRenderer
{
    if(_renderer||self.view.bounds.size.width<2||self.view.bounds.size.height<2)return;
    NSString* rootPath=[[NSBundle mainBundle] resourcePath]; NSString* media=[rootPath stringByAppendingPathComponent:@"OgreMedia/Main"]; NSString* content=[rootPath stringByAppendingPathComponent:@"Content/dafsemi"]; NSString* propMeshes=[rootPath stringByAppendingPathComponent:@"RoRResources/meshes"];
    try{_renderer=new OgreRenderer(self.view.bounds.size,std::string(media.UTF8String),std::string(content.UTF8String),std::string(propMeshes.UTF8String));_ogreView=_renderer->View();_ogreView.frame=self.view.bounds;[self.view insertSubview:_ogreView atIndex:0];}
    catch(const Ogre::Exception& e){_status.text=[NSString stringWithFormat:@"OGRE INIT FAILED\n%s",e.getFullDescription().c_str()];}
    catch(const std::exception& e){_status.text=[NSString stringWithFormat:@"RENDER INIT FAILED\n%s",e.what()];}
}

- (void)frame:(CADisplayLink*)link
{
    [self ensureRenderer]; if(_lastFrame>0){double dt=link.timestamp-_lastFrame;if(dt>.0001){float now=1.0f/(float)dt;_fps=_fps<1?now:_fps*.90f+now*.10f;}}_lastFrame=link.timestamp;
    Snapshot s=_simulation->GetSnapshot(); if(_renderer)_renderer->Draw(s,_simulation->Visual()); _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];
    if(!s.ready)_status.text=[NSString stringWithFormat:@"VEHICLE LOAD FAILED\n%s",s.error.c_str()]; else if(!s.finite)_status.text=@"PHYSICS STOPPED • TAP RESET"; else _status.text=[NSString stringWithFormat:@"OGRE 14.6 • ROR DAF + PROPS • %.0f FPS\nROR PHYSICS 2,000 HZ • %llu STEPS",_fps,s.telemetry.physics_steps];
}

- (void)push{_simulation->SetControls((_right?1.f:0.f)-(_left?1.f:0.f),_gas?1.f:0.f,_brake?1.f:0.f,_handbrake);}
- (void)leftDown:(id)x{(void)x;_left=YES;[self push];}- (void)leftUp:(id)x{(void)x;_left=NO;[self push];}
- (void)rightDown:(id)x{(void)x;_right=YES;[self push];}- (void)rightUp:(id)x{(void)x;_right=NO;[self push];}
- (void)gasDown:(id)x{(void)x;_gas=YES;[self push];}- (void)gasUp:(id)x{(void)x;_gas=NO;[self push];}
- (void)brakeDown:(id)x{(void)x;_brake=YES;[self push];}- (void)brakeUp:(id)x{(void)x;_brake=NO;[self push];}
- (void)hbDown:(id)x{(void)x;_handbrake=YES;[self push];}- (void)hbUp:(id)x{(void)x;_handbrake=NO;[self push];}
- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();if(_renderer)_renderer->ResetLook();}
@end

@interface RoRProbeAppDelegate:UIResponder<UIApplicationDelegate>@property(nonatomic,strong)UIWindow* window;@end
@implementation RoRProbeAppDelegate
- (BOOL)application:(UIApplication*)application didFinishLaunchingWithOptions:(NSDictionary*)options{(void)application;(void)options;self.window=[[UIWindow alloc]initWithFrame:UIScreen.mainScreen.bounds];self.window.rootViewController=[[RoRGameViewController alloc]init];[self.window makeKeyAndVisible];return YES;}
@end

int main(int argc,char* argv[]){@autoreleasepool{return UIApplicationMain(argc,argv,nil,NSStringFromClass([RoRProbeAppDelegate class]));}}
