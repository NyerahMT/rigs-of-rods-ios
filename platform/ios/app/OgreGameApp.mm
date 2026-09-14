#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>

#include "AuthoredVehicleRuntime.h"
#include "AuthoredVisualGeometry.h"
#include "SimConstants.h"

#include "Ogre.h"
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
constexpr int kWheelSegments = 14;
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
        for (std::size_t i = 0; i < runtime.NodeCount(); ++i)
            s.nodes.push_back(runtime.Node(i).position);
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
    OgreRenderer(CGSize size, const std::string& media_path) { Initialise(size, media_path); }

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

    void Draw(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (!root || !s.ready || !s.finite || s.nodes.empty()) return;
        UpdateBody(s, visual);
        UpdateWheels(s, visual);
        UpdateCamera(s.telemetry);
        root->renderOneFrame();
    }

private:
    static void V(Ogre::ManualObject* o, const Ogre::Vector3& p, const Ogre::ColourValue& c)
    {
        o->position(p); o->colour(c);
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

    void Initialise(CGSize size, const std::string& media_path)
    {
        NSString* log = [NSTemporaryDirectory() stringByAppendingPathComponent:@"RoROgre.log"];
        root = new Ogre::Root("", "", log.UTF8String);
        metal_plugin = new Ogre::MetalPlugin();
        root->installPlugin(metal_plugin);

        Ogre::RenderSystem* metal = root->getRenderSystemByName("Metal Rendering Subsystem");
        if (!metal)
            OGRE_EXCEPT(Ogre::Exception::ERR_RENDERINGAPI_ERROR, "Metal RenderSystem missing", "OgreRenderer");
        root->setRenderSystem(metal);
        root->initialise(false);

        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            media_path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        const unsigned int w = std::max(2u, static_cast<unsigned int>(size.width));
        const unsigned int h = std::max(2u, static_cast<unsigned int>(size.height));
        window = root->createRenderWindow("RoR iOS OGRE", w, h, false, nullptr);

        void* raw_view = nullptr;
        window->getCustomAttribute("UIView", &raw_view);
        if (!raw_view)
            OGRE_EXCEPT(Ogre::Exception::ERR_RENDERINGAPI_ERROR, "Metal window returned no UIView", "OgreRenderer");
        view = (__bridge_transfer UIView*)raw_view;
        view.frame = CGRectMake(0, 0, size.width, size.height);
        view.autoresizingMask = UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
        view.userInteractionEnabled = NO;

        Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
        CreateMaterial();

        scene = root->createSceneManager(Ogre::ST_GENERIC);
        camera = scene->createCamera("ChaseCamera");
        camera->setNearClipDistance(0.08f);
        camera->setFarClipDistance(1000.0f);
        camera->setFOVy(Ogre::Degree(58.0f));
        camera->setAspectRatio(static_cast<Ogre::Real>(size.width / std::max<CGFloat>(1.0, size.height)));
        Ogre::Viewport* viewport = window->addViewport(camera);
        viewport->setBackgroundColour(Ogre::ColourValue(0.34f, 0.52f, 0.70f, 1.0f));

        CreateGround();
        body = scene->createManualObject("AuthoredCab");
        body->setDynamic(true);
        scene->getRootSceneNode()->createChildSceneNode()->attachObject(body);
        wheels = scene->createManualObject("AuthoredWheels");
        wheels->setDynamic(true);
        scene->getRootSceneNode()->createChildSceneNode()->attachObject(wheels);
    }

    void CreateMaterial()
    {
        Ogre::GpuProgramPtr vp = Ogre::GpuProgramManager::getSingleton().createProgram(
            "RoRGameVP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_VERTEX_PROGRAM);
        vp->setSourceFile("RoRGame.metal");
        vp->setParameter("entry_point", "ror_game_vp");

        Ogre::GpuProgramPtr fp = Ogre::GpuProgramManager::getSingleton().createProgram(
            "RoRGameFP", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "metal", Ogre::GPT_FRAGMENT_PROGRAM);
        fp->setSourceFile("RoRGame.metal");
        fp->setParameter("entry_point", "ror_game_fp");
        fp->setParameter("shader_reflection_pair_hint", "RoRGameVP");
        vp->load(); fp->load();

        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
            "RoR/Game", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->removeAllTechniques();
        Ogre::Pass* pass = material->createTechnique()->createPass();
        pass->setLightingEnabled(false);
        pass->setCullingMode(Ogre::CULL_NONE);
        pass->setDepthCheckEnabled(true);
        pass->setDepthWriteEnabled(true);
        pass->setVertexProgram("RoRGameVP");
        pass->setFragmentProgram("RoRGameFP");
        pass->getVertexProgramParameters()->setNamedAutoConstant(
            "mvpMtx", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
        material->load();
    }

    void CreateGround()
    {
        Ogre::ManualObject* pad = scene->createManualObject("AsphaltPad");
        pad->begin("RoR/Game", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        const Ogre::ColourValue asphalt(0.115f, 0.125f, 0.135f, 1.0f);
        Quad(pad, {-300,-0.025f,-300}, {300,-0.025f,-300}, {300,-0.025f,300}, {-300,-0.025f,300}, asphalt);

        const Ogre::ColourValue grid(0.19f, 0.205f, 0.215f, 1.0f);
        for (int i = -20; i <= 20; ++i)
        {
            const float p = i * 10.0f;
            Quad(pad, {-200,-0.018f,p-0.035f}, {200,-0.018f,p-0.035f},
                      {200,-0.018f,p+0.035f}, {-200,-0.018f,p+0.035f}, grid);
            Quad(pad, {p-0.035f,-0.018f,-200}, {p+0.035f,-0.018f,-200},
                      {p+0.035f,-0.018f,200}, {p-0.035f,-0.018f,200}, grid);
        }

        const Ogre::ColourValue yellow(0.92f, 0.70f, 0.16f, 1.0f);
        for (int i = -20; i < 20; ++i)
        {
            const float x = i * 8.0f;
            Quad(pad, {x,-0.012f,-0.055f}, {x+4,-0.012f,-0.055f},
                      {x+4,-0.012f,0.055f}, {x,-0.012f,0.055f}, yellow);
        }
        pad->end();
        scene->getRootSceneNode()->createChildSceneNode()->attachObject(pad);
    }

    void UpdateBody(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (visual.cab_triangles.empty()) return;
        if (body_built) body->beginUpdate(0);
        else body->begin("RoR/Game", Ogre::RenderOperation::OT_TRIANGLE_LIST);

        const Ogre::Vector3 sun = Ogre::Vector3(-0.45f, 0.82f, 0.35f).normalisedCopy();
        for (const auto& t : visual.cab_triangles)
        {
            if (t.a >= s.nodes.size() || t.b >= s.nodes.size() || t.c >= s.nodes.size()) continue;
            const Ogre::Vector3 a = OgreVec(s.nodes[t.a]);
            const Ogre::Vector3 b = OgreVec(s.nodes[t.b]);
            const Ogre::Vector3 c = OgreVec(s.nodes[t.c]);
            Ogre::Vector3 n = (b-a).crossProduct(c-a);
            if (!n.isZeroLength()) n.normalise();
            const float light = 0.43f + 0.57f * std::fabs(n.dotProduct(sun));
            const float contact = t.contact ? 0.82f : 1.0f;
            Tri(body, a, b, c, {0.10f*light*contact, 0.39f*light*contact, 0.70f*light*contact, 1.0f});
        }
        body->end(); body_built = true;
    }

    void UpdateWheels(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        if (visual.wheels.empty()) return;
        if (wheels_built) wheels->beginUpdate(0);
        else wheels->begin("RoR/Game", Ogre::RenderOperation::OT_TRIANGLE_LIST);

        const Ogre::ColourValue tread(0.035f,0.04f,0.045f,1);
        const Ogre::ColourValue side(0.06f,0.065f,0.07f,1);
        const Ogre::ColourValue hub(0.32f,0.33f,0.35f,1);
        for (const auto& w : visual.wheels)
        {
            if (w.axis0 >= s.nodes.size() || w.axis1 >= s.nodes.size()) continue;
            const Ogre::Vector3 p0 = OgreVec(s.nodes[w.axis0]);
            const Ogre::Vector3 p1 = OgreVec(s.nodes[w.axis1]);
            Ogre::Vector3 axis = p1-p0;
            if (axis.isZeroLength()) continue;
            axis.normalise();
            Ogre::Vector3 r0 = Ogre::Vector3::UNIT_Y - axis * axis.dotProduct(Ogre::Vector3::UNIT_Y);
            if (r0.isZeroLength()) r0 = Ogre::Vector3::UNIT_X - axis * axis.dotProduct(Ogre::Vector3::UNIT_X);
            r0.normalise();
            Ogre::Vector3 r1 = axis.crossProduct(r0).normalisedCopy();
            const float radius = std::max(0.05f, w.radius);
            for (int i=0; i<kWheelSegments; ++i)
            {
                const float a0 = Ogre::Math::TWO_PI * i / static_cast<float>(kWheelSegments);
                const float a1 = Ogre::Math::TWO_PI * (i+1) / static_cast<float>(kWheelSegments);
                const Ogre::Vector3 q0 = (r0*std::cos(a0)+r1*std::sin(a0))*radius;
                const Ogre::Vector3 q1 = (r0*std::cos(a1)+r1*std::sin(a1))*radius;
                Quad(wheels, p0+q0, p1+q0, p1+q1, p0+q1, tread);
                Tri(wheels, p0, p0+q1, p0+q0, side);
                Tri(wheels, p1, p1+q0, p1+q1, side);
                Tri(wheels, p0, p0+q1*0.4f, p0+q0*0.4f, hub);
                Tri(wheels, p1, p1+q0*0.4f, p1+q1*0.4f, hub);
            }
        }
        wheels->end(); wheels_built = true;
    }

    void UpdateCamera(const RoR::IOSVehicleCore::AuthoredVehicleTelemetry& t)
    {
        const Ogre::Vector3 target_center = OgreVec(t.center);
        if (!camera_started)
        {
            camera_center = target_center;
            camera_heading = t.heading_radians;
            camera_started = true;
        }
        camera_center += (target_center-camera_center)*0.14f;
        camera_heading = WrapAngle(camera_heading + WrapAngle(t.heading_radians-camera_heading)*0.12f);
        const Ogre::Vector3 forward(std::cos(camera_heading),0,std::sin(camera_heading));
        const float chase = 10.5f + std::min(t.speed_mps,35.0f)*0.055f;
        camera->setPosition(camera_center-forward*chase+Ogre::Vector3(0,4.4f,0));
        camera->lookAt(camera_center+forward*2.6f+Ogre::Vector3(0,1.05f,0));
    }

    Ogre::Root* root = nullptr;
    Ogre::MetalPlugin* metal_plugin = nullptr;
    Ogre::RenderWindow* window = nullptr;
    Ogre::SceneManager* scene = nullptr;
    Ogre::Camera* camera = nullptr;
    Ogre::ManualObject* body = nullptr;
    Ogre::ManualObject* wheels = nullptr;
    __strong UIView* view = nil;
    bool body_built = false;
    bool wheels_built = false;
    bool camera_started = false;
    Ogre::Vector3 camera_center = Ogre::Vector3::ZERO;
    float camera_heading = 0;
};
} // namespace

@interface RoRGameViewController : UIViewController
@end

@implementation RoRGameViewController
{
    SimulationHost* _simulation;
    OgreRenderer* _renderer;
    UIView* _ogreView;
    CADisplayLink* _displayLink;
    UILabel* _speed;
    UILabel* _status;
    BOOL _left, _right, _gas, _brake, _handbrake;
    CFTimeInterval _lastFrame;
    float _fps;
}

- (UIButton*)button:(NSString*)title
{
    UIButton* b=[UIButton buttonWithType:UIButtonTypeSystem];
    b.translatesAutoresizingMaskIntoConstraints=NO;
    [b setTitle:title forState:UIControlStateNormal];
    [b setTitleColor:UIColor.whiteColor forState:UIControlStateNormal];
    b.titleLabel.font=[UIFont systemFontOfSize:16 weight:UIFontWeightBold];
    b.backgroundColor=[UIColor colorWithWhite:0.04 alpha:0.66];
    b.layer.cornerRadius=19; b.layer.borderWidth=1;
    b.layer.borderColor=[UIColor colorWithWhite:1 alpha:0.13].CGColor;
    return b;
}

- (void)viewDidLoad
{
    [super viewDidLoad]; self.view.backgroundColor=UIColor.blackColor;
    NSString* path=[[NSBundle mainBundle] pathForResource:@"b6b0UID-semi" ofType:@"truck" inDirectory:@"Content/dafsemi"];
    NSString* text=path?[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil]:nil;
    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));

    _speed=[[UILabel alloc] init]; _speed.translatesAutoresizingMaskIntoConstraints=NO;
    _speed.textColor=UIColor.whiteColor; _speed.font=[UIFont monospacedDigitSystemFontOfSize:25 weight:UIFontWeightBold];
    _speed.layer.shadowColor=UIColor.blackColor.CGColor; _speed.layer.shadowOpacity=.8; _speed.layer.shadowRadius=4;
    [self.view addSubview:_speed];
    _status=[[UILabel alloc] init]; _status.translatesAutoresizingMaskIntoConstraints=NO;
    _status.textColor=[UIColor colorWithWhite:1 alpha:.72]; _status.font=[UIFont monospacedSystemFontOfSize:10.5 weight:UIFontWeightSemibold];
    _status.numberOfLines=2; [self.view addSubview:_status];

    UIButton *left=[self button:@"◀"], *right=[self button:@"▶"], *brake=[self button:@"BRAKE"], *hb=[self button:@"HB"], *gas=[self button:@"GAS"], *reset=[self button:@"RESET"];
    gas.backgroundColor=[UIColor colorWithRed:.03 green:.30 blue:.11 alpha:.72];
    brake.backgroundColor=[UIColor colorWithRed:.34 green:.04 blue:.04 alpha:.72];
    hb.backgroundColor=[UIColor colorWithRed:.33 green:.15 blue:.02 alpha:.72];
    reset.titleLabel.font=[UIFont systemFontOfSize:11 weight:UIFontWeightBold];
    [left addTarget:self action:@selector(leftDown:) forControlEvents:UIControlEventTouchDown];
    [left addTarget:self action:@selector(leftUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [right addTarget:self action:@selector(rightDown:) forControlEvents:UIControlEventTouchDown];
    [right addTarget:self action:@selector(rightUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [gas addTarget:self action:@selector(gasDown:) forControlEvents:UIControlEventTouchDown];
    [gas addTarget:self action:@selector(gasUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [brake addTarget:self action:@selector(brakeDown:) forControlEvents:UIControlEventTouchDown];
    [brake addTarget:self action:@selector(brakeUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [hb addTarget:self action:@selector(hbDown:) forControlEvents:UIControlEventTouchDown];
    [hb addTarget:self action:@selector(hbUp:) forControlEvents:UIControlEventTouchUpInside|UIControlEventTouchUpOutside|UIControlEventTouchCancel];
    [reset addTarget:self action:@selector(reset:) forControlEvents:UIControlEventTouchUpInside];

    UIStackView* steer=[[UIStackView alloc] initWithArrangedSubviews:@[left,right]]; steer.translatesAutoresizingMaskIntoConstraints=NO; steer.spacing=12; steer.distribution=UIStackViewDistributionFillEqually; [self.view addSubview:steer];
    UIStackView* pedals=[[UIStackView alloc] initWithArrangedSubviews:@[brake,hb,gas]]; pedals.translatesAutoresizingMaskIntoConstraints=NO; pedals.spacing=10; pedals.distribution=UIStackViewDistributionFillEqually; [self.view addSubview:pedals];
    [self.view addSubview:reset];
    [NSLayoutConstraint activateConstraints:@[
        [_speed.leadingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.leadingAnchor constant:18], [_speed.topAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.topAnchor constant:14],
        [_status.leadingAnchor constraintEqualToAnchor:_speed.leadingAnchor], [_status.topAnchor constraintEqualToAnchor:_speed.bottomAnchor constant:2],
        [steer.leadingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.leadingAnchor constant:18], [steer.bottomAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.bottomAnchor constant:-16], [steer.widthAnchor constraintEqualToConstant:176], [steer.heightAnchor constraintEqualToConstant:72],
        [pedals.trailingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.trailingAnchor constant:-18], [pedals.bottomAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.bottomAnchor constant:-16], [pedals.widthAnchor constraintEqualToConstant:260], [pedals.heightAnchor constraintEqualToConstant:72],
        [reset.trailingAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.trailingAnchor constant:-16], [reset.topAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.topAnchor constant:12], [reset.widthAnchor constraintEqualToConstant:64], [reset.heightAnchor constraintEqualToConstant:34]
    ]];

    _displayLink=[CADisplayLink displayLinkWithTarget:self selector:@selector(frame:)];
    if (@available(iOS 15.0,*)) _displayLink.preferredFrameRateRange=CAFrameRateRangeMake(kRenderFps,kRenderFps,kRenderFps); else _displayLink.preferredFramesPerSecond=kRenderFps;
    [_displayLink addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
}

- (void)viewDidAppear:(BOOL)animated { [super viewDidAppear:animated]; [self ensureRenderer]; }
- (void)viewDidLayoutSubviews { [super viewDidLayoutSubviews]; if(_renderer){ _ogreView.frame=self.view.bounds; _renderer->Resize(self.view.bounds.size); } }
- (void)dealloc { [_displayLink invalidate]; delete _renderer; delete _simulation; }

- (void)ensureRenderer
{
    if(_renderer||self.view.bounds.size.width<2||self.view.bounds.size.height<2) return;
    NSString* media=[[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"OgreMedia/Main"];
    try {
        _renderer=new OgreRenderer(self.view.bounds.size,std::string(media.UTF8String));
        _ogreView=_renderer->View(); _ogreView.frame=self.view.bounds; [self.view insertSubview:_ogreView atIndex:0];
    } catch(const Ogre::Exception& e) {
        _status.text=[NSString stringWithFormat:@"OGRE INIT FAILED\n%s",e.getFullDescription().c_str()];
    } catch(const std::exception& e) {
        _status.text=[NSString stringWithFormat:@"RENDER INIT FAILED\n%s",e.what()];
    }
}

- (void)frame:(CADisplayLink*)link
{
    [self ensureRenderer];
    if(_lastFrame>0){ double dt=link.timestamp-_lastFrame; if(dt>.0001){ float now=1.0f/(float)dt; _fps=_fps<1?now:_fps*.90f+now*.10f; }} _lastFrame=link.timestamp;
    Snapshot s=_simulation->GetSnapshot();
    if(_renderer&&s.ready&&s.finite) _renderer->Draw(s,_simulation->Visual());
    _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];
    if(!s.ready) _status.text=[NSString stringWithFormat:@"VEHICLE LOAD FAILED\n%s",s.error.c_str()];
    else if(!s.finite) _status.text=@"PHYSICS STOPPED • TAP RESET";
    else _status.text=[NSString stringWithFormat:@"OGRE 14.6 • METAL • %.0f FPS\nROR PHYSICS 2,000 HZ • %llu STEPS",_fps,s.telemetry.physics_steps];
}

- (void)push { _simulation->SetControls((_right?1.f:0.f)-(_left?1.f:0.f),_gas?1.f:0.f,_brake?1.f:0.f,_handbrake); }
- (void)leftDown:(id)x{(void)x;_left=YES;[self push];} - (void)leftUp:(id)x{(void)x;_left=NO;[self push];}
- (void)rightDown:(id)x{(void)x;_right=YES;[self push];} - (void)rightUp:(id)x{(void)x;_right=NO;[self push];}
- (void)gasDown:(id)x{(void)x;_gas=YES;[self push];} - (void)gasUp:(id)x{(void)x;_gas=NO;[self push];}
- (void)brakeDown:(id)x{(void)x;_brake=YES;[self push];} - (void)brakeUp:(id)x{(void)x;_brake=NO;[self push];}
- (void)hbDown:(id)x{(void)x;_handbrake=YES;[self push];} - (void)hbUp:(id)x{(void)x;_handbrake=NO;[self push];}
- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();}
@end

@interface RoRProbeAppDelegate : UIResponder <UIApplicationDelegate>
@property(nonatomic,strong) UIWindow* window;
@end
@implementation RoRProbeAppDelegate
- (BOOL)application:(UIApplication*)application didFinishLaunchingWithOptions:(NSDictionary*)options
{ (void)application;(void)options; self.window=[[UIWindow alloc] initWithFrame:UIScreen.mainScreen.bounds]; self.window.rootViewController=[[RoRGameViewController alloc] init]; [self.window makeKeyAndVisible]; return YES; }
@end

int main(int argc,char* argv[]){ @autoreleasepool { return UIApplicationMain(argc,argv,nil,NSStringFromClass([RoRProbeAppDelegate class])); } }
