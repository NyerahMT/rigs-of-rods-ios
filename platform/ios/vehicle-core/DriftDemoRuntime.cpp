#include "DriftDemoRuntime.h"

#include "Differentials.h"
#include "GroundContact.h"
#include "SimConstants.h"
#include "WheelPhysics.h"

#include <algorithm>
#include <cmath>
#include <deque>

namespace RoR {
namespace IOSVehicleCore {
namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr int RAYS = 8;
float Clamp(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }
float Length(const PhysicsVec3& v) { return std::sqrt(v.squaredLength()); }
PhysicsVec3 Unit(const PhysicsVec3& v) { const float l = Length(v); return l > 1e-7f ? v * (1.0f / l) : PhysicsVec3(); }
bool Finite(const PhysicsVec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
}

struct DriftDemoRuntime::Impl
{
    struct Link { std::size_t a=0, b=0; BeamCoreState beam; };
    struct Wheel
    {
        std::size_t a0=0, a1=0;
        bool driven=false, handbraked=false;
        std::vector<std::size_t> tire;
        std::vector<WheelNodeBinding> bindings;
        WheelCoreState state;
    };

    std::deque<NodeCoreState> nodes;
    std::vector<Link> links;
    std::vector<std::pair<std::size_t,std::size_t>> pairs;
    std::vector<std::size_t> tire_nodes;
    std::vector<std::size_t> front_axes;
    std::vector<Wheel> wheels;
    GroundContactParams road;

    float steering=0, throttle=0, brake=0, throttle_state=0;
    bool handbrake=false, finite=true;
    std::uint64_t steps=0;

    std::size_t AddNode(float x,float y,float z,float mass)
    {
        nodes.emplace_back();
        NodeCoreState& n=nodes.back(); n.position=PhysicsVec3(x,y,z); n.mass=mass;
        n.force=PhysicsVec3(0,mass*DEFAULT_GRAVITY,0); return nodes.size()-1;
    }

    void AddBeam(std::size_t a,std::size_t b,float k,float d)
    {
        Link l; l.a=a; l.b=b; l.beam.rest_length=Length(nodes[b].position-nodes[a].position);
        l.beam.spring=k; l.beam.damping=d; links.push_back(l); pairs.emplace_back(a,b);
    }

    Wheel MakeWheel(float x,float zi,float zo,bool driven,bool hb)
    {
        Wheel w; w.a0=AddNode(x,.54f,zi,22); w.a1=AddNode(x,.54f,zo,22);
        w.driven=driven; w.handbraked=hb; w.state.radius=.43f; w.state.rotational_mass=35.2f;
        AddBeam(w.a0,w.a1,500000,6000);
        for(int r=0;r<RAYS;++r)
        {
            const float a=2*PI*r/RAYS; const PhysicsVec3 rad(.43f*std::cos(a),.43f*std::sin(a),0);
            const std::size_t ax[2]={w.a0,w.a1};
            for(int s=0;s<2;++s)
            {
                const PhysicsVec3 p=nodes[ax[s]].position+rad;
                const std::size_t o=AddNode(p.x,p.y,p.z,2.2f); w.tire.push_back(o); tire_nodes.push_back(o);
                WheelNodeBinding bind; bind.outer=&nodes[o]; bind.inner=&nodes[ax[s]]; w.bindings.push_back(bind);
                AddBeam(o,w.a0,200000,2500); AddBeam(o,w.a1,200000,2500);
            }
        }
        for(int r=0;r<RAYS;++r)
        {
            const int n=(r+1)%RAYS; const std::size_t a0=w.tire[r*2],a1=w.tire[r*2+1],b0=w.tire[n*2],b1=w.tire[n*2+1];
            AddBeam(a0,b0,150000,1600); AddBeam(a1,b1,150000,1600); AddBeam(a0,a1,190000,2000); AddBeam(a0,b1,150000,1600);
        }
        return w;
    }

    void Mount(Wheel& w,std::size_t low,std::size_t high,bool front)
    {
        // Same-side upper/lower mounts avoid the crossed suspension geometry
        // that was stable at rest but singular as soon as wheel torque arrived.
        AddBeam(w.a0,low,500000,6500); AddBeam(w.a1,low,500000,6500);
        AddBeam(w.a0,high,360000,6000); AddBeam(w.a1,high,360000,6000);
        if(front){ front_axes.push_back(w.a0); front_axes.push_back(w.a1); }
    }

    void Build()
    {
        nodes.clear(); links.clear(); pairs.clear(); tire_nodes.clear(); front_axes.clear(); wheels.clear();
        // lower FL,FR,RL,RR then upper FL,FR,RL,RR
        AddNode( 1.20f,.84f, .52f,65); AddNode( 1.20f,.84f,-.52f,65);
        AddNode(-1.20f,.84f, .52f,65); AddNode(-1.20f,.84f,-.52f,65);
        AddNode( .78f,1.18f, .42f,45); AddNode( .78f,1.18f,-.42f,45);
        AddNode(-.78f,1.18f, .42f,45); AddNode(-.78f,1.18f,-.42f,45);
        const int frame[][2]={{0,1},{0,2},{1,3},{2,3},{0,3},{1,2},{4,5},{4,6},{5,7},{6,7},{4,7},{5,6},{0,4},{1,5},{2,6},{3,7},{0,5},{1,4},{2,7},{3,6},{0,6},{1,7},{2,4},{3,5}};
        for(const auto& p:frame) AddBeam(p[0],p[1],760000,9000);

        wheels.reserve(4);
        wheels.push_back(MakeWheel( .98f, .68f, .90f,false,false));
        wheels.push_back(MakeWheel( .98f,-.90f,-.68f,false,false));
        wheels.push_back(MakeWheel(-.98f, .68f, .90f,true,true));
        wheels.push_back(MakeWheel(-.98f,-.90f,-.68f,true,true));
        Mount(wheels[0],0,4,true); Mount(wheels[1],1,5,true); Mount(wheels[2],2,6,false); Mount(wheels[3],3,7,false);

        road.friction.adhesion_velocity=.55f; road.friction.static_friction=1.15f; road.friction.sliding_friction=.72f;
        road.friction.hydrodynamic_friction=.018f; road.friction.stribeck_velocity=.35f; road.friction.stribeck_alpha=2;
        road.ground_strength=1; road.node_friction=1; road.solid_ground_level=0;
        steering=throttle=brake=throttle_state=0; handbrake=false; finite=true; steps=0;
    }

    PhysicsVec3 Center() const
    {
        PhysicsVec3 c; float m=0; for(const NodeCoreState& n:nodes){c+=n.position*n.mass;m+=n.mass;} return m?c*(1/m):c;
    }
    PhysicsVec3 Velocity() const
    {
        PhysicsVec3 v; float m=0; for(const NodeCoreState& n:nodes){v+=n.velocity*n.mass;m+=n.mass;} return m?v*(1/m):v;
    }
    PhysicsVec3 Forward() const
    {
        PhysicsVec3 f=(nodes[0].position+nodes[1].position)*.5f-(nodes[2].position+nodes[3].position)*.5f; f.y=0; return Unit(f);
    }

    void Steer()
    {
        const PhysicsVec3 f=Forward(); if(f.squaredLength()<.5f || front_axes.empty()) return;
        const PhysicsVec3 r(-f.z,0,f.x); PhysicsVec3 v;
        for(std::size_t i:front_axes) v+=nodes[i].velocity; v=v*(1.0f/front_axes.size());
        const float target=v.dot(f)*std::tan(steering*.55f); const float error=target-v.dot(r);
        const PhysicsVec3 force=r*(Clamp(error*4200,-14500,14500)/front_axes.size());
        for(std::size_t i:front_axes) nodes[i].force+=force;
    }

    void Step(float dt)
    {
        if(!finite||dt<=0) return;
        for(std::size_t i:tire_nodes) ApplyFlatGroundContact(nodes[i],0,road,dt);
        Steer();
        for(NodeCoreState& n:nodes){IntegrateNode(n,DEFAULT_GRAVITY,dt);if(!Finite(n.position)||!Finite(n.velocity)){finite=false;return;}}

        const float rate=2.4f*dt; throttle_state += Clamp(throttle-throttle_state,-rate*1.8f,rate);
        DifferentialData diff{}; diff.speed[0]=wheels[2].state.speed; diff.speed[1]=wheels[3].state.speed;
        diff.in_torque=throttle_state*1700.0f; diff.dt=dt; Differential::CalcSeparateDiff(diff);
        wheels[2].state.torque+=diff.out_torque[0]; wheels[3].state.torque+=diff.out_torque[1];

        for(Wheel& w:wheels)
        {
            float avail=brake*1800.0f + ((handbrake&&w.handbraked)?3200.0f:0.0f);
            if(avail>0) w.state.torque+=CalcWheelBrakeTorque(w.state.speed,w.state.average_speed,w.state.radius,w.state.rotational_mass,w.state.last_reaction_torque,avail,dt);
            StepWheelNodes(w.state,nodes[w.a0],nodes[w.a1],w.bindings,dt);
        }
        // Reaction torque stays disabled in this demo until the parsed rig's
        // authored reference-arm nodes are available. The core implementation
        // remains tested independently.
        for(Link& l:links){ApplyBeamForce(nodes[l.a],nodes[l.b],l.beam);if(!std::isfinite(l.beam.stress)){finite=false;return;}}
        ++steps;
    }
};

DriftDemoRuntime::DriftDemoRuntime():m_impl(new Impl()){m_impl->Build();}
DriftDemoRuntime::~DriftDemoRuntime()=default;
void DriftDemoRuntime::Reset(){m_impl->Build();}
void DriftDemoRuntime::SetControls(float s,float t,float b,bool h){m_impl->steering=Clamp(s,-1,1);m_impl->throttle=Clamp(t,0,1);m_impl->brake=Clamp(b,0,1);m_impl->handbrake=h;}
void DriftDemoRuntime::Step(float dt){m_impl->Step(dt);}
std::size_t DriftDemoRuntime::NodeCount() const{return m_impl->nodes.size();}
const NodeCoreState& DriftDemoRuntime::Node(std::size_t i) const{return m_impl->nodes.at(i);}
const std::vector<std::pair<std::size_t,std::size_t>>& DriftDemoRuntime::BeamPairs() const{return m_impl->pairs;}
const std::vector<std::size_t>& DriftDemoRuntime::TireNodeIndices() const{return m_impl->tire_nodes;}
DriftDemoTelemetry DriftDemoRuntime::Telemetry() const
{
    DriftDemoTelemetry t; t.center=m_impl->Center(); const PhysicsVec3 v=m_impl->Velocity(),f=m_impl->Forward();
    t.heading_radians=std::atan2(f.z,f.x); t.speed_mps=Length(v); t.forward_speed_mps=v.dot(f);
    t.rear_wheel_speed_mps=.5f*(std::fabs(m_impl->wheels[2].state.speed)+std::fabs(m_impl->wheels[3].state.speed));
    t.steering=m_impl->steering;t.throttle=m_impl->throttle;t.handbrake=m_impl->handbrake;t.physics_steps=m_impl->steps;return t;
}
bool DriftDemoRuntime::IsFinite() const{return m_impl->finite;}

} // namespace IOSVehicleCore
} // namespace RoR
