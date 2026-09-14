#include "DriftDemoRuntime.h"

#include "Differentials.h"
#include "GroundContact.h"
#include "SimConstants.h"
#include "WheelPhysics.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace RoR {
namespace IOSVehicleCore {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr int kWheelRays = 8;

float Clamp(float value, float low, float high)
{
    return std::max(low, std::min(value, high));
}

float Length(const PhysicsVec3& value)
{
    return std::sqrt(value.squaredLength());
}

PhysicsVec3 Normalized(const PhysicsVec3& value)
{
    const float len = Length(value);
    return (len > 1.0e-7f) ? value * (1.0f / len) : PhysicsVec3();
}

bool Finite(const PhysicsVec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

} // namespace

struct DriftDemoRuntime::Impl
{
    struct BeamLink
    {
        std::size_t a = 0;
        std::size_t b = 0;
        BeamCoreState beam;
    };

    struct WheelFixture
    {
        std::size_t axis0 = 0;
        std::size_t axis1 = 0;
        std::size_t reaction_arm = 0;
        std::size_t near_attach = 0;
        bool driven = false;
        bool handbraked = false;
        std::vector<std::size_t> outer_nodes;
        std::vector<WheelNodeBinding> bindings;
        WheelCoreState wheel;
    };

    std::deque<NodeCoreState> nodes;
    std::vector<BeamLink> beams;
    std::vector<std::pair<std::size_t, std::size_t>> beam_pairs;
    std::vector<std::size_t> tire_nodes;
    std::vector<WheelFixture> wheels;
    GroundContactParams road;

    float steering = 0.0f;
    float throttle = 0.0f;
    float brake = 0.0f;
    bool handbrake = false;
    float diff_delta_rotation = 0.0f;
    float throttle_state = 0.0f;
    std::uint64_t physics_steps = 0;
    bool finite = true;

    std::size_t chassis_front_left = 0;
    std::size_t chassis_front_right = 0;
    std::size_t chassis_rear_left = 0;
    std::size_t chassis_rear_right = 0;
    std::vector<std::size_t> front_axis_nodes;

    std::size_t AddNode(const PhysicsVec3& position, float mass)
    {
        nodes.emplace_back();
        NodeCoreState& node = nodes.back();
        node.position = position;
        node.mass = mass;
        node.force = PhysicsVec3(0.0f, mass * DEFAULT_GRAVITY, 0.0f);
        return nodes.size() - 1;
    }

    void AddBeam(std::size_t a, std::size_t b, float spring, float damping)
    {
        if (a == b || a >= nodes.size() || b >= nodes.size())
        {
            return;
        }
        BeamLink link;
        link.a = a;
        link.b = b;
        link.beam.rest_length = Length(nodes[b].position - nodes[a].position);
        link.beam.spring = spring;
        link.beam.damping = damping;
        beams.push_back(link);
        beam_pairs.emplace_back(a, b);
    }

    void AddFrameBeam(std::size_t a, std::size_t b)
    {
        AddBeam(a, b, 620000.0f, 8200.0f);
    }

    WheelFixture BuildWheel(
        float x,
        float z_inner,
        float z_outer,
        bool driven,
        bool parking_brake,
        std::size_t reaction_arm,
        std::size_t near_attach)
    {
        WheelFixture fixture;
        fixture.axis0 = AddNode(PhysicsVec3(x, 0.53f, z_inner), 18.0f);
        fixture.axis1 = AddNode(PhysicsVec3(x, 0.53f, z_outer), 18.0f);
        fixture.reaction_arm = reaction_arm;
        fixture.near_attach = near_attach;
        fixture.driven = driven;
        fixture.handbraked = parking_brake;
        fixture.wheel.radius = 0.43f;
        fixture.wheel.rotational_mass = static_cast<float>(kWheelRays * 2) * 2.2f;
        fixture.outer_nodes.reserve(kWheelRays * 2);

        for (int ray = 0; ray < kWheelRays; ++ray)
        {
            const float angle = 2.0f * kPi * static_cast<float>(ray) / static_cast<float>(kWheelRays);
            const PhysicsVec3 radial(
                fixture.wheel.radius * std::cos(angle),
                fixture.wheel.radius * std::sin(angle),
                0.0f);

            const std::size_t sides[2] = {fixture.axis0, fixture.axis1};
            for (int side = 0; side < 2; ++side)
            {
                const std::size_t outer = AddNode(nodes[sides[side]].position + radial, 2.2f);
                fixture.outer_nodes.push_back(outer);
                tire_nodes.push_back(outer);

                WheelNodeBinding binding;
                binding.outer = &nodes[outer];
                binding.inner = &nodes[sides[side]];
                fixture.bindings.push_back(binding);

                // Same soft-wheel geometric principle as RoR's generated wheels:
                // every tire node is located by both axle nodes.
                AddBeam(outer, fixture.axis0, 190000.0f, 2600.0f);
                AddBeam(outer, fixture.axis1, 190000.0f, 2600.0f);
            }
        }

        for (int ray = 0; ray < kWheelRays; ++ray)
        {
            const int next = (ray + 1) % kWheelRays;
            const std::size_t side0 = fixture.outer_nodes[ray * 2];
            const std::size_t side1 = fixture.outer_nodes[ray * 2 + 1];
            const std::size_t next0 = fixture.outer_nodes[next * 2];
            const std::size_t next1 = fixture.outer_nodes[next * 2 + 1];
            AddBeam(side0, next0, 145000.0f, 1800.0f);
            AddBeam(side1, next1, 145000.0f, 1800.0f);
            AddBeam(side0, side1, 190000.0f, 2200.0f);
            AddBeam(side0, next1, 145000.0f, 1800.0f);
        }

        return fixture;
    }

    void ConnectWheelToChassis(WheelFixture& wheel, bool front, bool left)
    {
        const std::size_t lower_near = front
            ? (left ? chassis_front_left : chassis_front_right)
            : (left ? chassis_rear_left : chassis_rear_right);
        const std::size_t lower_far = front
            ? (left ? chassis_front_right : chassis_front_left)
            : (left ? chassis_rear_right : chassis_rear_left);

        // Compliant locating links give the portable demo a real sprung mass
        // without introducing a rigid-body wheel shortcut.
        AddBeam(wheel.axis0, lower_near, 260000.0f, 6500.0f);
        AddBeam(wheel.axis1, lower_near, 260000.0f, 6500.0f);
        AddBeam(wheel.axis0, lower_far, 110000.0f, 4200.0f);
        AddBeam(wheel.axis1, lower_far, 110000.0f, 4200.0f);

        if (front)
        {
            front_axis_nodes.push_back(wheel.axis0);
            front_axis_nodes.push_back(wheel.axis1);
        }
    }

    void Build()
    {
        nodes.clear();
        beams.clear();
        beam_pairs.clear();
        tire_nodes.clear();
        wheels.clear();
        front_axis_nodes.clear();

        // Eight-node deformable space-frame. X is forward, Y is up, Z is lateral.
        chassis_front_left  = AddNode(PhysicsVec3( 1.20f, 0.82f,  0.53f), 62.0f);
        chassis_front_right = AddNode(PhysicsVec3( 1.20f, 0.82f, -0.53f), 62.0f);
        chassis_rear_left   = AddNode(PhysicsVec3(-1.20f, 0.82f,  0.53f), 62.0f);
        chassis_rear_right  = AddNode(PhysicsVec3(-1.20f, 0.82f, -0.53f), 62.0f);
        const std::size_t f_ul = AddNode(PhysicsVec3( 0.78f, 1.16f,  0.43f), 44.0f);
        const std::size_t f_ur = AddNode(PhysicsVec3( 0.78f, 1.16f, -0.43f), 44.0f);
        const std::size_t r_ul = AddNode(PhysicsVec3(-0.78f, 1.16f,  0.43f), 44.0f);
        const std::size_t r_ur = AddNode(PhysicsVec3(-0.78f, 1.16f, -0.43f), 44.0f);

        const std::size_t frame[][2] = {
            {0,1},{0,2},{1,3},{2,3},{0,3},{1,2},
            {4,5},{4,6},{5,7},{6,7},{4,7},{5,6},
            {0,4},{1,5},{2,6},{3,7},{0,5},{1,4},{2,7},{3,6},
            {0,6},{1,7},{2,4},{3,5}
        };
        for (const auto& pair : frame)
        {
            AddFrameBeam(pair[0], pair[1]);
        }

        wheels.reserve(4);
        // Axis order always points +Z so positive wheel torque produces the
        // same tread direction on all four corners.
        wheels.push_back(BuildWheel( 0.98f,  0.68f,  0.90f, false, false, chassis_front_left, 0));
        wheels.push_back(BuildWheel( 0.98f, -0.90f, -0.68f, false, false, chassis_front_right, 0));
        wheels.push_back(BuildWheel(-0.98f,  0.68f,  0.90f, true,  true,  chassis_rear_left, 0));
        wheels.push_back(BuildWheel(-0.98f, -0.90f, -0.68f, true,  true,  chassis_rear_right, 0));

        ConnectWheelToChassis(wheels[0], true, true);
        ConnectWheelToChassis(wheels[1], true, false);
        ConnectWheelToChassis(wheels[2], false, true);
        ConnectWheelToChassis(wheels[3], false, false);

        // The closest axis node is used for the reaction-arm force pair.
        wheels[0].near_attach = wheels[0].axis0;
        wheels[1].near_attach = wheels[1].axis1;
        wheels[2].near_attach = wheels[2].axis0;
        wheels[3].near_attach = wheels[3].axis1;

        road = GroundContactParams();
        road.friction.adhesion_velocity = 0.55f;
        road.friction.static_friction = 1.15f;
        road.friction.sliding_friction = 0.72f;
        road.friction.hydrodynamic_friction = 0.018f;
        road.friction.stribeck_velocity = 0.35f;
        road.friction.stribeck_alpha = 2.0f;
        road.ground_strength = 1.0f;
        road.node_friction = 1.0f;
        road.solid_ground_level = 0.0f;

        steering = 0.0f;
        throttle = 0.0f;
        brake = 0.0f;
        handbrake = false;
        diff_delta_rotation = 0.0f;
        throttle_state = 0.0f;
        physics_steps = 0;
        finite = true;
    }

    PhysicsVec3 Center() const
    {
        PhysicsVec3 weighted;
        float mass = 0.0f;
        for (const NodeCoreState& node : nodes)
        {
            weighted += node.position * node.mass;
            mass += node.mass;
        }
        return mass > 0.0f ? weighted * (1.0f / mass) : PhysicsVec3();
    }

    PhysicsVec3 CenterVelocity() const
    {
        PhysicsVec3 weighted;
        float mass = 0.0f;
        for (const NodeCoreState& node : nodes)
        {
            weighted += node.velocity * node.mass;
            mass += node.mass;
        }
        return mass > 0.0f ? weighted * (1.0f / mass) : PhysicsVec3();
    }

    PhysicsVec3 Forward() const
    {
        const PhysicsVec3 front = (nodes[chassis_front_left].position + nodes[chassis_front_right].position) * 0.5f;
        const PhysicsVec3 rear = (nodes[chassis_rear_left].position + nodes[chassis_rear_right].position) * 0.5f;
        PhysicsVec3 direction = front - rear;
        direction.y = 0.0f;
        return Normalized(direction);
    }

    float Heading() const
    {
        const PhysicsVec3 forward = Forward();
        return std::atan2(forward.z, forward.x);
    }

    void ApplySteeringForce()
    {
        const PhysicsVec3 forward = Forward();
        if (forward.squaredLength() < 0.5f)
        {
            return;
        }
        const PhysicsVec3 right(-forward.z, 0.0f, forward.x);
        PhysicsVec3 front_velocity;
        for (std::size_t index : front_axis_nodes)
        {
            front_velocity += nodes[index].velocity;
        }
        if (!front_axis_nodes.empty())
        {
            front_velocity = front_velocity * (1.0f / static_cast<float>(front_axis_nodes.size()));
        }

        const float forward_speed = front_velocity.dot(forward);
        const float lateral_speed = front_velocity.dot(right);
        const float steer_angle = steering * 0.58f; // ~33 degrees at full lock.
        const float desired_lateral = forward_speed * std::tan(steer_angle);
        const float error = desired_lateral - lateral_speed;
        const float lateral_force = Clamp(error * 5200.0f, -18500.0f, 18500.0f);
        const PhysicsVec3 per_node = right * (lateral_force / static_cast<float>(front_axis_nodes.size()));
        for (std::size_t index : front_axis_nodes)
        {
            nodes[index].force += per_node;
        }
    }

    void StepInternal(float dt)
    {
        if (!finite || dt <= 0.0f)
        {
            return;
        }

        // Tire/road contact is the same portable primitive used by the core probes.
        for (std::size_t index : tire_nodes)
        {
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
        }

        ApplySteeringForce();

        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, dt);
            if (!Finite(node.position) || !Finite(node.velocity))
            {
                finite = false;
                return;
            }
        }

        // Clutch/throttle ramp prevents an impossible torque impulse on tick one.
        const float throttle_rate = 2.8f * dt;
        if (throttle_state < throttle)
            throttle_state = std::min(throttle, throttle_state + throttle_rate);
        else
            throttle_state = std::max(throttle, throttle_state - throttle_rate * 1.8f);

        DifferentialData diff;
        diff.speed[0] = wheels[2].wheel.speed;
        diff.speed[1] = wheels[3].wheel.speed;
        diff.delta_rotation = diff_delta_rotation;
        diff.in_torque = throttle_state * 2550.0f;
        diff.dt = dt;
        Differential::CalcLockedDiff(diff);
        diff_delta_rotation = diff.delta_rotation;
        wheels[2].wheel.torque += diff.out_torque[0];
        wheels[3].wheel.torque += diff.out_torque[1];

        for (std::size_t i = 0; i < wheels.size(); ++i)
        {
            WheelFixture& fixture = wheels[i];
            float braking_torque = brake * 2200.0f;
            if (handbrake && fixture.handbraked)
            {
                braking_torque += 4300.0f;
            }
            if (braking_torque > 0.0f)
            {
                fixture.wheel.torque += CalcWheelBrakeTorque(
                    fixture.wheel.speed,
                    fixture.wheel.average_speed,
                    fixture.wheel.radius,
                    fixture.wheel.rotational_mass,
                    fixture.wheel.last_reaction_torque,
                    braking_torque,
                    dt);
            }

            const PhysicsVec3 axis = nodes[fixture.axis1].position - nodes[fixture.axis0].position;
            const WheelStepResult result = StepWheelNodes(
                fixture.wheel,
                nodes[fixture.axis0],
                nodes[fixture.axis1],
                fixture.bindings,
                dt);

            if (fixture.driven)
            {
                ApplyWheelReactionTorque(
                    result.applied_torque,
                    axis,
                    nodes[fixture.reaction_arm],
                    nodes[fixture.near_attach]);
            }
        }

        for (BeamLink& link : beams)
        {
            ApplyBeamForce(nodes[link.a], nodes[link.b], link.beam);
            if (!std::isfinite(link.beam.stress))
            {
                finite = false;
                return;
            }
        }

        ++physics_steps;
    }
};

DriftDemoRuntime::DriftDemoRuntime(): m_impl(new Impl())
{
    m_impl->Build();
}

DriftDemoRuntime::~DriftDemoRuntime() = default;

void DriftDemoRuntime::Reset()
{
    m_impl->Build();
}

void DriftDemoRuntime::SetControls(float steering, float throttle, float brake, bool handbrake)
{
    m_impl->steering = Clamp(steering, -1.0f, 1.0f);
    m_impl->throttle = Clamp(throttle, 0.0f, 1.0f);
    m_impl->brake = Clamp(brake, 0.0f, 1.0f);
    m_impl->handbrake = handbrake;
}

void DriftDemoRuntime::Step(float dt)
{
    m_impl->StepInternal(dt);
}

std::size_t DriftDemoRuntime::NodeCount() const
{
    return m_impl->nodes.size();
}

const NodeCoreState& DriftDemoRuntime::Node(std::size_t index) const
{
    return m_impl->nodes.at(index);
}

const std::vector<std::pair<std::size_t, std::size_t>>& DriftDemoRuntime::BeamPairs() const
{
    return m_impl->beam_pairs;
}

const std::vector<std::size_t>& DriftDemoRuntime::TireNodeIndices() const
{
    return m_impl->tire_nodes;
}

DriftDemoTelemetry DriftDemoRuntime::Telemetry() const
{
    DriftDemoTelemetry result;
    result.center = m_impl->Center();
    const PhysicsVec3 velocity = m_impl->CenterVelocity();
    const PhysicsVec3 forward = m_impl->Forward();
    result.heading_radians = m_impl->Heading();
    result.speed_mps = Length(velocity);
    result.forward_speed_mps = velocity.dot(forward);
    result.rear_wheel_speed_mps = 0.5f *
        (std::fabs(m_impl->wheels[2].wheel.speed) + std::fabs(m_impl->wheels[3].wheel.speed));
    result.steering = m_impl->steering;
    result.throttle = m_impl->throttle;
    result.handbrake = m_impl->handbrake;
    result.physics_steps = m_impl->physics_steps;
    return result;
}

bool DriftDemoRuntime::IsFinite() const
{
    return m_impl->finite;
}

} // namespace IOSVehicleCore
} // namespace RoR
