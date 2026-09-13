#include "BeamPhysics.h"
#include "Differentials.h"
#include "GroundContact.h"
#include "SimConstants.h"
#include "WheelPhysics.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <vector>

using namespace RoR;

namespace
{
constexpr int RAYS = 12;
constexpr float PI = 3.14159265358979323846f;

struct BeamLink
{
    NodeCoreState* a = nullptr;
    NodeCoreState* b = nullptr;
    BeamCoreState beam;
};

struct WheelFixture
{
    NodeCoreState* axis0 = nullptr;
    NodeCoreState* axis1 = nullptr;
    std::vector<NodeCoreState*> outer_nodes;
    std::vector<WheelNodeBinding> bindings;
    WheelCoreState wheel;
};

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

float Length(const PhysicsVec3& value)
{
    return std::sqrt(value.squaredLength());
}

float Distance(const PhysicsVec3& a, const PhysicsVec3& b)
{
    return Length(a - b);
}

bool Finite(const PhysicsVec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

NodeCoreState* AddNode(std::deque<NodeCoreState>& nodes, const PhysicsVec3& position, float mass)
{
    nodes.emplace_back();
    NodeCoreState& node = nodes.back();
    node.position = position;
    node.mass = mass;
    node.force = PhysicsVec3(0.0f, mass * DEFAULT_GRAVITY, 0.0f);
    return &node;
}

void AddBeam(std::vector<BeamLink>& beams, NodeCoreState* a, NodeCoreState* b, float spring, float damping)
{
    BeamLink link{};
    link.a = a;
    link.b = b;
    link.beam.rest_length = Distance(a->position, b->position);
    link.beam.spring = spring;
    link.beam.damping = damping;
    beams.push_back(link);
}

WheelFixture BuildWheel(
    std::deque<NodeCoreState>& nodes,
    std::vector<BeamLink>& beams,
    NodeCoreState* axis0,
    NodeCoreState* axis1)
{
    WheelFixture fixture{};
    fixture.axis0 = axis0;
    fixture.axis1 = axis1;
    fixture.wheel.radius = 0.50f;
    fixture.wheel.rotational_mass = static_cast<float>(RAYS * 2) * 3.0f;
    fixture.outer_nodes.reserve(RAYS * 2);
    fixture.bindings.reserve(RAYS * 2);

    for (int ray = 0; ray < RAYS; ++ray)
    {
        const float angle = 2.0f * PI * static_cast<float>(ray) / static_cast<float>(RAYS);
        const PhysicsVec3 radial(
            fixture.wheel.radius * std::cos(angle),
            fixture.wheel.radius * std::sin(angle),
            0.0f);

        for (int side = 0; side < 2; ++side)
        {
            NodeCoreState* inner = (side == 0) ? axis0 : axis1;
            NodeCoreState* outer = AddNode(nodes, inner->position + radial, 3.0f);
            fixture.outer_nodes.push_back(outer);

            WheelNodeBinding binding{};
            binding.outer = outer;
            binding.inner = inner;
            fixture.bindings.push_back(binding);

            // A point on the tire can rotate around the axle while keeping a
            // constant distance to both axis nodes. This is the same geometric
            // trick used by RoR's auto-generated node/beam wheels.
            AddBeam(beams, outer, axis0, 200000.0f, 2500.0f);
            AddBeam(beams, outer, axis1, 200000.0f, 2500.0f);
        }
    }

    for (int ray = 0; ray < RAYS; ++ray)
    {
        const int next = (ray + 1) % RAYS;
        NodeCoreState* side0 = fixture.outer_nodes[ray * 2];
        NodeCoreState* side1 = fixture.outer_nodes[ray * 2 + 1];
        NodeCoreState* next0 = fixture.outer_nodes[next * 2];
        NodeCoreState* next1 = fixture.outer_nodes[next * 2 + 1];

        AddBeam(beams, side0, next0, 150000.0f, 1500.0f);
        AddBeam(beams, side1, next1, 150000.0f, 1500.0f);
        AddBeam(beams, side0, side1, 200000.0f, 2000.0f);
        AddBeam(beams, side0, next1, 150000.0f, 1500.0f);
    }

    return fixture;
}

PhysicsVec3 CenterOfMass(const std::deque<NodeCoreState>& nodes)
{
    PhysicsVec3 weighted;
    float total_mass = 0.0f;
    for (const NodeCoreState& node : nodes)
    {
        weighted += node.position * node.mass;
        total_mass += node.mass;
    }
    return weighted * (1.0f / total_mass);
}

float AverageRadius(const WheelFixture& fixture)
{
    float total = 0.0f;
    for (size_t j = 0; j < fixture.outer_nodes.size(); ++j)
    {
        const NodeCoreState* inner = (j % 2 != 0) ? fixture.axis1 : fixture.axis0;
        total += Distance(fixture.outer_nodes[j]->position, inner->position);
    }
    return total / static_cast<float>(fixture.outer_nodes.size());
}
}

int main()
{
    std::deque<NodeCoreState> nodes;
    std::vector<BeamLink> beams;
    beams.reserve(220);

    // Rear axle nodes. Both wheel axes point +Z so positive torque produces
    // the same tread direction on both sides of the car.
    NodeCoreState* left_axis0  = AddNode(nodes, PhysicsVec3(0.0f, 0.55f,  0.65f), 60.0f);
    NodeCoreState* left_axis1  = AddNode(nodes, PhysicsVec3(0.0f, 0.55f,  0.85f), 60.0f);
    NodeCoreState* right_axis0 = AddNode(nodes, PhysicsVec3(0.0f, 0.55f, -0.85f), 60.0f);
    NodeCoreState* right_axis1 = AddNode(nodes, PhysicsVec3(0.0f, 0.55f, -0.65f), 60.0f);

    NodeCoreState* chassis_fl = AddNode(nodes, PhysicsVec3( 0.70f, 0.90f,  0.55f), 50.0f);
    NodeCoreState* chassis_rl = AddNode(nodes, PhysicsVec3(-0.70f, 0.90f,  0.55f), 50.0f);
    NodeCoreState* chassis_fr = AddNode(nodes, PhysicsVec3( 0.70f, 0.90f, -0.55f), 50.0f);
    NodeCoreState* chassis_rr = AddNode(nodes, PhysicsVec3(-0.70f, 0.90f, -0.55f), 50.0f);

    NodeCoreState* chassis_nodes[4] = {chassis_fl, chassis_rl, chassis_fr, chassis_rr};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            AddBeam(beams, chassis_nodes[i], chassis_nodes[j], 800000.0f, 8000.0f);
        }
    }

    AddBeam(beams, left_axis0,  chassis_fl, 500000.0f, 5000.0f);
    AddBeam(beams, left_axis0,  chassis_rl, 500000.0f, 5000.0f);
    AddBeam(beams, left_axis1,  chassis_fl, 500000.0f, 5000.0f);
    AddBeam(beams, left_axis1,  chassis_rl, 500000.0f, 5000.0f);
    AddBeam(beams, right_axis0, chassis_fr, 500000.0f, 5000.0f);
    AddBeam(beams, right_axis0, chassis_rr, 500000.0f, 5000.0f);
    AddBeam(beams, right_axis1, chassis_fr, 500000.0f, 5000.0f);
    AddBeam(beams, right_axis1, chassis_rr, 500000.0f, 5000.0f);
    AddBeam(beams, left_axis0, right_axis1, 800000.0f, 8000.0f);

    WheelFixture left = BuildWheel(nodes, beams, left_axis0, left_axis1);
    WheelFixture right = BuildWheel(nodes, beams, right_axis0, right_axis1);

    GroundContactParams road{};
    road.friction.adhesion_velocity = 0.5f;
    road.friction.static_friction = 1.2f;
    road.friction.sliding_friction = 0.85f;
    road.friction.hydrodynamic_friction = 0.02f;
    road.friction.stribeck_velocity = 0.35f;
    road.friction.stribeck_alpha = 2.0f;
    road.ground_strength = 1.0f;
    road.node_friction = 1.0f;

    const PhysicsVec3 initial_com = CenterOfMass(nodes);
    float diff_delta_rotation = 0.0f;
    int contact_steps = 0;
    float speed_before_handbrake = 0.0f;
    PhysicsVec3 com_before_handbrake;

    for (int step = 0; step < 6000; ++step) // 3 seconds at RoR's 2 kHz timestep
    {
        bool contacted = false;
        for (NodeCoreState* node : left.outer_nodes)
        {
            contacted = ApplyFlatGroundContact(*node, 0.0f, road, PHYSICS_DT) || contacted;
        }
        for (NodeCoreState* node : right.outer_nodes)
        {
            contacted = ApplyFlatGroundContact(*node, 0.0f, road, PHYSICS_DT) || contacted;
        }
        if (contacted)
        {
            ++contact_steps;
        }

        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, PHYSICS_DT);
            Require(Finite(node.position) && Finite(node.velocity), "driven axle state remains finite");
        }

        if (step < 4000)
        {
            DifferentialData diff{};
            diff.speed[0] = left.wheel.speed;
            diff.speed[1] = right.wheel.speed;
            diff.delta_rotation = diff_delta_rotation;
            diff.in_torque = 1200.0f;
            diff.dt = PHYSICS_DT;
            Differential::CalcLockedDiff(diff);
            diff_delta_rotation = diff.delta_rotation;
            left.wheel.torque += diff.out_torque[0];
            right.wheel.torque += diff.out_torque[1];
        }
        else
        {
            left.wheel.torque += CalcWheelBrakeTorque(
                left.wheel.speed, left.wheel.average_speed, left.wheel.radius,
                left.wheel.rotational_mass, left.wheel.last_reaction_torque,
                2500.0f, PHYSICS_DT);
            right.wheel.torque += CalcWheelBrakeTorque(
                right.wheel.speed, right.wheel.average_speed, right.wheel.radius,
                right.wheel.rotational_mass, right.wheel.last_reaction_torque,
                2500.0f, PHYSICS_DT);
        }

        StepWheelNodes(left.wheel, *left.axis0, *left.axis1, left.bindings, PHYSICS_DT);
        StepWheelNodes(right.wheel, *right.axis0, *right.axis1, right.bindings, PHYSICS_DT);

        for (BeamLink& link : beams)
        {
            ApplyBeamForce(*link.a, *link.b, link.beam);
            Require(std::isfinite(link.beam.stress), "driven axle beam stress remains finite");
        }

        if (step == 3999)
        {
            speed_before_handbrake = 0.5f * (std::fabs(left.wheel.speed) + std::fabs(right.wheel.speed));
            com_before_handbrake = CenterOfMass(nodes);
        }
    }

    const PhysicsVec3 final_com = CenterOfMass(nodes);
    const float final_wheel_speed = 0.5f * (std::fabs(left.wheel.speed) + std::fabs(right.wheel.speed));
    const float driven_distance = std::fabs(com_before_handbrake.x - initial_com.x);

    Require(contact_steps > 500, "powered tires establish sustained road contact");
    Require(speed_before_handbrake > 2.0f, "locked rear axle spins up under drive torque");
    Require(driven_distance > 1.0f, "deformable powered axle propels the chassis across the road");
    Require(std::fabs(left.wheel.speed - right.wheel.speed) < 0.25f, "locked differential keeps rear wheel speeds together");
    Require(final_wheel_speed < speed_before_handbrake * 0.20f, "handbrake arrests rear wheel rotation");
    Require(std::fabs(AverageRadius(left) - left.wheel.radius) < 0.04f, "left deformable tire retains its radius");
    Require(std::fabs(AverageRadius(right) - right.wheel.radius) < 0.04f, "right deformable tire retains its radius");
    Require(Finite(final_com), "final driven chassis center of mass remains finite");

    std::cout << "RoR driven axle probe passed: " << nodes.size() << " nodes, "
              << beams.size() << " beams, " << driven_distance << " m under power, wheel speed "
              << speed_before_handbrake << " -> " << final_wheel_speed << " m/s with handbrake.\n";
    return EXIT_SUCCESS;
}
