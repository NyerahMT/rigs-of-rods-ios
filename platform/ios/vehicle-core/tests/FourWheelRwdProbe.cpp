#include "BeamPhysics.h"
#include "Differentials.h"
#include "GroundContact.h"
#include "HydroPhysics.h"
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

BeamLink* AddBeam(
    std::vector<BeamLink>& beams,
    NodeCoreState* a,
    NodeCoreState* b,
    float spring,
    float damping)
{
    BeamLink link{};
    link.a = a;
    link.b = b;
    link.beam.rest_length = Distance(a->position, b->position);
    link.beam.spring = spring;
    link.beam.damping = damping;
    beams.push_back(link);
    return &beams.back();
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
            fixture.bindings.push_back({outer, inner});

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

float SteeringAngle(const WheelFixture& wheel)
{
    const PhysicsVec3 axis = wheel.axis1->position - wheel.axis0->position;
    return std::atan2(axis.x, axis.z);
}

float AverageRadius(const WheelFixture& fixture)
{
    float total = 0.0f;
    for (size_t i = 0; i < fixture.outer_nodes.size(); ++i)
    {
        const NodeCoreState* inner = (i % 2 != 0) ? fixture.axis1 : fixture.axis0;
        total += Distance(fixture.outer_nodes[i]->position, inner->position);
    }
    return total / static_cast<float>(fixture.outer_nodes.size());
}

void ApplyTireContact(WheelFixture& wheel, const GroundContactParams& road, bool& contacted)
{
    for (NodeCoreState* node : wheel.outer_nodes)
    {
        contacted = ApplyFlatGroundContact(*node, 0.0f, road, PHYSICS_DT) || contacted;
    }
}
}

int main()
{
    std::deque<NodeCoreState> nodes;
    std::vector<BeamLink> beams;
    beams.reserve(600);

    NodeCoreState* fl_low  = AddNode(nodes, PhysicsVec3( 1.30f, 0.48f,  0.68f), 45.0f);
    NodeCoreState* fr_low  = AddNode(nodes, PhysicsVec3( 1.30f, 0.48f, -0.68f), 45.0f);
    NodeCoreState* rl_low  = AddNode(nodes, PhysicsVec3(-1.30f, 0.48f,  0.68f), 55.0f);
    NodeCoreState* rr_low  = AddNode(nodes, PhysicsVec3(-1.30f, 0.48f, -0.68f), 55.0f);
    NodeCoreState* fl_high = AddNode(nodes, PhysicsVec3( 1.30f, 0.98f,  0.68f), 45.0f);
    NodeCoreState* fr_high = AddNode(nodes, PhysicsVec3( 1.30f, 0.98f, -0.68f), 45.0f);
    NodeCoreState* rl_high = AddNode(nodes, PhysicsVec3(-1.30f, 0.98f,  0.68f), 55.0f);
    NodeCoreState* rr_high = AddNode(nodes, PhysicsVec3(-1.30f, 0.98f, -0.68f), 55.0f);

    NodeCoreState* body[8] = {fl_low, fr_low, rl_low, rr_low, fl_high, fr_high, rl_high, rr_high};
    for (int i = 0; i < 8; ++i)
    {
        for (int j = i + 1; j < 8; ++j)
        {
            if (Distance(body[i]->position, body[j]->position) < 3.25f)
            {
                AddBeam(beams, body[i], body[j], 900000.0f, 9000.0f);
            }
        }
    }

    NodeCoreState* rear_left_a0  = AddNode(nodes, PhysicsVec3(-1.30f, 0.62f,  0.80f), 30.0f);
    NodeCoreState* rear_left_a1  = AddNode(nodes, PhysicsVec3(-1.30f, 0.62f,  1.00f), 30.0f);
    NodeCoreState* rear_right_a0 = AddNode(nodes, PhysicsVec3(-1.30f, 0.62f, -1.00f), 30.0f);
    NodeCoreState* rear_right_a1 = AddNode(nodes, PhysicsVec3(-1.30f, 0.62f, -0.80f), 30.0f);

    AddBeam(beams, rear_left_a0, rl_low,  600000.0f, 6000.0f);
    AddBeam(beams, rear_left_a0, rl_high, 600000.0f, 6000.0f);
    AddBeam(beams, rear_left_a1, rl_low,  600000.0f, 6000.0f);
    AddBeam(beams, rear_left_a1, rl_high, 600000.0f, 6000.0f);
    AddBeam(beams, rear_right_a0, rr_low,  600000.0f, 6000.0f);
    AddBeam(beams, rear_right_a0, rr_high, 600000.0f, 6000.0f);
    AddBeam(beams, rear_right_a1, rr_low,  600000.0f, 6000.0f);
    AddBeam(beams, rear_right_a1, rr_high, 600000.0f, 6000.0f);

    NodeCoreState* left_kp_low  = AddNode(nodes, PhysicsVec3(1.30f, 0.48f,  0.90f), 8.0f);
    NodeCoreState* left_kp_high = AddNode(nodes, PhysicsVec3(1.30f, 0.98f,  0.90f), 8.0f);
    NodeCoreState* right_kp_low  = AddNode(nodes, PhysicsVec3(1.30f, 0.48f, -0.90f), 8.0f);
    NodeCoreState* right_kp_high = AddNode(nodes, PhysicsVec3(1.30f, 0.98f, -0.90f), 8.0f);

    AddBeam(beams, left_kp_low,  fl_low,  900000.0f, 9000.0f);
    AddBeam(beams, left_kp_low,  fl_high, 900000.0f, 9000.0f);
    AddBeam(beams, left_kp_low,  fr_low,  900000.0f, 9000.0f);
    AddBeam(beams, left_kp_high, fl_high, 900000.0f, 9000.0f);
    AddBeam(beams, left_kp_high, fl_low,  900000.0f, 9000.0f);
    AddBeam(beams, left_kp_high, fr_high, 900000.0f, 9000.0f);

    AddBeam(beams, right_kp_low,  fr_low,  900000.0f, 9000.0f);
    AddBeam(beams, right_kp_low,  fr_high, 900000.0f, 9000.0f);
    AddBeam(beams, right_kp_low,  fl_low,  900000.0f, 9000.0f);
    AddBeam(beams, right_kp_high, fr_high, 900000.0f, 9000.0f);
    AddBeam(beams, right_kp_high, fr_low,  900000.0f, 9000.0f);
    AddBeam(beams, right_kp_high, fl_high, 900000.0f, 9000.0f);

    NodeCoreState* front_left_a0  = AddNode(nodes, PhysicsVec3(1.30f, 0.68f,  0.80f), 22.0f);
    NodeCoreState* front_left_a1  = AddNode(nodes, PhysicsVec3(1.30f, 0.68f,  1.00f), 22.0f);
    NodeCoreState* front_right_a0 = AddNode(nodes, PhysicsVec3(1.30f, 0.68f, -1.00f), 22.0f);
    NodeCoreState* front_right_a1 = AddNode(nodes, PhysicsVec3(1.30f, 0.68f, -0.80f), 22.0f);

    AddBeam(beams, left_kp_low,  front_left_a0,  700000.0f, 7000.0f);
    AddBeam(beams, left_kp_high, front_left_a0,  700000.0f, 7000.0f);
    AddBeam(beams, left_kp_low,  front_left_a1,  700000.0f, 7000.0f);
    AddBeam(beams, left_kp_high, front_left_a1,  700000.0f, 7000.0f);
    AddBeam(beams, front_left_a0, front_left_a1, 700000.0f, 7000.0f);

    AddBeam(beams, right_kp_low,  front_right_a0, 700000.0f, 7000.0f);
    AddBeam(beams, right_kp_high, front_right_a0, 700000.0f, 7000.0f);
    AddBeam(beams, right_kp_low,  front_right_a1, 700000.0f, 7000.0f);
    AddBeam(beams, right_kp_high, front_right_a1, 700000.0f, 7000.0f);
    AddBeam(beams, front_right_a0, front_right_a1, 700000.0f, 7000.0f);

    NodeCoreState* rack_left  = AddNode(nodes, PhysicsVec3(1.00f, 0.68f,  0.65f), 8.0f);
    NodeCoreState* rack_right = AddNode(nodes, PhysicsVec3(1.00f, 0.68f, -0.65f), 8.0f);
    AddBeam(beams, rack_left, fl_low,   900000.0f, 9000.0f);
    AddBeam(beams, rack_left, fl_high,  900000.0f, 9000.0f);
    AddBeam(beams, rack_left, fr_low,   900000.0f, 9000.0f);
    AddBeam(beams, rack_right, fr_low,  900000.0f, 9000.0f);
    AddBeam(beams, rack_right, fr_high, 900000.0f, 9000.0f);
    AddBeam(beams, rack_right, fl_low,  900000.0f, 9000.0f);

    // Steering hydros are intentionally much stiffer than tire beams so road
    // forces cannot stretch the tie rods into huge toe/over-center excursions.
    BeamLink* left_hydro = AddBeam(beams, rack_left, front_left_a0, 1500000.0f, 15000.0f);
    BeamLink* right_hydro = AddBeam(beams, rack_right, front_right_a1, 1500000.0f, 15000.0f);
    const float left_hydro_reference = left_hydro->beam.rest_length;
    const float right_hydro_reference = right_hydro->beam.rest_length;

    WheelFixture front_left  = BuildWheel(nodes, beams, front_left_a0,  front_left_a1);
    WheelFixture front_right = BuildWheel(nodes, beams, front_right_a0, front_right_a1);
    WheelFixture rear_left   = BuildWheel(nodes, beams, rear_left_a0,   rear_left_a1);
    WheelFixture rear_right  = BuildWheel(nodes, beams, rear_right_a0,  rear_right_a1);

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
    PhysicsVec3 com_before_steer = initial_com;
    float diff_delta_rotation = 0.0f;
    float steering_state = 0.0f;
    int four_tire_contact_steps = 0;
    float left_presteer = 0.0f;
    float right_presteer = 0.0f;

    for (int step = 0; step < 10000; ++step)
    {
        bool fl_contact = false;
        bool fr_contact = false;
        bool rl_contact = false;
        bool rr_contact = false;
        ApplyTireContact(front_left,  road, fl_contact);
        ApplyTireContact(front_right, road, fr_contact);
        ApplyTireContact(rear_left,   road, rl_contact);
        ApplyTireContact(rear_right,  road, rr_contact);
        if (fl_contact && fr_contact && rl_contact && rr_contact)
        {
            ++four_tire_contact_steps;
        }

        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, PHYSICS_DT);
            Require(Finite(node.position) && Finite(node.velocity), "four-wheel vehicle state remains finite");
        }

        const float steering_command = (step < 6000) ? 0.0f : 0.70f;
        const float road_speed = 0.5f * (std::fabs(rear_left.wheel.speed) + std::fabs(rear_right.wheel.speed));
        steering_state = StepHydroSteeringState(
            steering_state, steering_command, road_speed, true, 1.0f, 1.0f, PHYSICS_DT);
        left_hydro->beam.rest_length = CalcHydroTargetLength(
            left_hydro_reference, steering_state, 0.10f, 0.10f, 0.10f);
        right_hydro->beam.rest_length = CalcHydroTargetLength(
            right_hydro_reference, steering_state, -0.10f, 0.10f, 0.10f);

        DifferentialData diff{};
        diff.speed[0] = rear_left.wheel.speed;
        diff.speed[1] = rear_right.wheel.speed;
        diff.delta_rotation = diff_delta_rotation;
        // Approximate clutch engagement instead of applying maximum driveline
        // torque on the first 0.5 ms simulation step.
        const float clutch_ramp = std::min(1.0f, static_cast<float>(step) / 1000.0f);
        diff.in_torque = 3200.0f * clutch_ramp;
        diff.dt = PHYSICS_DT;
        Differential::CalcLockedDiff(diff);
        diff_delta_rotation = diff.delta_rotation;
        rear_left.wheel.torque += diff.out_torque[0];
        rear_right.wheel.torque += diff.out_torque[1];

        StepWheelNodes(front_left.wheel,  *front_left.axis0,  *front_left.axis1,  front_left.bindings,  PHYSICS_DT);
        StepWheelNodes(front_right.wheel, *front_right.axis0, *front_right.axis1, front_right.bindings, PHYSICS_DT);
        StepWheelNodes(rear_left.wheel,   *rear_left.axis0,   *rear_left.axis1,   rear_left.bindings,   PHYSICS_DT);
        StepWheelNodes(rear_right.wheel,  *rear_right.axis0,  *rear_right.axis1,  rear_right.bindings,  PHYSICS_DT);

        for (BeamLink& link : beams)
        {
            ApplyBeamForce(*link.a, *link.b, link.beam);
            Require(std::isfinite(link.beam.stress), "four-wheel beam stress remains finite");
        }

        if (step == 5999)
        {
            com_before_steer = CenterOfMass(nodes);
            left_presteer = SteeringAngle(front_left);
            right_presteer = SteeringAngle(front_right);
        }
    }

    const PhysicsVec3 final_com = CenterOfMass(nodes);
    const float straight_driven_distance = std::fabs(com_before_steer.x - initial_com.x);
    const float final_horizontal_displacement = std::sqrt(
        (final_com.x - initial_com.x) * (final_com.x - initial_com.x) +
        (final_com.z - initial_com.z) * (final_com.z - initial_com.z));
    const float left_steer = SteeringAngle(front_left);
    const float right_steer = SteeringAngle(front_right);

    std::cerr << "four-wheel metrics: straight=" << straight_driven_distance
              << " m, horizontal=" << final_horizontal_displacement
              << " m, contacts=" << four_tire_contact_steps
              << ", rear speeds=" << rear_left.wheel.speed << "/" << rear_right.wheel.speed
              << " m/s, presteer=" << (left_presteer * 180.0f / PI) << "/"
              << (right_presteer * 180.0f / PI) << " deg, steer="
              << (left_steer * 180.0f / PI) << "/"
              << (right_steer * 180.0f / PI) << " deg\n";

    Require(four_tire_contact_steps > 500, "all four deformable tires establish road contact together");
    Require(straight_driven_distance > 1.5f, "rear-wheel drive propels the complete four-wheel chassis before steering");
    Require(std::fabs(left_presteer) < 3.0f * PI / 180.0f, "left front remains near straight-ahead under acceleration");
    Require(std::fabs(right_presteer) < 3.0f * PI / 180.0f, "right front remains near straight-ahead under acceleration");
    Require(std::fabs(rear_left.wheel.speed - rear_right.wheel.speed) < 0.5f, "locked rear diff keeps wheel speeds coupled");
    Require(std::fabs(left_steer) > 2.0f * PI / 180.0f, "left front structural carrier steers under hydro command");
    Require(std::fabs(right_steer) > 2.0f * PI / 180.0f, "right front structural carrier steers under hydro command");
    Require(std::fabs(left_steer) < 45.0f * PI / 180.0f, "left steering carrier does not pass over-center");
    Require(std::fabs(right_steer) < 45.0f * PI / 180.0f, "right steering carrier does not pass over-center");
    Require(left_steer * right_steer > 0.0f, "mirrored front linkages steer in the same vehicle direction");
    Require(std::fabs(AverageRadius(front_left)  - front_left.wheel.radius)  < 0.05f, "front-left tire retains radius");
    Require(std::fabs(AverageRadius(front_right) - front_right.wheel.radius) < 0.05f, "front-right tire retains radius");
    Require(std::fabs(AverageRadius(rear_left)   - rear_left.wheel.radius)   < 0.05f, "rear-left tire retains radius");
    Require(std::fabs(AverageRadius(rear_right)  - rear_right.wheel.radius)  < 0.05f, "rear-right tire retains radius");
    Require(Finite(final_com), "four-wheel final center of mass remains finite");

    std::cout << "RoR four-wheel RWD probe passed: " << nodes.size() << " nodes, "
              << beams.size() << " beams, " << straight_driven_distance
              << " m straight-line drive before steering, steering "
              << (left_steer * 180.0f / PI) << "/" << (right_steer * 180.0f / PI)
              << " deg.\n";
    return EXIT_SUCCESS;
}
