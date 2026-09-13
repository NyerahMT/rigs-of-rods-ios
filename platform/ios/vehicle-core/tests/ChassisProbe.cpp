#include "BeamPhysics.h"
#include "GroundContact.h"
#include "SimConstants.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace RoR;

namespace
{
struct BeamLink
{
    int a;
    int b;
    BeamCoreState beam;
};

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

float Distance(const PhysicsVec3& a, const PhysicsVec3& b)
{
    const PhysicsVec3 d = a - b;
    return std::sqrt(d.squaredLength());
}

bool Finite(const PhysicsVec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void AddBeam(std::vector<BeamLink>& beams, const std::array<NodeCoreState, 8>& nodes, int a, int b)
{
    BeamLink link{};
    link.a = a;
    link.b = b;
    link.beam.rest_length = Distance(nodes[a].position, nodes[b].position);
    link.beam.spring = 500000.0f;
    link.beam.damping = 8000.0f;
    beams.push_back(link);
}
}

int main()
{
    std::array<NodeCoreState, 8> nodes{};

    const PhysicsVec3 positions[8] = {
        PhysicsVec3( 1.8f, 0.65f,  0.8f),
        PhysicsVec3( 1.8f, 0.65f, -0.8f),
        PhysicsVec3(-1.8f, 0.65f, -0.8f),
        PhysicsVec3(-1.8f, 0.65f,  0.8f),
        PhysicsVec3( 1.8f, 1.15f,  0.8f),
        PhysicsVec3( 1.8f, 1.15f, -0.8f),
        PhysicsVec3(-1.8f, 1.15f, -0.8f),
        PhysicsVec3(-1.8f, 1.15f,  0.8f)
    };

    for (int i = 0; i < 8; ++i)
    {
        nodes[i].position = positions[i];
        nodes[i].velocity = PhysicsVec3(2.0f, 0.0f, 7.0f);
        nodes[i].mass = 100.0f;
        nodes[i].force = PhysicsVec3(0.0f, nodes[i].mass * DEFAULT_GRAVITY, 0.0f);
    }

    std::vector<BeamLink> beams;
    beams.reserve(28);

    // Lower and upper perimeter.
    AddBeam(beams, nodes, 0, 1); AddBeam(beams, nodes, 1, 2);
    AddBeam(beams, nodes, 2, 3); AddBeam(beams, nodes, 3, 0);
    AddBeam(beams, nodes, 4, 5); AddBeam(beams, nodes, 5, 6);
    AddBeam(beams, nodes, 6, 7); AddBeam(beams, nodes, 7, 4);

    // Vertical structure.
    AddBeam(beams, nodes, 0, 4); AddBeam(beams, nodes, 1, 5);
    AddBeam(beams, nodes, 2, 6); AddBeam(beams, nodes, 3, 7);

    // Face triangulation.
    AddBeam(beams, nodes, 0, 2); AddBeam(beams, nodes, 1, 3);
    AddBeam(beams, nodes, 4, 6); AddBeam(beams, nodes, 5, 7);
    AddBeam(beams, nodes, 0, 5); AddBeam(beams, nodes, 1, 4);
    AddBeam(beams, nodes, 1, 6); AddBeam(beams, nodes, 2, 5);
    AddBeam(beams, nodes, 2, 7); AddBeam(beams, nodes, 3, 6);
    AddBeam(beams, nodes, 3, 4); AddBeam(beams, nodes, 0, 7);

    // Body diagonals resist parallelogram collapse.
    AddBeam(beams, nodes, 0, 6); AddBeam(beams, nodes, 1, 7);
    AddBeam(beams, nodes, 2, 4); AddBeam(beams, nodes, 3, 5);

    const float initial_width = Distance(nodes[0].position, nodes[1].position);
    const float initial_length = Distance(nodes[0].position, nodes[3].position);
    const float initial_lateral_speed = 7.0f;

    GroundContactParams road{};
    road.friction.adhesion_velocity = 0.5f;
    road.friction.static_friction = 1.2f;
    road.friction.sliding_friction = 0.85f;
    road.friction.hydrodynamic_friction = 0.02f;
    road.friction.stribeck_velocity = 0.35f;
    road.friction.stribeck_alpha = 2.0f;
    road.ground_strength = 1.0f;
    road.node_friction = 1.0f;

    int contact_steps = 0;

    for (int step = 0; step < 8000; ++step)
    {
        bool contacted_this_step = false;
        for (NodeCoreState& node : nodes)
        {
            contacted_this_step = ApplyFlatGroundContact(node, 0.0f, road, PHYSICS_DT) || contacted_this_step;
        }
        if (contacted_this_step)
        {
            ++contact_steps;
        }

        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, PHYSICS_DT);
            Require(Finite(node.position) && Finite(node.velocity) && Finite(node.force), "chassis state remains finite");
        }

        for (BeamLink& link : beams)
        {
            ApplyBeamForce(nodes[link.a], nodes[link.b], link.beam);
            Require(std::isfinite(link.beam.stress), "beam stress remains finite");
        }
    }

    float min_y = nodes[0].position.y;
    float average_lateral_speed = 0.0f;
    for (const NodeCoreState& node : nodes)
    {
        min_y = std::min(min_y, node.position.y);
        average_lateral_speed += node.velocity.z;
    }
    average_lateral_speed /= static_cast<float>(nodes.size());

    const float final_width = Distance(nodes[0].position, nodes[1].position);
    const float final_length = Distance(nodes[0].position, nodes[3].position);

    Require(contact_steps > 100, "chassis reaches and remains in road contact");
    Require(min_y > -0.20f, "ground reaction prevents runaway penetration");
    Require(std::fabs(final_width - initial_width) < initial_width * 0.08f, "beam lattice retains chassis width");
    Require(std::fabs(final_length - initial_length) < initial_length * 0.08f, "beam lattice retains chassis length");
    Require(std::fabs(average_lateral_speed) < initial_lateral_speed * 0.65f, "road friction scrubs lateral chassis speed");

    std::cout << "RoR chassis probe passed: " << beams.size() << " beams, "
              << contact_steps << " contact steps, lateral speed "
              << average_lateral_speed << " m/s.\n";
    return EXIT_SUCCESS;
}
