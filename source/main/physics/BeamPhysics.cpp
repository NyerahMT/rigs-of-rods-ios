/*
    This source file is part of Rigs of Rods.

    Portable node/beam helpers extracted from the core equations used by
    Actor::CalcBeams() and Actor::CalcNodes().
*/

#include "BeamPhysics.h"
#include "ApproxMath.h"
#include "SimConstants.h"

#include <cmath>

namespace RoR {

float CalcBeamStress(float length_error, float relative_speed, float spring, float damping)
{
    return -spring * length_error - damping * relative_speed;
}

void ApplyBeamForce(NodeCoreState& node1, NodeCoreState& node2, BeamCoreState& beam)
{
    const PhysicsVec3 displacement = node1.position - node2.position;
    const float squared_length = displacement.squaredLength();

    // RoR truck beams have a nonzero minimum length. Keep a tiny guard here so
    // malformed portable test data cannot feed zero into fast_invSqrt().
    if (squared_length <= 1.0e-12f)
    {
        beam.stress = 0.0f;
        return;
    }

    const float inverse_length = fast_invSqrt(squared_length);
    const float length = squared_length * inverse_length;
    const float length_error = length - beam.rest_length;
    const float relative_speed = (node1.velocity - node2.velocity).dot(displacement) * inverse_length;

    float spring = beam.spring;
    float damping = beam.damping;

    // Actor::CalcBeams() SHOCK1 handling. Wheel spokes use the global normal
    // beam constants at the limiter. Authored `shocks`/`shocks2` instead use
    // the BeamDefaults captured when that shock was spawned (shock_t::sbd_*).
    if (beam.bounded)
    {
        float interp_ratio = 0.0f;
        if (length_error > beam.longbound * beam.rest_length)
            interp_ratio = length_error - beam.longbound * beam.rest_length;
        else if (length_error < -beam.shortbound * beam.rest_length)
            interp_ratio = -length_error - beam.shortbound * beam.rest_length;

        if (interp_ratio != 0.0f)
        {
            const float target_spring = beam.bump_spring > 0.0f ? beam.bump_spring : DEFAULT_SPRING;
            const float target_damping = beam.bump_damping > 0.0f ? beam.bump_damping : DEFAULT_DAMP;
            spring += (target_spring - spring) * interp_ratio;
            damping += (target_damping - damping) * interp_ratio;
        }
    }

    beam.stress = CalcBeamStress(length_error, relative_speed, spring, damping);

    const PhysicsVec3 force = displacement * (beam.stress * inverse_length);
    node1.force += force;
    node2.force -= force;
}

void IntegrateNode(NodeCoreState& node, float gravity, float dt)
{
    if (!node.immovable && node.mass > 0.0f)
    {
        node.velocity += node.force * (dt / node.mass);
        node.position += node.velocity * dt;
    }

    node.force = PhysicsVec3(0.0f, node.mass * gravity, 0.0f);
}

} // namespace RoR
