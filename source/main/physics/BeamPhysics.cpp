/*
    This source file is part of Rigs of Rods.

    Portable node/beam helpers extracted from the core equations used by
    Actor::CalcBeams() and Actor::CalcNodes().
*/

#include "BeamPhysics.h"
#include "ApproxMath.h"
#include "SimConstants.h"

#include <algorithm>
#include <cmath>

namespace RoR {
namespace {

float ProgressiveRatio(float travel, float bound, float length)
{
    if (bound == 0.0f || length <= 0.0f)
        return 1.0f;
    const float ratio = travel / (bound * length);
    return std::min(ratio * ratio, 1.0f);
}

float BumpSpring(const BeamCoreState& beam)
{
    return beam.bump_spring > 0.0f ? beam.bump_spring : DEFAULT_SPRING;
}

float BumpDamping(const BeamCoreState& beam)
{
    return beam.bump_damping > 0.0f ? beam.bump_damping : DEFAULT_DAMP;
}

} // namespace

float CalcBeamStress(float length_error, float relative_speed, float spring, float damping)
{
    return -spring * length_error - damping * relative_speed;
}

ShockCoefficients CalcShock2Coefficients(
    const BeamCoreState& beam,
    float length_error,
    float relative_speed)
{
    ShockCoefficients out;

    // Actor::CalcShocks2(): choose rebound/compression bases from instantaneous
    // beam velocity, then apply the authored squared travel progression.
    if (relative_speed > 0.0f) // Extension / rebound
    {
        out.spring = beam.spring_out;
        out.damping = beam.damp_out;
        const float factor = ProgressiveRatio(length_error, beam.longbound, beam.rest_length);
        out.spring += beam.progress_spring_out * out.spring * factor;
        out.damping += beam.progress_damp_out * out.damping * factor;
    }
    else // Compression (upstream also uses this branch at exactly zero speed)
    {
        out.spring = beam.spring_in;
        out.damping = beam.damp_in;
        const float factor = ProgressiveRatio(length_error, beam.shortbound, beam.rest_length);
        out.spring += beam.progress_spring_in * out.spring * factor;
        out.damping += beam.progress_damp_in * out.damping * factor;
    }

    if (beam.soft_bump)
    {
        // Desktop's soft-bump pre-limit begins at 80% of the normal bound and
        // ramps across the remaining 20% (factor * 5). Preserve the historical
        // use of the *out* progression fields for the extra bump ramp on both
        // ends; it looks odd, but it is the upstream behavior.
        const float prelimit_length = beam.rest_length * 0.8f;
        const float long_prelimit = beam.longbound * prelimit_length;
        const float short_prelimit = -beam.shortbound * prelimit_length;

        if (length_error > long_prelimit)
        {
            out.spring = beam.spring_out;
            out.damping = beam.damp_out;
            float factor = ProgressiveRatio(length_error, beam.longbound, beam.rest_length);
            out.spring += beam.progress_spring_out * out.spring * factor;
            out.damping += beam.progress_damp_out * out.damping * factor;

            factor = 1.0f;
            if (beam.longbound != 0.0f && beam.rest_length > 0.0f)
            {
                const float ramp = ((length_error - long_prelimit) * 5.0f) /
                    (beam.longbound * beam.rest_length);
                factor = std::min(ramp * ramp, 1.0f);
            }
            out.spring += (out.spring + 100.0f) * beam.progress_spring_out * factor;
            out.damping += (out.damping + 100.0f) * beam.progress_damp_out * factor;

            // Oscillating-beam workaround in the desktop solver: if the beam is
            // already moving back inward, restore compression-side base values.
            if (relative_speed < 0.0f)
            {
                out.spring = beam.spring_in;
                out.damping = beam.damp_in;
            }
        }
        else if (length_error < short_prelimit)
        {
            out.spring = beam.spring_in;
            out.damping = beam.damp_in;
            float factor = ProgressiveRatio(length_error, beam.shortbound, beam.rest_length);
            out.spring += beam.progress_spring_in * out.spring * factor;
            out.damping += beam.progress_damp_in * out.damping * factor;

            factor = 1.0f;
            if (beam.shortbound != 0.0f && beam.rest_length > 0.0f)
            {
                const float ramp = ((length_error - short_prelimit) * 5.0f) /
                    (beam.shortbound * beam.rest_length);
                factor = std::min(ramp * ramp, 1.0f);
            }
            out.spring += (out.spring + 100.0f) * beam.progress_spring_out * factor;
            out.damping += (out.damping + 100.0f) * beam.progress_damp_out * factor;

            if (relative_speed > 0.0f)
            {
                out.spring = beam.spring_out;
                out.damping = beam.damp_out;
            }
        }

        if (length_error > beam.longbound * beam.rest_length ||
            length_error < -beam.shortbound * beam.rest_length)
        {
            // Upstream uses max() in soft mode rather than replacing the tuned
            // progressive rate. This retains a stiffer authored progression.
            out.spring = std::max(out.spring, BumpSpring(beam));
            out.damping = std::max(out.damping, BumpDamping(beam));
        }
    }
    else if (length_error > beam.longbound * beam.rest_length ||
             length_error < -beam.shortbound * beam.rest_length)
    {
        // Hard SHOCK2 limiter: switch directly to the raw BeamDefaults stored
        // in shock_t::sbd_spring / sbd_damp.
        out.spring = BumpSpring(beam);
        out.damping = BumpDamping(beam);
    }

    return out;
}

ShockCoefficients CalcShock3Coefficients(
    const BeamCoreState& beam,
    float length_error,
    float relative_speed)
{
    ShockCoefficients out{beam.spring, beam.damping};

    // Actor::CalcShocks3() checks the limiter before the velocity split. Unlike
    // SHOCK2, it interpolates toward the BeamDefaults bump rate by the amount of
    // travel beyond the authored bound.
    if (length_error > beam.longbound * beam.rest_length)
    {
        const float ratio = length_error - beam.longbound * beam.rest_length;
        out.spring += (BumpSpring(beam) - out.spring) * ratio;
        out.damping += (BumpDamping(beam) - out.damping) * ratio;
    }
    else if (length_error < -beam.shortbound * beam.rest_length)
    {
        const float ratio = -length_error - beam.shortbound * beam.rest_length;
        out.spring += (BumpSpring(beam) - out.spring) * ratio;
        out.damping += (BumpDamping(beam) - out.damping) * ratio;
    }
    else if (relative_speed > 0.0f) // Extension
    {
        const float speed = std::max(0.15f, std::min(20.0f, std::fabs(relative_speed)));
        out.spring = beam.spring_out;
        out.damping =
            beam.damp_out * beam.damp_out_slow * std::min(speed, beam.split_vel_out) +
            beam.damp_out * beam.damp_out_fast * std::max(0.0f, speed - beam.split_vel_out);
        out.damping /= speed;
    }
    else if (relative_speed < 0.0f) // Compression
    {
        const float speed = std::max(0.15f, std::min(20.0f, std::fabs(relative_speed)));
        out.spring = beam.spring_in;
        out.damping =
            beam.damp_in * beam.damp_in_slow * std::min(speed, beam.split_vel_in) +
            beam.damp_in * beam.damp_in_fast * std::max(0.0f, speed - beam.split_vel_in);
        out.damping /= speed;
    }

    return out;
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

    if (beam.shock_model == ShockModel::Shock2)
    {
        const ShockCoefficients coeff = CalcShock2Coefficients(beam, length_error, relative_speed);
        spring = coeff.spring;
        damping = coeff.damping;
    }
    else if (beam.shock_model == ShockModel::Shock3)
    {
        const ShockCoefficients coeff = CalcShock3Coefficients(beam, length_error, relative_speed);
        spring = coeff.spring;
        damping = coeff.damping;
    }
    else if (beam.bounded)
    {
        // Actor::CalcBeams() SHOCK1 handling. Wheel spokes use the global normal
        // beam constants at the limiter. Classic authored `shocks` use the raw
        // BeamDefaults captured when the shock was spawned (shock_t::sbd_*).
        float interp_ratio = 0.0f;
        if (length_error > beam.longbound * beam.rest_length)
            interp_ratio = length_error - beam.longbound * beam.rest_length;
        else if (length_error < -beam.shortbound * beam.rest_length)
            interp_ratio = -length_error - beam.shortbound * beam.rest_length;

        if (interp_ratio != 0.0f)
        {
            spring += (BumpSpring(beam) - spring) * interp_ratio;
            damping += (BumpDamping(beam) - damping) * interp_ratio;
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
