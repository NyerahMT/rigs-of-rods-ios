/*
    This source file is part of Rigs of Rods.

    Portable node/beam helpers extracted from the core equations used by
    Actor::CalcBeams() and Actor::CalcNodes().
*/

#pragma once

namespace RoR {

struct PhysicsVec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    PhysicsVec3() = default;
    PhysicsVec3(float px, float py, float pz): x(px), y(py), z(pz) {}

    PhysicsVec3 operator+(const PhysicsVec3& other) const { return PhysicsVec3(x + other.x, y + other.y, z + other.z); }
    PhysicsVec3 operator-(const PhysicsVec3& other) const { return PhysicsVec3(x - other.x, y - other.y, z - other.z); }
    PhysicsVec3 operator-() const { return PhysicsVec3(-x, -y, -z); }
    PhysicsVec3 operator*(float scalar) const { return PhysicsVec3(x * scalar, y * scalar, z * scalar); }

    PhysicsVec3& operator+=(const PhysicsVec3& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    PhysicsVec3& operator-=(const PhysicsVec3& other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    float dot(const PhysicsVec3& other) const { return x * other.x + y * other.y + z * other.z; }
    PhysicsVec3 cross(const PhysicsVec3& other) const
    {
        return PhysicsVec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x);
    }
    float squaredLength() const { return dot(*this); }
};

struct NodeCoreState
{
    PhysicsVec3 position;
    PhysicsVec3 velocity;
    PhysicsVec3 force;
    float mass = 1.0f;
    // Upstream node_t carries set_node_defaults friction per node; collision
    // moderation multiplies ground strength by this coefficient.
    float friction_coef = 1.0f;
    bool immovable = false;
};

enum class ShockModel
{
    None,
    Shock1,
    Shock2,
    Shock3
};

struct BeamCoreState
{
    float rest_length = 1.0f;
    float spring = 0.0f;
    float damping = 0.0f;
    float stress = 0.0f;

    // RoR's generated wheel spokes and classic authored `shocks` are SHOCK1
    // bounded beams. The advanced models below mirror shock_t fields consumed
    // by Actor::CalcShocks2()/CalcShocks3().
    bool bounded = false;
    ShockModel shock_model = ShockModel::None;
    bool soft_bump = false;
    float shortbound = 0.0f;
    float longbound = 0.0f;
    float bump_spring = 0.0f;   // shock_t::sbd_spring
    float bump_damping = 0.0f;  // shock_t::sbd_damp

    // SHOCK2 + SHOCK3 asymmetric base spring/damper values.
    float spring_in = 0.0f;
    float damp_in = 0.0f;
    float spring_out = 0.0f;
    float damp_out = 0.0f;

    // SHOCK2 progressive terms.
    float progress_spring_in = 0.0f;
    float progress_damp_in = 0.0f;
    float progress_spring_out = 0.0f;
    float progress_damp_out = 0.0f;

    // SHOCK3 digressive/velocity-split damping terms. These are multipliers of
    // damp_in/damp_out exactly like shock_t::dslow*/dfast* upstream.
    float split_vel_in = 0.0f;
    float damp_in_slow = 0.0f;
    float damp_in_fast = 0.0f;
    float split_vel_out = 0.0f;
    float damp_out_slow = 0.0f;
    float damp_out_fast = 0.0f;
};

struct ShockCoefficients
{
    float spring;
    float damping;

    ShockCoefficients(): spring(0.0f), damping(0.0f) {}
    ShockCoefficients(float spring_value, float damping_value):
        spring(spring_value), damping(damping_value) {}
};

// Exact scalar spring/damper law used in Actor::CalcBeams().
float CalcBeamStress(float length_error, float relative_speed, float spring, float damping);

// Portable equivalents of Actor::CalcShocks2()/CalcShocks3(). Exposed so the
// conformance probes can compare the coefficient law directly against desktop.
ShockCoefficients CalcShock2Coefficients(const BeamCoreState& beam, float length_error, float relative_speed);
ShockCoefficients CalcShock3Coefficients(const BeamCoreState& beam, float length_error, float relative_speed);

// Applies equal/opposite force to the two nodes using RoR's beam direction math,
// including SHOCK1/SHOCK2/SHOCK3 suspension semantics.
void ApplyBeamForce(NodeCoreState& node1, NodeCoreState& node2, BeamCoreState& beam);

// Base semi-implicit Euler integration from Actor::CalcNodes().
// The force accumulator is reset to gravity for the next 0.5 ms physics step.
void IntegrateNode(NodeCoreState& node, float gravity, float dt);

} // namespace RoR
