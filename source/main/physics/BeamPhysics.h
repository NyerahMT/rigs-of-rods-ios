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

struct BeamCoreState
{
    float rest_length = 1.0f;
    float spring = 0.0f;
    float damping = 0.0f;
    float stress = 0.0f;

    // RoR's generated wheel spokes use SpecialBeam::SHOCK1 bounds even though
    // they are wheel beams rather than authored shocks. Keep the same state in
    // the portable core so meshwheels/meshwheels2 do not become unconstrained
    // generic springs on iOS.
    bool bounded = false;
    float shortbound = 0.0f;
    float longbound = 0.0f;
};

// Exact scalar spring/damper law used in Actor::CalcBeams().
float CalcBeamStress(float length_error, float relative_speed, float spring, float damping);

// Applies equal/opposite force to the two nodes using RoR's beam direction math,
// including the SHOCK1-style hard-bump interpolation used by generated wheel beams.
void ApplyBeamForce(NodeCoreState& node1, NodeCoreState& node2, BeamCoreState& beam);

// Base semi-implicit Euler integration from Actor::CalcNodes().
// The force accumulator is reset to gravity for the next 0.5 ms physics step.
void IntegrateNode(NodeCoreState& node, float gravity, float dt);

} // namespace RoR
