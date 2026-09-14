/*
    This source file is part of Rigs of Rods.

    Integrated iOS bring-up runtime: deformable node/beam chassis, four
    generated soft wheels, locked rear differential, RoR wheel torque/braking,
    flat-ground contact and a lightweight front-cornering control layer.

    This is deliberately a portable bring-up actor, not a replacement for the
    full desktop Actor/ActorSpawner path. It lets the iPhone app exercise the
    already-ported RoR physics pieces together while native rig spawning is
    expanded.
*/

#pragma once

#include "BeamPhysics.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace RoR {
namespace IOSVehicleCore {

struct DriftDemoTelemetry
{
    PhysicsVec3 center;
    float heading_radians = 0.0f;
    float speed_mps = 0.0f;
    float forward_speed_mps = 0.0f;
    float rear_wheel_speed_mps = 0.0f;
    float steering = 0.0f;
    float throttle = 0.0f;
    bool handbrake = false;
    std::uint64_t physics_steps = 0;
};

class DriftDemoRuntime
{
public:
    DriftDemoRuntime();
    ~DriftDemoRuntime();

    DriftDemoRuntime(const DriftDemoRuntime&) = delete;
    DriftDemoRuntime& operator=(const DriftDemoRuntime&) = delete;

    void Reset();
    void SetControls(float steering, float throttle, float brake, bool handbrake);
    void Step(float dt);

    std::size_t NodeCount() const;
    const NodeCoreState& Node(std::size_t index) const;
    const std::vector<std::pair<std::size_t, std::size_t>>& BeamPairs() const;
    const std::vector<std::size_t>& TireNodeIndices() const;
    DriftDemoTelemetry Telemetry() const;
    bool IsFinite() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace IOSVehicleCore
} // namespace RoR
