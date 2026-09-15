/*
    This source file is part of Rigs of Rods.

    Renderer-free runtime for authored RoR truck-format vehicles on iOS.
    The runtime consumes PortableRigDef rather than embedding a demo chassis,
    so complete authored rigs can drive the same portable physics core.
*/
#pragma once
#include "BeamPhysics.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>
namespace RoR { namespace IOSVehicleCore {
struct AuthoredVehicleTelemetry { PhysicsVec3 center; float heading_radians=0, speed_mps=0, forward_speed_mps=0, driven_wheel_speed_mps=0, engine_rpm=0; int gear=0; float steering=0, throttle=0; bool handbrake=false; std::uint64_t physics_steps=0; };
struct EarlyStepDiagnostics { std::uint64_t step=0; std::size_t worst_node=static_cast<std::size_t>(-1); PhysicsVec3 start_position,position,velocity,force; float mass=0,displacement=0; std::size_t worst_beam=static_cast<std::size_t>(-1),beam_a=static_cast<std::size_t>(-1),beam_b=static_cast<std::size_t>(-1); float beam_stress=0,beam_length=0,beam_rest_length=0,beam_spring=0,beam_damping=0; };
struct SpawnGroundDiagnostics { float min_node_y=0,max_node_y=0,min_tire_y=0,max_penetration=0; std::size_t min_node=static_cast<std::size_t>(-1),min_tire=static_cast<std::size_t>(-1); bool node27_is_tire=false,node27_is_contacter=false; PhysicsVec3 node27_position; PhysicsVec3 node27_ground_force_step1; };
class AuthoredVehicleRuntime {
public:
 explicit AuthoredVehicleRuntime(const std::string& truck_text); ~AuthoredVehicleRuntime(); AuthoredVehicleRuntime(const AuthoredVehicleRuntime&)=delete; AuthoredVehicleRuntime& operator=(const AuthoredVehicleRuntime&)=delete;
 void Reset(); void SetControls(float steering,float throttle,float brake,bool handbrake); void Step(float dt);
 bool Ready() const; bool IsFinite() const; const std::string& VehicleName() const; const std::vector<std::string>& Errors() const; const std::vector<std::string>& Warnings() const;
 std::size_t NodeCount() const; const NodeCoreState& Node(std::size_t index) const; const std::vector<std::pair<std::size_t,std::size_t>>& BeamPairs() const; const std::vector<std::size_t>& TireNodeIndices() const; AuthoredVehicleTelemetry Telemetry() const; const std::vector<EarlyStepDiagnostics>& EarlyDiagnostics() const; const SpawnGroundDiagnostics& SpawnDiagnostics() const;
private: struct Impl; std::unique_ptr<Impl> m_impl;
};
} }
