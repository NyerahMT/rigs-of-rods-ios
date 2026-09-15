#include "AuthoredVehicleRuntime.h"
#include "PortableRigDef.h"
#include "SimConstants.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace RoR;
using namespace RoR::IOSVehicleCore;

#ifndef ROR_DAF_FIXTURE_PATH
#error ROR_DAF_FIXTURE_PATH must point to the pinned authored DAF truck file
#endif

namespace
{
void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

std::string ReadFile(const char* path)
{
    std::ifstream input(path, std::ios::binary);
    Require(static_cast<bool>(input), "pinned DAF authored vehicle fixture opens");
    std::ostringstream output;
    output << input.rdbuf();
    return output.str();
}

float HorizontalDistance(const PhysicsVec3& a, const PhysicsVec3& b)
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dz * dz);
}

float WrappedAngleDelta(float after, float before)
{
    float delta = after - before;
    while (delta > 3.14159265f) delta -= 6.28318531f;
    while (delta < -3.14159265f) delta += 6.28318531f;
    return delta;
}
}

int main()
{
    static_assert(PHYSICS_DT == 0.0005f, "authored vehicle runtime requires RoR's 2 kHz timestep");

    const std::string truck_text = ReadFile(ROR_DAF_FIXTURE_PATH);
    const PortableRigDef::Document parsed = PortableRigDef::Parse(truck_text);

    Require(parsed.name == "Daf Semi truck", "real authored vehicle title survives parsing");
    Require(parsed.globals.present, "globals section is parsed");
    Require(std::fabs(parsed.globals.dry_mass - 10000.0f) < 0.1f, "authored dry mass is retained");
    Require(parsed.nodes.size() == 79, "all 79 authored DAF nodes are parsed");
    Require(parsed.beams.size() > 200, "hundreds of authored DAF chassis beams are parsed");
    Require(parsed.shocks.size() == 8, "all authored suspension shocks are parsed");
    Require(parsed.hydros.size() == 4, "all authored steering hydros are parsed");
    Require(parsed.wheels.size() == 4, "all authored wheel definitions are parsed");
    Require(parsed.contacters.size() == 15, "authored ground contact nodes are parsed");
    Require(parsed.engine.present && std::fabs(parsed.engine.torque - 8000.0f) < 0.1f,
            "authored engine definition is retained");

    AuthoredVehicleRuntime truck(truck_text);
    if (!truck.Ready())
    {
        for (const std::string& error : truck.Errors())
            std::cerr << "runtime error: " << error << '\n';
    }
    Require(truck.Ready(), "real authored DAF builds into portable runtime");
    Require(truck.NodeCount() == 175, "79 authored nodes plus 96 generated wheel nodes are live");
    Require(truck.TireNodeIndices().size() == 96, "four 12-ray authored wheels generate 96 tire nodes");
    Require(truck.BeamPairs().size() > 600, "authored chassis, shocks, hydros and generated wheels form one large beam network");

    truck.SetControls(0.0f, 0.0f, 0.0f, false);
    for (int i = 0; i < 3000; ++i)
        truck.Step(PHYSICS_DT);
    Require(truck.IsFinite(), "real authored vehicle settles onto road without divergence");

    const AuthoredVehicleTelemetry settled = truck.Telemetry();
    const PhysicsVec3 settled_front_axis = truck.Node(36).position - truck.Node(6).position;

    truck.SetControls(0.0f, 0.60f, 0.0f, false);
    for (int i = 0; i < 5000; ++i)
        truck.Step(PHYSICS_DT);
    Require(truck.IsFinite(), "real authored vehicle remains finite under powered rear axle");

    const AuthoredVehicleTelemetry powered = truck.Telemetry();
    const float powered_travel = HorizontalDistance(powered.center, settled.center);
    std::cerr << "authored launch diagnostics: travel=" << powered_travel
              << " m, body=" << powered.speed_mps
              << " m/s, forward=" << powered.forward_speed_mps
              << " m/s, driven tread=" << powered.driven_wheel_speed_mps
              << " m/s\n";

    // This is a behavior/integration probe, not an acceleration benchmark.  RoR's
    // DEFAULT_MINIMASS materially changes acceleration versus the old portable
    // runtime's 0.25 kg node floor, so require clear powered motion without
    // encoding the old non-parity mass model into CI.
    Require(powered_travel > 0.01f,
            "authored powered wheels propel the complete truck");
    Require(powered.driven_wheel_speed_mps > 0.01f,
            "authored rear wheels rotate under engine torque");
    Require(std::fabs(powered.forward_speed_mps) > 0.01f,
            "powered truck develops longitudinal motion");

    const float heading_before = powered.heading_radians;
    truck.SetControls(0.72f, 0.30f, 0.0f, false);
    for (int i = 0; i < 4200; ++i)
        truck.Step(PHYSICS_DT);
    Require(truck.IsFinite(), "real authored vehicle remains finite while steering through hydros");

    const AuthoredVehicleTelemetry steering = truck.Telemetry();
    const PhysicsVec3 steered_front_axis = truck.Node(36).position - truck.Node(6).position;
    const float hydro_motion = HorizontalDistance(steered_front_axis, settled_front_axis);
    const float heading_delta = std::fabs(WrappedAngleDelta(steering.heading_radians, heading_before));
    Require(std::fabs(steering.steering) > 0.25f, "steering command advances native hydro state");
    Require(hydro_motion > 0.01f, "authored steering hydro moves the front axle structure");
    Require(heading_delta > 0.002f, "authored steering/road forces yaw the vehicle");

    const float tread_before_brake = steering.driven_wheel_speed_mps;
    truck.SetControls(-0.20f, 0.0f, 1.0f, true);
    for (int i = 0; i < 3200; ++i)
        truck.Step(PHYSICS_DT);
    Require(truck.IsFinite(), "real authored vehicle remains finite through service + parking braking");

    const AuthoredVehicleTelemetry stopped = truck.Telemetry();
    Require(stopped.driven_wheel_speed_mps < tread_before_brake * 0.80f,
            "authored brakes materially reduce driven-wheel tread speed");

    std::cout << "RoR authored DAF runtime passed: "
              << truck.NodeCount() << " nodes, " << truck.BeamPairs().size() << " beams, "
              << powered_travel << " m powered travel, "
              << heading_delta * 180.0f / 3.14159265f << " deg yaw, rear tread "
              << tread_before_brake << " -> " << stopped.driven_wheel_speed_mps << " m/s.\n";
    return EXIT_SUCCESS;
}
