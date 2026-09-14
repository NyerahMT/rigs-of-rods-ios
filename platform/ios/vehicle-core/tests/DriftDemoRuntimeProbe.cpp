#include "DriftDemoRuntime.h"
#include "SimConstants.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace RoR;
using namespace RoR::IOSVehicleCore;

namespace {
void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}
}

int main()
{
    static_assert(PHYSICS_DT == 0.0005f, "demo runtime requires RoR's 2 kHz timestep");

    DriftDemoRuntime car;
    Require(car.NodeCount() > 60, "integrated car includes generated deformable tire nodes");
    Require(car.BeamPairs().size() > 180, "integrated car includes a substantial beam structure");
    Require(car.TireNodeIndices().size() >= 64, "four soft tires are generated");

    // Settle the sprung structure onto the road.
    car.SetControls(0.0f, 0.0f, 0.0f, false);
    for (int i = 0; i < 2400; ++i)
        car.Step(PHYSICS_DT);
    Require(car.IsFinite(), "car remains finite while settling");

    const DriftDemoTelemetry start = car.Telemetry();

    // Progressive RWD launch.
    car.SetControls(0.0f, 0.72f, 0.0f, false);
    for (int i = 0; i < 5000; ++i)
        car.Step(PHYSICS_DT);
    Require(car.IsFinite(), "car remains finite under rear-wheel drive torque");

    const DriftDemoTelemetry powered = car.Telemetry();
    const float dx = powered.center.x - start.center.x;
    const float dz = powered.center.z - start.center.z;
    const float travel = std::sqrt(dx * dx + dz * dz);
    Require(travel > 0.60f, "RWD torque moves the complete soft-body car");
    Require(powered.rear_wheel_speed_mps > 1.0f, "rear wheels spin under power");

    // Command steering while maintaining power; the complete chassis must yaw.
    const float heading_before = powered.heading_radians;
    car.SetControls(0.72f, 0.48f, 0.0f, false);
    for (int i = 0; i < 3600; ++i)
        car.Step(PHYSICS_DT);
    Require(car.IsFinite(), "car remains finite while cornering");

    const DriftDemoTelemetry cornering = car.Telemetry();
    float heading_delta = cornering.heading_radians - heading_before;
    while (heading_delta > 3.14159265f) heading_delta -= 6.28318531f;
    while (heading_delta < -3.14159265f) heading_delta += 6.28318531f;
    Require(std::fabs(heading_delta) > 0.025f, "steering input yaws the soft-body chassis");

    // Handbrake must materially reduce rear tread speed.
    const float speed_before_handbrake = cornering.rear_wheel_speed_mps;
    car.SetControls(-0.35f, 0.0f, 0.0f, true);
    for (int i = 0; i < 2600; ++i)
        car.Step(PHYSICS_DT);
    Require(car.IsFinite(), "car remains finite during handbrake phase");

    const DriftDemoTelemetry stopped = car.Telemetry();
    Require(stopped.rear_wheel_speed_mps < speed_before_handbrake * 0.70f,
            "handbrake materially slows rear tire rotation");

    std::cout << "RoR integrated RWD demo passed: "
              << car.NodeCount() << " nodes, " << car.BeamPairs().size() << " beams, "
              << travel << " m powered travel, heading delta "
              << (heading_delta * 180.0f / 3.14159265f) << " deg, rear tread "
              << speed_before_handbrake << " -> " << stopped.rear_wheel_speed_mps << " m/s.\n";
    return EXIT_SUCCESS;
}
