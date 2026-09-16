#include "RoRDrivetrain.h"
#include "SimConstants.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

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

bool Near(float a, float b, float epsilon = .01f)
{
    return std::fabs(a - b) <= epsilon;
}
}

int main()
{
    RoRDrivetrainConfig c;
    c.shift_down_rpm = 1000.0f;
    c.shift_up_rpm = 1500.0f;
    c.engine_torque = 8000.0f;
    c.differential_ratio = 2.0f;
    c.reverse_gear_ratio = 10.85f;
    c.neutral_gear_ratio = 13.86f;
    const float g[] = {9.52f,6.56f,5.48f,4.58f,3.83f,3.02f,2.53f,2.08f,1.74f,1.43f,1.20f,1.00f};
    c.forward_gears.assign(g, g + 12);

    c.engine_inertia = 10.0f;
    c.engine_type = 'c';
    c.clutch_force = -1.0f; // upstream car default => 5000
    c.shift_time = 0.60f;
    c.clutch_time = 0.20f;
    c.post_shift_time = 0.30f;
    c.idle_rpm = 925.0f;
    c.stall_rpm = 550.0f;
    c.max_idle_mixture = 0.14f;
    c.min_idle_mixture = 0.03f;
    c.engine_braking_torque = 175.0f;

    RoRDrivetrain d(c);
    Require(d.Gear() == 1, "startEngine automatic state starts in first forward gear");
    Require(Near(d.DriveRatio(), 19.04f), "first gear includes global differential ratio");
    Require(Near(d.Telemetry().engine_rpm, 925.0f), "authored idle RPM reaches engine start state");
    Require(!d.Telemetry().shifting && !d.Telemetry().post_shifting, "engine starts outside shift state");

    // Keep driveshaft just below gearbox speed. This gives the automatic clutch
    // a small positive reaction torque, allowing it to reach full engagement
    // exactly as Engine::UpdateEngine() does once RPM is above m_engine_min_rpm.
    int guard = 0;
    while (!d.Telemetry().shifting && guard++ < 10000)
    {
        const float ratio = std::max(1.0f, std::fabs(d.DriveRatio()));
        const float wheel = std::max(0.0f, d.Telemetry().engine_rpm / ratio - 1.0f);
        d.Step(1.0f, wheel, PHYSICS_DT);
    }

    Require(d.Telemetry().shifting, "automatic redline request enters timed shift state");
    Require(d.Gear() == 1, "gear does not change at shift request instant");
    Require(d.Telemetry().clutch > 0.99f, "automatic clutch reached full engagement before upshift");

    const float clutch_at_request = d.Telemetry().clutch;
    for (int i = 0; i < 100; ++i) // 0.05 s < 0.20 s declutch phase
    {
        const float wheel = d.Telemetry().engine_rpm / std::max(1.0f, std::fabs(d.DriveRatio()));
        d.Step(1.0f, wheel, PHYSICS_DT);
    }
    Require(d.Gear() == 1, "gear remains engaged during declutch phase");
    Require(d.Telemetry().clutch < clutch_at_request, "declutch phase releases clutch progressively");
    Require(d.Telemetry().current_acc < 1.0f, "declutch phase also cuts engine acceleration input");

    guard = 0;
    while (d.Gear() == 1 && guard++ < 2000)
    {
        const float wheel = d.Telemetry().engine_rpm / std::max(1.0f, std::fabs(d.DriveRatio()));
        d.Step(1.0f, wheel, PHYSICS_DT);
    }
    Require(d.Gear() == 2, "second gear engages after authored declutch interval");
    Require(d.Telemetry().shifting, "gear engagement occurs before total shift time expires");

    guard = 0;
    while (d.Telemetry().shifting && guard++ < 3000)
    {
        const float wheel = d.Telemetry().engine_rpm / std::max(1.0f, std::fabs(d.DriveRatio()));
        d.Step(1.0f, wheel, PHYSICS_DT);
    }
    Require(!d.Telemetry().shifting, "shift state ends after authored shift_time");
    Require(d.Telemetry().post_shifting, "post-shift torque ramp begins after shift");

    const float acc_at_post_start = d.Telemetry().current_acc;
    guard = 0;
    while (d.Telemetry().post_shifting && guard++ < 2000)
    {
        const float wheel = d.Telemetry().engine_rpm / std::max(1.0f, std::fabs(d.DriveRatio()));
        d.Step(1.0f, wheel, PHYSICS_DT);
    }
    Require(!d.Telemetry().post_shifting, "post-shift state ends after authored post_shift_time");
    Require(acc_at_post_start <= 0.51f, "post-shift ramp starts near half requested throttle");
    Require(Near(d.Telemetry().current_acc, 1.0f, .02f), "post-shift ramp restores requested throttle");

    Require(std::isfinite(d.Telemetry().engine_rpm) && std::isfinite(d.OutputTorque()),
            "drivetrain remains finite through full timed shift");

    std::cout << "RoR drivetrain parity probe passed: timed 1->2 shift at "
              << d.Telemetry().engine_rpm << " rpm\n";
    return EXIT_SUCCESS;
}
