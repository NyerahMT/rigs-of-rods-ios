/*
    Deterministic portable-core trace used to compare iOS physics against an
    upstream Rigs of Rods oracle run.
*/
#include "AuthoredVehicleRuntime.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using RoR::IOSVehicleCore::AuthoredVehicleRuntime;
using RoR::IOSVehicleCore::AuthoredVehicleTelemetry;
using RoR::NodeCoreState;
using RoR::PhysicsVec3;

namespace {
std::string ReadFile(const char* path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error(std::string("cannot open vehicle: ") + path);
    std::ostringstream out;
    out << input.rdbuf();
    return out.str();
}

void ControlsForStep(std::uint64_t step, float& steering, float& throttle, float& brake, bool& handbrake)
{
    // One deterministic scenario deliberately exercises straight-line launch,
    // coast, braking, steering and combined throttle/steering. With RoR's
    // 0.0005 s fixed step these boundaries are exact integer steps.
    steering = 0.0f;
    throttle = 0.0f;
    brake = 0.0f;
    handbrake = false;

    if (step >= 4000 && step < 24000)               // 2-12 s
        throttle = 1.0f;
    else if (step >= 34000 && step < 44000)         // 17-22 s
        brake = 0.45f;
    else if (step >= 44000 && step < 54000)         // 22-27 s
        steering = 0.35f;
    else if (step >= 54000 && step < 64000)         // 27-32 s
    {
        steering = -0.25f;
        throttle = 0.55f;
    }
}

void WriteVec(std::ostream& out, const PhysicsVec3& v)
{
    out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
}

void WriteSample(std::ostream& out, const AuthoredVehicleRuntime& runtime)
{
    const AuthoredVehicleTelemetry t = runtime.Telemetry();
    out << "{\"step\":" << t.physics_steps
        << ",\"telemetry\":{";
    out << "\"center\":"; WriteVec(out, t.center);
    out << ",\"heading\":" << t.heading_radians
        << ",\"speed\":" << t.speed_mps
        << ",\"forward_speed\":" << t.forward_speed_mps
        << ",\"driven_wheel_speed\":" << t.driven_wheel_speed_mps
        << ",\"engine_rpm\":" << t.engine_rpm
        << ",\"gear\":" << t.gear
        << ",\"steering\":" << t.steering
        << ",\"throttle\":" << t.throttle
        << "},\"nodes\":[";

    for (std::size_t i = 0; i < runtime.NodeCount(); ++i)
    {
        if (i) out << ',';
        const NodeCoreState& n = runtime.Node(i);
        out << "{\"i\":" << i << ",\"p\":"; WriteVec(out, n.position);
        out << ",\"v\":"; WriteVec(out, n.velocity);
        out << ",\"f\":"; WriteVec(out, n.force);
        out << ",\"m\":" << n.mass
            << ",\"mu\":" << n.friction_coef << '}';
    }
    out << "]}\n";
}
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: ror_physics_parity_trace <vehicle.truck> [steps=64000] [stride=100] [output.jsonl]\n";
        return 2;
    }

    try
    {
        const std::uint64_t steps = argc > 2 ? static_cast<std::uint64_t>(std::strtoull(argv[2], nullptr, 10)) : 64000ULL;
        const std::uint64_t stride = std::max<std::uint64_t>(1ULL, argc > 3 ? static_cast<std::uint64_t>(std::strtoull(argv[3], nullptr, 10)) : 100ULL);
        std::ofstream file;
        std::ostream* out = &std::cout;
        if (argc > 4)
        {
            file.open(argv[4], std::ios::binary | std::ios::trunc);
            if (!file) throw std::runtime_error("cannot open trace output");
            out = &file;
        }
        *out << std::setprecision(9);

        AuthoredVehicleRuntime runtime(ReadFile(argv[1]));
        if (!runtime.Ready())
        {
            for (const std::string& e : runtime.Errors()) std::cerr << e << '\n';
            return 3;
        }

        // Emit spawn state as step zero, then run the exact upstream fixed dt.
        WriteSample(*out, runtime);
        constexpr float kPhysicsDt = 0.0005f;
        for (std::uint64_t step = 0; step < steps; ++step)
        {
            float steering, throttle, brake;
            bool handbrake;
            ControlsForStep(step, steering, throttle, brake, handbrake);
            runtime.SetControls(steering, throttle, brake, handbrake);
            runtime.Step(kPhysicsDt);
            if (!runtime.IsFinite())
            {
                std::cerr << "non-finite physics at step " << (step + 1) << '\n';
                return 4;
            }
            if (((step + 1) % stride) == 0 || step + 1 == steps)
                WriteSample(*out, runtime);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 5;
    }
    return 0;
}
