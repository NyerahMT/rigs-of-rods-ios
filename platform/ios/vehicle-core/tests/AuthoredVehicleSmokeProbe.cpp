#include "AuthoredVehicleRuntime.h"
#include "SimConstants.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

using namespace RoR::IOSVehicleCore;

namespace {

std::string Read(const char* path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) return {};
    std::ostringstream stream;
    stream << file.rdbuf();
    return stream.str();
}

float Dist(const RoR::PhysicsVec3& a, const RoR::PhysicsVec3& b)
{
    const float x = a.x - b.x;
    const float z = a.z - b.z;
    return std::sqrt(x * x + z * z);
}

void PrintMassSummary(const AuthoredVehicleRuntime& vehicle)
{
    float total = 0.0f;
    float minimum = std::numeric_limits<float>::max();
    float maximum = 0.0f;
    std::size_t minimum_index = 0;
    std::size_t maximum_index = 0;
    for (std::size_t i = 0; i < vehicle.NodeCount(); ++i)
    {
        const auto& node = vehicle.Node(i);
        total += node.mass;
        if (node.mass < minimum)
        {
            minimum = node.mass;
            minimum_index = i;
        }
        if (node.mass > maximum)
        {
            maximum = node.mass;
            maximum_index = i;
        }
    }
    std::cerr << "mass_total=" << total
              << " min=" << minimum << "@" << minimum_index
              << " max=" << maximum << "@" << maximum_index << '\n';
}

void PrintWorstNode(const AuthoredVehicleRuntime& vehicle, const char* phase, int step)
{
    float worst_speed_sq = -1.0f;
    std::size_t worst_index = 0;
    for (std::size_t i = 0; i < vehicle.NodeCount(); ++i)
    {
        const auto& node = vehicle.Node(i);
        const float speed_sq = node.velocity.squaredLength();
        if (!std::isfinite(node.position.x) || !std::isfinite(node.position.y) || !std::isfinite(node.position.z) ||
            !std::isfinite(node.velocity.x) || !std::isfinite(node.velocity.y) || !std::isfinite(node.velocity.z))
        {
            worst_index = i;
            worst_speed_sq = std::numeric_limits<float>::infinity();
            break;
        }
        if (speed_sq > worst_speed_sq)
        {
            worst_speed_sq = speed_sq;
            worst_index = i;
        }
    }

    const auto& node = vehicle.Node(worst_index);
    std::cerr << "unstable phase=" << phase << " step=" << step
              << " node=" << worst_index
              << " mass=" << node.mass
              << " pos=(" << node.position.x << ',' << node.position.y << ',' << node.position.z << ')'
              << " vel=(" << node.velocity.x << ',' << node.velocity.y << ',' << node.velocity.z << ')'
              << " speed=" << std::sqrt(worst_speed_sq) << '\n';
}

struct TireContactStats
{
    std::size_t samples = 0;
    std::size_t min_contacts = std::numeric_limits<std::size_t>::max();
    std::size_t max_contacts = 0;
    std::size_t zero_contact_steps = 0;
    double contact_sum = 0.0;
    double vertical_speed_sq_sum = 0.0;
    std::size_t vertical_speed_samples = 0;
    float max_abs_vertical_speed = 0.0f;

    void Sample(const AuthoredVehicleRuntime& vehicle)
    {
        const auto& tyres = vehicle.TireNodeIndices();
        std::size_t contacts = 0;
        for (std::size_t index : tyres)
        {
            if (index >= vehicle.NodeCount()) continue;
            const auto& node = vehicle.Node(index);
            // The current portable terrain contact is the Y=0 plane. A node at
            // or below it is part of the instantaneous tyre contact patch.
            if (node.position.y <= 0.0f) ++contacts;
            const float ay = std::fabs(node.velocity.y);
            max_abs_vertical_speed = std::max(max_abs_vertical_speed, ay);
            vertical_speed_sq_sum += static_cast<double>(node.velocity.y) * node.velocity.y;
            ++vertical_speed_samples;
        }
        min_contacts = std::min(min_contacts, contacts);
        max_contacts = std::max(max_contacts, contacts);
        if (contacts == 0) ++zero_contact_steps;
        contact_sum += static_cast<double>(contacts);
        ++samples;
    }

    void Print(const char* phase) const
    {
        const double average_contacts = samples ? contact_sum / static_cast<double>(samples) : 0.0;
        const double rms_vy = vertical_speed_samples
            ? std::sqrt(vertical_speed_sq_sum / static_cast<double>(vertical_speed_samples))
            : 0.0;
        std::cerr << "tire_contact phase=" << phase
                  << " avg=" << average_contacts
                  << " min=" << (samples ? min_contacts : 0)
                  << " max=" << max_contacts
                  << " zero_steps=" << zero_contact_steps << '/' << samples
                  << " rms_vy=" << rms_vy
                  << " max_abs_vy=" << max_abs_vertical_speed << '\n';
    }
};

bool RunPhase(AuthoredVehicleRuntime& vehicle, const char* phase, int steps, TireContactStats* stats = nullptr)
{
    for (int i = 0; i < steps; ++i)
    {
        vehicle.Step(PHYSICS_DT);
        if (stats) stats->Sample(vehicle);
        if (!vehicle.IsFinite())
        {
            PrintWorstNode(vehicle, phase, i + 1);
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: ror_authored_vehicle_smoke <vehicle.truck>\n";
        return 64;
    }

    const std::string text = Read(argv[1]);
    if (text.empty())
    {
        std::cerr << "FAIL: cannot read vehicle\n";
        return 1;
    }

    AuthoredVehicleRuntime vehicle(text);
    std::cerr << "vehicle=" << vehicle.VehicleName()
              << " nodes=" << vehicle.NodeCount()
              << " tyres=" << vehicle.TireNodeIndices().size()
              << " beams=" << vehicle.BeamPairs().size() << '\n';
    for (const auto& warning : vehicle.Warnings()) std::cerr << "warning: " << warning << '\n';
    for (const auto& error : vehicle.Errors()) std::cerr << "error: " << error << '\n';

    if (!vehicle.Ready())
    {
        std::cerr << "FAIL: runtime did not build\n";
        return 2;
    }

    PrintMassSummary(vehicle);

    TireContactStats settle_stats;
    vehicle.SetControls(0.0f, 0.0f, 0.0f, false);
    if (!RunPhase(vehicle, "settle", 3000, &settle_stats))
    {
        std::cerr << "FAIL: vehicle unstable while settling\n";
        return 3;
    }
    settle_stats.Print("settle");
    const auto settled = vehicle.Telemetry();

    TireContactStats power_stats;
    vehicle.SetControls(0.0f, 0.60f, 0.0f, false);
    if (!RunPhase(vehicle, "power", 5000, &power_stats))
    {
        std::cerr << "FAIL: vehicle unstable under power\n";
        return 4;
    }
    power_stats.Print("power");
    const auto powered = vehicle.Telemetry();

    const float travel = Dist(settled.center, powered.center);
    std::cerr << "travel=" << travel
              << " speed=" << powered.speed_mps
              << " forward=" << powered.forward_speed_mps
              << " tread=" << powered.driven_wheel_speed_mps << '\n';

    if (!(travel > .05f && powered.driven_wheel_speed_mps > .10f))
    {
        std::cerr << "FAIL: reference vehicle does not drive\n";
        return 5;
    }

    std::cout << "authored reference vehicle smoke probe passed\n";
    return 0;
}
