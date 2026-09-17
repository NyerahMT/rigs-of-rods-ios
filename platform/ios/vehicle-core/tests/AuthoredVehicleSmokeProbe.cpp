#include "AuthoredVehicleRuntime.h"
#include "PortableRigDef.h"
#include "SimConstants.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

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

float Length(const RoR::PhysicsVec3& v)
{
    return std::sqrt(v.squaredLength());
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

struct WheelDiagnosticDef
{
    std::size_t axis0 = 0;
    std::size_t axis1 = 0;
    float radius = 0.0f;
    std::vector<std::pair<std::size_t, std::size_t>> tyre_nodes; // tyre node, matching axle node
};

std::vector<WheelDiagnosticDef> BuildWheelDiagnosticDefs(const std::string& text)
{
    const RoR::PortableRigDef::Document rig = RoR::PortableRigDef::Parse(text);
    std::unordered_map<std::string, std::size_t> authored;
    for (std::size_t i = 0; i < rig.nodes.size(); ++i)
        authored[rig.nodes[i].id] = i;

    std::vector<WheelDiagnosticDef> out;
    std::size_t generated = rig.nodes.size();
    for (const RoR::PortableRigDef::Wheel& wheel : rig.wheels)
    {
        const int rays = std::max(0, wheel.num_rays);
        const auto a = authored.find(wheel.axis_node_0);
        const auto b = authored.find(wheel.axis_node_1);
        std::size_t axis0 = a == authored.end() ? static_cast<std::size_t>(-1) : a->second;
        std::size_t axis1 = b == authored.end() ? static_cast<std::size_t>(-1) : b->second;

        // Match the runtime/upstream GetWheelAxisNodes() ordering.
        if (axis0 != static_cast<std::size_t>(-1) && axis1 != static_cast<std::size_t>(-1) &&
            rig.nodes[axis0].z > rig.nodes[axis1].z)
            std::swap(axis0, axis1);

        WheelDiagnosticDef def;
        def.axis0 = axis0;
        def.axis1 = axis1;
        def.radius = wheel.tire_radius;

        if (wheel.wheels2)
        {
            // BuildWheel2 emits rim0,tire0,rim1,tire1 for every ray.
            for (int ray = 0; ray < rays; ++ray)
            {
                const std::size_t base = generated + static_cast<std::size_t>(ray * 4);
                def.tyre_nodes.emplace_back(base + 1, axis0);
                def.tyre_nodes.emplace_back(base + 3, axis1);
            }
            generated += static_cast<std::size_t>(rays * 4);
        }
        else
        {
            // wheels/meshwheels/meshwheels2 emit alternating outer/inner tyre nodes.
            for (int ray = 0; ray < rays; ++ray)
            {
                const std::size_t base = generated + static_cast<std::size_t>(ray * 2);
                def.tyre_nodes.emplace_back(base, axis0);
                def.tyre_nodes.emplace_back(base + 1, axis1);
            }
            generated += static_cast<std::size_t>(rays * 2);
        }

        if (axis0 != static_cast<std::size_t>(-1) && axis1 != static_cast<std::size_t>(-1))
            out.push_back(std::move(def));
    }
    return out;
}

struct TireContactStats
{
    explicit TireContactStats(std::vector<WheelDiagnosticDef> definitions)
        : defs(std::move(definitions)), contact_sum_by_wheel(defs.size(), 0.0),
          contact_min_by_wheel(defs.size(), std::numeric_limits<std::size_t>::max()),
          contact_max_by_wheel(defs.size(), 0)
    {
    }

    std::vector<WheelDiagnosticDef> defs;
    std::vector<double> contact_sum_by_wheel;
    std::vector<std::size_t> contact_min_by_wheel;
    std::vector<std::size_t> contact_max_by_wheel;
    std::size_t samples = 0;
    std::size_t zero_contact_steps = 0;
    double axle_vertical_speed_sq_sum = 0.0;
    std::size_t axle_vertical_speed_samples = 0;
    float max_abs_axle_vertical_speed = 0.0f;
    double radial_speed_sq_sum = 0.0;
    std::size_t radial_speed_samples = 0;
    float max_abs_radial_speed = 0.0f;
    double radial_error_sq_sum = 0.0;
    std::size_t radial_error_samples = 0;
    float max_abs_radial_error = 0.0f;

    void Sample(const AuthoredVehicleRuntime& vehicle)
    {
        std::size_t total_contacts = 0;
        for (std::size_t wi = 0; wi < defs.size(); ++wi)
        {
            const WheelDiagnosticDef& def = defs[wi];
            if (def.axis0 >= vehicle.NodeCount() || def.axis1 >= vehicle.NodeCount()) continue;

            const auto& axis0 = vehicle.Node(def.axis0);
            const auto& axis1 = vehicle.Node(def.axis1);
            const float axle_vy = 0.5f * (axis0.velocity.y + axis1.velocity.y);
            axle_vertical_speed_sq_sum += static_cast<double>(axle_vy) * axle_vy;
            ++axle_vertical_speed_samples;
            max_abs_axle_vertical_speed = std::max(max_abs_axle_vertical_speed, std::fabs(axle_vy));

            std::size_t wheel_contacts = 0;
            for (const auto& binding : def.tyre_nodes)
            {
                if (binding.first >= vehicle.NodeCount() || binding.second >= vehicle.NodeCount()) continue;
                const auto& tyre = vehicle.Node(binding.first);
                const auto& axle = vehicle.Node(binding.second);
                if (tyre.position.y <= 0.0f) ++wheel_contacts;

                const RoR::PhysicsVec3 radial = tyre.position - axle.position;
                const float radius = Length(radial);
                if (radius > 1.0e-7f)
                {
                    const RoR::PhysicsVec3 radial_unit = radial * (1.0f / radius);
                    const RoR::PhysicsVec3 relative_velocity = tyre.velocity - axle.velocity;
                    const float radial_speed = relative_velocity.dot(radial_unit);
                    const float radial_error = radius - def.radius;
                    radial_speed_sq_sum += static_cast<double>(radial_speed) * radial_speed;
                    ++radial_speed_samples;
                    max_abs_radial_speed = std::max(max_abs_radial_speed, std::fabs(radial_speed));
                    radial_error_sq_sum += static_cast<double>(radial_error) * radial_error;
                    ++radial_error_samples;
                    max_abs_radial_error = std::max(max_abs_radial_error, std::fabs(radial_error));
                }
            }

            total_contacts += wheel_contacts;
            contact_sum_by_wheel[wi] += static_cast<double>(wheel_contacts);
            contact_min_by_wheel[wi] = std::min(contact_min_by_wheel[wi], wheel_contacts);
            contact_max_by_wheel[wi] = std::max(contact_max_by_wheel[wi], wheel_contacts);
        }
        if (total_contacts == 0) ++zero_contact_steps;
        ++samples;
    }

    void Print(const char* phase) const
    {
        const double axle_rms_vy = axle_vertical_speed_samples
            ? std::sqrt(axle_vertical_speed_sq_sum / static_cast<double>(axle_vertical_speed_samples))
            : 0.0;
        const double radial_rms_v = radial_speed_samples
            ? std::sqrt(radial_speed_sq_sum / static_cast<double>(radial_speed_samples))
            : 0.0;
        const double radial_rms_error = radial_error_samples
            ? std::sqrt(radial_error_sq_sum / static_cast<double>(radial_error_samples))
            : 0.0;

        std::cerr << "wheel_chatter phase=" << phase
                  << " zero_contact_steps=" << zero_contact_steps << '/' << samples
                  << " axle_rms_vy=" << axle_rms_vy
                  << " axle_max_abs_vy=" << max_abs_axle_vertical_speed
                  << " radial_rms_v=" << radial_rms_v
                  << " radial_max_abs_v=" << max_abs_radial_speed
                  << " radial_rms_error=" << radial_rms_error
                  << " radial_max_abs_error=" << max_abs_radial_error << '\n';

        for (std::size_t i = 0; i < defs.size(); ++i)
        {
            const double average = samples ? contact_sum_by_wheel[i] / static_cast<double>(samples) : 0.0;
            std::cerr << "wheel_contact phase=" << phase
                      << " wheel=" << i
                      << " avg=" << average
                      << " min=" << (samples ? contact_min_by_wheel[i] : 0)
                      << " max=" << contact_max_by_wheel[i] << '\n';
        }
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

    const std::vector<WheelDiagnosticDef> wheel_defs = BuildWheelDiagnosticDefs(text);
    AuthoredVehicleRuntime vehicle(text);
    std::cerr << "vehicle=" << vehicle.VehicleName()
              << " nodes=" << vehicle.NodeCount()
              << " tyres=" << vehicle.TireNodeIndices().size()
              << " beams=" << vehicle.BeamPairs().size()
              << " diagnostic_wheels=" << wheel_defs.size() << '\n';
    for (const auto& warning : vehicle.Warnings()) std::cerr << "warning: " << warning << '\n';
    for (const auto& error : vehicle.Errors()) std::cerr << "error: " << error << '\n';

    if (!vehicle.Ready())
    {
        std::cerr << "FAIL: runtime did not build\n";
        return 2;
    }

    PrintMassSummary(vehicle);

    TireContactStats settle_stats(wheel_defs);
    vehicle.SetControls(0.0f, 0.0f, 0.0f, false);
    if (!RunPhase(vehicle, "settle", 3000, &settle_stats))
    {
        std::cerr << "FAIL: vehicle unstable while settling\n";
        return 3;
    }
    settle_stats.Print("settle");
    const auto settled = vehicle.Telemetry();

    TireContactStats power_stats(wheel_defs);
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
