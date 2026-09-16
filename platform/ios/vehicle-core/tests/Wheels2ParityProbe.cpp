#include "AuthoredVehicleRuntime.h"
#include "SimConstants.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace RoR;
using namespace RoR::IOSVehicleCore;

namespace {

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "wheels2 parity probe failed: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool Near(float a, float b, float epsilon = 1.0e-3f)
{
    return std::fabs(a - b) <= epsilon * std::max(1.0f, std::max(std::fabs(a), std::fabs(b)));
}

float Length(const PhysicsVec3& v)
{
    return std::sqrt(v.squaredLength());
}

} // namespace

int main()
{
    constexpr int rays = 6;
    constexpr float authored_wheel_mass = 120.0f;
    constexpr float rim_radius = 0.30f;
    constexpr float tyre_radius = 0.50f;
    constexpr float axle_width = 0.40f;

    const std::string truck = R"ROR(wheels2 parity fixture
globals
900, 0, tracks/beam
nodes
0, 0, 1, -0.2
1, 0, 1, 0.2
wheels2
0.30, 0.50, 9.99, 6, 0, 1, -1, 0, 1, -1, 120, 300000, 1800, 180000, 1600
end
)ROR";

    AuthoredVehicleRuntime runtime(truck);
    Require(runtime.Ready(), "fixture did not spawn");
    Require(runtime.IsFinite(), "fixture is non-finite at spawn");

    // ActorSpawner::CalcMemoryRequirements(): wheels2 = 4 nodes/ray and exactly
    // 24 beams/ray without rigidity. Two authored axle nodes precede them.
    Require(runtime.NodeCount() == static_cast<std::size_t>(2 + 4 * rays),
            "wheels2 generated node count differs from upstream");
    Require(runtime.BeamPairs().size() == static_cast<std::size_t>(24 * rays),
            "wheels2 generated beam count differs from upstream 24/ray topology");
    Require(runtime.TireNodeIndices().size() == static_cast<std::size_t>(2 * rays),
            "wheels2 wheel.wh_nodes/tyre-node count differs from upstream");

    // Generated ordering is the same as ProcessWheel2(): all 2*rays rim nodes,
    // then all 2*rays tyre nodes. The tyre ring begins half a ray out of phase.
    const std::size_t first_outer = runtime.TireNodeIndices()[0];
    const std::size_t first_inner = runtime.TireNodeIndices()[1];
    const PhysicsVec3 outer_radial = runtime.Node(first_outer).position - runtime.Node(0).position;
    const PhysicsVec3 inner_radial = runtime.Node(first_inner).position - runtime.Node(1).position;
    Require(Near(Length(outer_radial), tyre_radius), "outer tyre radius changed during spawn");
    Require(Near(Length(inner_radial), tyre_radius), "inner tyre radius changed during spawn");

    const float half_ray_x = tyre_radius * std::sin(3.14159265358979323846f / rays);
    Require(Near(std::fabs(outer_radial.x), half_ray_x),
            "tyre ring is not offset by half a ray from rim ring");
    Require(Near(outer_radial.x, inner_radial.x) && Near(outer_radial.y, inner_radial.y),
            "outer/inner tyre nodes of one ray do not share angular position");

    // Historical ProcessWheel2 mass split: outer tyre nodes get 67% and inner
    // nodes 33% of the tyre half of authored wheel mass.
    const float expected_outer_mass = (0.67f * authored_wheel_mass) / (2.0f * rays);
    const float expected_inner_mass = (0.33f * authored_wheel_mass) / (2.0f * rays);
    Require(Near(runtime.Node(first_outer).mass, expected_outer_mass),
            "wheels2 outer tyre 67% mass split lost");
    Require(Near(runtime.Node(first_inner).mass, expected_inner_mass),
            "wheels2 inner tyre 33% mass split lost");

    float tyre_mass_sum = 0.0f;
    for (std::size_t index : runtime.TireNodeIndices())
        tyre_mass_sum += runtime.Node(index).mass;
    Require(Near(tyre_mass_sum, authored_wheel_mass * 0.5f),
            "wheel_t::wh_nodes no longer sum to upstream wheels2 half-mass");

    // ProcessWheel2 ignores the authored width token (9.99 above) and instead
    // uses axle-node distance. Tire friction is wheel width * 2.0 upstream.
    Require(Near(runtime.Node(first_outer).friction_coef, axle_width * WHEEL_FRICTION_COEF),
            "wheels2 tyre friction did not use axle width * WHEEL_FRICTION_COEF");

    std::cout << "wheels2 parity probe passed: " << rays << " rays, "
              << runtime.NodeCount() << " nodes, " << runtime.BeamPairs().size()
              << " beams, tyre mass " << tyre_mass_sum << " kg\n";
    return EXIT_SUCCESS;
}
