#include "AuthoredVehicleRuntime.h"
#include "BeamPhysics.h"
#include "SimConstants.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

float Length(const RoR::PhysicsVec3& v)
{
    return std::sqrt(v.squaredLength());
}

bool Near(float a, float b, float epsilon)
{
    return std::fabs(a - b) <= epsilon;
}

} // namespace

int main()
{
    // Deliberately list axle refs as 1,0. Upstream GetWheelAxisNodes() reorders
    // them by Z before generating the alternating outer/inner ring.
    const std::string truck = R"ROR(Meshwheel parity probe
globals
1200, 0, testmat
nodes
0, 0.0, 1.0, -0.5
1, 0.0, 1.0,  0.5
2, 1.0, 1.0,  0.0
meshwheels2
0.50, 0.30, 1.00, 8, 1, 0, 9999, 1, 1, 2, 40, 100000, 1500, l, testwheel.mesh testtire
engine
1000, 5000, 300, 3.0, 3.0, 1.0, 3.0, 2.0, 1.0, -1
brakes
2000, 4000
end
)ROR";

    RoR::IOSVehicleCore::AuthoredVehicleRuntime runtime(truck);
    if (!runtime.Ready())
    {
        std::cerr << "FAIL: meshwheel fixture did not build";
        if (!runtime.Errors().empty()) std::cerr << ": " << runtime.Errors().front();
        std::cerr << '\n';
        return EXIT_FAILURE;
    }

    // ActorSpawner::BuildWheelObjectAndNodes() generates exactly 2*num_rays
    // tyre nodes for meshwheels2, and BuildWheelBeams() adds 8 beams per ray
    // when there is no rigidity node.
    if (runtime.NodeCount() != 19)
    {
        std::cerr << "FAIL: expected 3 authored + 16 generated nodes, got "
                  << runtime.NodeCount() << '\n';
        return EXIT_FAILURE;
    }
    if (runtime.TireNodeIndices().size() != 16)
    {
        std::cerr << "FAIL: expected 16 meshwheel tyre nodes, got "
                  << runtime.TireNodeIndices().size() << '\n';
        return EXIT_FAILURE;
    }
    if (runtime.BeamPairs().size() != 64)
    {
        std::cerr << "FAIL: expected 64 generated wheel beams, got "
                  << runtime.BeamPairs().size() << '\n';
        return EXIT_FAILURE;
    }

    const RoR::PhysicsVec3 r_outer = runtime.Node(3).position - runtime.Node(0).position;
    const RoR::PhysicsVec3 r_inner = runtime.Node(4).position - runtime.Node(1).position;
    const float outer_len = Length(r_outer);
    const float inner_len = Length(r_inner);
    if (!Near(outer_len, 0.50f, 0.002f) || !Near(inner_len, 0.50f, 0.002f))
    {
        std::cerr << "FAIL: generated wheel nodes were not attached to Z-ordered axle nodes: "
                  << outer_len << ", " << inner_len << '\n';
        return EXIT_FAILURE;
    }

    // Inner node is rotated by one half-ray step (pi / num_rays) relative to
    // the outer node. The old iOS wheel incorrectly put both at the same angle.
    const float cosine = r_outer.dot(r_inner) / (outer_len * inner_len);
    const float expected_cosine = std::cos(3.14159265358979323846f / 8.0f);
    if (!Near(cosine, expected_cosine, 0.01f))
    {
        std::cerr << "FAIL: outer/inner ring staggering mismatch: got " << cosine
                  << " expected " << expected_cosine << '\n';
        return EXIT_FAILURE;
    }

    // Verify the portable beam helper retains Actor::CalcBeams() SHOCK1 hard
    // bump interpolation used by the first two spokes in BuildWheelBeams().
    RoR::NodeCoreState a;
    RoR::NodeCoreState b;
    a.position = RoR::PhysicsVec3(1.30f, 0.0f, 0.0f);
    b.position = RoR::PhysicsVec3(0.0f, 0.0f, 0.0f);
    RoR::BeamCoreState beam;
    beam.rest_length = 1.0f;
    beam.spring = 1000.0f;
    beam.damping = 0.0f;
    beam.bounded = true;
    beam.shortbound = 0.66f;
    beam.longbound = 0.10f;
    RoR::ApplyBeamForce(a, b, beam);

    const float excess = 0.30f - 0.10f;
    const float effective_spring = 1000.0f + (DEFAULT_SPRING - 1000.0f) * excess;
    const float expected_stress = -effective_spring * 0.30f;
    if (!Near(beam.stress, expected_stress, std::fabs(expected_stress) * 0.002f))
    {
        std::cerr << "FAIL: bounded wheel-beam stress mismatch: got " << beam.stress
                  << " expected " << expected_stress << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "Meshwheel parity passed: 2*rays staggered tyre nodes, 8*rays beam topology, "
              << "Z-ordered axle and SHOCK1 spoke bounds.\n";
    return EXIT_SUCCESS;
}
