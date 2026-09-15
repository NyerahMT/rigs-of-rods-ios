#include "BeamPhysics.h"
#include "PortableRigDef.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace {

bool Near(float a, float b, float eps = 1.0e-3f)
{
    return std::fabs(a - b) <= eps * std::max(1.0f, std::max(std::fabs(a), std::fabs(b)));
}

int Fail(const std::string& message)
{
    std::cerr << "shock parity probe failed: " << message << "\n";
    return 1;
}

} // namespace

int main()
{
    using namespace RoR;
    using namespace RoR::PortableRigDef;

    // BeamDefaults are raw upstream data plus a separate scale. Ordinary beams
    // consume GetScaled*(), while classic shock bump-stops retain the raw values.
    const std::string truck = R"ROR(Shock parity fixture
set_beam_defaults_scale 0.5, 0.25, 0.2, 0.3
set_beam_defaults 8000000, 4000, 300000, 500000
nodes
0, 0, 1, 0
1, 1, 1, 0
beams
0, 1
shocks
0, 1, 64000, 3700, 0.29, 0.11, 1.07, im
shocks2
0, 1, 61000, 3300, 1.2, 1.3, 52000, 2800, 1.4, 1.5, 0.32, 0.12, 0.975, M
end
)ROR";

    const Document rig = Parse(truck);
    if (rig.beams.size() != 1) return Fail("expected one ordinary beam");
    if (rig.shocks.size() != 2) return Fail("expected classic shock plus shocks2");

    if (!Near(rig.beams[0].spring, 4000000.0f))
        return Fail("normal beam did not apply set_beam_defaults_scale spring");
    if (!Near(rig.beams[0].damping, 1000.0f))
        return Fail("normal beam did not apply set_beam_defaults_scale damping");

    const Shock& classic = rig.shocks[0];
    if (classic.shock2) return Fail("classic shocks entry marked as shock2");
    if (!Near(classic.spring, 64000.0f) || !Near(classic.damping, 3700.0f))
        return Fail("classic shock spring/damping lost");
    if (!Near(classic.short_bound, 0.29f) || !Near(classic.long_bound, 0.11f) ||
        !Near(classic.precompression, 1.07f))
        return Fail("classic shock limits/precompression lost");
    if (classic.options != "im") return Fail("classic shock options lost");
    if (!Near(classic.bump_spring, 8000000.0f) || !Near(classic.bump_damping, 4000.0f))
        return Fail("classic shock did not preserve raw BeamDefaults bump stop");

    const Shock& advanced = rig.shocks[1];
    if (!advanced.shock2) return Fail("shocks2 entry not marked as shock2");
    if (!Near(advanced.spring_in, 61000.0f) || !Near(advanced.damp_in, 3300.0f) ||
        !Near(advanced.progress_spring_in, 1.2f) || !Near(advanced.progress_damp_in, 1.3f) ||
        !Near(advanced.spring_out, 52000.0f) || !Near(advanced.damp_out, 2800.0f) ||
        !Near(advanced.progress_spring_out, 1.4f) || !Near(advanced.progress_damp_out, 1.5f))
        return Fail("shocks2 asymmetric/progressive metadata lost");
    if (!Near(advanced.short_bound, 0.32f) || !Near(advanced.long_bound, 0.12f) ||
        !Near(advanced.precompression, 0.975f) || advanced.options != "M")
        return Fail("shocks2 limits/options lost");

    // Exercise the SHOCK1 force branch with an authored bump target. The same
    // extension with a 100 N/m base spring would produce about 50 N. Once the
    // 10% long bound is exceeded, the effective spring must climb toward the
    // authored 1000 N/m bump-stop value.
    NodeCoreState a;
    NodeCoreState b;
    a.position = PhysicsVec3(1.5f, 0.0f, 0.0f);
    b.position = PhysicsVec3(0.0f, 0.0f, 0.0f);
    a.mass = b.mass = 1.0f;

    BeamCoreState beam;
    beam.rest_length = 1.0f;
    beam.spring = 100.0f;
    beam.damping = 0.0f;
    beam.bounded = true;
    beam.shortbound = 0.1f;
    beam.longbound = 0.1f;
    beam.bump_spring = 1000.0f;
    beam.bump_damping = 50.0f;

    ApplyBeamForce(a, b, beam);
    if (!(beam.stress < -100.0f))
        return Fail("bounded shock did not stiffen into authored bump stop");
    if (!(a.force.x < 0.0f && b.force.x > 0.0f))
        return Fail("bounded shock force direction is wrong");

    std::cout << "shock parity probe passed\n";
    return 0;
}
