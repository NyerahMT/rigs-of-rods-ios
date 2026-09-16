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
    // consume GetScaled*(), while shock bump-stops retain the raw values.
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
0, 1, 61000, 3300, 1.2, 1.3, 52000, 2800, 1.4, 1.5, 0.32, 0.12, 0.975, sM
shocks3
0, 1, 62000, 3400, 0.75, 0.8, 0.35, 53000, 2900, 0.85, 1.1, 0.45, 0.31, 0.13, 0.98, M
end
)ROR";

    const Document rig = Parse(truck);
    if (rig.beams.size() != 1) return Fail("expected one ordinary beam");
    if (rig.shocks.size() != 3) return Fail("expected classic shock, shocks2 and shocks3");

    if (!Near(rig.beams[0].spring, 4000000.0f))
        return Fail("normal beam did not apply set_beam_defaults_scale spring");
    if (!Near(rig.beams[0].damping, 1000.0f))
        return Fail("normal beam did not apply set_beam_defaults_scale damping");

    const Shock& classic = rig.shocks[0];
    if (classic.shock2 || classic.shock3) return Fail("classic shocks entry marked advanced");
    if (!Near(classic.spring, 64000.0f) || !Near(classic.damping, 3700.0f))
        return Fail("classic shock spring/damping lost");
    if (!Near(classic.short_bound, 0.29f) || !Near(classic.long_bound, 0.11f) ||
        !Near(classic.precompression, 1.07f))
        return Fail("classic shock limits/precompression lost");
    if (classic.options != "im") return Fail("classic shock options lost");
    if (!Near(classic.bump_spring, 8000000.0f) || !Near(classic.bump_damping, 4000.0f))
        return Fail("classic shock did not preserve raw BeamDefaults bump stop");

    const Shock& advanced = rig.shocks[1];
    if (!advanced.shock2 || advanced.shock3) return Fail("shocks2 model flags wrong");
    if (!Near(advanced.spring_in, 61000.0f) || !Near(advanced.damp_in, 3300.0f) ||
        !Near(advanced.progress_spring_in, 1.2f) || !Near(advanced.progress_damp_in, 1.3f) ||
        !Near(advanced.spring_out, 52000.0f) || !Near(advanced.damp_out, 2800.0f) ||
        !Near(advanced.progress_spring_out, 1.4f) || !Near(advanced.progress_damp_out, 1.5f))
        return Fail("shocks2 asymmetric/progressive metadata lost");
    if (!Near(advanced.short_bound, 0.32f) || !Near(advanced.long_bound, 0.12f) ||
        !Near(advanced.precompression, 0.975f) || advanced.options != "sM")
        return Fail("shocks2 limits/options lost");

    const Shock& digressive = rig.shocks[2];
    if (digressive.shock2 || !digressive.shock3) return Fail("shocks3 model flags wrong");
    if (!Near(digressive.spring_in, 62000.0f) || !Near(digressive.damp_in, 3400.0f) ||
        !Near(digressive.damp_in_slow, 0.75f) || !Near(digressive.split_vel_in, 0.8f) ||
        !Near(digressive.damp_in_fast, 0.35f) || !Near(digressive.spring_out, 53000.0f) ||
        !Near(digressive.damp_out, 2900.0f) || !Near(digressive.damp_out_slow, 0.85f) ||
        !Near(digressive.split_vel_out, 1.1f) || !Near(digressive.damp_out_fast, 0.45f))
        return Fail("shocks3 velocity-split metadata lost");
    if (!Near(digressive.short_bound, 0.31f) || !Near(digressive.long_bound, 0.13f) ||
        !Near(digressive.precompression, 0.98f) || digressive.options != "M")
        return Fail("shocks3 limits/options lost");

    // SHOCK1 authored bump target.
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
    beam.shock_model = ShockModel::Shock1;
    beam.shortbound = 0.1f;
    beam.longbound = 0.1f;
    beam.bump_spring = 1000.0f;
    beam.bump_damping = 50.0f;

    ApplyBeamForce(a, b, beam);
    if (!(beam.stress < -100.0f))
        return Fail("bounded SHOCK1 did not stiffen into authored bump stop");
    if (!(a.force.x < 0.0f && b.force.x > 0.0f))
        return Fail("bounded shock force direction is wrong");

    // SHOCK2 base progression: at half of the long travel limit the squared
    // factor is 0.25, so 100 N/m with progress=1 becomes 125 N/m.
    BeamCoreState s2;
    s2.rest_length = 1.0f;
    s2.shock_model = ShockModel::Shock2;
    s2.shortbound = 0.5f;
    s2.longbound = 0.5f;
    s2.spring_in = 200.0f;
    s2.damp_in = 20.0f;
    s2.spring_out = 100.0f;
    s2.damp_out = 10.0f;
    s2.progress_spring_in = 0.5f;
    s2.progress_damp_in = 0.5f;
    s2.progress_spring_out = 1.0f;
    s2.progress_damp_out = 1.0f;
    s2.bump_spring = 1000.0f;
    s2.bump_damping = 100.0f;

    ShockCoefficients c = CalcShock2Coefficients(s2, 0.25f, 1.0f);
    if (!Near(c.spring, 125.0f) || !Near(c.damping, 12.5f))
        return Fail("SHOCK2 rebound progression differs from upstream squared law");

    // Soft-bump prelimit begins at 80% of the authored bound. At 0.45 m travel
    // on a 0.5 bound, the extra 5x ramp is already active before hard contact.
    s2.soft_bump = true;
    c = CalcShock2Coefficients(s2, 0.45f, 1.0f);
    if (!(c.spring > 200.0f && c.spring < s2.bump_spring))
        return Fail("SHOCK2 soft bump did not ramp before hard limiter");
    c = CalcShock2Coefficients(s2, 0.60f, 1.0f);
    if (c.spring < s2.bump_spring || c.damping < s2.bump_damping)
        return Fail("SHOCK2 soft bump did not enforce BeamDefaults hard floor");

    // SHOCK3 exact velocity split. For extension at 3 m/s with split=2,
    // 100*1*2 + 100*0.5*1 = 250 N*s/m integrated over speed => 83.3333.
    BeamCoreState s3;
    s3.rest_length = 1.0f;
    s3.spring = s3.spring_in = 120.0f;
    s3.damping = s3.damp_in = 120.0f;
    s3.spring_out = 90.0f;
    s3.damp_out = 100.0f;
    s3.shortbound = 0.5f;
    s3.longbound = 0.5f;
    s3.damp_out_slow = 1.0f;
    s3.split_vel_out = 2.0f;
    s3.damp_out_fast = 0.5f;
    s3.damp_in_slow = 0.8f;
    s3.split_vel_in = 1.0f;
    s3.damp_in_fast = 0.4f;
    s3.bump_spring = 900.0f;
    s3.bump_damping = 300.0f;

    c = CalcShock3Coefficients(s3, 0.1f, 3.0f);
    if (!Near(c.spring, 90.0f) || !Near(c.damping, 83.333333f))
        return Fail("SHOCK3 rebound velocity-split damping differs from upstream");
    c = CalcShock3Coefficients(s3, -0.1f, -4.0f);
    if (!Near(c.spring, 120.0f) || !Near(c.damping, 60.0f))
        return Fail("SHOCK3 compression velocity-split damping differs from upstream");

    std::cout << "shock parity probe passed\n";
    return 0;
}
