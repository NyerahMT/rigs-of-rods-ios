#include "PortableRigDef.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace RoR::PortableRigDef;

namespace {

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool Near(float a, float b, float epsilon = .001f)
{
    return std::fabs(a - b) <= epsilon;
}

} // namespace

int main()
{
    const char* rig_text = R"ROR(Portable Drift Coupe
set_beam_defaults_scale 0.5, 1, 1, 1
set_beam_defaults 800000, 9000, 300000, 500000
set_node_defaults 0, 1.25, 0.8, 1.1, c
minimass
42
nodes
0,1.25,0.55,0.78
1,1.25,0.55,-0.78,l
2,-1.25,0.55,0.78,l 120
set_default_minimass 12
set_node_defaults 15, 0.75, 1, 1
3,-1.25,0.55,-0.78
4,1.05,0.85,0.25
5,1.05,0.85,-0.25
beams
0,2
1,3
0,1
2,3
hydros
4,0,0.12,s
5,1,-0.12,s
shocks
0,4,220000,4200,0.70,1.25,1.00
1,5,220000,4200,0.70,1.25,1.00
props
0,1,2,0,0,0,0,0,0,dummy.mesh
wheels2
0.30,0.50,0.26,12,0,4,2,1,0,2,18,350000,4500,180000,2500
0.30,0.50,0.26,12,1,5,3,1,0,3,18,350000,4500,180000,2500
0.30,0.50,0.28,12,2,4,0,2,1,0,20,380000,4800,170000,2400
0.30,0.50,0.28,12,3,5,1,2,1,1,20,380000,4800,170000,2400
engine
1200,6500,410,4.10,3.20,0,3.05,2.10,1.55,1.18,1.00,-1
brakes
30000,55000
end
)ROR";

    Document r = Parse(rig_text);
    Require(r.name == "Portable Drift Coupe", "name");
    Require(r.nodes.size() == 6, "nodes");
    Require(r.beams.size() == 4, "beams");
    Require(r.wheels.size() == 4, "wheels");
    Require(Near(r.nodes[0].minimass, 42), "minimass section applies");
    Require(Near(r.nodes[0].friction, 1.25), "node friction default");
    Require(Near(r.nodes[0].volume, .8), "node volume default");
    Require(r.nodes[0].options.find('c') != std::string::npos, "default node option");
    Require(r.nodes[1].loaded_mass && !r.nodes[1].override_mass, "plain l node uses shared globals load mass");
    Require(r.nodes[2].loaded_mass && r.nodes[2].override_mass && Near(r.nodes[2].load_weight, 120), "explicit l mass overrides load mass");
    Require(Near(r.nodes[3].minimass, 12), "set_default_minimass applies forward");
    Require(r.nodes[3].loaded_mass && r.nodes[3].override_mass && Near(r.nodes[3].load_weight, 15), "default loadweight creates loaded override node");
    Require(Near(r.nodes[3].friction, .75), "changed friction default");
    Require(Near(r.beams[0].spring, 400000), "beam-default scale applies");
    Require(Near(r.beams[0].damping, 9000), "beam damping scale");
    Require(Near(r.hydros[0].spring, 400000), "scaled defaults reach hydros");
    Require(Near(r.hydros[0].lengthening_factor, .12), "hydro factor");
    Require(r.engine.present && Near(r.engine.torque, 410), "engine");
    Require(Near(r.engine.differential_ratio, 4.10), "engine differential ratio");
    Require(Near(r.engine.reverse_gear_ratio, 3.20), "engine reverse ratio");
    Require(Near(r.engine.neutral_gear_ratio, 0), "engine neutral ratio");
    Require(r.engine.gear_ratios.size() == 5, "forward gears only");
    Require(Near(r.engine.gear_ratios[0], 3.05) && Near(r.engine.gear_ratios[4], 1.00), "forward gear values");
    Require(r.brakes.present && Near(r.brakes.parking_force, 55000), "brakes");
    Require(r.warnings.empty(), "supported fixture warns cleanly");

    const char* legacy_text = R"ROR(Legacy MeshWheel Coupe
globals
50,1000,bodymat
engine
1800,4500,840,2.95,3.3,5.0,3.35,2.1,1.4,1.0,-1
engoption
0.075,c,1000,0.30,0.60,0.25,550,925,0.14,0.03,175
nodes
0,-1.0,0.5,-0.7
1,-1.0,0.5,0.7
2,1.0,0.5,-0.7
3,1.0,0.5,0.7
extcamera node 0
beams
0,2
1,3
detacher_group 8
0,1
detacher_group 0
2,3
set_beam_defaults 2950000,200,25000000,3000000000
meshwheels2
0.335,0.20,0.205,14,0,1,9999,4,0,2,48,86000,1150,l,wheel.mesh,tiremat
brakes
1870,4400
end
)ROR";

    Document legacy = Parse(legacy_text);
    Require(legacy.nodes.size() == 4, "legacy extcamera does not become a node");
    Require(legacy.beams.size() == 4, "legacy detacher_group does not become a beam");
    Require(legacy.engine.present && legacy.engine.gear_ratios.size() == 4, "engoption does not corrupt engine block");
    Require(legacy.engoption.present, "engoption parsed");
    Require(Near(legacy.engoption.inertia, .075f), "engoption inertia");
    Require(legacy.engoption.type == 'c', "engoption engine type");
    Require(Near(legacy.engoption.clutch_force, 1000.0f), "engoption clutch force");
    Require(Near(legacy.engoption.shift_time, .30f), "engoption authored shift time");
    Require(Near(legacy.engoption.clutch_time, .60f), "engoption authored clutch time");
    Require(Near(legacy.engoption.post_shift_time, .25f), "engoption post-shift time");
    Require(Near(legacy.engoption.stall_rpm, 550.0f), "engoption stall RPM");
    Require(Near(legacy.engoption.idle_rpm, 925.0f), "engoption idle RPM");
    Require(Near(legacy.engoption.max_idle_mixture, .14f), "engoption max idle mixture");
    Require(Near(legacy.engoption.min_idle_mixture, .03f), "engoption min idle mixture");
    Require(Near(legacy.engoption.braking_torque, 175.0f), "engoption braking torque");
    Require(legacy.wheels.size() == 1, "meshwheels2 parsed as a physical wheel");
    Require(!legacy.wheels[0].wheels2, "meshwheels2 uses upstream two-node-per-ray topology");
    Require(Near(legacy.wheels[0].tire_radius, .335f) && Near(legacy.wheels[0].rim_radius, .20f), "meshwheels2 radius order");
    Require(Near(legacy.wheels[0].rim_spring, 2950000.0f) && Near(legacy.wheels[0].rim_damping, 200.0f), "meshwheels2 rim uses active beam defaults");
    Require(Near(legacy.wheels[0].tire_spring, 86000.0f) && Near(legacy.wheels[0].tire_damping, 1150.0f), "meshwheels2 tyre spring/damping");
    Require(legacy.warnings.empty(), "legacy compatibility fixture warns cleanly");

    std::cout << "RigDef parity probe passed\n";
    return EXIT_SUCCESS;
}
