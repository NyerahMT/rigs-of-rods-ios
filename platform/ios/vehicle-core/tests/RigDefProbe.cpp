#include "PortableRigDef.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace RoR::PortableRigDef;

namespace
{
void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool Near(float a, float b, float epsilon = 0.001f)
{
    return std::fabs(a - b) <= epsilon;
}
}

int main()
{
    const char* rig_text = R"ROR(Portable Drift Coupe
set_beam_defaults 800000, 9000, 300000, 500000
set_node_defaults 0, 42

nodes
0,  1.25, 0.55,  0.78
1,  1.25, 0.55, -0.78
2, -1.25, 0.55,  0.78
set_node_defaults 15, 75
3, -1.25, 0.55, -0.78
4,  1.05, 0.85,  0.25
5,  1.05, 0.85, -0.25

beams
0, 2
1, 3
0, 1
2, 3

hydros
4, 0, 0.12, s
5, 1, -0.12, s

shocks
0, 4, 220000, 4200, 0.70, 1.25, 1.00
1, 5, 220000, 4200, 0.70, 1.25, 1.00

props
0, 1, 2, 0, 0, 0, 0, 0, 0, dummy.mesh

wheels2
0.30, 0.50, 0.26, 12, 0, 4, 2, 1, 0, 2, 18, 350000, 4500, 180000, 2500
0.30, 0.50, 0.26, 12, 1, 5, 3, 1, 0, 3, 18, 350000, 4500, 180000, 2500
0.30, 0.50, 0.28, 12, 2, 4, 0, 2, 1, 0, 20, 380000, 4800, 170000, 2400
0.30, 0.50, 0.28, 12, 3, 5, 1, 2, 1, 1, 20, 380000, 4800, 170000, 2400

engine
1200, 6500, 410, 4.10, -3.20, 0, 3.05, 2.10, 1.55, 1.18, 1.00

brakes
30000, 55000

end
)ROR";

    const Document rig = Parse(rig_text);

    Require(rig.name == "Portable Drift Coupe", "rig name is preserved");
    Require(rig.nodes.size() == 6, "nodes section is parsed");
    Require(rig.beams.size() == 4, "beams section is parsed");
    Require(rig.hydros.size() == 2, "hydros section is parsed");
    Require(rig.shocks.size() == 2, "shocks section is parsed");
    Require(rig.wheels.size() == 4, "wheels2 section is parsed after unsupported visual blocks");

    Require(Near(rig.nodes[0].load_weight, 0.0f), "first node-default load weight is inherited");
    Require(Near(rig.nodes[0].minimass, 42.0f), "first node-default minimass is inherited");
    Require(Near(rig.nodes[2].minimass, 42.0f), "node defaults remain active until changed");
    Require(Near(rig.nodes[3].load_weight, 15.0f), "changed node-default load weight is inherited");
    Require(Near(rig.nodes[3].minimass, 75.0f), "changed node-default minimass is inherited");
    Require(Near(rig.nodes[5].minimass, 75.0f), "changed node defaults remain active");

    Require(Near(rig.beams[0].spring, 800000.0f), "set_beam_defaults spring reaches beams");
    Require(Near(rig.beams[0].damping, 9000.0f), "set_beam_defaults damping reaches beams");
    Require(Near(rig.hydros[0].spring, 800000.0f), "beam defaults reach hydros");
    Require(Near(rig.hydros[0].lengthening_factor, 0.12f), "left steering hydro factor is preserved");
    Require(Near(rig.hydros[1].lengthening_factor, -0.12f), "right steering hydro factor is preserved");

    Require(Near(rig.shocks[0].short_bound, 0.70f), "shock compression bound is parsed");
    Require(Near(rig.shocks[0].long_bound, 1.25f), "shock extension bound is parsed");

    const Wheel& rear_left = rig.wheels[2];
    Require(rear_left.wheels2, "wheels2 is identified");
    Require(rear_left.num_rays == 12, "wheel ray count is parsed");
    Require(rear_left.propulsion == 1, "rear wheel propulsion flag is parsed");
    Require(rear_left.braking == 2, "rear wheel braking flag is parsed");
    Require(rear_left.reference_arm_node == "0", "wheel drivetrain reaction-arm node is preserved");
    Require(Near(rear_left.rim_radius, 0.30f), "rim radius is parsed");
    Require(Near(rear_left.tire_radius, 0.50f), "tire radius is parsed");
    Require(Near(rear_left.rim_spring, 380000.0f), "rim spring is parsed");
    Require(Near(rear_left.tire_spring, 170000.0f), "tire spring is parsed");

    Require(rig.engine.present, "engine section is parsed");
    Require(Near(rig.engine.torque, 410.0f), "engine torque is parsed");
    Require(Near(rig.engine.differential_ratio, 4.10f), "differential ratio is parsed");
    Require(rig.engine.gear_ratios.size() == 7, "gear ratios are retained");
    Require(Near(rig.engine.gear_ratios[2], 3.05f), "first forward gear ratio is retained");

    Require(rig.brakes.present, "brakes section is parsed");
    Require(Near(rig.brakes.service_force, 30000.0f), "service brake force is parsed");
    Require(Near(rig.brakes.parking_force, 55000.0f), "parking brake force is parsed");
    Require(rig.warnings.empty(), "well-formed supported structural subset produces no warnings");

    std::cout << "RoR portable rig probe passed: "
              << rig.nodes.size() << " nodes, "
              << rig.beams.size() << " beams, "
              << rig.hydros.size() << " hydros, "
              << rig.shocks.size() << " shocks, "
              << rig.wheels.size() << " wheels.\n";
    return EXIT_SUCCESS;
}
