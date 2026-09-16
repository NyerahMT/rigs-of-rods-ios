#include "PortableRigDef.h"
#include "RoRMassDistribution.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace RoR::IOSVehicleCore;

namespace {
bool Near(float a, float b, float eps = 1.0e-5f)
{
    return std::fabs(a - b) <= eps;
}
}

int main()
{
    const char* truck =
        "Minimass parity fixture\n"
        "globals\n"
        "10, 0, tracks/transred\n"
        "minimass\n"
        "50, l\n"
        "nodes\n"
        "0, 0, 1, 0, L\n"
        "1, 2, 1, 0\n"
        "beams\n"
        "0, 1\n"
        "end\n";

    const RoR::PortableRigDef::Document rig = RoR::PortableRigDef::Parse(truck);
    if (!rig.minimass_skip_loaded)
    {
        std::cerr << "minimass l option was not preserved\n";
        return 1;
    }
    if (rig.nodes.size() != 2 || !rig.nodes[0].loaded_mass || rig.nodes[1].loaded_mass)
    {
        std::cerr << "fixture load-node semantics drifted\n";
        return 2;
    }

    std::vector<MassNodeInput> nodes(2);
    nodes[0].loaded = true;
    nodes[0].minimass = 50.0f;
    nodes[1].minimass = 50.0f;

    MassBeamInput beam;
    beam.a = 0;
    beam.b = 1;
    beam.reference_length = 2.0f;
    std::vector<MassBeamInput> beams(1, beam);

    const MassDistributionResult upstream_l =
        CalculateRoRNodeMasses(10.0f, 0.0f, nodes, beams, rig.minimass_skip_loaded);
    if (upstream_l.masses.size() != 2 ||
        !Near(upstream_l.masses[0], 5.0f) ||
        !Near(upstream_l.masses[1], 50.0f))
    {
        std::cerr << "minimass,l mass result drifted: loaded=" << upstream_l.masses[0]
                  << " ordinary=" << upstream_l.masses[1] << '\n';
        return 3;
    }

    const MassDistributionResult without_l =
        CalculateRoRNodeMasses(10.0f, 0.0f, nodes, beams, false);
    if (!Near(without_l.masses[0], 50.0f) || !Near(without_l.masses[1], 50.0f))
    {
        std::cerr << "control minimass behavior drifted\n";
        return 4;
    }

    std::cout << "upstream minimass,l loaded-node semantics match\n";
    return 0;
}
