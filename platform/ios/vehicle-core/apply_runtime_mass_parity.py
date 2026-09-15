#!/usr/bin/env python3
from pathlib import Path
import sys
p=Path(sys.argv[1]);s=p.read_text()
def one(old,new):
 global s
 if s.count(old)!=1: raise SystemExit('runtime parity anchor drifted: '+old[:60])
 s=s.replace(old,new,1)
one('#include "PortableRigDef.h"\n','#include "PortableRigDef.h"\n#include "RoRMassDistribution.h"\n')
one('''        float dry_mass = rig.globals.present ? rig.globals.dry_mass : 0.0f;
        if (dry_mass <= 0.0f)
        {
            dry_mass = static_cast<float>(rig.nodes.size()) * 50.0f;
            Warn("vehicle has no usable globals dry mass; using portable fallback mass");
        }
        const float authored_node_mass = std::max(1.0f, dry_mass / static_cast<float>(rig.nodes.size()));
''','''        float dry_mass = rig.globals.present ? rig.globals.dry_mass : 0.0f;
        if (dry_mass <= 0.0f)
        {
            dry_mass = static_cast<float>(rig.nodes.size()) * 50.0f;
            Warn("vehicle has no usable globals dry mass; using portable fallback mass");
        }
''')
one('AddNode(PhysicsVec3(source.x, source.y, source.z), authored_node_mass);','AddNode(PhysicsVec3(source.x, source.y, source.z), 1.0f);')
anchor='''        for (const PortableRigDef::Wheel& source : rig.wheels)
            BuildWheel(source);

        if (wheels.empty())
'''
insert='''        for (const PortableRigDef::Wheel& source : rig.wheels)
            BuildWheel(source);

        // Match Actor::recalculateNodeMasses(): preserve tyre masses, initialize
        // loaded/override nodes, distribute dry mass by non-virtual beam refL,
        // then enforce each node's effective minimass.
        std::vector<MassNodeInput> mass_nodes(nodes.size());
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            mass_nodes[i].minimass = 50.0f;
            if (std::find(tire_nodes.begin(), tire_nodes.end(), i) != tire_nodes.end())
            {
                mass_nodes[i].tyre = true;
                mass_nodes[i].tyre_mass = nodes[i].mass;
            }
        }
        for (const PortableRigDef::Node& source : rig.nodes)
        {
            const std::size_t i = ResolveNode(source.id, false);
            if (i == kInvalidIndex) continue;
            mass_nodes[i].loaded = source.loaded_mass;
            mass_nodes[i].override_mass = source.override_mass;
            mass_nodes[i].override_weight = source.load_weight;
            mass_nodes[i].minimass = source.minimass;
        }
        std::vector<MassBeamInput> mass_beams;
        mass_beams.reserve(beams.size());
        for (const BeamLink& link : beams)
        {
            MassBeamInput b;
            b.a = link.a; b.b = link.b; b.reference_length = link.beam.rest_length;
            b.virtual_beam = false;
            mass_beams.push_back(b);
        }
        const MassDistributionResult distributed = CalculateRoRNodeMasses(
            dry_mass, rig.globals.present ? rig.globals.load_mass : 0.0f,
            mass_nodes, mass_beams, false);
        if (distributed.masses.size() == nodes.size())
            for (std::size_t i = 0; i < nodes.size(); ++i) nodes[i].mass = distributed.masses[i];

        if (wheels.empty())
'''
one(anchor,insert)
one('''        if (!rig.globals.present)
            Warn("portable authored runtime is using fallback mass distribution");
        else
            Warn("portable authored runtime currently distributes globals dry mass uniformly across authored nodes");
''','''        if (!rig.globals.present)
            Warn("portable authored runtime is using fallback dry mass");
''')
p.write_text(s)
print('applied upstream-compatible RoR node mass distribution')
