#!/usr/bin/env python3
from pathlib import Path
import sys
p=Path(sys.argv[1]);s=p.read_text()
def one(old,new):
 global s
 if s.count(old)!=1: raise SystemExit('runtime parity anchor drifted: '+old[:80])
 s=s.replace(old,new,1)

one('#include "PortableRigDef.h"\n','#include "PortableRigDef.h"\n#include "RoRMassDistribution.h"\n#include "RoRDrivetrain.h"\n')
one('''    GroundContactParams road;

    float steering = 0.0f;
''','''    GroundContactParams road;
    RoRDrivetrain drivetrain;

    float steering = 0.0f;
''')
one('''        const float spring = def.tire_spring > 0.0f ? def.tire_spring : 200000.0f;
        const float damping = def.tire_damping > 0.0f ? def.tire_damping : 2500.0f;
''','''        const float spring = def.tire_spring > 0.0f ? def.tire_spring : 200000.0f;
        const float damping = def.tire_damping > 0.0f ? def.tire_damping : 2500.0f;
        // Upstream meshwheels2 shares the simple two-node-per-ray topology.
        // Its spoke beams use tyre spring/damping while the ring reinforcement
        // uses the active set_beam_defaults values carried in rim_*.
        const float ring_spring = def.rim_spring > 0.0f ? def.rim_spring : spring;
        const float ring_damping = def.rim_damping > 0.0f ? def.rim_damping : damping;
''')
one('''            AddBeam(side0, next0, spring, damping, BeamKind::Wheel);
            AddBeam(side1, next1, spring, damping, BeamKind::Wheel);
            AddBeam(side0, side1, spring, damping, BeamKind::Wheel);
            AddBeam(side0, next1, spring, damping, BeamKind::Wheel);
''','''            AddBeam(side0, next0, ring_spring, ring_damping, BeamKind::Wheel);
            AddBeam(side1, next1, ring_spring, ring_damping, BeamKind::Wheel);
            AddBeam(side0, side1, ring_spring, ring_damping, BeamKind::Wheel);
            // Match ActorSpawner::BuildWheelBeams(): inner ring -> next outer ring.
            AddBeam(side1, next0, ring_spring, ring_damping, BeamKind::Wheel);
''')
one('''        fixture.braking = def.braking;
        fixture.propulsion = def.propulsion;
        fixture.reference_arm = ResolveNode(def.reference_arm_node);
''','''        fixture.braking = def.braking;
        fixture.propulsion = def.propulsion;
        // RoR::CalcWheels reverses radius only for WheelPropulsion::BACKWARD
        // (numeric truck-file value 2). Do not infer torque direction from mesh geometry.
        fixture.wheel.reverse_rotation = (def.propulsion == 2);
        fixture.reference_arm = ResolveNode(def.reference_arm_node);
''')
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

        RoRDrivetrainConfig drivetrain_config;
        if (rig.engine.present)
        {
            drivetrain_config.shift_down_rpm = rig.engine.shift_down_rpm;
            drivetrain_config.shift_up_rpm = rig.engine.shift_up_rpm;
            drivetrain_config.engine_torque = rig.engine.torque;
            drivetrain_config.differential_ratio = rig.engine.differential_ratio;
            drivetrain_config.reverse_gear_ratio = rig.engine.reverse_gear_ratio;
            drivetrain_config.neutral_gear_ratio = rig.engine.neutral_gear_ratio;
            drivetrain_config.forward_gears = rig.engine.gear_ratios;
        }
        drivetrain.Configure(drivetrain_config);
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
        {
            for (std::size_t i = 0; i < nodes.size(); ++i)
            {
                nodes[i].mass = distributed.masses[i];
                // AddNode seeded gravity using temporary construction mass; refresh it.
                nodes[i].force = PhysicsVec3(0.0f, nodes[i].mass * DEFAULT_GRAVITY, 0.0f);
            }
        }

        if (wheels.empty())
'''
one(anchor,insert)
one('''        AlignTiresToGround();
        ConfigureWheelDirections();
''','''        AlignTiresToGround();
''')
one('''        const float authored_engine_torque = rig.engine.present ? std::fabs(rig.engine.torque) : 1200.0f;
        const float total_drive_torque = std::min(authored_engine_torque, 8000.0f) * throttle_state * 0.55f;
        const float per_driven_torque = driven_count > 0
            ? total_drive_torque / static_cast<float>(driven_count)
            : 0.0f;
        const float service_force = rig.brakes.present ? std::max(0.0f, rig.brakes.service_force) : 30000.0f;
''','''        float driven_wheel_rpm = 0.0f;
        if (driven_count > 0)
        {
            for (const WheelFixture& fixture : wheels)
            {
                if (fixture.propulsion != 0 && fixture.wheel.radius > 1.0e-6f)
                    driven_wheel_rpm += (fixture.wheel.speed / fixture.wheel.radius) * RAD_PER_SEC_TO_RPM;
            }
            driven_wheel_rpm /= static_cast<float>(driven_count);
        }
        drivetrain.Step(throttle_state, driven_wheel_rpm, dt);
        const float per_driven_torque = driven_count > 0
            ? drivetrain.OutputTorque() / static_cast<float>(driven_count)
            : 0.0f;
        const float service_force = rig.brakes.present ? std::max(0.0f, rig.brakes.service_force) : 30000.0f;
''')
one('''        if (!rig.globals.present)
            Warn("portable authored runtime is using fallback mass distribution");
        else
            Warn("portable authored runtime currently distributes globals dry mass uniformly across authored nodes");
''','''        if (!rig.globals.present)
            Warn("portable authored runtime is using fallback dry mass");
''')
p.write_text(s)
print('applied upstream-compatible RoR mass, wheel direction, topology, and drivetrain parity')