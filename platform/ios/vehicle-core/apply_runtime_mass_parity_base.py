#!/usr/bin/env python3
from pathlib import Path
import sys

p = Path(sys.argv[1])
s = p.read_text()

def one(old, new):
    global s
    if s.count(old) != 1:
        raise SystemExit('runtime parity anchor drifted: ' + old[:80])
    s = s.replace(old, new, 1)

one('#include "PortableRigDef.h"\n',
    '#include "PortableRigDef.h"\n#include "RoRMassDistribution.h"\n#include "RoRDrivetrain.h"\n')

one('''    GroundContactParams road;

    float steering = 0.0f;
''','''    GroundContactParams road;
    RoRDrivetrain drivetrain;

    float steering = 0.0f;
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
        if (rig.engoption.present)
        {
            // RigDef::Engoption is not cosmetic. The Bandit, for example,
            // authors 0.075 kg*m^2 inertia rather than Engine's 10.0 default.
            // Ignoring it makes throttle response more than two orders of
            // magnitude too slow before the clutch can transfer torque.
            drivetrain_config.engine_inertia = rig.engoption.inertia;
            if (rig.engoption.clutch_force >= 0.0f)
                drivetrain_config.clutch_force = rig.engoption.clutch_force;
            if (rig.engoption.shift_time > 0.0f)
                drivetrain_config.shift_time = rig.engoption.shift_time;
            if (rig.engoption.clutch_time > 0.0f)
                drivetrain_config.clutch_time = rig.engoption.clutch_time;
            if (rig.engoption.post_shift_time > 0.0f)
                drivetrain_config.post_shift_time = rig.engoption.post_shift_time;
        }
        drivetrain.Configure(drivetrain_config);
''')

one('AddNode(PhysicsVec3(source.x, source.y, source.z), authored_node_mass);',
    'AddNode(PhysicsVec3(source.x, source.y, source.z), 1.0f);')

anchor = '''        for (const PortableRigDef::Wheel& source : rig.wheels)
            BuildWheel(source);

        if (wheels.empty())
'''
insert = '''        for (const PortableRigDef::Wheel& source : rig.wheels)
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
            b.a = link.a;
            b.b = link.b;
            b.reference_length = link.beam.rest_length;
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
                nodes[i].force = PhysicsVec3(0.0f, nodes[i].mass * DEFAULT_GRAVITY, 0.0f);
            }
        }

        if (wheels.empty())
'''
one(anchor, insert)

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

# Publish the drivetrain state already being used for wheel torque. This avoids
# inventing a second RPM model in the iOS HUD/audio layer.
one('''    telemetry.throttle = m_impl->throttle_state;
    telemetry.handbrake = m_impl->handbrake;
    telemetry.physics_steps = m_impl->physics_steps;
''','''    telemetry.throttle = m_impl->throttle_state;
    telemetry.handbrake = m_impl->handbrake;
    telemetry.physics_steps = m_impl->physics_steps;
    telemetry.engine_rpm = m_impl->drivetrain.Telemetry().engine_rpm;
    telemetry.gear = m_impl->drivetrain.Telemetry().gear;
''')

# The workflow restores AuthoredVehicleRuntime.cpp from an older complete runtime
# snapshot. Provide the diagnostic API added after that snapshot so device failures
# still report the worst live node and beam.
one('''    return telemetry;
}

} // namespace IOSVehicleCore
} // namespace RoR
''','''    return telemetry;
}

const std::vector<EarlyStepDiagnostics>& AuthoredVehicleRuntime::EarlyDiagnostics() const
{
    static thread_local std::vector<EarlyStepDiagnostics> diagnostics;
    diagnostics.clear();
    if (!m_impl || m_impl->nodes.empty()) return diagnostics;

    EarlyStepDiagnostics d;
    d.step = m_impl->physics_steps;
    float worst_speed_sq = -1.0f;
    for (std::size_t i = 0; i < m_impl->nodes.size(); ++i)
    {
        const NodeCoreState& node = m_impl->nodes[i];
        const float speed_sq = node.velocity.squaredLength();
        if (speed_sq > worst_speed_sq || !std::isfinite(speed_sq))
        {
            worst_speed_sq = speed_sq;
            d.worst_node = i;
            d.start_position = node.position;
            d.position = node.position;
            d.velocity = node.velocity;
            d.force = node.force;
            d.mass = node.mass;
            d.displacement = 0.0f;
            if (!std::isfinite(speed_sq)) break;
        }
    }

    float worst_stress = -1.0f;
    for (std::size_t i = 0; i < m_impl->beams.size(); ++i)
    {
        const auto& link = m_impl->beams[i];
        const float magnitude = std::fabs(link.beam.stress);
        if (magnitude > worst_stress || !std::isfinite(magnitude))
        {
            worst_stress = magnitude;
            d.worst_beam = i;
            d.beam_a = link.a;
            d.beam_b = link.b;
            d.beam_stress = link.beam.stress;
            d.beam_length = Distance(m_impl->nodes[link.a].position, m_impl->nodes[link.b].position);
            d.beam_rest_length = link.beam.rest_length;
            d.beam_spring = link.beam.spring;
            d.beam_damping = link.beam.damping;
            if (!std::isfinite(magnitude)) break;
        }
    }
    diagnostics.push_back(d);
    return diagnostics;
}

const SpawnGroundDiagnostics& AuthoredVehicleRuntime::SpawnDiagnostics() const
{
    static thread_local SpawnGroundDiagnostics diagnostics;
    diagnostics = SpawnGroundDiagnostics();
    if (!m_impl || m_impl->nodes.empty()) return diagnostics;

    diagnostics.min_node_y = diagnostics.max_node_y = m_impl->nodes.front().position.y;
    diagnostics.min_node = 0;
    for (std::size_t i = 1; i < m_impl->nodes.size(); ++i)
    {
        const float y = m_impl->nodes[i].position.y;
        if (y < diagnostics.min_node_y)
        {
            diagnostics.min_node_y = y;
            diagnostics.min_node = i;
        }
        diagnostics.max_node_y = std::max(diagnostics.max_node_y, y);
    }

    bool have_tire = false;
    for (std::size_t i : m_impl->tire_nodes)
    {
        if (i >= m_impl->nodes.size()) continue;
        const float y = m_impl->nodes[i].position.y;
        if (!have_tire || y < diagnostics.min_tire_y)
        {
            diagnostics.min_tire_y = y;
            diagnostics.min_tire = i;
            have_tire = true;
        }
    }
    if (have_tire) diagnostics.max_penetration = std::max(0.0f, -diagnostics.min_tire_y);

    if (m_impl->nodes.size() > 27)
    {
        diagnostics.node27_position = m_impl->nodes[27].position;
        diagnostics.node27_is_tire = std::find(m_impl->tire_nodes.begin(), m_impl->tire_nodes.end(), 27) != m_impl->tire_nodes.end();
        diagnostics.node27_is_contacter = std::find(m_impl->contact_nodes.begin(), m_impl->contact_nodes.end(), 27) != m_impl->contact_nodes.end();
    }
    return diagnostics;
}

const std::vector<NodeBeamDiagnostic>& AuthoredVehicleRuntime::Node27BeamDiagnostics() const
{
    static thread_local std::vector<NodeBeamDiagnostic> diagnostics;
    diagnostics.clear();
    if (!m_impl || m_impl->nodes.size() <= 27) return diagnostics;

    for (std::size_t i = 0; i < m_impl->beams.size(); ++i)
    {
        const auto& link = m_impl->beams[i];
        if (link.a != 27 && link.b != 27) continue;
        NodeBeamDiagnostic d;
        d.beam = i;
        d.other = link.a == 27 ? link.b : link.a;
        d.initial_length = Distance(m_impl->nodes[link.a].position, m_impl->nodes[link.b].position);
        d.rest_length = link.beam.rest_length;
        d.spring = link.beam.spring;
        d.damping = link.beam.damping;
        d.initial_stress = link.beam.stress;
        d.initial_force = std::fabs(link.beam.stress);
        diagnostics.push_back(d);
    }
    return diagnostics;
}

} // namespace IOSVehicleCore
} // namespace RoR
''')

p.write_text(s)
print('applied upstream-compatible RoR mass, wheel direction, drivetrain, engoption, telemetry, and diagnostic parity')
