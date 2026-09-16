#!/usr/bin/env python3
"""Replace the portable wheels2 approximation with ActorSpawner::ProcessWheel2().

Preserves upstream node geometry/masses/friction and the exact 10 rim + 14 tyre
beams per ray (+1 virtual rigidity beam when authored). The tyre ring is offset
by half a ray from the rim ring, matching desktop RoR.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_wheels2_topology_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1]).resolve()
s = p.read_text()

start = '    void BuildWheel2(const PortableRigDef::Wheel& def, WheelFixture& fixture)\n'
end = '    void BuildWheel(const PortableRigDef::Wheel& def)\n'
a = s.find(start)
b = s.find(end, a + len(start)) if a >= 0 else -1
if a < 0 or b < 0:
    raise SystemExit('wheels2 topology range drifted')

replacement = r'''    void BuildWheel2(const PortableRigDef::Wheel& def, WheelFixture& fixture)
    {
        const int rays = def.num_rays;
        const PhysicsVec3 raw_axis = nodes[fixture.axis1].position - nodes[fixture.axis0].position;
        const float wheel_width = Length(raw_axis);
        const PhysicsVec3 axis = Normalized(raw_axis);
        if (axis.squaredLength() < 0.5f || rays < 3)
        {
            Error("wheels2 has invalid axis or ray count");
            return;
        }

        const auto rotate_around_axis = [](const PhysicsVec3& value, const PhysicsVec3& unit_axis, float angle)
        {
            const float c = std::cos(angle);
            const float si = std::sin(angle);
            return value * c + unit_axis.cross(value) * si +
                unit_axis * (unit_axis.dot(value) * (1.0f - c));
        };

        const float full_step = -2.0f * kPi / static_cast<float>(rays);
        const float half_step = -kPi / static_cast<float>(rays);
        const float rim_node_mass = def.mass / (4.0f * static_cast<float>(rays));
        const float tyre_outer_mass = (0.67f * def.mass) / (2.0f * static_cast<float>(rays));
        const float tyre_inner_mass = (0.33f * def.mass) / (2.0f * static_cast<float>(rays));
        const float rim_spring = def.rim_spring;
        const float rim_damping = def.rim_damping;
        const float tyre_spring = def.tire_spring;
        const float tyre_damping = def.tire_damping;

        std::vector<std::size_t> rim_nodes;
        rim_nodes.reserve(static_cast<std::size_t>(rays * 2));
        fixture.tire_nodes.reserve(static_cast<std::size_t>(rays * 2));
        fixture.bindings.reserve(static_cast<std::size_t>(rays * 2));

        // ActorSpawner::ProcessWheel2(): rim starts at world +Y and advances one
        // full ray at a time around the axle. Both sides of a ray share angle.
        PhysicsVec3 rim_ray(0.0f, def.rim_radius, 0.0f);
        for (int ray = 0; ray < rays; ++ray)
        {
            const std::size_t outer = AddNode(nodes[fixture.axis0].position + rim_ray, rim_node_mass);
            const std::size_t inner = AddNode(nodes[fixture.axis1].position + rim_ray, rim_node_mass);
            nodes[outer].friction_coef = def.node_friction;
            nodes[inner].friction_coef = def.node_friction;
            rim_nodes.push_back(outer);
            rim_nodes.push_back(inner);
            rim_ray = rotate_around_axis(rim_ray, axis, full_step);
        }

        // Tyre ring starts half a ray out of phase with the rim. Upstream gives
        // the axis0/outer tyre nodes 67% and axis1/inner nodes 33% of tyre mass.
        PhysicsVec3 tyre_ray = rotate_around_axis(
            PhysicsVec3(0.0f, def.tire_radius, 0.0f), axis, half_step);
        const float tyre_friction = wheel_width * WHEEL_FRICTION_COEF;
        for (int ray = 0; ray < rays; ++ray)
        {
            const std::size_t outer = AddNode(nodes[fixture.axis0].position + tyre_ray, tyre_outer_mass);
            const std::size_t inner = AddNode(nodes[fixture.axis1].position + tyre_ray, tyre_inner_mass);
            nodes[outer].friction_coef = tyre_friction;
            nodes[inner].friction_coef = tyre_friction;
            fixture.tire_nodes.push_back(outer);
            fixture.tire_nodes.push_back(inner);
            tire_nodes.push_back(outer);
            tire_nodes.push_back(inner);

            WheelNodeBinding outer_binding;
            outer_binding.outer = &nodes[outer];
            outer_binding.inner = &nodes[fixture.axis0];
            fixture.bindings.push_back(outer_binding);
            WheelNodeBinding inner_binding;
            inner_binding.outer = &nodes[inner];
            inner_binding.inner = &nodes[fixture.axis1];
            fixture.bindings.push_back(inner_binding);

            tyre_ray = rotate_around_axis(tyre_ray, axis, full_step);
        }

        const std::size_t rigidity = ResolveNode(def.rigidity_node);
        const bool rigidity_side0 = rigidity != kInvalidIndex &&
            Distance(nodes[rigidity].position, nodes[fixture.axis0].position) <
            Distance(nodes[rigidity].position, nodes[fixture.axis1].position);

        for (int ray = 0; ray < rays; ++ray)
        {
            const int next = (ray + 1) % rays;
            const std::size_t r0 = rim_nodes[static_cast<std::size_t>(ray * 2)];
            const std::size_t r1 = rim_nodes[static_cast<std::size_t>(ray * 2 + 1)];
            const std::size_t rn0 = rim_nodes[static_cast<std::size_t>(next * 2)];
            const std::size_t rn1 = rim_nodes[static_cast<std::size_t>(next * 2 + 1)];
            const std::size_t t0 = fixture.tire_nodes[static_cast<std::size_t>(ray * 2)];
            const std::size_t t1 = fixture.tire_nodes[static_cast<std::size_t>(ray * 2 + 1)];
            const std::size_t tn0 = fixture.tire_nodes[static_cast<std::size_t>(next * 2)];
            const std::size_t tn1 = fixture.tire_nodes[static_cast<std::size_t>(next * 2 + 1)];

            // Rim: exact ten AddWheelRimBeam() calls per ray. The first outer
            // axis beam is intentionally duplicated later; this is upstream.
            const std::size_t bounded_outer = AddBeam(fixture.axis0, r0, rim_spring, rim_damping, BeamKind::Wheel);
            const std::size_t bounded_inner = AddBeam(fixture.axis1, r1, rim_spring, rim_damping, BeamKind::Wheel);
            if (bounded_outer != kInvalidIndex) beams[bounded_outer].beam.shortbound = 0.66f;
            if (bounded_inner != kInvalidIndex) beams[bounded_inner].beam.shortbound = 0.66f;
            AddBeam(fixture.axis1, r0, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(fixture.axis0, r1, rim_spring, rim_damping, BeamKind::Wheel);

            AddBeam(fixture.axis0, r0, rim_spring, rim_damping, BeamKind::Wheel); // historical duplicate
            AddBeam(r0, r1, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r0, rn0, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r1, rn1, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r0, rn1, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r1, rn0, rim_spring, rim_damping, BeamKind::Wheel);

            if (rigidity != kInvalidIndex)
            {
                AddBeam(rigidity, rigidity_side0 ? r0 : r1,
                        rim_spring, rim_damping, BeamKind::WheelVirtual);
            }

            // Tyre band: four triangulated links to next ray.
            AddBeam(t0, tn0, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t0, tn1, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t1, tn0, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t1, tn1, tyre_spring, tyre_damping, BeamKind::Wheel);

            // Sidewalls: same-side current and next rim nodes.
            AddBeam(t0, r0, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t0, rn0, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t1, r1, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t1, rn1, tyre_spring, tyre_damping, BeamKind::Wheel);

            // Reinforcement: cross-side current and next rim nodes.
            AddBeam(t0, r1, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t0, rn1, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t1, r0, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(t1, rn0, tyre_spring, tyre_damping, BeamKind::Wheel);

            // Backpressure spokes. Despite the historical source comment saying
            // "bounded", AddTyreBeam() creates ordinary BEAM_NORMAL links here.
            AddBeam(fixture.axis0, t0, tyre_spring, tyre_damping, BeamKind::Wheel);
            AddBeam(fixture.axis1, t1, tyre_spring, tyre_damping, BeamKind::Wheel);
        }
    }
'''

s = s[:a] + replacement + '\n' + s[b:]
p.write_text(s)
print('replaced wheels2 with upstream half-ray geometry, 67/33 tyre mass, width friction and 24/25-beam topology')
