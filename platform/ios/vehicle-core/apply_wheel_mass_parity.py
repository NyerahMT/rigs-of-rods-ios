#!/usr/bin/env python3
"""Match upstream generated wheel node masses and wheel_t::wh_mass.

Desktop RoR assigns wheel node masses directly from the authored wheel mass and
ray count, without the portable core's old arbitrary 0.25/0.35 kg floors. After
Actor::recalculateNodeMasses(), ActorSpawner computes wheel_t::wh_mass by summing
exactly wheel.wh_nodes. For wheels2 that deliberately means the tyre nodes only;
rim nodes are separate generated nodes and do not participate in CalcWheels().
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_wheel_mass_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1]).resolve()
s = p.read_text()


def once(old: str, new: str, label: str) -> None:
    global s
    if new in s:
        return
    count = s.count(old)
    if count != 1:
        raise SystemExit(f"wheel mass parity anchor '{label}' expected once, found {count}")
    s = s.replace(old, new, 1)


# GetFreeNode()/InitNode() does not impose a minimum on an explicit wheel-node
# mass; ActorSpawner writes the authored wheel mass fraction directly afterward.
once(
    '        node.mass = std::max(mass, 0.25f);\n',
    '        node.mass = mass;\n',
    'AddNode arbitrary mass floor')

# BuildWheelObjectAndNodes(): two deformable tyre nodes per ray.
once(
    '        const float node_mass = std::max(0.25f, def.mass / static_cast<float>(rays * 2));\n',
    '        const float node_mass = def.mass / static_cast<float>(rays * 2);\n',
    'classic/meshwheel node mass')

# ProcessWheel2(): four generated nodes per ray (two rim + two tyre), each gets
# one quarter-ray share of authored mass. Only the tyre pair enters wh_nodes.
once(
    '        const float node_mass = std::max(0.35f, def.mass / static_cast<float>(rays * 4));\n',
    '        const float node_mass = def.mass / static_cast<float>(rays * 4);\n',
    'wheels2 generated node mass')

mass_tail = '''        if (distributed.masses.size() == nodes.size())
        {
            for (std::size_t i = 0; i < nodes.size(); ++i)
            {
                nodes[i].mass = distributed.masses[i];
                nodes[i].force = PhysicsVec3(0.0f, nodes[i].mass * DEFAULT_GRAVITY, 0.0f);
            }
        }

        if (wheels.empty())
'''
mass_tail_new = '''        if (distributed.masses.size() == nodes.size())
        {
            for (std::size_t i = 0; i < nodes.size(); ++i)
            {
                nodes[i].mass = distributed.masses[i];
                nodes[i].force = PhysicsVec3(0.0f, nodes[i].mass * DEFAULT_GRAVITY, 0.0f);
            }
        }

        // ActorSpawner finalization: calculate wheel_t::wh_mass only after the
        // node-mass pass, by summing wheel.wh_nodes. bindings.outer is the exact
        // portable equivalent of that set (tyre ring only for wheels2).
        for (WheelFixture& fixture : wheels)
        {
            const float wheel_node_mass = CalcWheelNodeMass(fixture.bindings);
            if (wheel_node_mass > 0.0f)
                fixture.wheel.rotational_mass = wheel_node_mass;
        }

        if (wheels.empty())
'''
once(mass_tail, mass_tail_new, 'post-recalculate wh_mass')

p.write_text(s)
print('matched upstream wheel node mass fractions and post-recalc wheel_t::wh_mass sum')
