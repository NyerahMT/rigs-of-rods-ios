#!/usr/bin/env python3
"""Move portable ground contact to the same force phase as upstream RoR.

Upstream CalcForcesEulerCompute() starts with CalcNodes(); groundCollision() runs
there against the force accumulator produced by the previous force phase.  The
rest of that compute pass then runs wheels, shocks, hydros and beams to produce
the next accumulator.

The portable runtime previously called ground contact at the start of a step
while the accumulator contained gravity only, then added wheel/suspension/beam
forces and immediately integrated.  Because primitiveCollision's static/sliding
friction decision explicitly reads node->Forces, this made normal load and slip
force blind to the suspension and drive forces that actually load the tyre.

For our single-phase portable integrator the equivalent ordering is: accumulate
wheel/suspension/beam forces -> resolve ground contact from that complete force
state -> integrate/reset nodes.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_force_order_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1])
s = p.read_text()

old_front = '''        // Ground contact for generated tire nodes plus explicitly authored contacters.
        for (std::size_t index : tire_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
        for (std::size_t index : contact_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);

'''
new_front = '''        // Match RoR's force phasing: ground contact is resolved only after the
        // wheel/suspension/beam force accumulator for this integration step is complete.

'''
if new_front not in s:
    if s.count(old_front) != 1:
        raise SystemExit(f"force-order front contact anchor expected once, found {s.count(old_front)}")
    s = s.replace(old_front, new_front, 1)

old_integrate = '''        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, dt);
'''
new_integrate = '''        // primitiveCollision() uses the complete node force vector to derive
        // normal reaction and the tangential force that static friction must
        // cancel. Resolve contact here, immediately before node integration.
        for (std::size_t index : tire_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
        for (std::size_t index : contact_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);

        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, dt);
'''
if new_integrate not in s:
    if s.count(old_integrate) != 1:
        raise SystemExit(f"force-order integration anchor expected once, found {s.count(old_integrate)}")
    s = s.replace(old_integrate, new_integrate, 1)

p.write_text(s)
print('moved RoR ground contact after wheel/suspension/beam force accumulation and before integration')
