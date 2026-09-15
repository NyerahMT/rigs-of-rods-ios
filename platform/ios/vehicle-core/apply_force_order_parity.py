#!/usr/bin/env python3
"""Match the portable iOS step ordering to RoR's Euler force pipeline.

Desktop RoR does *not* calculate a fresh set of beam/wheel forces and integrate
those forces in the same call. Actor::CalcForcesEulerCompute() begins with
CalcNodes(): groundCollision() augments the force accumulator left by the
previous physics pass, the node is integrated, and its accumulator is reset to
gravity. Only after that do CalcDifferentials(), CalcWheels(), CalcShocks(),
CalcHydros() and CalcBeams() populate the accumulator for the *next* 0.5 ms pass.

That one-step force phase is important here because primitiveCollision() reads
node->Forces when it decides static-vs-sliding friction. The previous portable
runtime either saw gravity only or, after an earlier parity attempt, fed contact
the just-computed wheel/beam forces and integrated them immediately. Neither is
RoR's solver and both can create an artificial contact/traction feedback loop.

This transform keeps ground contact at the beginning of StepInternal(), moves
node integration/reset directly behind it, and leaves wheel/suspension/beam
force accumulation at the end for the next step.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_force_order_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1])
s = p.read_text()

contact = '''        // Ground contact for generated tire nodes plus explicitly authored contacters.
        for (std::size_t index : tire_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
        for (std::size_t index : contact_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);

'''

integrate = '''        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, dt);
            if (!Finite(node.position) || !Finite(node.velocity) || node.velocity.squaredLength() > 1.0e12f)
            {
                finite = false;
                return;
            }
        }
'''

phased_front = contact + '''        // RoR::CalcNodes(): consume the force accumulator from the previous
        // force pass, then reset every node to gravity. The wheel, hydro and
        // beam code below therefore writes forces for the next physics pass.
''' + integrate + '''
'''

if phased_front not in s:
    if s.count(contact) != 1:
        raise SystemExit(f"Euler-phase contact anchor expected once, found {s.count(contact)}")
    if s.count(integrate) != 1:
        raise SystemExit(f"Euler-phase integration anchor expected once, found {s.count(integrate)}")
    s = s.replace(contact, phased_front, 1)
    # Remove the old end-of-step integration. Keep ++physics_steps at the end,
    # after force accumulation, matching one completed RoR physics pass.
    if s.count(integrate) != 2:
        raise SystemExit(f"Euler-phase integration duplication expected twice, found {s.count(integrate)}")
    # Delete the second occurrence only.
    first = s.find(integrate)
    second = s.find(integrate, first + len(integrate))
    s = s[:second] + s[second + len(integrate):]

p.write_text(s)
print('matched RoR CalcNodes -> wheels/shocks/hydros/beams one-step Euler force phasing')
