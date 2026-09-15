#!/usr/bin/env python3
"""Instrument AuthoredVehicleRuntime with step 0-2 node/beam diagnostics."""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_early_step_debug.py <AuthoredVehicleRuntime.cpp>")
p = Path(sys.argv[1]); text = p.read_text()

def r(old, new, label):
    global text
    n=text.count(old)
    if n != 1: raise SystemExit(f"early-step debug anchor {label!r}: expected 1, found {n}")
    text=text.replace(old,new,1)

r('''    std::vector<std::string> warnings;\n    GroundContactParams road;\n''','''    std::vector<std::string> warnings;\n    std::vector<PhysicsVec3> debug_start_positions;\n    std::vector<EarlyStepDiagnostics> early_diagnostics;\n    GroundContactParams road;\n''','storage')

r('''        finite = true;\n        physics_steps = 0;\n\n        if (rig.nodes.empty())\n''','''        finite = true;\n        physics_steps = 0;\n        debug_start_positions.clear();\n        early_diagnostics.clear();\n\n        if (rig.nodes.empty())\n''','reset')

r('''        AlignTiresToGround();\n        ConfigureWheelDirections();\n\n        road = GroundContactParams();\n''','''        AlignTiresToGround();\n        ConfigureWheelDirections();\n        debug_start_positions.reserve(nodes.size());\n        for (const NodeCoreState& node : nodes) debug_start_positions.push_back(node.position);\n\n        road = GroundContactParams();\n''','initial positions')

r('''        for (BeamLink& link : beams)\n        {\n            ApplyBeamForce(nodes[link.a], nodes[link.b], link.beam);\n            if (!std::isfinite(link.beam.stress))\n''','''        std::size_t debug_worst_beam = kInvalidIndex;\n        float debug_worst_stress = 0.0f;\n        for (std::size_t beam_index = 0; beam_index < beams.size(); ++beam_index)\n        {\n            BeamLink& link = beams[beam_index];\n            ApplyBeamForce(nodes[link.a], nodes[link.b], link.beam);\n            if (std::isfinite(link.beam.stress) && std::fabs(link.beam.stress) > std::fabs(debug_worst_stress))\n            {\n                debug_worst_stress = link.beam.stress;\n                debug_worst_beam = beam_index;\n            }\n            if (!std::isfinite(link.beam.stress))\n''','beam scan')

r('''        ++physics_steps;\n    }\n};\n''','''        ++physics_steps;\n        if (physics_steps <= 2)\n        {\n            EarlyStepDiagnostics d;\n            d.step = physics_steps;\n            float worst_displacement = -1.0f;\n            for (std::size_t i = 0; i < nodes.size() && i < debug_start_positions.size(); ++i)\n            {\n                const float displacement = Distance(nodes[i].position, debug_start_positions[i]);\n                if (displacement > worst_displacement)\n                {\n                    worst_displacement = displacement; d.worst_node = i;\n                    d.start_position = debug_start_positions[i]; d.position = nodes[i].position;\n                    d.velocity = nodes[i].velocity; d.force = nodes[i].force; d.mass = nodes[i].mass;\n                    d.displacement = displacement;\n                }\n            }\n            d.worst_beam = debug_worst_beam;\n            if (debug_worst_beam != kInvalidIndex && debug_worst_beam < beams.size())\n            {\n                const BeamLink& b = beams[debug_worst_beam];\n                d.beam_a=b.a; d.beam_b=b.b; d.beam_stress=b.beam.stress;\n                d.beam_length=Distance(nodes[b.a].position,nodes[b.b].position);\n                d.beam_rest_length=b.beam.rest_length; d.beam_spring=b.beam.spring; d.beam_damping=b.beam.damping;\n            }\n            early_diagnostics.push_back(d);\n        }\n    }\n};\n''','capture')

r('''AuthoredVehicleTelemetry AuthoredVehicleRuntime::Telemetry() const\n''','''const std::vector<EarlyStepDiagnostics>& AuthoredVehicleRuntime::EarlyDiagnostics() const\n{\n    return m_impl->early_diagnostics;\n}\n\nAuthoredVehicleTelemetry AuthoredVehicleRuntime::Telemetry() const\n''','accessor')

p.write_text(text)
print(f"Instrumented early physics steps in {p}")
