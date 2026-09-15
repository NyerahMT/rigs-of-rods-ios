#!/usr/bin/env python3
"""Instrument AuthoredVehicleRuntime with early-step and spawn-ground diagnostics."""
from pathlib import Path
import sys
if len(sys.argv)!=2: raise SystemExit("usage: apply_early_step_debug.py <AuthoredVehicleRuntime.cpp>")
p=Path(sys.argv[1]); text=p.read_text()
if "AuthoredVehicleRuntime::SpawnDiagnostics() const" in text: print("diagnostics already present"); raise SystemExit(0)
def r(a,b,n):
 global text
 if text.count(a)!=1: raise SystemExit(f"anchor {n}: {text.count(a)}")
 text=text.replace(a,b,1)
r('    std::vector<std::string> warnings;\n    GroundContactParams road;\n','    std::vector<std::string> warnings;\n    std::vector<PhysicsVec3> debug_start_positions;\n    std::vector<EarlyStepDiagnostics> early_diagnostics;\n    SpawnGroundDiagnostics spawn_diagnostics;\n    GroundContactParams road;\n','storage')
r('        finite = true;\n        physics_steps = 0;\n\n        if (rig.nodes.empty())\n','        finite = true; physics_steps = 0; debug_start_positions.clear(); early_diagnostics.clear(); spawn_diagnostics=SpawnGroundDiagnostics();\n\n        if (rig.nodes.empty())\n','reset')
r('        AlignTiresToGround();\n        ConfigureWheelDirections();\n\n        road = GroundContactParams();\n','''        AlignTiresToGround(); ConfigureWheelDirections();
        debug_start_positions.reserve(nodes.size()); for(const NodeCoreState& n:nodes) debug_start_positions.push_back(n.position);
        if(!nodes.empty()){spawn_diagnostics.min_node_y=spawn_diagnostics.max_node_y=nodes[0].position.y;for(std::size_t i=0;i<nodes.size();++i){if(nodes[i].position.y<spawn_diagnostics.min_node_y){spawn_diagnostics.min_node_y=nodes[i].position.y;spawn_diagnostics.min_node=i;}spawn_diagnostics.max_node_y=std::max(spawn_diagnostics.max_node_y,nodes[i].position.y);}spawn_diagnostics.max_penetration=std::max(0.0f,-spawn_diagnostics.min_node_y);}
        if(!tire_nodes.empty()){spawn_diagnostics.min_tire=tire_nodes[0];spawn_diagnostics.min_tire_y=nodes[tire_nodes[0]].position.y;for(std::size_t i:tire_nodes)if(nodes[i].position.y<spawn_diagnostics.min_tire_y){spawn_diagnostics.min_tire_y=nodes[i].position.y;spawn_diagnostics.min_tire=i;}}
        if(nodes.size()>27) spawn_diagnostics.node27_position=nodes[27].position;
        spawn_diagnostics.node27_is_tire=std::find(tire_nodes.begin(),tire_nodes.end(),27)!=tire_nodes.end(); spawn_diagnostics.node27_is_contacter=std::find(contact_nodes.begin(),contact_nodes.end(),27)!=contact_nodes.end();

        road = GroundContactParams();
''','spawn')
r('        for (std::size_t index : tire_nodes)\n            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);\n        for (std::size_t index : contact_nodes)\n            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);\n','''        PhysicsVec3 n27_before; if(physics_steps==0 && nodes.size()>27)n27_before=nodes[27].force;
        for (std::size_t index : tire_nodes) ApplyFlatGroundContact(nodes[index],0.0f,road,dt);
        for (std::size_t index : contact_nodes) ApplyFlatGroundContact(nodes[index],0.0f,road,dt);
        if(physics_steps==0 && nodes.size()>27)spawn_diagnostics.node27_ground_force_step1=nodes[27].force-n27_before;
''','ground')
r('        for (BeamLink& link : beams)\n        {\n            ApplyBeamForce(nodes[link.a], nodes[link.b], link.beam);\n            if (!std::isfinite(link.beam.stress))\n','        std::size_t debug_worst_beam=kInvalidIndex; float debug_worst_stress=0; for(std::size_t beam_index=0;beam_index<beams.size();++beam_index){ BeamLink& link=beams[beam_index]; ApplyBeamForce(nodes[link.a],nodes[link.b],link.beam); if(std::isfinite(link.beam.stress)&&std::fabs(link.beam.stress)>std::fabs(debug_worst_stress)){debug_worst_stress=link.beam.stress;debug_worst_beam=beam_index;} if (!std::isfinite(link.beam.stress))\n','beam')
r('        ++physics_steps;\n    }\n};\n','''        ++physics_steps;
        if(physics_steps<=2){EarlyStepDiagnostics d;d.step=physics_steps;float worst=-1;for(std::size_t i=0;i<nodes.size()&&i<debug_start_positions.size();++i){float x=Distance(nodes[i].position,debug_start_positions[i]);if(x>worst){worst=x;d.worst_node=i;d.start_position=debug_start_positions[i];d.position=nodes[i].position;d.velocity=nodes[i].velocity;d.force=nodes[i].force;d.mass=nodes[i].mass;d.displacement=x;}}d.worst_beam=debug_worst_beam;if(debug_worst_beam!=kInvalidIndex&&debug_worst_beam<beams.size()){const BeamLink& b=beams[debug_worst_beam];d.beam_a=b.a;d.beam_b=b.b;d.beam_stress=b.beam.stress;d.beam_length=Distance(nodes[b.a].position,nodes[b.b].position);d.beam_rest_length=b.beam.rest_length;d.beam_spring=b.beam.spring;d.beam_damping=b.beam.damping;}early_diagnostics.push_back(d);}
    }
};
''','capture')
r('AuthoredVehicleTelemetry AuthoredVehicleRuntime::Telemetry() const\n','const std::vector<EarlyStepDiagnostics>& AuthoredVehicleRuntime::EarlyDiagnostics() const{return m_impl->early_diagnostics;}\nconst SpawnGroundDiagnostics& AuthoredVehicleRuntime::SpawnDiagnostics() const{return m_impl->spawn_diagnostics;}\n\nAuthoredVehicleTelemetry AuthoredVehicleRuntime::Telemetry() const\n','access')
p.write_text(text)
