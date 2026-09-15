#!/usr/bin/env python3
"""Inject temporary on-device diagnostics into OgreGameApp.mm before packaging."""
from pathlib import Path
import sys
if len(sys.argv)!=2: raise SystemExit("usage: apply_physics_debug.py <OgreGameApp.mm>")
path=Path(sys.argv[1]); text=path.read_text()
def r(a,b,n):
 global text
 if text.count(a)!=1: raise SystemExit(f"anchor {n}: {text.count(a)}")
 text=text.replace(a,b,1)
r('    std::string error;\n    std::vector<RoR::PhysicsVec3> nodes;\n    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;\n','    std::string error; std::string early_debug;\n    std::vector<RoR::PhysicsVec3> nodes; std::size_t first_bad_node=static_cast<std::size_t>(-1); float max_abs_node_position=0; RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;\n','snapshot')
r('        s.nodes.reserve(runtime.NodeCount());\n        for (std::size_t i = 0; i < runtime.NodeCount(); ++i) s.nodes.push_back(runtime.Node(i).position);\n        std::lock_guard<std::mutex> guard(snapshot_mutex);\n','''        s.nodes.reserve(runtime.NodeCount()); for(std::size_t i=0;i<runtime.NodeCount();++i){auto p=runtime.Node(i).position;s.nodes.push_back(p);if(s.first_bad_node==static_cast<std::size_t>(-1)&&(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)))s.first_bad_node=i;s.max_abs_node_position=std::max(s.max_abs_node_position,std::max(std::fabs(p.x),std::max(std::fabs(p.y),std::fabs(p.z))));}
        std::ostringstream dbg; const auto& g=runtime.SpawnDiagnostics(); dbg<<"SPAWN Y="<<g.min_node_y<<".."<<g.max_node_y<<" TIREMIN="<<g.min_tire_y<<" PEN="<<g.max_penetration<<" N27=("<<g.node27_position.x<<","<<g.node27_position.y<<","<<g.node27_position.z<<") T="<<g.node27_is_tire<<" C="<<g.node27_is_contacter<<" GF="<<std::sqrt(g.node27_ground_force_step1.squaredLength())<<"\\n"; for(const auto& d:runtime.EarlyDiagnostics())dbg<<"S"<<d.step<<" N"<<d.worst_node<<" D="<<d.displacement<<" V="<<std::sqrt(d.velocity.squaredLength())<<" M="<<d.mass<<" B"<<d.worst_beam<<"("<<d.beam_a<<"-"<<d.beam_b<<") ST="<<d.beam_stress<<" L="<<d.beam_length<<"/"<<d.beam_rest_length<<" K="<<d.beam_spring<<" C="<<d.beam_damping<<"\\n"; s.early_debug=dbg.str();
        std::lock_guard<std::mutex> guard(snapshot_mutex);
''','publish')
r('else if(!s.finite)_status.text=@"PHYSICS STOPPED • TAP RESET";','''else if(!s.finite){NSString* bad=s.first_bad_node==static_cast<std::size_t>(-1)?@"NONE":[NSString stringWithFormat:@"%llu",(unsigned long long)s.first_bad_node];_status.numberOfLines=6;_status.text=[NSString stringWithFormat:@"PHYSICS STOPPED • BAD NODE %@ • TAP RESET\\nNODES %llu • MAX|POS| %.2e m • SPEED %.1f m/s • STEP %llu\\n%s",bad,(unsigned long long)s.nodes.size(),s.max_abs_node_position,s.telemetry.speed_mps,(unsigned long long)s.telemetry.physics_steps,s.early_debug.c_str()];}''','hud')
path.write_text(text)
