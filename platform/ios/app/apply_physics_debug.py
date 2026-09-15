#!/usr/bin/env python3
from pathlib import Path
import sys
if len(sys.argv)!=2: raise SystemExit(2)
p=Path(sys.argv[1]);t=p.read_text()
def r(a,b):
 global t
 if t.count(a)!=1: raise SystemExit(f"anchor count {t.count(a)}")
 t=t.replace(a,b,1)
r('    std::string error;\n    std::vector<RoR::PhysicsVec3> nodes;\n    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;\n','    std::string error,early_debug; std::vector<RoR::PhysicsVec3> nodes; std::size_t first_bad_node=static_cast<std::size_t>(-1); float max_abs_node_position=0; RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;\n')
r('        s.nodes.reserve(runtime.NodeCount());\n        for (std::size_t i = 0; i < runtime.NodeCount(); ++i) s.nodes.push_back(runtime.Node(i).position);\n        std::lock_guard<std::mutex> guard(snapshot_mutex);\n','''        s.nodes.reserve(runtime.NodeCount());for(std::size_t i=0;i<runtime.NodeCount();++i){auto q=runtime.Node(i).position;s.nodes.push_back(q);if(s.first_bad_node==static_cast<std::size_t>(-1)&&(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)))s.first_bad_node=i;s.max_abs_node_position=std::max(s.max_abs_node_position,std::max(std::fabs(q.x),std::max(std::fabs(q.y),std::fabs(q.z))));}
        std::ostringstream d;const auto& g=runtime.SpawnDiagnostics();d<<"SPAWN Y="<<g.min_node_y<<".."<<g.max_node_y<<" PEN="<<g.max_penetration<<" N27Y="<<g.node27_position.y<<" GF="<<std::sqrt(g.node27_ground_force_step1.squaredLength())<<"\\n";
        d<<"N27 BEAMS:";for(const auto& b:runtime.Node27BeamDiagnostics())d<<" B"<<b.beam<<"->"<<b.other<<" L="<<b.initial_length<<"/"<<b.rest_length<<" F="<<b.initial_force<<" K="<<b.spring<<";";d<<"\\n";
        for(const auto& x:runtime.EarlyDiagnostics())d<<"S"<<x.step<<" N"<<x.worst_node<<" D="<<x.displacement<<" V="<<std::sqrt(x.velocity.squaredLength())<<" B"<<x.worst_beam<<"("<<x.beam_a<<"-"<<x.beam_b<<") ST="<<x.beam_stress<<" L="<<x.beam_length<<"/"<<x.beam_rest_length<<"\\n";s.early_debug=d.str();
        std::lock_guard<std::mutex> guard(snapshot_mutex);
''')
r('else if(!s.finite)_status.text=@"PHYSICS STOPPED • TAP RESET";','else if(!s.finite){NSString* bad=s.first_bad_node==static_cast<std::size_t>(-1)?@"NONE":[NSString stringWithFormat:@"%llu",(unsigned long long)s.first_bad_node];_status.numberOfLines=8;_status.adjustsFontSizeToFitWidth=YES;_status.minimumScaleFactor=0.55;_status.text=[NSString stringWithFormat:@"PHYSICS STOPPED • BAD NODE %@ • TAP RESET\\nNODES %llu • MAX %.2e • SPEED %.1f • STEP %llu\\n%s",bad,(unsigned long long)s.nodes.size(),s.max_abs_node_position,s.telemetry.speed_mps,(unsigned long long)s.telemetry.physics_steps,s.early_debug.c_str()];}')
p.write_text(t)
