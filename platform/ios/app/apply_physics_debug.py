#!/usr/bin/env python3
"""Inject temporary on-device diagnostics into OgreGameApp.mm before packaging."""
from pathlib import Path
import sys
if len(sys.argv)!=2: raise SystemExit("usage: apply_physics_debug.py <OgreGameApp.mm>")
path=Path(sys.argv[1]); text=path.read_text()
def r(old,new,label):
    global text
    n=text.count(old)
    if n!=1: raise SystemExit(f"physics debug anchor {label!r} expected once, found {n}")
    text=text.replace(old,new,1)
r('    std::string error;\n    std::vector<RoR::PhysicsVec3> nodes;\n    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;\n','    std::string error;\n    std::string early_debug;\n    std::vector<RoR::PhysicsVec3> nodes;\n    std::size_t first_bad_node = static_cast<std::size_t>(-1);\n    float max_abs_node_position = 0.0f;\n    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;\n','snapshot')
r('        s.nodes.reserve(runtime.NodeCount());\n        for (std::size_t i = 0; i < runtime.NodeCount(); ++i) s.nodes.push_back(runtime.Node(i).position);\n        std::lock_guard<std::mutex> guard(snapshot_mutex);\n','''        s.nodes.reserve(runtime.NodeCount());
        for (std::size_t i=0;i<runtime.NodeCount();++i){const RoR::PhysicsVec3 p=runtime.Node(i).position;s.nodes.push_back(p);if(s.first_bad_node==static_cast<std::size_t>(-1)&&(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)))s.first_bad_node=i;if(std::isfinite(p.x))s.max_abs_node_position=std::max(s.max_abs_node_position,std::fabs(p.x));if(std::isfinite(p.y))s.max_abs_node_position=std::max(s.max_abs_node_position,std::fabs(p.y));if(std::isfinite(p.z))s.max_abs_node_position=std::max(s.max_abs_node_position,std::fabs(p.z));}
        std::ostringstream dbg;
        for(const auto& d:runtime.EarlyDiagnostics()){
            dbg<<"S"<<d.step<<" N"<<d.worst_node<<" D="<<d.displacement<<" V="<<std::sqrt(d.velocity.squaredLength())<<" M="<<d.mass<<" B"<<d.worst_beam<<"("<<d.beam_a<<"-"<<d.beam_b<<") ST="<<d.beam_stress<<" L="<<d.beam_length<<"/"<<d.beam_rest_length<<" K="<<d.beam_spring<<" C="<<d.beam_damping<<"\\n";
        }
        s.early_debug=dbg.str();
        std::lock_guard<std::mutex> guard(snapshot_mutex);
''','publish')
r('else if(!s.finite)_status.text=@"PHYSICS STOPPED • TAP RESET";','''else if(!s.finite){
        NSString* bad=s.first_bad_node==static_cast<std::size_t>(-1)?@"NONE":[NSString stringWithFormat:@"%llu",(unsigned long long)s.first_bad_node];
        _status.numberOfLines=5;
        _status.text=[NSString stringWithFormat:@"PHYSICS STOPPED • BAD NODE %@ • TAP RESET\\nNODES %llu • MAX|POS| %.2e m • SPEED %.1f m/s • STEP %llu\\n%s",bad,(unsigned long long)s.nodes.size(),s.max_abs_node_position,s.telemetry.speed_mps,(unsigned long long)s.telemetry.physics_steps,s.early_debug.c_str()];
    }''','hud')
path.write_text(text); print(f"Injected physics diagnostics into {path}")
