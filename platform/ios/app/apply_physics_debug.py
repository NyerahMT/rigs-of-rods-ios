#!/usr/bin/env python3
"""Inject temporary on-device diagnostics into OgreGameApp.mm before packaging."""

from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_physics_debug.py <OgreGameApp.mm>")

path = Path(sys.argv[1])
text = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"physics debug anchor '{label}' expected once, found {count}")
    text = text.replace(old, new, 1)


replace_once(
    '''    std::string error;
    std::vector<RoR::PhysicsVec3> nodes;
    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;
''',
    '''    std::string error;
    std::vector<RoR::PhysicsVec3> nodes;
    std::size_t first_bad_node = static_cast<std::size_t>(-1);
    float max_abs_node_position = 0.0f;
    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;
''',
    "snapshot diagnostics",
)

replace_once(
    '''        s.nodes.reserve(runtime.NodeCount());
        for (std::size_t i = 0; i < runtime.NodeCount(); ++i) s.nodes.push_back(runtime.Node(i).position);
        std::lock_guard<std::mutex> guard(snapshot_mutex);
''',
    '''        s.nodes.reserve(runtime.NodeCount());
        for (std::size_t i = 0; i < runtime.NodeCount(); ++i)
        {
            const RoR::PhysicsVec3 p = runtime.Node(i).position;
            s.nodes.push_back(p);
            if (s.first_bad_node == static_cast<std::size_t>(-1) &&
                (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)))
                s.first_bad_node = i;
            if (std::isfinite(p.x)) s.max_abs_node_position = std::max(s.max_abs_node_position, std::fabs(p.x));
            if (std::isfinite(p.y)) s.max_abs_node_position = std::max(s.max_abs_node_position, std::fabs(p.y));
            if (std::isfinite(p.z)) s.max_abs_node_position = std::max(s.max_abs_node_position, std::fabs(p.z));
        }
        std::lock_guard<std::mutex> guard(snapshot_mutex);
''',
    "publish node diagnostics",
)

replace_once(
    '''else if(!s.finite)_status.text=@"PHYSICS STOPPED • TAP RESET";''',
    '''else if(!s.finite){
        NSString* bad=s.first_bad_node==static_cast<std::size_t>(-1)?@"NONE":[NSString stringWithFormat:@"%llu",(unsigned long long)s.first_bad_node];
        _status.text=[NSString stringWithFormat:@"PHYSICS STOPPED • BAD NODE %@ • TAP RESET\\nNODES %llu • MAX|POS| %.2e m • SPEED %.1f m/s • STEP %llu",bad,(unsigned long long)s.nodes.size(),s.max_abs_node_position,s.telemetry.speed_mps,(unsigned long long)s.telemetry.physics_steps];
    }''',
    "non-finite HUD",
)

path.write_text(text)
print(f"Injected physics diagnostics into {path}")
