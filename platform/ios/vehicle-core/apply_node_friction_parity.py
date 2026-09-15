#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_node_friction_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]


def one(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    if text.count(old) != 1:
        raise SystemExit(f"friction parity anchor '{label}' expected once, found {text.count(old)}")
    path.write_text(text.replace(old, new, 1))


header = root / "source/main/resources/rig_def_fileformat/PortableRigDef.h"
one(
    header,
    "float mass=0,rim_spring=0,rim_damping=0,tire_spring=0,tire_damping=0; char side='r'; std::string mesh_name,material_name;",
    "float mass=0,rim_spring=0,rim_damping=0,tire_spring=0,tire_damping=0; float node_friction=1.0f; char side='r'; std::string mesh_name,material_name;",
    "wheel node friction field")

parser = root / "source/main/resources/rig_def_fileformat/PortableRigDef.cpp"
text = parser.read_text()
needle = "                    document.wheels.push_back(wheel);\n"
replacement = "                    wheel.node_friction = document.node_defaults.friction;\n                    document.wheels.push_back(wheel);\n"
if replacement not in text:
    count = text.count(needle)
    if count != 3:
        raise SystemExit(f"wheel friction capture expected 3 wheel pushes, found {count}")
    text = text.replace(needle, replacement)
parser.write_text(text)

text = runtime.read_text()
# Authored nodes carry the active set_node_defaults coefficient already captured
# by PortableRigDef::Node.
old = '''            const std::size_t index = AddNode(PhysicsVec3(source.x, source.y, source.z), 1.0f);
            node_indices[source.id] = index;
'''
new = '''            const std::size_t index = AddNode(PhysicsVec3(source.x, source.y, source.z), 1.0f);
            nodes[index].friction_coef = source.friction;
            node_indices[source.id] = index;
'''
if new not in text:
    if text.count(old) != 1:
        raise SystemExit("authored node friction anchor drifted")
    text = text.replace(old, new, 1)

# Upstream InitNode(..., node_defaults) applies the same coefficient to every
# generated meshwheel tyre node. Front/rear Bandit values are 0.96/0.99.
old = '''            const std::size_t outer = AddNode(nodes[fixture.axis0].position + outer_radial, node_mass);
            const std::size_t inner = AddNode(nodes[fixture.axis1].position + inner_radial, node_mass);
            fixture.tire_nodes.push_back(outer);
'''
new = '''            const std::size_t outer = AddNode(nodes[fixture.axis0].position + outer_radial, node_mass);
            const std::size_t inner = AddNode(nodes[fixture.axis1].position + inner_radial, node_mass);
            nodes[outer].friction_coef = def.node_friction;
            nodes[inner].friction_coef = def.node_friction;
            fixture.tire_nodes.push_back(outer);
'''
if new not in text:
    if text.count(old) != 1:
        raise SystemExit("2-node wheel friction anchor drifted")
    text = text.replace(old, new, 1)

# wheels2 uses separate rim/tyre rings; preserve friction on both like InitNode.
old = '''                const std::size_t rim = AddNode(nodes[sides[side]].position + unit * def.rim_radius, node_mass);
                const std::size_t tire = AddNode(nodes[sides[side]].position + unit * def.tire_radius, node_mass);
                rim_nodes.push_back(rim);
'''
new = '''                const std::size_t rim = AddNode(nodes[sides[side]].position + unit * def.rim_radius, node_mass);
                const std::size_t tire = AddNode(nodes[sides[side]].position + unit * def.tire_radius, node_mass);
                nodes[rim].friction_coef = def.node_friction;
                nodes[tire].friction_coef = def.node_friction;
                rim_nodes.push_back(rim);
'''
if new not in text:
    if text.count(old) != 1:
        raise SystemExit("wheels2 friction anchor drifted")
    text = text.replace(old, new, 1)

runtime.write_text(text)
print('carried authored set_node_defaults friction into chassis and generated tyre nodes')
