#!/usr/bin/env python3
"""Make ground contact eligibility match RoR node semantics.

Desktop RoR calls groundCollision() for every actor node unless that node carries
the `c` (NO_GROUND_CONTACT) option. `contacters` are a separate vehicle/contact
feature; they are not the list of nodes allowed to touch terrain.

The portable runtime previously collided only generated tyre nodes plus explicit
`contacters`, allowing ordinary suspension/chassis/axle nodes to pass through the
ground and feed unrealistic geometry/load states back into the tyres.

Generated wheel nodes default to terrain contact, as RoR InitNode does. Authored
nodes honor their parsed node/default options.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_ground_contact_node_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1])
s = p.read_text()


def one(old: str, new: str, label: str) -> None:
    global s
    if new in s:
        return
    count = s.count(old)
    if count != 1:
        raise SystemExit(f"ground-contact parity anchor '{label}' expected once, found {count}")
    s = s.replace(old, new, 1)


one(
'''    std::vector<std::size_t> tire_nodes;
    std::vector<std::size_t> contact_nodes;
    std::unordered_map<std::string, std::size_t> node_indices;
''',
'''    std::vector<std::size_t> tire_nodes;
    std::vector<std::size_t> contact_nodes;
    // RoR::CalcNodes() terrain-tests every node except OPTION_c_NO_GROUND_CONTACT.
    std::vector<std::size_t> ground_contact_nodes;
    std::unordered_map<std::string, std::size_t> node_indices;
''',
"ground-contact node list")

one(
'''    std::size_t AddNode(const PhysicsVec3& position, float mass)
    {
        nodes.emplace_back();
        NodeCoreState& node = nodes.back();
        node.position = position;
        node.mass = std::max(mass, 0.25f);
        node.force = PhysicsVec3(0.0f, node.mass * DEFAULT_GRAVITY, 0.0f);
        return nodes.size() - 1;
    }
''',
'''    std::size_t AddNode(const PhysicsVec3& position, float mass, bool ground_contact = true)
    {
        nodes.emplace_back();
        NodeCoreState& node = nodes.back();
        node.position = position;
        node.mass = std::max(mass, 0.25f);
        node.force = PhysicsVec3(0.0f, node.mass * DEFAULT_GRAVITY, 0.0f);
        const std::size_t index = nodes.size() - 1;
        if (ground_contact)
            ground_contact_nodes.push_back(index);
        return index;
    }
''',
"AddNode contact eligibility")

one(
'''        tire_nodes.clear();
        contact_nodes.clear();
        node_indices.clear();
''',
'''        tire_nodes.clear();
        contact_nodes.clear();
        ground_contact_nodes.clear();
        node_indices.clear();
''',
"clear ground-contact list")

one(
'''            const std::size_t index = AddNode(PhysicsVec3(source.x, source.y, source.z), 1.0f);
            nodes[index].friction_coef = source.friction;
            node_indices[source.id] = index;
''',
'''            // Upstream Node::OPTION_c_NO_GROUND_CONTACT is the only normal
            // opt-out from terrain collision. NodeDefaults options are already
            // folded into source.options by PortableRigDef.
            const bool can_touch_ground = source.options.find('c') == std::string::npos;
            const std::size_t index = AddNode(
                PhysicsVec3(source.x, source.y, source.z), 1.0f, can_touch_ground);
            nodes[index].friction_coef = source.friction;
            node_indices[source.id] = index;
''',
"authored node c option")

one(
'''        // Ground contact for generated tire nodes plus explicitly authored contacters.
        for (std::size_t index : tire_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
        for (std::size_t index : contact_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
''',
'''        // RoR::CalcNodes() checks terrain contact for every node unless the
        // node explicitly has the `c` no-ground-contact option. `contacters`
        // belong to actor/cab collision bookkeeping and do not gate terrain.
        for (std::size_t index : ground_contact_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
''',
"all-node ground collision")

p.write_text(s)
print('matched RoR all-node terrain collision with node option c opt-out')
