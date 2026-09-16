#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_wheel_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
s = runtime.read_text()
root = runtime.parents[3]


def one(old: str, new: str, label: str) -> None:
    global s
    if s.count(old) != 1:
        raise SystemExit(f"wheel parity anchor '{label}' expected once, found {s.count(old)}")
    s = s.replace(old, new, 1)


def between(start: str, end: str, replacement: str, label: str) -> None:
    global s
    a = s.find(start)
    b = s.find(end, a + len(start)) if a >= 0 else -1
    if a < 0 or b < 0:
        raise SystemExit(f"wheel parity range '{label}' drifted")
    s = s[:a] + replacement + "\n\n" + s[b:]


one(
'''    enum class BeamKind
    {
        Normal,
        Shock,
        Hydro,
        Wheel
    };
''',
'''    enum class BeamKind
    {
        Normal,
        Shock,
        Hydro,
        Wheel,
        WheelVirtual
    };
''',
"virtual wheel beam kind")

# Add the wheel-specific SHOCK1 bounds used by ActorSpawner::BuildWheelBeams().
insert_after_addbeam = '''        beam_pairs.emplace_back(a, b);
        return beams.size() - 1;
    }
'''
add_wheel_helper = '''        beam_pairs.emplace_back(a, b);
        return beams.size() - 1;
    }

    std::size_t AddGeneratedWheelBeam(
        std::size_t a,
        std::size_t b,
        float spring,
        float damping,
        bool bounded = false,
        float max_extension = 0.0f,
        BeamKind kind = BeamKind::Wheel)
    {
        const std::size_t index = AddBeam(a, b, spring, damping, kind);
        if (index != kInvalidIndex && bounded)
        {
            // ActorSpawner::AddWheelBeam(..., 0.66f, max_extension)
            // marks these generated spokes as SpecialBeam::SHOCK1.
            beams[index].beam.bounded = true;
            beams[index].beam.shortbound = 0.66f;
            beams[index].beam.longbound = max_extension;
        }
        return index;
    }
'''
one(insert_after_addbeam, add_wheel_helper, "generated wheel beam helper")

# Match ActorSpawner::BuildWheelObjectAndNodes() + BuildWheelBeams() for the
# 2-node-per-ray wheel family used by wheels, meshwheels and meshwheels2.
# The important details are the half-ray staggering between outer/inner nodes,
# the four axle spokes, the four reinforcement beams and SHOCK1 spoke bounds.
plain_wheel = r'''    void BuildPlainWheel(const PortableRigDef::Wheel& def, WheelFixture& fixture)
    {
        const int rays = std::max(3, def.num_rays);
        const PhysicsVec3 axis = Normalized(nodes[fixture.axis1].position - nodes[fixture.axis0].position);
        if (axis.squaredLength() < 0.5f)
        {
            Error("wheel has coincident axis nodes");
            return;
        }

        PhysicsVec3 radial0 = PhysicsVec3(0.0f, 1.0f, 0.0f) - axis * axis.y;
        if (radial0.squaredLength() < 1.0e-5f)
            radial0 = PhysicsVec3(1.0f, 0.0f, 0.0f) - axis * axis.x;
        radial0 = Normalized(radial0);
        const PhysicsVec3 radial1 = Normalized(axis.cross(radial0));

        const float node_mass = std::max(0.25f, def.mass / static_cast<float>(rays * 2));
        const float tyre_spring = def.tire_spring > 0.0f ? def.tire_spring : 200000.0f;
        const float tyre_damping = def.tire_damping > 0.0f ? def.tire_damping : 2500.0f;
        const float rim_spring = def.rim_spring > 0.0f ? def.rim_spring : tyre_spring;
        const float rim_damping = def.rim_damping > 0.0f ? def.rim_damping : tyre_damping;

        fixture.tire_nodes.reserve(static_cast<std::size_t>(rays * 2));
        fixture.bindings.reserve(static_cast<std::size_t>(rays * 2));

        // Upstream rotates the ray after every generated node, not after every
        // pair. That makes the inner ring half a ray out of phase with the outer
        // ring and is essential to the triangulated wheel structure.
        const float half_step = kPi / static_cast<float>(rays);
        for (int ray = 0; ray < rays; ++ray)
        {
            const float outer_angle = half_step * static_cast<float>(ray * 2);
            const float inner_angle = half_step * static_cast<float>(ray * 2 + 1);
            const PhysicsVec3 outer_radial =
                radial0 * (def.tire_radius * std::cos(outer_angle)) +
                radial1 * (def.tire_radius * std::sin(outer_angle));
            const PhysicsVec3 inner_radial =
                radial0 * (def.tire_radius * std::cos(inner_angle)) +
                radial1 * (def.tire_radius * std::sin(inner_angle));

            const std::size_t outer = AddNode(nodes[fixture.axis0].position + outer_radial, node_mass);
            const std::size_t inner = AddNode(nodes[fixture.axis1].position + inner_radial, node_mass);
            fixture.tire_nodes.push_back(outer);
            fixture.tire_nodes.push_back(inner);
            tire_nodes.push_back(outer);
            tire_nodes.push_back(inner);

            WheelNodeBinding outer_binding;
            outer_binding.outer = &nodes[outer];
            outer_binding.inner = &nodes[fixture.axis0];
            fixture.bindings.push_back(outer_binding);
            WheelNodeBinding inner_binding;
            inner_binding.outer = &nodes[inner];
            inner_binding.inner = &nodes[fixture.axis1];
            fixture.bindings.push_back(inner_binding);
        }

        // meshwheels2 is the only parsed 2-node wheel format with a separate rim
        // radius in the current portable RigDef model. Upstream gives its bounded
        // spokes 15% extension; classic wheels/meshwheels use the default 0.
        const float spoke_max_extension = (!def.wheels2 && def.rim_radius > 0.0f) ? 0.15f : 0.0f;

        const std::string normalized_rigidity = NormalizeLegacyNodeRef(def.rigidity_node);
        const std::size_t rigidity = ResolveNode(def.rigidity_node);
        const bool rigidity_side0 = rigidity != kInvalidIndex &&
            Distance(nodes[rigidity].position, nodes[fixture.axis0].position) <
            Distance(nodes[rigidity].position, nodes[fixture.axis1].position);

        for (int ray = 0; ray < rays; ++ray)
        {
            const int next = (ray + 1) % rays;
            const std::size_t outer = fixture.tire_nodes[static_cast<std::size_t>(ray * 2)];
            const std::size_t inner = fixture.tire_nodes[static_cast<std::size_t>(ray * 2 + 1)];
            const std::size_t next_outer = fixture.tire_nodes[static_cast<std::size_t>(next * 2)];
            const std::size_t next_inner = fixture.tire_nodes[static_cast<std::size_t>(next * 2 + 1)];

            AddGeneratedWheelBeam(fixture.axis0, outer, tyre_spring, tyre_damping, true, spoke_max_extension);
            AddGeneratedWheelBeam(fixture.axis1, inner, tyre_spring, tyre_damping, true, spoke_max_extension);
            AddGeneratedWheelBeam(fixture.axis1, outer, tyre_spring, tyre_damping);
            AddGeneratedWheelBeam(fixture.axis0, inner, tyre_spring, tyre_damping);

            AddGeneratedWheelBeam(outer, inner, rim_spring, rim_damping);
            AddGeneratedWheelBeam(outer, next_outer, rim_spring, rim_damping);
            AddGeneratedWheelBeam(inner, next_inner, rim_spring, rim_damping);
            AddGeneratedWheelBeam(inner, next_outer, rim_spring, rim_damping);

            if (rigidity != kInvalidIndex)
            {
                AddGeneratedWheelBeam(
                    rigidity,
                    rigidity_side0 ? outer : inner,
                    tyre_spring,
                    tyre_damping,
                    false,
                    0.0f,
                    BeamKind::WheelVirtual);
            }
        }

        if (normalized_rigidity != def.rigidity_node && def.rigidity_node != "-1")
            Warn("normalized legacy signed wheel rigidity-node reference " + def.rigidity_node + " -> " + normalized_rigidity);
    }
'''
between('    void BuildPlainWheel(const PortableRigDef::Wheel& def, WheelFixture& fixture)\n',
        '    void BuildWheel2(const PortableRigDef::Wheel& def, WheelFixture& fixture)\n',
        plain_wheel,
        "2-node generated wheel topology")

# Upstream GetWheelAxisNodes() always orders the axle pair by Z before generating
# the wheel. Preserve that deterministic handedness so steering and rim orientation
# do not depend on how a legacy truck happened to list its two axle nodes.
old_axis = '''        const std::size_t axis0 = ResolveNode(def.axis_node_0, false);
        const std::size_t axis1 = ResolveNode(def.axis_node_1, false);
        if (axis0 == kInvalidIndex || axis1 == kInvalidIndex)
'''
new_axis = '''        std::size_t axis0 = ResolveNode(def.axis_node_0, false);
        std::size_t axis1 = ResolveNode(def.axis_node_1, false);
        if (axis0 == kInvalidIndex || axis1 == kInvalidIndex)
'''
one(old_axis, new_axis, "mutable wheel axis refs")
axis_guard = '''        if (def.tire_radius <= 0.0f || def.num_rays < 3)
        {
            Error("wheel has invalid radius or ray count");
            return;
        }

        WheelFixture fixture;
'''
axis_guard_new = '''        if (def.tire_radius <= 0.0f || def.num_rays < 3)
        {
            Error("wheel has invalid radius or ray count");
            return;
        }

        if (nodes[axis0].position.z > nodes[axis1].position.z)
            std::swap(axis0, axis1);

        WheelFixture fixture;
'''
one(axis_guard, axis_guard_new, "GetWheelAxisNodes Z ordering")

# Virtual rigidity beams must not participate in dry-mass beam-length weighting.
one('            b.virtual_beam = false;\n',
    '            b.virtual_beam = (link.kind == BeamKind::WheelVirtual);\n',
    "virtual rigidity beam mass exclusion")

runtime.write_text(s)

# The build generates a vehicle-specific Objective-C++ controller after the base
# audio/flexbody transform. Chain the meshwheel renderer first, then the shared
# 60 FPS render bridge so interpolation also covers the generated wheel visuals.
build_ipa = root / "platform/ios/app/build-ipa.sh"
b = build_ipa.read_text()
needle = '''python3 "$ROOT/platform/ios/app/prepare_audio_game_source.py" \\
    "$ROOT/platform/ios/app/OgreGameApp.mm" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_render_interpolation.py" "$AUDIO_GAME_SRC"
grep -q 'kSnapshotPublishHz = 120.0' "$AUDIO_GAME_SRC"
grep -q 'flexbody_node_cache' "$AUDIO_GAME_SRC"

"$CXX" \\
'''
replacement = '''python3 "$ROOT/platform/ios/app/prepare_audio_game_source.py" \\
    "$ROOT/platform/ios/app/OgreGameApp.mm" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_meshwheel_renderer.py" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_render_interpolation.py" "$AUDIO_GAME_SRC"
grep -q 'kSnapshotPublishHz = 120.0' "$AUDIO_GAME_SRC"
grep -q 'flexbody_node_cache' "$AUDIO_GAME_SRC"

"$CXX" \\
'''
if replacement not in b:
    if b.count(needle) != 1:
        raise SystemExit("build-ipa meshwheel/render transform anchor drifted")
    b = b.replace(needle, replacement, 1)
build_ipa.write_text(b)

# Register a construction-level regression probe. It checks the exact 2*rays node
# count, 8*rays beam topology, half-step staggering and the bounded-beam equation.
cmake = root / "platform/ios/vehicle-core/CMakeLists.txt"
c = cmake.read_text()
probe_marker = 'add_executable(ror_meshwheel_parity_probe tests/MeshWheelParityProbe.cpp)'
if probe_marker not in c:
    anchor = '    add_executable(ror_authored_vehicle_smoke tests/AuthoredVehicleSmokeProbe.cpp)\n'
    if c.count(anchor) != 1:
        raise SystemExit("CMake meshwheel probe anchor drifted")
    c = c.replace(anchor, anchor + '    add_executable(ror_meshwheel_parity_probe tests/MeshWheelParityProbe.cpp)\n', 1)
    tail = '''    target_link_libraries(ror_authored_vehicle_smoke PRIVATE ror_vehicle_core)
    target_compile_features(ror_authored_vehicle_smoke PRIVATE cxx_std_11)
'''
    repl = tail + '''    target_link_libraries(ror_meshwheel_parity_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_meshwheel_parity_probe PRIVATE cxx_std_11)
    add_test(NAME ror_meshwheel_parity_probe COMMAND ror_meshwheel_parity_probe)
'''
    if c.count(tail) != 1:
        raise SystemExit("CMake meshwheel probe tail drifted")
    c = c.replace(tail, repl, 1)
cmake.write_text(c)

print('applied upstream RoR generated-wheel topology, bounds, axis ordering, virtual-beam mass, meshwheel renderer, 60 FPS render bridge hook, and probe')
