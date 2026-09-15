#!/usr/bin/env python3
"""Apply upstream RoR shock/suspension semantics to the portable iOS core.

The Bandit uses classic `shocks`.  Upstream spawns those as SHOCK1-bounded
BEAM_HYDRO beams, preserves the active BeamDefaults as the hard bump-stop
spring/damper, and converts metric bounds before applying precompression.
The old portable runtime treated shocks as unbounded spring/damper beams, so
suspension travel could run straight through the authored limits.

This transform also parses all `shocks2` fields so future work can implement its
asymmetric/progressive force law without throwing away the authored data.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_shock_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]
parser = root / "source/main/resources/rig_def_fileformat/PortableRigDef.cpp"
cmake = root / "platform/ios/vehicle-core/CMakeLists.txt"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"shock parity anchor '{label}' expected once, found {count}")
    return text.replace(old, new, 1)


# ---------------------------------------------------------------------------
# Portable RigDef: keep BeamDefaults raw, just like upstream RigDef::BeamDefaults.
# Normal beams/hydros/meshwheel reinforcement use GetScaled*() semantics, while
# shock_t::sbd_spring/sbd_damp intentionally remember the unscaled raw defaults.
# ---------------------------------------------------------------------------
s = parser.read_text()
s = replace_once(
    s,
'''                beam_defaults.spring = Resettable(tokens[1], 9000000.0f) * beam_scale.spring;
                beam_defaults.damping = Resettable(tokens[2], 12000.0f) * beam_scale.damping;
                beam_defaults.deform = Resettable(tokens[3], 400000.0f) * beam_scale.deform;
                beam_defaults.strength = Resettable(tokens[4], 100000.0f) * beam_scale.strength;
''',
'''                // Upstream BeamDefaults stores raw values and keeps the scale
                // separately. GetScaled*() is applied when ordinary beams are
                // spawned; classic shocks deliberately retain the raw defaults
                // as their hard bump-stop target (shock_t::sbd_*).
                beam_defaults.spring = Resettable(tokens[1], 9000000.0f);
                beam_defaults.damping = Resettable(tokens[2], 12000.0f);
                beam_defaults.deform = Resettable(tokens[3], 400000.0f);
                beam_defaults.strength = Resettable(tokens[4], 100000.0f);
''',
    "raw BeamDefaults storage")

s = replace_once(
    s,
'''                    beam.spring = beam_defaults.spring; beam.damping = beam_defaults.damping;
                    beam.deform = beam_defaults.deform; beam.strength = beam_defaults.strength;
''',
'''                    beam.spring = beam_defaults.spring * beam_scale.spring;
                    beam.damping = beam_defaults.damping * beam_scale.damping;
                    beam.deform = beam_defaults.deform * beam_scale.deform;
                    beam.strength = beam_defaults.strength * beam_scale.strength;
''',
    "scaled normal beam defaults")

s = replace_once(
    s,
'''                    hydro.spring = beam_defaults.spring; hydro.damping = beam_defaults.damping;
''',
'''                    hydro.spring = beam_defaults.spring * beam_scale.spring;
                    hydro.damping = beam_defaults.damping * beam_scale.damping;
''',
    "scaled hydro defaults")

s = replace_once(
    s,
'''                    wheel.mass = F(tokens[10]); wheel.rim_spring = beam_defaults.spring; wheel.rim_damping = beam_defaults.damping;
''',
'''                    wheel.mass = F(tokens[10]);
                    wheel.rim_spring = beam_defaults.spring * beam_scale.spring;
                    wheel.rim_damping = beam_defaults.damping * beam_scale.damping;
''',
    "scaled meshwheel2 rim defaults")

old_shocks = '''            case Section::Shocks:
            case Section::Shocks2:
                if (tokens.size() >= 7)
                {
                    Shock shock;
                    shock.node_a = tokens[0]; shock.node_b = tokens[1];
                    shock.spring = F(tokens[2]); shock.damping = F(tokens[3]);
                    if (section == Section::Shocks)
                    {
                        shock.short_bound = F(tokens[4]); shock.long_bound = F(tokens[5]);
                        shock.precompression = F(tokens[6], 1.0f);
                    }
                    else if (tokens.size() >= 13)
                    {
                        shock.short_bound = F(tokens[10]); shock.long_bound = F(tokens[11]);
                        shock.precompression = F(tokens[12], 1.0f);
                    }
                    document.shocks.push_back(shock);
                }
                break;
'''
new_shocks = '''            case Section::Shocks:
                if (tokens.size() >= 7)
                {
                    Shock shock;
                    shock.node_a = tokens[0]; shock.node_b = tokens[1];
                    shock.spring = F(tokens[2]); shock.damping = F(tokens[3]);
                    shock.short_bound = F(tokens[4]); shock.long_bound = F(tokens[5]);
                    shock.precompression = F(tokens[6], 1.0f);
                    shock.options = tokens.size() > 7 ? tokens[7] : "";
                    // ActorSpawner::ProcessShock stores the raw BeamDefaults in
                    // shock_t::sbd_spring/sbd_damp for hard bump-stop behavior.
                    shock.bump_spring = beam_defaults.spring;
                    shock.bump_damping = beam_defaults.damping;
                    document.shocks.push_back(shock);
                }
                else Warn(document, line_number, "shocks requires seven values");
                break;

            case Section::Shocks2:
                if (tokens.size() >= 13)
                {
                    Shock shock;
                    shock.shock2 = true;
                    shock.node_a = tokens[0]; shock.node_b = tokens[1];
                    shock.spring_in = F(tokens[2]);
                    shock.damp_in = F(tokens[3]);
                    shock.progress_spring_in = F(tokens[4]);
                    shock.progress_damp_in = F(tokens[5]);
                    shock.spring_out = F(tokens[6]);
                    shock.damp_out = F(tokens[7]);
                    shock.progress_spring_out = F(tokens[8]);
                    shock.progress_damp_out = F(tokens[9]);
                    shock.short_bound = F(tokens[10]); shock.long_bound = F(tokens[11]);
                    shock.precompression = F(tokens[12], 1.0f);
                    shock.options = tokens.size() > 13 ? tokens[13] : "";
                    shock.bump_spring = beam_defaults.spring;
                    shock.bump_damping = beam_defaults.damping;
                    // Keep old portable callers meaningful until the dedicated
                    // SHOCK2 progressive force path is ported.
                    shock.spring = shock.spring_in;
                    shock.damping = shock.damp_in;
                    document.shocks.push_back(shock);
                }
                else Warn(document, line_number, "shocks2 requires thirteen values");
                break;
'''
s = replace_once(s, old_shocks, new_shocks, "classic/shocks2 parser split")
parser.write_text(s)


# ---------------------------------------------------------------------------
# Runtime: spawn classic shocks as bounded SHOCK1 beams, including m/M bound
# conversion and the authored BeamDefaults bump-stop target.
# ---------------------------------------------------------------------------
r = runtime.read_text()
old_runtime = '''        for (const PortableRigDef::Shock& source : rig.shocks)
        {
            const std::size_t a = ResolveNode(source.node_a, false);
            const std::size_t b = ResolveNode(source.node_b, false);
            if (a == kInvalidIndex || b == kInvalidIndex)
            {
                Error("shock references missing node");
                continue;
            }
            const float base = Distance(nodes[a].position, nodes[b].position);
            const float rest = base * (source.precompression > 0.0f ? source.precompression : 1.0f);
            AddBeam(a, b, source.spring, source.damping, BeamKind::Shock, rest);
        }
'''
new_runtime = '''        for (const PortableRigDef::Shock& source : rig.shocks)
        {
            const std::size_t a = ResolveNode(source.node_a, false);
            const std::size_t b = ResolveNode(source.node_b, false);
            if (a == kInvalidIndex || b == kInvalidIndex)
            {
                Error("shock references missing node");
                continue;
            }

            const float base = Distance(nodes[a].position, nodes[b].position);
            if (base <= 1.0e-6f)
            {
                Error("shock has coincident nodes");
                continue;
            }

            float short_bound = source.short_bound;
            float long_bound = source.long_bound;

            // ActorSpawner::ProcessShock/ProcessShock2 metric options. `m`
            // interprets both limits as metres of travel. `M` (SHOCK2) stores
            // absolute min/max lengths and converts them to relative bounds.
            if (source.shock2 && source.options.find('M') != std::string::npos)
            {
                short_bound = (base - short_bound) / base;
                long_bound = (long_bound - base) / base;
            }
            else if (source.options.find('m') != std::string::npos)
            {
                short_bound /= base;
                long_bound /= base;
            }

            const float rest = base * (source.precompression > 0.0f ? source.precompression : 1.0f);
            const std::size_t beam_index = AddBeam(
                a, b, source.spring, source.damping, BeamKind::Shock, rest);
            if (beam_index == kInvalidIndex)
                continue;

            BeamCoreState& beam = beams[beam_index].beam;
            beam.bounded = true;
            beam.shortbound = std::max(0.0f, short_bound);
            beam.longbound = std::max(0.0f, long_bound);
            beam.bump_spring = std::max(0.0f, source.bump_spring);
            beam.bump_damping = std::max(0.0f, source.bump_damping);

            if (source.shock2)
                Warn("portable runtime preserves shocks2 metadata but currently uses its compression-side base spring/damper");
        }
'''
r = replace_once(r, old_runtime, new_runtime, "bounded authored shock construction")
runtime.write_text(r)


# ---------------------------------------------------------------------------
# Register a small regression probe.  It catches both the parser metadata and
# SHOCK1 bump-stop law, so these semantics cannot silently fall back to the old
# unbounded suspension again.
# ---------------------------------------------------------------------------
c = cmake.read_text()
probe = '    add_executable(ror_shock_parity_probe tests/ShockParityProbe.cpp)\n'
if probe not in c:
    anchor = '    add_executable(ror_authored_vehicle_smoke tests/AuthoredVehicleSmokeProbe.cpp)\n'
    if c.count(anchor) != 1:
        raise SystemExit("shock parity CMake executable anchor drifted")
    c = c.replace(anchor, anchor + probe, 1)

    tail = '''    target_link_libraries(ror_authored_vehicle_smoke PRIVATE ror_vehicle_core)
    target_compile_features(ror_authored_vehicle_smoke PRIVATE cxx_std_11)
'''
    replacement = tail + '''    target_link_libraries(ror_shock_parity_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_shock_parity_probe PRIVATE cxx_std_11)
    add_test(NAME ror_shock_parity_probe COMMAND ror_shock_parity_probe)
'''
    if c.count(tail) != 1:
        raise SystemExit("shock parity CMake link anchor drifted")
    c = c.replace(tail, replacement, 1)
cmake.write_text(c)

print('applied RoR classic SHOCK1 bounds, authored bump stops, metric limits, BeamDefaults scale semantics, and shocks2 metadata')
