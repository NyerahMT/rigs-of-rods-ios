#!/usr/bin/env python3
"""Finish upstream SHOCK2/SHOCK3 semantics after apply_shock_parity.py.

This transform runs after the classic SHOCK1 parity layer. It adds SHOCK3 to the
portable RigDef grammar and maps all SHOCK2/SHOCK3 authored state into the exact
coefficient laws implemented by BeamPhysics.cpp.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_shock23_force_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]
parser = root / "source/main/resources/rig_def_fileformat/PortableRigDef.cpp"


def once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"shock23 parity anchor '{label}' expected once, found {count}")
    return text.replace(old, new, 1)


# ---------------------------------------------------------------------------
# Portable RigDef: SHOCK3 is a first-class physics section, not an ignored block.
# Token order mirrors RigDef::Parser::ParseShock3() exactly.
# ---------------------------------------------------------------------------
s = parser.read_text()
s = once(
    s,
'''    Shocks,
    Shocks2,
    Wheels,
''',
'''    Shocks,
    Shocks2,
    Shocks3,
    Wheels,
''',
    "SHOCK3 section enum")

s = once(
    s,
'''    if (keyword == "shocks") return Section::Shocks;
    if (keyword == "shocks2") return Section::Shocks2;
    if (keyword == "wheels") return Section::Wheels;
''',
'''    if (keyword == "shocks") return Section::Shocks;
    if (keyword == "shocks2") return Section::Shocks2;
    if (keyword == "shocks3") return Section::Shocks3;
    if (keyword == "wheels") return Section::Wheels;
''',
    "SHOCK3 section keyword")

s = once(
    s,
'        "ropables", "ropes", "rotators", "rotators2", "screwprops", "scripts", "shocks3",\n',
'        "ropables", "ropes", "rotators", "rotators2", "screwprops", "scripts",\n',
    "SHOCK3 unsupported removal")

shock3_case = '''            case Section::Shocks3:
                if (tokens.size() >= 15)
                {
                    Shock shock;
                    shock.shock3 = true;
                    shock.node_a = tokens[0]; shock.node_b = tokens[1];
                    shock.spring_in = F(tokens[2]);
                    shock.damp_in = F(tokens[3]);
                    shock.damp_in_slow = F(tokens[4]);
                    shock.split_vel_in = F(tokens[5]);
                    shock.damp_in_fast = F(tokens[6]);
                    shock.spring_out = F(tokens[7]);
                    shock.damp_out = F(tokens[8]);
                    shock.damp_out_slow = F(tokens[9]);
                    shock.split_vel_out = F(tokens[10]);
                    shock.damp_out_fast = F(tokens[11]);
                    shock.short_bound = F(tokens[12]);
                    shock.long_bound = F(tokens[13]);
                    shock.precompression = F(tokens[14], 1.0f);
                    shock.options = tokens.size() > 15 ? tokens[15] : "";
                    shock.bump_spring = beam_defaults.spring;
                    shock.bump_damping = beam_defaults.damping;
                    shock.spring = shock.spring_in;
                    shock.damping = shock.damp_in;
                    document.shocks.push_back(shock);
                }
                else Warn(document, line_number, "shocks3 requires fifteen values");
                break;

'''
s = once(
    s,
'            case Section::Wheels:\n',
    shock3_case + '            case Section::Wheels:\n',
    "SHOCK3 parser case")
parser.write_text(s)


# ---------------------------------------------------------------------------
# Runtime: the SHOCK1 layer already creates a bounded beam and converts metric
# bounds. Replace its temporary SHOCK2 warning with full state transfer to the
# portable upstream-equivalent coefficient solver.
# ---------------------------------------------------------------------------
r = runtime.read_text()
old = '''            beam.bump_spring = std::max(0.0f, source.bump_spring);
            beam.bump_damping = std::max(0.0f, source.bump_damping);

            if (source.shock2)
                Warn("portable runtime preserves shocks2 metadata but currently uses its compression-side base spring/damper");
'''
new = '''            beam.bump_spring = std::max(0.0f, source.bump_spring);
            beam.bump_damping = std::max(0.0f, source.bump_damping);

            if (source.shock2)
            {
                beam.shock_model = ShockModel::Shock2;
                beam.soft_bump = source.options.find('s') != std::string::npos;
                beam.spring_in = source.spring_in;
                beam.damp_in = source.damp_in;
                beam.spring_out = source.spring_out;
                beam.damp_out = source.damp_out;
                beam.progress_spring_in = source.progress_spring_in;
                beam.progress_damp_in = source.progress_damp_in;
                beam.progress_spring_out = source.progress_spring_out;
                beam.progress_damp_out = source.progress_damp_out;
            }
            else if (source.shock3)
            {
                beam.shock_model = ShockModel::Shock3;
                beam.spring_in = source.spring_in;
                beam.damp_in = source.damp_in;
                beam.spring_out = source.spring_out;
                beam.damp_out = source.damp_out;
                beam.damp_in_slow = source.damp_in_slow;
                beam.split_vel_in = source.split_vel_in;
                beam.damp_in_fast = source.damp_in_fast;
                beam.damp_out_slow = source.damp_out_slow;
                beam.split_vel_out = source.split_vel_out;
                beam.damp_out_fast = source.damp_out_fast;
            }
            else
            {
                beam.shock_model = ShockModel::Shock1;
            }
'''
r = once(r, old, new, "advanced shock runtime state")
runtime.write_text(r)

print('ported SHOCK2 progression/soft-bump state and SHOCK3 velocity-split state into the live beam solver')
