#!/usr/bin/env python3
"""Match upstream `minimass` / `set_default_minimass` mass semantics.

Desktop RoR's `minimass <kg>, l` sets ar_minimass_skip_loaded_nodes, which means
minimum mass is not forced onto nodes carrying the load-mass option. The portable
core previously parsed only the kilogram value and always passed false to the
mass distributor.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_minimass_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]
header = root / "source/main/resources/rig_def_fileformat/PortableRigDef.h"
parser = root / "source/main/resources/rig_def_fileformat/PortableRigDef.cpp"


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"minimass parity anchor '{label}' expected once, found {count}")
    path.write_text(text.replace(old, new, 1))


replace_once(
    header,
    '''    float minimass=50.0f;
    std::vector<Node> nodes;
''',
    '''    float minimass=50.0f;
    // Upstream RigDef::MinimassOption::l_SKIP_LOADED. When true the global
    // minimum applies only to ordinary dry-mass nodes, not authored load nodes.
    bool minimass_skip_loaded=false;
    std::vector<Node> nodes;
''',
    "document minimass option",
)

replace_once(
    parser,
    '''            case Section::Minimass:
                document.minimass = F(tokens[0], 50.0f);
                active_minimass = document.minimass;
                section = Section::None;
                break;
''',
    '''            case Section::Minimass:
                document.minimass = F(tokens[0], 50.0f);
                // RigDef::Parser::ParseMinimass(): optional second argument `l`
                // maps to MinimassOption::l_SKIP_LOADED.
                document.minimass_skip_loaded =
                    tokens.size() > 1 && !tokens[1].empty() && tokens[1][0] == 'l';
                active_minimass = document.minimass;
                section = Section::None;
                break;
''',
    "minimass parser option",
)

replace_once(
    runtime,
    '''            dry_mass, rig.globals.present ? rig.globals.load_mass : 0.0f,
            mass_nodes, mass_beams, false);
''',
    '''            dry_mass, rig.globals.present ? rig.globals.load_mass : 0.0f,
            mass_nodes, mass_beams, rig.minimass_skip_loaded);
''',
    "runtime minimass skip-loaded flag",
)

print('matched upstream minimass `l` option and loaded-node minimum-mass exclusion')
