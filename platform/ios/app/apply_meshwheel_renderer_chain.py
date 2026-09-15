#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_meshwheel_renderer_chain.py <generated-OgreGameApp.mm>")

path = Path(sys.argv[1])
text = path.read_text()
prepared = '    bool body_built=false, wheels_built=false, props_built=false, flexbodies_built=false, camera_started=false;\n'
base_anchor = '    bool body_built=false, wheels_built=false, props_built=false, camera_started=false;\n'
if prepared in text:
    text = text.replace(prepared, base_anchor, 1)
elif base_anchor not in text:
    raise SystemExit('meshwheel renderer state bridge anchor drifted')
path.write_text(text)

here = Path(__file__).resolve().parent
subprocess.check_call([
    sys.executable,
    str(here / 'apply_meshwheel_renderer.py'),
    str(path),
])
subprocess.check_call([
    sys.executable,
    str(here / 'apply_render_interpolation.py'),
    str(path),
])
