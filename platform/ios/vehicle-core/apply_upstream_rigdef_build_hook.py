#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_upstream_rigdef_build_hook.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]
build = root / "platform/ios/app/build-ipa.sh"
text = build.read_text()

# apply_meshwheel_build_bridge.py intentionally rewrites the direct meshwheel
# transform to its chain wrapper before this hook runs. Match that final chain
# instead of reaching back to the stale pre-bridge spelling.
old = '''python3 "$ROOT/platform/ios/app/prepare_audio_game_source.py" \\
    "$ROOT/platform/ios/app/OgreGameApp.mm" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_meshwheel_renderer_chain.py" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_render_interpolation.py" "$AUDIO_GAME_SRC"
'''
new = '''python3 "$ROOT/platform/ios/app/prepare_audio_game_source.py" \\
    "$ROOT/platform/ios/app/OgreGameApp.mm" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_upstream_rigdef_physics.py" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_meshwheel_renderer_chain.py" "$AUDIO_GAME_SRC"
python3 "$ROOT/platform/ios/app/apply_render_interpolation.py" "$AUDIO_GAME_SRC"
'''

if new not in text:
    if text.count(old) != 1:
        raise SystemExit(f"upstream RigDef build hook expected one final transform chain, found {text.count(old)}")
    text = text.replace(old, new, 1)

build.write_text(text)
print('inserted upstream RigDef physics canonicalization into generated app source chain')
