#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_meshwheel_build_bridge.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]
build = root / "platform/ios/app/build-ipa.sh"
text = build.read_text()
old = 'python3 "$ROOT/platform/ios/app/apply_meshwheel_renderer.py" "$AUDIO_GAME_SRC"\n'
new = 'python3 "$ROOT/platform/ios/app/apply_meshwheel_renderer_chain.py" "$AUDIO_GAME_SRC"\n'
if new not in text:
    if text.count(old) != 1:
        raise SystemExit("meshwheel build bridge anchor drifted")
    text = text.replace(old, new, 1)
build.write_text(text)
print('bridged meshwheel renderer after flexbody/audio generated-source transform')
