#!/usr/bin/env bash
set -euo pipefail

# Community Repository resource 712: PoLi's 1979-82 Foxbody Mustang.
# We intentionally fetch it at build time rather than vendoring third-party
# vehicle assets into the source tree. The resource is published for RoR use.
OUT_DIR="${1:-build/foxbody-resource}"
URL="https://forum.rigsofrods.org/resources/712/download"
ZIP="$OUT_DIR/foxbody.zip"
EXTRACT="$OUT_DIR/extracted"

rm -rf "$OUT_DIR"
mkdir -p "$EXTRACT"

curl --fail --location --retry 3 --connect-timeout 20 \
    --user-agent 'Rigs of Rods Client/2026.01' \
    --output "$ZIP" "$URL"

# XenForo may return an HTML challenge/error with status 200. Fail explicitly
# instead of feeding that to unzip and obscuring the real problem.
python3 - "$ZIP" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
data = p.read_bytes()[:4]
if data != b'PK\x03\x04':
    raise SystemExit(f"Community resource download is not a ZIP (magic={data!r})")
PY

unzip -q "$ZIP" -d "$EXTRACT"

echo '--- Foxbody vehicle definitions ---'
find "$EXTRACT" -type f \( -iname '*.truck' -o -iname '*.car' \) -print | sort

echo '--- Foxbody sound scripts ---'
find "$EXTRACT" -type f -iname '*.soundscript' -print | sort

echo '--- Foxbody audio assets ---'
find "$EXTRACT" -type f \( -iname '*.wav' -o -iname '*.ogg' -o -iname '*.flac' \) -print | sort

echo '--- Foxbody visual assets (sample) ---'
find "$EXTRACT" -type f \( -iname '*.mesh' -o -iname '*.material' -o -iname '*.dds' -o -iname '*.png' \) -print | sort | sed -n '1,120p'

echo '--- Flexbody/cab/soundsource markers ---'
find "$EXTRACT" -type f \( -iname '*.truck' -o -iname '*.car' \) -print0 | while IFS= read -r -d '' file; do
    echo "### $file"
    grep -En '^(flexbodies|flexbody|cab|submesh|soundsources|soundsources2|engine|engoption|engineoptions)$' "$file" || true
done
