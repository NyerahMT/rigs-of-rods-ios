#!/usr/bin/env bash
set -euo pipefail

# RoR Community Repository resource 277: Gabester's Gavril Bandit pack.
# This is an archived RoR reference vehicle family used only as a compatibility
# fixture; it is fetched at build time and is not vendored into this repository.
OUT_DIR="${1:-build/gabester-bandit-resource}"
URL="https://forum.rigsofrods.org/resources/277/download"
ZIP="$OUT_DIR/bandit.zip"
EXTRACT="$OUT_DIR/extracted"
rm -rf "$OUT_DIR"
mkdir -p "$EXTRACT"
curl --fail --location --retry 3 --connect-timeout 20 \
  --user-agent 'Rigs of Rods Client/2026.01' \
  --output "$ZIP" "$URL"
python3 - "$ZIP" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
if p.read_bytes()[:4] != b'PK\x03\x04':
    raise SystemExit('Bandit repository download is not a ZIP')
PY
unzip -q "$ZIP" -d "$EXTRACT"

# The archived pack uses CRLF RigDef files. Normalize only text vehicle
# definitions so shell exact-line checks, the portable parser, and the iPhone
# packaged control fixture all see identical bytes. Do not touch binary assets.
python3 - "$EXTRACT" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1])
for path in root.rglob('*'):
    if path.is_file() and path.suffix.lower() in {'.truck', '.car'}:
        data = path.read_bytes()
        normalized = data.replace(b'\r\n', b'\n').replace(b'\r', b'\n')
        if normalized != data:
            path.write_bytes(normalized)
PY

echo '--- Gabester Bandit vehicle definitions ---'
find "$EXTRACT" -type f \( -iname '*.truck' -o -iname '*.car' \) -print | sort
