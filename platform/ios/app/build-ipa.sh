#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
CORE_BUILD="${1:-$ROOT/build/vehicle-core-ios}"
OUT_DIR="${2:-$ROOT/build/ios-app}"
APP_NAME="RoRIOSProbe"
APP_DIR="$OUT_DIR/Payload/$APP_NAME.app"
IPA="$OUT_DIR/$APP_NAME.ipa"

SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
CXX="$(xcrun --sdk iphoneos --find clang++)"
LIB="$(find "$CORE_BUILD" -name 'libror_vehicle_core.a' -print -quit)"

if [[ -z "$LIB" ]]; then
    echo "error: libror_vehicle_core.a not found under $CORE_BUILD" >&2
    exit 1
fi

rm -rf "$OUT_DIR"
mkdir -p "$APP_DIR"
cp "$ROOT/platform/ios/app/Info.plist" "$APP_DIR/Info.plist"

"$CXX" \
    -arch arm64 \
    -isysroot "$SDK" \
    -miphoneos-version-min=16.0 \
    -fobjc-arc \
    -std=c++17 \
    -stdlib=libc++ \
    -I"$ROOT/source/main/physics" \
    -I"$ROOT/source/main/resources/rig_def_fileformat" \
    -I"$ROOT/platform/ios/vehicle-core" \
    "$ROOT/platform/ios/app/main.mm" \
    "$LIB" \
    -framework UIKit \
    -framework Foundation \
    -framework QuartzCore \
    -framework CoreGraphics \
    -o "$APP_DIR/$APP_NAME"

chmod +x "$APP_DIR/$APP_NAME"
plutil -lint "$APP_DIR/Info.plist"
file "$APP_DIR/$APP_NAME"
lipo -info "$APP_DIR/$APP_NAME"

(
    cd "$OUT_DIR"
    /usr/bin/zip -qry "$APP_NAME.ipa" Payload
)

[[ -f "$IPA" ]]
echo "Built unsigned IPA: $IPA"
