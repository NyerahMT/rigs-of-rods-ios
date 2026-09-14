#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
CORE_BUILD="${1:-$ROOT/build/vehicle-core-ios}"
OUT_DIR="${2:-$ROOT/build/ios-app}"
OGRE_BUILD="${3:-$ROOT/build/ogre-ios}"
OGRE_SRC="${OGRE_SRC:-$ROOT/build/ogre-src}"
APP_NAME="RoRIOSProbe"
APP_DIR="$OUT_DIR/Payload/$APP_NAME.app"
IPA="$OUT_DIR/$APP_NAME.ipa"
FIXTURE="$ROOT/platform/ios/vehicle-core/fixtures/dafsemi/b6b0UID-semi.truck"

SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
CXX="$(xcrun --sdk iphoneos --find clang++)"
CORE_LIB="$(find "$CORE_BUILD" -name 'libror_vehicle_core.a' -print -quit)"
OGRE_MAIN="$(find "$OGRE_BUILD" -name 'libOgreMainStatic.a' -print -quit)"
OGRE_METAL="$(find "$OGRE_BUILD" -name 'libRenderSystem_MetalStatic.a' -print -quit)"

for REQUIRED in \
    "$CORE_LIB" \
    "$OGRE_MAIN" \
    "$OGRE_METAL" \
    "$FIXTURE" \
    "$OGRE_SRC/Media/Main/OgreUnifiedShader.h" \
    "$OGRE_SRC/Media/Main/DefaultShaders.metal" \
    "$OGRE_SRC/Media/Main/HLSL_SM4Support.hlsl" \
    "$OGRE_SRC/Media/Main/GLSL_GL3Support.glsl"; do
    if [[ -z "$REQUIRED" || ! -f "$REQUIRED" ]]; then
        echo "error: required iOS/OGRE input missing: $REQUIRED" >&2
        exit 1
    fi
done

rm -rf "$OUT_DIR"
mkdir -p "$APP_DIR/Content/dafsemi" "$APP_DIR/OgreMedia/Main"
cp "$ROOT/platform/ios/app/Info.plist" "$APP_DIR/Info.plist"
cp "$FIXTURE" "$APP_DIR/Content/dafsemi/b6b0UID-semi.truck"

# OGRE's runtime include resolver walks unified shader includes before Metal's
# preprocessor eliminates the HLSL/GLSL branches. Keep the complete pinned
# Media/Main support set together instead of cherry-picking Metal-only files.
cp -R "$OGRE_SRC/Media/Main/." "$APP_DIR/OgreMedia/Main/"
cp "$ROOT/platform/ios/ogre/RoRGame.metal" "$APP_DIR/OgreMedia/Main/RoRGame.metal"

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
    -I"$ROOT/platform/ios/ogre" \
    -I"$OGRE_SRC/OgreMain/include" \
    -I"$OGRE_BUILD/include" \
    -I"$OGRE_SRC/RenderSystems/Metal/include" \
    -I"$OGRE_SRC/RenderSystems/Metal/include/Windowing/iOS" \
    "$ROOT/platform/ios/app/OgreGameApp.mm" \
    "$ROOT/platform/ios/app/OgreRuntimeDiagnostics.mm" \
    "$ROOT/platform/ios/ogre/AuthoredVisualGeometry.cpp" \
    "$CORE_LIB" \
    "$OGRE_METAL" \
    "$OGRE_MAIN" \
    -framework UIKit \
    -framework Foundation \
    -framework QuartzCore \
    -framework CoreGraphics \
    -framework Metal \
    -framework CoreFoundation \
    -o "$APP_DIR/$APP_NAME"

chmod +x "$APP_DIR/$APP_NAME"
plutil -lint "$APP_DIR/Info.plist"
file "$APP_DIR/$APP_NAME"
lipo -info "$APP_DIR/$APP_NAME"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi.truck"
test -s "$APP_DIR/OgreMedia/Main/RoRGame.metal"
test -s "$APP_DIR/OgreMedia/Main/OgreUnifiedShader.h"
test -s "$APP_DIR/OgreMedia/Main/HLSL_SM4Support.hlsl"
test -s "$APP_DIR/OgreMedia/Main/GLSL_GL3Support.glsl"

# Static linking should leave concrete OGRE/Metal symbols in the final image.
xcrun --sdk iphoneos nm "$APP_DIR/$APP_NAME" | grep -E 'MetalPlugin|MetalRenderSystem|Ogre.*Root' | head -20

(
    cd "$OUT_DIR"
    /usr/bin/zip -qry "$APP_NAME.ipa" Payload
)

[[ -f "$IPA" ]]
echo "Built unsigned OGRE 14 / Metal authored-vehicle IPA: $IPA"
