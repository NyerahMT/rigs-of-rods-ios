#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
CORE_BUILD="${1:-$ROOT/build/vehicle-core-ios}"
OUT_DIR="${2:-$ROOT/build/ios-app}"
OGRE_BUILD="${3:-$ROOT/build/ogre-ios}"
RIGDEF_BUILD="${4:-$ROOT/build/ror-native-rigdef-ios}"
OGRE_SRC="${OGRE_SRC:-$ROOT/build/ogre-src}"
CONTENT_SRC="${ROR_CONTENT_SRC:-$ROOT/build/ror-content}"
CONTENT_COMMIT="34fefdd126784bf87b068fc283f812525d159dd7"
APP_NAME="RoRIOSProbe"
APP_DIR="$OUT_DIR/Payload/$APP_NAME.app"
IPA="$OUT_DIR/$APP_NAME.ipa"
FIXTURE="$ROOT/platform/ios/vehicle-core/fixtures/dafsemi/b6b0UID-semi.truck"

SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
CXX="$(xcrun --sdk iphoneos --find clang++)"
METALC="$(xcrun --sdk iphoneos --find metal)"
CORE_LIB="$(find "$CORE_BUILD" -name 'libror_vehicle_core.a' -print -quit)"
RIGDEF_LIB="$(find "$RIGDEF_BUILD" -name 'libror_native_rigdef.a' -print -quit)"
OGRE_MAIN="$(find "$OGRE_BUILD" -name 'libOgreMainStatic.a' -print -quit)"
OGRE_METAL="$(find "$OGRE_BUILD" -name 'libRenderSystem_MetalStatic.a' -print -quit)"

if [[ ! -d "$CONTENT_SRC/.git" ]] || [[ "$(git -C "$CONTENT_SRC" rev-parse HEAD 2>/dev/null || true)" != "$CONTENT_COMMIT" ]]; then
    rm -rf "$CONTENT_SRC"
    git init -q "$CONTENT_SRC"
    git -C "$CONTENT_SRC" remote add origin https://github.com/RigsOfRods/content.git
    git -C "$CONTENT_SRC" fetch --depth 1 origin "$CONTENT_COMMIT"
    git -C "$CONTENT_SRC" checkout -q FETCH_HEAD
fi

for REQUIRED in \
    "$CORE_LIB" \
    "$RIGDEF_LIB" \
    "$OGRE_MAIN" \
    "$OGRE_METAL" \
    "$FIXTURE" \
    "$CONTENT_SRC/dafsemi/b6b0UID-semi.truck" \
    "$CONTENT_SRC/dafsemi/b6b0UID-semi.dds" \
    "$CONTENT_SRC/dafsemi/b6b0UID-ampliroll_emissive.dds" \
    "$CONTENT_SRC/dafsemi/b6b0UID-semi.material" \
    "$OGRE_SRC/Media/Main/OgreUnifiedShader.h" \
    "$OGRE_SRC/Media/Main/DefaultShaders.metal" \
    "$OGRE_SRC/Media/Main/HLSL_SM4Support.hlsl" \
    "$OGRE_SRC/Media/Main/GLSL_GL3Support.glsl"; do
    if [[ -z "$REQUIRED" || ! -f "$REQUIRED" ]]; then
        echo "error: required iOS/OGRE/RoR input missing: $REQUIRED" >&2
        exit 1
    fi
done

rm -rf "$OUT_DIR"
mkdir -p "$APP_DIR/Content/dafsemi" "$APP_DIR/OgreMedia/Main"
cp "$ROOT/platform/ios/app/Info.plist" "$APP_DIR/Info.plist"
cp -R "$CONTENT_SRC/dafsemi/." "$APP_DIR/Content/dafsemi/"
cmp "$FIXTURE" "$APP_DIR/Content/dafsemi/b6b0UID-semi.truck"
echo "$CONTENT_COMMIT" > "$APP_DIR/Content/DEFAULT_CONTENT_COMMIT.txt"

cp -R "$OGRE_SRC/Media/Main/." "$APP_DIR/OgreMedia/Main/"
cp "$ROOT/platform/ios/ogre/RoRGame.metal" "$APP_DIR/OgreMedia/Main/RoRGame.metal"

# OGRE compiles Metal source at runtime. Compile the exact packaged shader in
# CI as well so invalid MSL can never produce a green build and a black screen
# only after installation. OGRE's Metal backend supplies OGRE_METAL=0 today;
# the unified shader include only needs that language selector for this source.
"$METALC" \
    -c \
    -DOGRE_METAL=0 \
    -I"$APP_DIR/OgreMedia/Main" \
    "$APP_DIR/OgreMedia/Main/RoRGame.metal" \
    -o "$OUT_DIR/RoRGame.air"

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
    -I"$ROOT/platform/ios/ror-native" \
    -I"$ROOT/platform/ios/ogre" \
    -I"$OGRE_SRC/OgreMain/include" \
    -I"$OGRE_BUILD/include" \
    -I"$OGRE_SRC/RenderSystems/Metal/include" \
    -I"$OGRE_SRC/RenderSystems/Metal/include/Windowing/iOS" \
    "$ROOT/platform/ios/app/OgreGameApp.mm" \
    "$ROOT/platform/ios/ror-native/NativeRigDefLaunchProbe.mm" \
    "$ROOT/platform/ios/ogre/AuthoredVisualGeometry.cpp" \
    "$CORE_LIB" \
    "$RIGDEF_LIB" \
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
test -s "$OUT_DIR/RoRGame.air"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi.truck"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi.dds"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-ampliroll_emissive.dds"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi.material"
test -s "$APP_DIR/OgreMedia/Main/RoRGame.metal"
test -s "$APP_DIR/OgreMedia/Main/OgreUnifiedShader.h"
test -s "$APP_DIR/OgreMedia/Main/HLSL_SM4Support.hlsl"
test -s "$APP_DIR/OgreMedia/Main/GLSL_GL3Support.glsl"

NM_RAW="$OUT_DIR/${APP_NAME}.nm.txt"
NM_DEMANGLED="$OUT_DIR/${APP_NAME}.nm.demangled.txt"
xcrun --sdk iphoneos nm "$APP_DIR/$APP_NAME" > "$NM_RAW"
c++filt < "$NM_RAW" > "$NM_DEMANGLED"
grep -E 'MetalPlugin|MetalRenderSystem|Ogre.*Root' "$NM_RAW" | sed -n '1,20p'
grep -q 'RoR::IOSNative::ParseRigDef' "$NM_DEMANGLED"
grep -q 'RigDef::Parser::ProcessRawLine' "$NM_DEMANGLED"

(
    cd "$OUT_DIR"
    /usr/bin/zip -qry "$APP_NAME.ipa" Payload
)

[[ -f "$IPA" ]]
echo "Built unsigned OGRE 14 / Metal + native RoR RigDef + official DAF content iPhone IPA: $IPA"
