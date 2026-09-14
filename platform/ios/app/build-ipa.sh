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
HOST_SDK="$(xcrun --sdk macosx --show-sdk-path)"
HOST_CXX="$(xcrun --sdk macosx --find clang++)"
CORE_LIB="$(find "$CORE_BUILD" -name 'libror_vehicle_core.a' -print -quit)"
RIGDEF_LIB="$(find "$RIGDEF_BUILD" -name 'libror_native_rigdef.a' -print -quit)"
OGRE_MAIN="$(find "$OGRE_BUILD" -name 'libOgreMainStatic.a' -print -quit)"
OGRE_METAL="$(find "$OGRE_BUILD" -name 'libRenderSystem_MetalStatic.a' -print -quit)"
OGRE_RTSS="$(find "$OGRE_BUILD" -name 'libOgreRTShaderSystemStatic.a' -print -quit)"
OGRE_TERRAIN="$(find "$OGRE_BUILD" -name 'libOgreTerrainStatic.a' -print -quit)"

if [[ ! -d "$CONTENT_SRC/.git" ]] || [[ "$(git -C "$CONTENT_SRC" rev-parse HEAD 2>/dev/null || true)" != "$CONTENT_COMMIT" ]]; then
    rm -rf "$CONTENT_SRC"
    git init -q "$CONTENT_SRC"
    git -C "$CONTENT_SRC" remote add origin https://github.com/RigsOfRods/content.git
    git -C "$CONTENT_SRC" fetch --depth 1 origin "$CONTENT_COMMIT"
    git -C "$CONTENT_SRC" checkout -q FETCH_HEAD
fi

# Community Repository resource 712 provides the faster Foxbody target and its
# authored 351 Windsor soundscript/recordings. Fetch it at build time rather
# than vendoring third-party vehicle content into the source tree.
bash "$ROOT/platform/ios/app/fetch-foxbody.sh" "$ROOT/build/foxbody-resource"
FOXBODY_ROOT="$(find "$ROOT/build/foxbody-resource/extracted" -type f -name '351Wmustang.soundscript' -print -quit | xargs dirname)"
if [[ -z "$FOXBODY_ROOT" || ! -d "$FOXBODY_ROOT" ]]; then
    echo "error: Foxbody resource did not contain 351Wmustang.soundscript" >&2
    exit 1
fi
FOXBODY_TRUCK="$(find "$FOXBODY_ROOT" -maxdepth 1 -type f \( -iname '*.truck' -o -iname '*.car' \) -print | sort | head -n 1)"
if [[ -z "$FOXBODY_TRUCK" || ! -f "$FOXBODY_TRUCK" ]]; then
    echo "error: Foxbody resource did not contain a top-level vehicle definition" >&2
    exit 1
fi

echo "Using Foxbody vehicle definition: $FOXBODY_TRUCK"

for REQUIRED in \
    "$CORE_LIB" \
    "$RIGDEF_LIB" \
    "$OGRE_MAIN" \
    "$OGRE_METAL" \
    "$OGRE_RTSS" \
    "$OGRE_TERRAIN" \
    "$FIXTURE" \
    "$CONTENT_SRC/dafsemi/b6b0UID-semi.truck" \
    "$CONTENT_SRC/dafsemi/b6b0UID-semi.dds" \
    "$CONTENT_SRC/dafsemi/b6b0UID-ampliroll_emissive.dds" \
    "$CONTENT_SRC/dafsemi/b6b0UID-semi.material" \
    "$CONTENT_SRC/simple2-terrain/simple2.terrn2" \
    "$CONTENT_SRC/simple2-terrain/simple2.otc" \
    "$CONTENT_SRC/simple2-terrain/simple2-page-0-0.otc" \
    "$CONTENT_SRC/simple2-terrain/simple2-gravel_diffusespecular.dds" \
    "$CONTENT_SRC/simple2-terrain/simple2-gravel_normalheight.dds" \
    "$FOXBODY_TRUCK" \
    "$FOXBODY_ROOT/351Wmustang.soundscript" \
    "$FOXBODY_ROOT/351Wlowidle.wav" \
    "$FOXBODY_ROOT/351Whighidle.wav" \
    "$FOXBODY_ROOT/351Whighrev.wav" \
    "$FOXBODY_ROOT/351Wstarter2.wav" \
    "$ROOT/resources/meshes/dashboard.mesh" \
    "$ROOT/resources/meshes/leftmirror.mesh" \
    "$ROOT/resources/meshes/rightmirror.mesh" \
    "$ROOT/resources/meshes/seat.mesh" \
    "$OGRE_SRC/Media/Main/OgreUnifiedShader.h" \
    "$OGRE_SRC/Media/Main/DefaultShaders.metal" \
    "$OGRE_SRC/Media/Main/HLSL_SM4Support.hlsl" \
    "$OGRE_SRC/Media/Main/GLSL_GL3Support.glsl" \
    "$ROOT/platform/ios/ogre/MetalShaderProbe.mm" \
    "$ROOT/platform/ios/ogre/RoRTerrainAdapter.cpp" \
    "$ROOT/platform/ios/ogre/RoRTerrainAdapter.h" \
    "$ROOT/platform/ios/ogre/transcode_dxt_dds_for_ios.py" \
    "$ROOT/platform/ios/app/RoREngineAudio.h" \
    "$ROOT/platform/ios/app/RoREngineAudio.mm" \
    "$ROOT/platform/ios/app/prepare_audio_game_source.py"; do
    if [[ -z "$REQUIRED" || ! -f "$REQUIRED" ]]; then
        echo "error: required iOS/OGRE/RoR input missing: $REQUIRED" >&2
        exit 1
    fi
done

rm -rf "$OUT_DIR"
mkdir -p "$APP_DIR/Content/dafsemi" "$APP_DIR/Content/simple2-terrain" "$APP_DIR/Content/foxbody-audio" "$APP_DIR/Content/foxbody-mustang" "$APP_DIR/OgreMedia/Main" "$APP_DIR/RoRResources/meshes"
cp "$ROOT/platform/ios/app/Info.plist" "$APP_DIR/Info.plist"
cp -R "$CONTENT_SRC/dafsemi/." "$APP_DIR/Content/dafsemi/"
cp -R "$CONTENT_SRC/simple2-terrain/." "$APP_DIR/Content/simple2-terrain/"
cmp "$FIXTURE" "$APP_DIR/Content/dafsemi/b6b0UID-semi.truck"
echo "$CONTENT_COMMIT" > "$APP_DIR/Content/DEFAULT_CONTENT_COMMIT.txt"

# Keep the entire authored Foxbody package together: the flexbody mesh, materials,
# textures, props and the truck definition refer to one another by filename. A
# stable alias lets the Objective-C++ shell select the vehicle without baking a
# community-resource filename into the renderer.
cp -R "$FOXBODY_ROOT/." "$APP_DIR/Content/foxbody-mustang/"
cp "$FOXBODY_TRUCK" "$APP_DIR/Content/foxbody-mustang/Foxbody.truck"
basename "$FOXBODY_TRUCK" > "$APP_DIR/Content/foxbody-mustang/PRIMARY_VEHICLE.txt"

# Preserve the actual RoR soundscript and recordings. The AVFoundation bridge
# parses these same RPM anchors at runtime; no synthesized placeholder tone.
cp "$FOXBODY_ROOT/351Wmustang.soundscript" "$APP_DIR/Content/foxbody-audio/"
for AUDIO in 351Wlowidle.wav 351Whighidle.wav 351Wmedrev.wav 351Whighrev.wav 351Wstarter2.wav; do
    if [[ -f "$FOXBODY_ROOT/$AUDIO" ]]; then
        cp "$FOXBODY_ROOT/$AUDIO" "$APP_DIR/Content/foxbody-audio/$AUDIO"
    fi
done

for MESH in dashboard.mesh leftmirror.mesh rightmirror.mesh seat.mesh; do
    cp "$ROOT/resources/meshes/$MESH" "$APP_DIR/RoRResources/meshes/$MESH"
done

# Keep the official artwork but remove on-device BC/DXT decoding from the Metal
# path. This is the same deterministic BGRA8 DDS packaging that fixed the DAF.
cp "$CONTENT_SRC/dafsemi/b6b0UID-semi.dds" "$APP_DIR/Content/dafsemi/b6b0UID-semi-source.dds"
cp "$CONTENT_SRC/dafsemi/b6b0UID-ampliroll_emissive.dds" "$APP_DIR/Content/dafsemi/b6b0UID-ampliroll_emissive-source.dds"
python3 "$ROOT/platform/ios/ogre/transcode_dxt_dds_for_ios.py" \
    "$CONTENT_SRC/dafsemi/b6b0UID-semi.dds" \
    "$APP_DIR/Content/dafsemi/b6b0UID-semi.dds"
python3 "$ROOT/platform/ios/ogre/transcode_dxt_dds_for_ios.py" \
    "$CONTENT_SRC/dafsemi/b6b0UID-ampliroll_emissive.dds" \
    "$APP_DIR/Content/dafsemi/b6b0UID-ampliroll_emissive.dds"

for TERRAIN_TEX in simple2-gravel_diffusespecular.dds simple2-gravel_normalheight.dds; do
    cp "$CONTENT_SRC/simple2-terrain/$TERRAIN_TEX" "$APP_DIR/Content/simple2-terrain/${TERRAIN_TEX%.dds}-source.dds"
    python3 "$ROOT/platform/ios/ogre/transcode_dxt_dds_for_ios.py" \
        "$CONTENT_SRC/simple2-terrain/$TERRAIN_TEX" \
        "$APP_DIR/Content/simple2-terrain/$TERRAIN_TEX"
done

python3 - \
    "$APP_DIR/Content/dafsemi/b6b0UID-semi.dds" \
    "$APP_DIR/Content/dafsemi/b6b0UID-ampliroll_emissive.dds" \
    "$APP_DIR/Content/simple2-terrain/simple2-gravel_diffusespecular.dds" \
    "$APP_DIR/Content/simple2-terrain/simple2-gravel_normalheight.dds" <<'PY'
import struct, sys
for path in sys.argv[1:]:
    data = open(path, 'rb').read(128)
    assert data[:4] == b'DDS ', path
    assert data[84:88] == b'\0\0\0\0', f'{path}: still FourCC/DXT'
    assert struct.unpack_from('<I', data, 88)[0] == 32, f'{path}: not 32bpp'
    masks = tuple(struct.unpack_from('<I', data, o)[0] for o in (92,96,100,104))
    assert masks == (0x00FF0000,0x0000FF00,0x000000FF,0xFF000000), f'{path}: wrong BGRA masks'
PY

cp -R "$OGRE_SRC/Media/Main/." "$APP_DIR/OgreMedia/Main/"
cp "$ROOT/platform/ios/ogre/RoRGame.metal" "$APP_DIR/OgreMedia/Main/RoRGame.metal"

"$HOST_CXX" \
    -isysroot "$HOST_SDK" \
    -fobjc-arc \
    -std=c++17 \
    "$ROOT/platform/ios/ogre/MetalShaderProbe.mm" \
    -framework Foundation \
    -framework Metal \
    -o "$OUT_DIR/metal-shader-probe"
"$OUT_DIR/metal-shader-probe" \
    "$APP_DIR/OgreMedia/Main/RoRGame.metal" \
    "$APP_DIR/OgreMedia/Main/OgreUnifiedShader.h"

# Generate the controller source with the audio bridge explicitly connected to
# vehicle telemetry. The transform is strict and fails if its anchors drift.
AUDIO_GAME_SRC="$OUT_DIR/OgreGameApp.audio.mm"
python3 "$ROOT/platform/ios/app/prepare_audio_game_source.py" \
    "$ROOT/platform/ios/app/OgreGameApp.mm" "$AUDIO_GAME_SRC"

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
    -I"$ROOT/platform/ios/app" \
    -I"$OGRE_SRC/OgreMain/include" \
    -I"$OGRE_BUILD/include" \
    -I"$OGRE_SRC/RenderSystems/Metal/include" \
    -I"$OGRE_SRC/RenderSystems/Metal/include/Windowing/iOS" \
    -I"$OGRE_SRC/Components/Terrain/include" \
    -I"$OGRE_SRC/Components/RTShaderSystem/include" \
    "$AUDIO_GAME_SRC" \
    "$ROOT/platform/ios/app/RoREngineAudio.mm" \
    "$ROOT/platform/ios/ror-native/NativeRigDefLaunchProbe.mm" \
    "$ROOT/platform/ios/ogre/AuthoredVisualGeometry.cpp" \
    "$ROOT/platform/ios/ogre/RoRTerrainAdapter.cpp" \
    "$CORE_LIB" \
    "$RIGDEF_LIB" \
    "$OGRE_TERRAIN" \
    "$OGRE_RTSS" \
    "$OGRE_METAL" \
    "$OGRE_MAIN" \
    -framework UIKit \
    -framework Foundation \
    -framework AVFoundation \
    -framework QuartzCore \
    -framework CoreGraphics \
    -framework Metal \
    -framework CoreFoundation \
    -o "$APP_DIR/$APP_NAME"

chmod +x "$APP_DIR/$APP_NAME"
plutil -lint "$APP_DIR/Info.plist"
file "$APP_DIR/$APP_NAME"
lipo -info "$APP_DIR/$APP_NAME"
test -x "$OUT_DIR/metal-shader-probe"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi.truck"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi.dds"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-ampliroll_emissive.dds"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi-source.dds"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-ampliroll_emissive-source.dds"
test -s "$APP_DIR/Content/dafsemi/b6b0UID-semi.material"
test -s "$APP_DIR/Content/simple2-terrain/simple2.terrn2"
test -s "$APP_DIR/Content/simple2-terrain/simple2.otc"
test -s "$APP_DIR/Content/simple2-terrain/simple2-page-0-0.otc"
test -s "$APP_DIR/Content/simple2-terrain/simple2-gravel_diffusespecular.dds"
test -s "$APP_DIR/Content/simple2-terrain/simple2-gravel_normalheight.dds"
test -s "$APP_DIR/Content/foxbody-mustang/Foxbody.truck"
test -s "$APP_DIR/Content/foxbody-mustang/PRIMARY_VEHICLE.txt"
find "$APP_DIR/Content/foxbody-mustang" -maxdepth 1 -type f -iname '*.mesh' -print -quit | grep -q .
grep -Eiq '^[[:space:]]*flexbodies([[:space:]]|$)' "$APP_DIR/Content/foxbody-mustang/Foxbody.truck"
test -s "$APP_DIR/Content/foxbody-audio/351Wmustang.soundscript"
test -s "$APP_DIR/Content/foxbody-audio/351Wlowidle.wav"
test -s "$APP_DIR/Content/foxbody-audio/351Whighidle.wav"
test -s "$APP_DIR/Content/foxbody-audio/351Whighrev.wav"
test -s "$APP_DIR/Content/foxbody-audio/351Wstarter2.wav"
for MESH in dashboard.mesh leftmirror.mesh rightmirror.mesh seat.mesh; do
    test -s "$APP_DIR/RoRResources/meshes/$MESH"
done
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
grep -q 'RoR::IOSOgre::RoRTerrainScene' "$NM_DEMANGLED"
grep -q 'RoR::IOSAudio::EngineAudio' "$NM_DEMANGLED"

(
    cd "$OUT_DIR"
    /usr/bin/zip -qry "$APP_NAME.ipa" Payload
)

[[ -f "$IPA" ]]
echo "Built unsigned OGRE 14 / Metal + Simple2 + full Foxbody vehicle/audio iPhone IPA: $IPA"