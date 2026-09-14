#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
OGRE_SRC="${OGRE_SRC:-$ROOT/build/ogre-src}"
OGRE_BUILD="${OGRE_BUILD:-$ROOT/build/ogre-ios}"
FMT_COMMIT="1be298e1bd68957e4cd352e1f676f00e07dcfb57" # fmt 12.2.0, same version as upstream RoR
FMT_SRC="${FMT_SRC:-$ROOT/build/fmt-src}"
OUT="${1:-$ROOT/build/ror-native-rigdef-ios}"
SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
CXX="$(xcrun --sdk iphoneos --find clang++)"
AR="$(xcrun --sdk iphoneos --find ar)"

if [[ ! -d "$OGRE_SRC" || ! -d "$OGRE_BUILD" ]]; then
    echo "error: OGRE source/build must exist before native RigDef build" >&2
    exit 1
fi

if [[ ! -d "$FMT_SRC/.git" ]]; then
    rm -rf "$FMT_SRC"
    git clone --filter=blob:none --no-checkout https://github.com/fmtlib/fmt.git "$FMT_SRC"
fi
git -C "$FMT_SRC" fetch --depth=1 origin "$FMT_COMMIT"
git -C "$FMT_SRC" checkout --detach --force "$FMT_COMMIT"

rm -rf "$OUT"
mkdir -p "$OUT/obj"

COMMON=(
    -arch arm64
    -isysroot "$SDK"
    -miphoneos-version-min=16.0
    -std=c++17
    -stdlib=libc++
    -O2
    -DFMT_HEADER_ONLY=1
    -I"$ROOT/platform/ios/ror-native/stubs"
    -I"$ROOT/platform/ios/ror-native"
    -I"$ROOT/source/main"
    -I"$ROOT/source/main/datatypes"
    -I"$ROOT/source/main/system"
    -I"$ROOT/source/main/utils"
    -I"$ROOT/source/main/utils/memory"
    -I"$ROOT/source/main/physics"
    -I"$ROOT/source/main/resources"
    -I"$ROOT/source/main/resources/rig_def_fileformat"
    -I"$ROOT/external/header_only"
    -I"$OGRE_SRC/OgreMain/include"
    -I"$OGRE_BUILD/include"
    -I"$FMT_SRC/include"
)

SOURCES=(
    "$ROOT/platform/ios/ror-native/NativeRoRHost.cpp"
    "$ROOT/platform/ios/ror-native/NativeRigDefBridge.cpp"
    "$ROOT/source/main/resources/rig_def_fileformat/RigDef_File.cpp"
    "$ROOT/source/main/resources/rig_def_fileformat/RigDef_Node.cpp"
    "$ROOT/source/main/resources/rig_def_fileformat/RigDef_Parser.cpp"
    "$ROOT/source/main/resources/rig_def_fileformat/RigDef_SequentialImporter.cpp"
)

OBJECTS=()
for SRC in "${SOURCES[@]}"; do
    BASE="$(basename "$SRC")"
    OBJ="$OUT/obj/${BASE%.cpp}.o"
    echo "Compiling native RoR source: ${SRC#$ROOT/}"
    "$CXX" "${COMMON[@]}" -c "$SRC" -o "$OBJ"
    OBJECTS+=("$OBJ")
done

LIB="$OUT/libror_native_rigdef.a"
"$AR" rcs "$LIB" "${OBJECTS[@]}"

file "$LIB"
lipo -info "$LIB" | grep -q arm64
printf '%s\n' "$FMT_COMMIT" > "$OUT/FMT_PINNED_COMMIT.txt"
echo "Built upstream RoR RigDef parser + SequentialImporter for iPhone ARM64: $LIB"
