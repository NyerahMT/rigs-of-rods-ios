#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
OGRE_COMMIT="5cccbeec824798629931cfd9184bde729610e30d"
OGRE_SRC="${OGRE_SRC:-$ROOT/build/ogre-src}"
OGRE_BUILD="${OGRE_BUILD:-$ROOT/build/ogre-ios}"
SDK="$(xcrun --sdk iphoneos --show-sdk-path)"

if [[ ! -d "$OGRE_SRC/.git" ]]; then
    rm -rf "$OGRE_SRC"
    git clone --filter=blob:none --no-checkout https://github.com/OGRECave/ogre.git "$OGRE_SRC"
fi

git -C "$OGRE_SRC" fetch --depth=1 origin "$OGRE_COMMIT"
git -C "$OGRE_SRC" checkout --detach --force "$OGRE_COMMIT"

echo "Building pinned OGRE commit: $(git -C "$OGRE_SRC" rev-parse HEAD)"
grep -q 'project(OGRE VERSION 14.6.0)' "$OGRE_SRC/CMakeLists.txt"

rm -rf "$OGRE_BUILD"

cmake \
    -S "$OGRE_SRC" \
    -B "$OGRE_BUILD" \
    -DCMAKE_SYSTEM_NAME=iOS \
    -DCMAKE_OSX_SYSROOT="$SDK" \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=16.0 \
    -DCMAKE_BUILD_TYPE=Release \
    -DAPPLE_IOS=TRUE \
    -DIOS=TRUE \
    -DIOS_PLATFORM=OS \
    -DOGRE_STATIC=ON \
    -DOGRE_BUILD_LIBS_AS_FRAMEWORKS=OFF \
    -DOGRE_BUILD_DEPENDENCIES=OFF \
    -DOGRE_BUILD_RENDERSYSTEM_METAL=ON \
    -DOGRE_BUILD_SAMPLES=OFF \
    -DOGRE_BUILD_TOOLS=OFF \
    -DOGRE_BUILD_TESTS=OFF \
    -DOGRE_BUILD_COMPONENT_BITES=OFF \
    -DOGRE_BUILD_COMPONENT_OVERLAY=OFF \
    -DOGRE_BUILD_COMPONENT_OVERLAY_IMGUI=OFF \
    -DOGRE_BUILD_COMPONENT_RTSHADERSYSTEM=OFF \
    -DOGRE_BUILD_COMPONENT_TERRAIN=OFF \
    -DOGRE_BUILD_COMPONENT_PAGING=OFF \
    -DOGRE_BUILD_COMPONENT_VOLUME=OFF \
    -DOGRE_BUILD_COMPONENT_MESHLODGENERATOR=OFF \
    -DOGRE_BUILD_COMPONENT_PROPERTY=OFF \
    -DOGRE_BUILD_COMPONENT_BULLET=OFF \
    -DOGRE_BUILD_COMPONENT_PYTHON=OFF \
    -DOGRE_BUILD_COMPONENT_JAVA=OFF \
    -DOGRE_BUILD_COMPONENT_CSHARP=OFF \
    -DOGRE_BUILD_PLUGIN_ASSIMP=OFF \
    -DOGRE_BUILD_PLUGIN_BSP=OFF \
    -DOGRE_BUILD_PLUGIN_OCTREE=OFF \
    -DOGRE_BUILD_PLUGIN_PFX=OFF \
    -DOGRE_BUILD_PLUGIN_PCZ=OFF \
    -DOGRE_BUILD_PLUGIN_DOT_SCENE=OFF \
    -DOGRE_BUILD_PLUGIN_FREEIMAGE=OFF \
    -DOGRE_BUILD_PLUGIN_EXRCODEC=OFF \
    -DOGRE_BUILD_PLUGIN_STBI=OFF \
    -DOGRE_CONFIG_ENABLE_ZIP=OFF \
    -DOGRE_CONFIG_ENABLE_DDS=OFF \
    -DOGRE_CONFIG_ENABLE_PVRTC=OFF \
    -DOGRE_CONFIG_ENABLE_ETC=OFF \
    -DOGRE_CONFIG_ENABLE_ASTC=OFF \
    -DOGRE_CONFIG_THREADS=0 \
    -DOGRE_ENABLE_PRECOMPILED_HEADERS=OFF

cmake --build "$OGRE_BUILD" --config Release --target OgreMain RenderSystem_Metal -j 3

OGRE_MAIN="$(find "$OGRE_BUILD" -name 'libOgreMainStatic.a' -print -quit)"
OGRE_METAL="$(find "$OGRE_BUILD" -name 'libRenderSystem_MetalStatic.a' -print -quit)"

if [[ -z "$OGRE_MAIN" || -z "$OGRE_METAL" ]]; then
    echo "error: expected OGRE static libraries were not produced" >&2
    find "$OGRE_BUILD" -maxdepth 5 -type f -name '*.a' -print >&2 || true
    exit 1
fi

for LIB in "$OGRE_MAIN" "$OGRE_METAL"; do
    echo "Built OGRE library: $LIB"
    file "$LIB"
    lipo -info "$LIB"
    lipo -info "$LIB" | grep -q 'arm64'
done

printf '%s\n' "$OGRE_COMMIT" > "$OGRE_BUILD/OGRE_PINNED_COMMIT.txt"
echo "OGRE 14.6 / Metal iPhone build complete."
