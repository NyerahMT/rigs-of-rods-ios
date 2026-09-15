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

# OGRE 14.6's iOS MetalRenderWindow is created before its OgreMetalView is
# attached to a UIWindow. Seed/repair CAMetalLayer's pixel size explicitly so a
# UIKit-hosted first frame cannot remain 0x0 and black.
python3 - "$OGRE_SRC" <<'PY'
from pathlib import Path
import sys
src = Path(sys.argv[1]) / "RenderSystems/Metal/src/OgreMetalRenderWindow.mm"
text = src.read_text()
create_old = """        mMetalLayer.framebufferOnly = YES;\n\n        this->init( nil, nil );\n"""
create_new = """        mMetalLayer.framebufferOnly = YES;\n\n#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS\n        const CGFloat initialScale = [UIScreen mainScreen].nativeScale;\n        [mMetalView setContentScaleFactor:initialScale];\n        mMetalLayer.contentsScale = initialScale;\n        mMetalLayer.drawableSize = CGSizeMake(frame.size.width * initialScale,\n                                               frame.size.height * initialScale);\n        mMetalView.layerSizeDidUpdate = YES;\n#endif\n\n        this->init( nil, nil );\n"""
if create_old not in text:
    raise SystemExit("OGRE iOS Metal create() patch anchor changed")
text = text.replace(create_old, create_new, 1)
next_old = """                if( mMetalView.layerSizeDidUpdate )\n                    checkLayerSizeChanges();\n\n                // do not retain current drawable beyond the frame.\n"""
next_new = """#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS\n                if( mMetalView.window )\n                {\n                    const CGFloat scale = mMetalView.contentScaleFactor > 0.0 ?\n                                              mMetalView.contentScaleFactor :\n                                              mMetalView.window.screen.nativeScale;\n                    const CGSize expected = CGSizeMake(mMetalView.bounds.size.width * scale,\n                                                       mMetalView.bounds.size.height * scale);\n                    if( !CGSizeEqualToSize(mMetalLayer.drawableSize, expected) )\n                    {\n                        mMetalLayer.contentsScale = scale;\n                        mMetalLayer.drawableSize = expected;\n                        mMetalView.layerSizeDidUpdate = YES;\n                    }\n                }\n#endif\n                if( mMetalView.layerSizeDidUpdate )\n                    checkLayerSizeChanges();\n\n                // do not retain current drawable beyond the frame.\n"""
if next_old not in text:
    raise SystemExit("OGRE iOS Metal nextDrawable() patch anchor changed")
text = text.replace(next_old, next_new, 1)
src.write_text(text)
print("Applied iOS Metal drawable-size synchronization patch")
PY

# OGRE 14.6 binds all GPU parameter blocks with setVertexBytes(), including
# fragment programs and zero-length blocks. iOS 27 Metal validation rejects
# that. Bind only non-empty blocks and bind them to the correct shader stage.
python3 - "$OGRE_SRC" <<'PY'
from pathlib import Path
import sys
src = Path(sys.argv[1]) / "RenderSystems/Metal/src/OgreMetalRenderSystem.mm"
text = src.read_text()
old = """        // update const buffer\n        #if 1\n        [mActiveRenderEncoder setVertexBytes:params->getFloatPointer(0)\n            length:params->getConstantList().size() atIndex:MetalProgram::UNIFORM_INDEX_START];\n        #else\n        // TODO rather use this, but buffer seems to be never updated\n        size_t unused;\n        mAutoParamsBuffer->writeData(0, params->getConstantList().size(), params->getFloatPointer(0));\n        [mActiveRenderEncoder setVertexBuffer:mAutoParamsBuffer->getBufferName(unused) offset:0 atIndex:MetalProgram::UNIFORM_INDEX_START];\n        #endif\n"""
new = """        const size_t constantBytes = params->getConstantList().size();\n        if( constantBytes == 0u )\n            return;\n\n        switch( gptype )\n        {\n        case GPT_VERTEX_PROGRAM:\n            [mActiveRenderEncoder setVertexBytes:params->getFloatPointer(0)\n                length:constantBytes atIndex:MetalProgram::UNIFORM_INDEX_START];\n            break;\n        case GPT_FRAGMENT_PROGRAM:\n            [mActiveRenderEncoder setFragmentBytes:params->getFloatPointer(0)\n                length:constantBytes atIndex:MetalProgram::UNIFORM_INDEX_START];\n            break;\n        default:\n            break;\n        }\n"""
if old not in text:
    raise SystemExit("OGRE Metal GPU-parameter patch anchor changed")
text = text.replace(old, new, 1)
src.write_text(text)
print("Applied OGRE Metal stage-correct/zero-length GPU parameter patch")
PY

# OGRE 14.6 sizes Metal constant buffers by summing reflected members. That
# misses C/Metal struct tail padding (for example float4x4 + float2 is 80 bytes,
# not 72). The iOS debug layer validates the full reflected bufferDataSize at
# draw time and aborts if setVertexBytes() is shorter. Preserve OGRE's member
# offsets, but grow the backing parameter block to the size Metal reports.
python3 - "$OGRE_SRC" <<'PY'
from pathlib import Path
import sys
src = Path(sys.argv[1]) / "RenderSystems/Metal/src/OgreMetalProgram.mm"
text = src.read_text()
old = """                mConstantDefs->map.insert( GpuConstantDefinitionMap::value_type( varName, def ) );\n            }\n        }\n    }\n    //-----------------------------------------------------------------------\n    void MetalProgram::unloadHighLevelImpl(void)\n"""
new = """                mConstantDefs->map.insert( GpuConstantDefinitionMap::value_type( varName, def ) );\n            }\n        }\n\n        const size_t reflectedBufferFloats =\n                (arg.bufferDataSize + sizeof(float) - 1u) / sizeof(float);\n        if( mConstantDefs->bufferSize < reflectedBufferFloats )\n        {\n            mConstantDefs->bufferSize = reflectedBufferFloats;\n            mLogicalToPhysical->bufferSize = reflectedBufferFloats;\n        }\n    }\n    //-----------------------------------------------------------------------\n    void MetalProgram::unloadHighLevelImpl(void)\n"""
if old not in text:
    raise SystemExit("OGRE Metal reflected-buffer-size patch anchor changed")
text = text.replace(old, new, 1)
src.write_text(text)
print("Applied OGRE Metal reflected constant-buffer padding patch")
PY

# RoR-era meshes legitimately use OGRE's v1.30-v1.41 serializers, whose own
# loader comments note that old chunk sizes are sometimes wrong. OGRE 14 added
# a strict whole-stream bounds check in Serializer::readChunk(); it rejects
# these otherwise readable legacy files before the version-specific reader can
# apply its compatibility logic. Preserve strict checking for modern formats,
# but clamp only pre-1.8 mesh chunks to the bytes remaining in the stream.
python3 - "$OGRE_SRC" <<'PY'
from pathlib import Path
import sys
src = Path(sys.argv[1]) / "OgreMain/src/OgreSerializer.cpp"
text = src.read_text()
old = """        if (!stream->eof())\n        {\n            // chunk size cant be smaller than the header size, and must fit within the stream\n            if (mCurrentstreamLen < calcChunkHeaderSize() || pos + mCurrentstreamLen > stream->size())\n                OGRE_EXCEPT(Exception::ERR_INTERNAL_ERROR, \"Corrupt chunk length\");\n        }\n"""
new = """        if (!stream->eof())\n        {\n            // Pre-1.8 OGRE mesh exporters are known to have emitted incorrect\n            // chunk lengths. Keep modern streams strict, but let their dedicated\n            // legacy readers consume the actual bytes that remain.\n            const size_t remaining = stream->size() >= pos ? stream->size() - pos : 0u;\n            if (mCurrentstreamLen < calcChunkHeaderSize())\n                OGRE_EXCEPT(Exception::ERR_INTERNAL_ERROR, \"Corrupt chunk length\");\n\n            if (static_cast<size_t>(mCurrentstreamLen) > remaining)\n            {\n                const bool legacyMeshVersion =\n                    mVersion == \"[MeshSerializer_v1.41]\" ||\n                    mVersion == \"[MeshSerializer_v1.40]\" ||\n                    mVersion == \"[MeshSerializer_v1.30]\" ||\n                    mVersion == \"[MeshSerializer_v1.20]\" ||\n                    mVersion == \"[MeshSerializer_v1.10]\";\n                if (!legacyMeshVersion)\n                    OGRE_EXCEPT(Exception::ERR_INTERNAL_ERROR, \"Corrupt chunk length\");\n                mCurrentstreamLen = static_cast<uint32>(remaining);\n            }\n        }\n"""
if old not in text:
    raise SystemExit("OGRE legacy mesh chunk-bounds patch anchor changed")
text = text.replace(old, new, 1)
src.write_text(text)
print("Applied OGRE pre-1.8 legacy mesh chunk-bounds compatibility patch")
PY

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
    -DOGRE_BUILD_COMPONENT_RTSHADERSYSTEM=ON \
    -DOGRE_BUILD_COMPONENT_TERRAIN=ON \
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
    -DOGRE_CONFIG_ENABLE_DDS=ON \
    -DOGRE_CONFIG_ENABLE_PVRTC=OFF \
    -DOGRE_CONFIG_ENABLE_ETC=OFF \
    -DOGRE_CONFIG_ENABLE_ASTC=OFF \
    -DOGRE_CONFIG_THREADS=0 \
    -DOGRE_ENABLE_PRECOMPILED_HEADERS=OFF

# Terrain links RTShaderSystem in OGRE 14. Build the same component stack RoR's
# TerrainGeometryManager sits on instead of drawing another bespoke ground pad.
cmake --build "$OGRE_BUILD" --config Release \
    --target OgreMain RenderSystem_Metal OgreRTShaderSystem OgreTerrain -j 3

OGRE_MAIN="$(find "$OGRE_BUILD" -name 'libOgreMainStatic.a' -print -quit)"
OGRE_METAL="$(find "$OGRE_BUILD" -name 'libRenderSystem_MetalStatic.a' -print -quit)"
OGRE_RTSS="$(find "$OGRE_BUILD" -name 'libOgreRTShaderSystemStatic.a' -print -quit)"
OGRE_TERRAIN="$(find "$OGRE_BUILD" -name 'libOgreTerrainStatic.a' -print -quit)"

if [[ -z "$OGRE_MAIN" || -z "$OGRE_METAL" || -z "$OGRE_RTSS" || -z "$OGRE_TERRAIN" ]]; then
    echo "error: expected OGRE static libraries were not produced" >&2
    find "$OGRE_BUILD" -maxdepth 6 -type f -name '*.a' -print >&2 || true
    exit 1
fi

for LIB in "$OGRE_MAIN" "$OGRE_METAL" "$OGRE_RTSS" "$OGRE_TERRAIN"; do
    echo "Built OGRE library: $LIB"
    file "$LIB"
    lipo -info "$LIB"
    lipo -info "$LIB" | grep -q 'arm64'
done

printf '%s\n' "$OGRE_COMMIT" > "$OGRE_BUILD/OGRE_PINNED_COMMIT.txt"
echo "OGRE 14.6 / Metal iPhone build complete (DDS + RTShaderSystem + Terrain enabled)."
