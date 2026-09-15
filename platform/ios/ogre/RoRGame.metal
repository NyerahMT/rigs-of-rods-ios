#include "OgreUnifiedShader.h"

struct RasterizerData
{
    vec4 pos [[position]];
    vec4 colour;
};

struct PropRasterizerData
{
    vec4 pos [[position]];
};

struct TexturedRasterizerData
{
    vec4 pos [[position]];
    vec4 colour;
    vec2 uv;
};

struct FlexRasterizerData
{
    vec4 pos [[position]];
    vec2 uv;
};

struct TerrainRasterizerData
{
    vec4 pos [[position]];
    vec2 uv;
};

struct Vertex
{
    IN(vec3 pos, POSITION);
    IN(vec4 colour, COLOR0);
};

struct PropVertex
{
    IN(vec3 pos, POSITION);
};

struct TexturedVertex
{
    IN(vec3 pos, POSITION);
    IN(vec4 colour, COLOR0);
    IN(vec2 uv, TEXCOORD0);
};

// Authored RoR .mesh flexbodies carry positions, normals and UVs but generally
// no vertex-colour stream. Keep a distinct input layout rather than asking the
// DAF collision-cab shader for COLOR0 that the flexbody mesh never authored.
struct FlexVertex
{
    IN(vec3 pos, POSITION);
    IN(vec2 uv, TEXCOORD0);
};

// OGRE Terrain uses POSITION + TEXCOORD0 when vertex compression is disabled.
// TEXCOORD1 contains its LOD delta data but does not need to be consumed by the
// first flat Simple2 pass because morphing is explicitly disabled by the adapter.
struct TerrainVertex
{
    IN(vec3 pos, POSITION);
    IN(vec2 uv, TEXCOORD0);
};

struct Uniform
{
    mat4 mvpMtx;
};

struct TerrainUniform
{
    mat4 mvpMtx;
    vec2 uvScale;
};

#define UNIFORM_INDEX_START 16
#define ROR_ALPHA_REJECT (128.0h / 255.0h)

vertex RasterizerData ror_game_vp(Vertex in [[stage_in]],
                                  constant Uniform& u [[buffer(UNIFORM_INDEX_START)]])
{
    RasterizerData out;
    out.pos = u.mvpMtx * vec4(in.pos, 1.0);
    out.colour = in.colour;
    return out;
}

fragment half4 ror_game_fp(RasterizerData in [[stage_in]])
{
    return half4(in.colour);
}

vertex PropRasterizerData ror_prop_vp(PropVertex in [[stage_in]],
                                      constant Uniform& u [[buffer(UNIFORM_INDEX_START)]])
{
    PropRasterizerData out;
    out.pos = u.mvpMtx * vec4(in.pos, 1.0);
    return out;
}

fragment half4 ror_prop_fp(PropRasterizerData in [[stage_in]])
{
    (void)in;
    return half4(0.32h, 0.34h, 0.36h, 1.0h);
}

vertex TerrainRasterizerData ror_terrain_vp(TerrainVertex in [[stage_in]],
                                            constant TerrainUniform& u [[buffer(UNIFORM_INDEX_START)]])
{
    TerrainRasterizerData out;
    out.pos = u.mvpMtx * vec4(in.pos, 1.0);
    out.uv = in.uv * u.uvScale;
    return out;
}

fragment half4 ror_terrain_fp(TerrainRasterizerData in [[stage_in]],
                              metal::texture2d<half> albedo_specular [[texture(0)]],
                              metal::sampler terrain_sampler [[sampler(0)]])
{
    const half4 texel = albedo_specular.sample(terrain_sampler, in.uv);
    return half4(texel.rgb, 1.0h);
}

vertex TexturedRasterizerData ror_vehicle_vp(TexturedVertex in [[stage_in]],
                                             constant Uniform& u [[buffer(UNIFORM_INDEX_START)]])
{
    TexturedRasterizerData out;
    out.pos = u.mvpMtx * vec4(in.pos, 1.0);
    out.colour = in.colour;
    out.uv = in.uv;
    return out;
}

fragment half4 ror_vehicle_fp(TexturedRasterizerData in [[stage_in]],
                              metal::texture2d<half> diffuse_map [[texture(0)]],
                              metal::sampler diffuse_sampler [[sampler(0)]])
{
    const half4 texel = diffuse_map.sample(diffuse_sampler, in.uv);
    if (texel.a <= ROR_ALPHA_REJECT)
        metal::discard_fragment();
    return texel * half4(in.colour);
}

fragment half4 ror_vehicle_emissive_fp(TexturedRasterizerData in [[stage_in]],
                                       metal::texture2d<half> emissive_map [[texture(0)]],
                                       metal::sampler emissive_sampler [[sampler(0)]])
{
    return emissive_map.sample(emissive_sampler, in.uv);
}

vertex FlexRasterizerData ror_flex_vp(FlexVertex in [[stage_in]],
                                      constant Uniform& u [[buffer(UNIFORM_INDEX_START)]])
{
    FlexRasterizerData out;
    out.pos = u.mvpMtx * vec4(in.pos, 1.0);
    out.uv = in.uv;
    return out;
}

fragment half4 ror_flex_fp(FlexRasterizerData in [[stage_in]],
                           metal::texture2d<half> diffuse_map [[texture(0)]],
                           metal::sampler diffuse_sampler [[sampler(0)]])
{
    const half4 texel = diffuse_map.sample(diffuse_sampler, in.uv);
    if (texel.a <= ROR_ALPHA_REJECT)
        metal::discard_fragment();
    return texel;
}
