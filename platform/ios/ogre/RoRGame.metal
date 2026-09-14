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

struct Vertex
{
    IN(vec3 pos, POSITION);
    IN(vec4 colour, COLOR0);
};

// Stock RoR .mesh props do not necessarily carry a vertex-colour stream.
// Keep the bring-up shader intentionally position-only so OGRE's Metal PSO
// never requires COLOR0 from dashboard/mirror/seat meshes.
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

struct Uniform
{
    mat4 mvpMtx;
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

    // The stock DAF material uses `alpha_rejection greater 128`. On Metal the
    // programmable fragment stage owns this test, so reproduce the RoR/OGRE
    // material rule here instead of drawing transparent atlas pixels as black.
    if (texel.a <= ROR_ALPHA_REJECT)
        metal::discard_fragment();

    return texel * half4(in.colour);
}

fragment half4 ror_vehicle_emissive_fp(TexturedRasterizerData in [[stage_in]],
                                       metal::texture2d<half> emissive_map [[texture(0)]],
                                       metal::sampler emissive_sampler [[sampler(0)]])
{
    // Kept separate from the lit diffuse shader so OGRE can reproduce the
    // stock material's additive, depth-write-off emissive pass exactly.
    return emissive_map.sample(emissive_sampler, in.uv);
}
