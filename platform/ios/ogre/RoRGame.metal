#include "OgreUnifiedShader.h"

struct RasterizerData
{
    vec4 pos [[position]];
    vec4 colour;
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
    return texel * half4(in.colour);
}
