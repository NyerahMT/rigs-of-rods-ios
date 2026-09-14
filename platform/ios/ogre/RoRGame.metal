#include "OgreUnifiedShader.h"

struct RasterizerData
{
    vec4 pos [[position]];
    vec4 colour;
};

struct Vertex
{
    IN(vec3 pos, POSITION);
    IN(vec4 colour, COLOR0);
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
