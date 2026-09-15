/*
    Visual geometry extraction for the iOS OGRE bring-up.
    This keeps rendering data separate from the portable physics parser while
    preserving the authored RoR cab triangles, texture coordinates, wheel definitions,
    prop attachments, flexbody attachment/forset metadata, and managed materials.
*/

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace RoR {
namespace IOSOgre {

struct TextureCoord
{
    float u = 0.0f;
    float v = 0.0f;
};

struct CabTriangle
{
    std::size_t a = 0;
    std::size_t b = 0;
    std::size_t c = 0;
    TextureCoord uv_a;
    TextureCoord uv_b;
    TextureCoord uv_c;
    bool has_uv = false;
    bool contact = false;
};

struct WheelVisual
{
    std::size_t axis0 = 0;
    std::size_t axis1 = 0;
    float radius = 0.0f;
    float width = 0.0f;
};

struct PropVisual
{
    std::size_t node_ref = 0;
    std::size_t node_x = 0;
    std::size_t node_y = 0;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
    float rot_x_degrees = 0.0f;
    float rot_y_degrees = 0.0f;
    float rot_z_degrees = 0.0f;
    std::string mesh_name;
};

struct ManagedMaterialVisual
{
    std::string name;
    std::string type;
    std::string diffuse_texture;
    std::string specular_texture;
    std::string damage_texture;
    bool transparent = false;
};

// Mirrors the authored fields consumed by upstream ActorSpawner::ProcessFlexbody().
// `node_indices` is RoR's `forset`: the candidate rig nodes used to bind each
// mesh vertex to a deforming local basis. Keeping this data explicit lets the
// iOS renderer use the same nearest-node locator algorithm as FlexBody.cpp.
struct FlexBodyVisual
{
    std::size_t node_ref = 0;
    std::size_t node_x = 0;
    std::size_t node_y = 0;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
    float rot_x_degrees = 0.0f;
    float rot_y_degrees = 0.0f;
    float rot_z_degrees = 0.0f;
    std::string mesh_name;
    std::vector<std::size_t> node_indices;
    bool has_forvert_directives = false;
};

struct AuthoredVisualGeometry
{
    std::vector<CabTriangle> cab_triangles;
    std::vector<WheelVisual> wheels;
    std::vector<PropVisual> props;
    std::vector<FlexBodyVisual> flexbodies;
    std::vector<ManagedMaterialVisual> managed_materials;
    std::vector<std::string> warnings;
};

AuthoredVisualGeometry ParseAuthoredVisualGeometry(const std::string& truck_text);

} // namespace IOSOgre
} // namespace RoR
