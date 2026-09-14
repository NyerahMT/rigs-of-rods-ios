/*
    Visual geometry extraction for the iOS OGRE bring-up.
    This keeps rendering data separate from the portable physics parser while
    preserving the authored RoR cab triangles and wheel definitions.
*/

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace RoR {
namespace IOSOgre {

struct CabTriangle
{
    std::size_t a = 0;
    std::size_t b = 0;
    std::size_t c = 0;
    bool contact = false;
};

struct WheelVisual
{
    std::size_t axis0 = 0;
    std::size_t axis1 = 0;
    float radius = 0.0f;
    float width = 0.0f;
};

struct AuthoredVisualGeometry
{
    std::vector<CabTriangle> cab_triangles;
    std::vector<WheelVisual> wheels;
    std::vector<std::string> warnings;
};

AuthoredVisualGeometry ParseAuthoredVisualGeometry(const std::string& truck_text);

} // namespace IOSOgre
} // namespace RoR
