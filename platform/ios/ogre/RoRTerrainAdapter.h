#pragma once

#include "OgreVector3.h"

#include <memory>
#include <string>
#include <vector>

namespace Ogre { class SceneManager; }

namespace RoR {
namespace IOSOgre {

struct TerrainLayerDefinition
{
    float world_size = 10.0f;
    std::string diffuse_specular;
    std::string normal_height;
};

struct TerrainDefinition
{
    std::string name;
    std::string geometry_config;
    std::string object_config;
    Ogre::Vector3 start_position = Ogre::Vector3::ZERO;

    bool flat = false;
    int pages_x = 0;
    int pages_y = 0;
    unsigned int page_size = 1025;
    float world_size_x = 1024.0f;
    float world_size_z = 1024.0f;
    float world_size_y = 0.0f;
    unsigned int min_batch_size = 17;
    unsigned int max_batch_size = 65;
    unsigned int layer_blend_map_size = 128;
    unsigned int composite_map_size = 1024;
    float composite_map_distance = 4000.0f;
    float skirt_size = 30.0f;
    unsigned int light_map_size = 1024;
    float max_pixel_error = 5.0f;
    std::string page_config;
    std::vector<TerrainLayerDefinition> layers;
};

// Small platform adapter around the real RoR terrn2/OTC files and OGRE Terrain.
// It intentionally supports the subset used by the first stock iOS terrain
// (Simple2) and fails loudly for unsupported layouts rather than inventing a
// separate terrain format or renderer.
class RoRTerrainScene
{
public:
    RoRTerrainScene(Ogre::SceneManager* scene,
                    const std::string& terrain_directory,
                    const std::string& material_template_name);
    ~RoRTerrainScene();

    RoRTerrainScene(const RoRTerrainScene&) = delete;
    RoRTerrainScene& operator=(const RoRTerrainScene&) = delete;

    bool Ready() const;
    const std::string& Error() const;
    const TerrainDefinition& Definition() const;
    Ogre::Vector3 StartPosition() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace IOSOgre
} // namespace RoR
