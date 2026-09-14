#include "RoRTerrainAdapter.h"

#include "OgreException.h"
#include "OgreMaterial.h"
#include "OgreMaterialManager.h"
#include "OgreResourceGroupManager.h"
#include "OgreString.h"
#include "OgreTerrain.h"
#include "OgreTerrainGroup.h"
#include "OgreTerrainMaterialGenerator.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace RoR {
namespace IOSOgre {
namespace {

std::string Trim(const std::string& input)
{
    std::size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) ++begin;
    std::size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) --end;
    return input.substr(begin, end - begin);
}

std::string StripComment(const std::string& input)
{
    std::size_t end = input.size();
    const std::size_t semicolon = input.find(';');
    const std::size_t hash = input.find('#');
    const std::size_t slash = input.find("//");
    if (semicolon != std::string::npos) end = std::min(end, semicolon);
    if (hash != std::string::npos) end = std::min(end, hash);
    if (slash != std::string::npos) end = std::min(end, slash);
    return Trim(input.substr(0, end));
}

std::string NormalizeKey(std::string key)
{
    std::string out;
    for (unsigned char c : key)
        if (std::isalnum(c)) out.push_back(static_cast<char>(std::tolower(c)));
    return out;
}

std::string JoinPath(const std::string& dir, const std::string& file)
{
    if (dir.empty()) return file;
    if (file.empty()) return dir;
    if (dir.back() == '/') return dir + file;
    return dir + "/" + file;
}

std::vector<std::string> ReadDataLines(const std::string& path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open " + path);
    std::vector<std::string> lines;
    std::string raw;
    while (std::getline(input, raw))
    {
        std::string line = StripComment(raw);
        if (!line.empty()) lines.push_back(std::move(line));
    }
    return lines;
}

std::unordered_map<std::string, std::string> ReadKeyValues(const std::string& path)
{
    std::unordered_map<std::string, std::string> values;
    for (const std::string& line : ReadDataLines(path))
    {
        if (!line.empty() && line.front() == '[') continue;
        const std::size_t equal = line.find('=');
        if (equal == std::string::npos) continue;
        values[NormalizeKey(line.substr(0, equal))] = Trim(line.substr(equal + 1));
    }
    return values;
}

std::string GetString(const std::unordered_map<std::string, std::string>& values,
                      const char* key, const std::string& fallback = {})
{
    auto it = values.find(NormalizeKey(key));
    return it == values.end() ? fallback : it->second;
}

int GetInt(const std::unordered_map<std::string, std::string>& values, const char* key, int fallback)
{
    const std::string text = GetString(values, key);
    if (text.empty()) return fallback;
    return std::stoi(text);
}

float GetFloat(const std::unordered_map<std::string, std::string>& values, const char* key, float fallback)
{
    const std::string text = GetString(values, key);
    if (text.empty()) return fallback;
    return std::stof(text);
}

bool GetBool(const std::unordered_map<std::string, std::string>& values, const char* key, bool fallback)
{
    std::string text = GetString(values, key);
    if (text.empty()) return fallback;
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text == "1" || text == "true" || text == "yes" || text == "on";
}

Ogre::Vector3 ParseVector3(std::string text, Ogre::Vector3 fallback)
{
    for (char& c : text) if (c == ',') c = ' ';
    std::istringstream stream(text);
    float x = 0, y = 0, z = 0;
    if (stream >> x >> y >> z) return {x, y, z};
    return fallback;
}

std::vector<std::string> SplitComma(const std::string& line)
{
    std::vector<std::string> values;
    std::size_t start = 0;
    while (start <= line.size())
    {
        const std::size_t comma = line.find(',', start);
        values.push_back(Trim(line.substr(start, comma == std::string::npos ? std::string::npos : comma - start)));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return values;
}

TerrainDefinition ParseDefinition(const std::string& directory)
{
    TerrainDefinition result;
    const std::string terrn2_path = JoinPath(directory, "simple2.terrn2");
    const auto terrn2 = ReadKeyValues(terrn2_path);
    result.name = GetString(terrn2, "Name", "RoR Terrain");
    result.geometry_config = GetString(terrn2, "GeometryConfig");
    result.start_position = ParseVector3(GetString(terrn2, "StartPosition"), Ogre::Vector3::ZERO);
    result.object_config = GetString(terrn2, "Objects");
    if (result.geometry_config.empty())
        throw std::runtime_error("terrn2 has no GeometryConfig");

    const auto otc = ReadKeyValues(JoinPath(directory, result.geometry_config));
    result.flat = GetBool(otc, "Flat", false);
    result.pages_x = GetInt(otc, "Pages_X", 0);
    result.pages_y = GetInt(otc, "Pages_Y", 0);
    result.page_size = static_cast<unsigned int>(GetInt(otc, "PageSize", 1025));
    result.world_size_x = GetFloat(otc, "WorldSizeX", 1024.0f);
    result.world_size_z = GetFloat(otc, "WorldSizeZ", 1024.0f);
    result.world_size_y = GetFloat(otc, "WorldSizeY", 0.0f);
    result.min_batch_size = static_cast<unsigned int>(GetInt(otc, "minBatchSize", 17));
    result.max_batch_size = static_cast<unsigned int>(GetInt(otc, "maxBatchSize", 65));
    result.layer_blend_map_size = static_cast<unsigned int>(GetInt(otc, "LayerBlendMapSize", 128));
    result.composite_map_size = static_cast<unsigned int>(GetInt(otc, "CompositeMapSize", 1024));
    result.composite_map_distance = GetFloat(otc, "CompositeMapDistance", 4000.0f);
    result.skirt_size = GetFloat(otc, "SkirtSize", 30.0f);
    result.light_map_size = static_cast<unsigned int>(GetInt(otc, "LightMapSize", 1024));
    result.max_pixel_error = GetFloat(otc, "MaxPixelError", 5.0f);
    result.page_config = GetString(otc, "PageFileFormat");

    if (result.pages_x != 0 || result.pages_y != 0)
        throw std::runtime_error("first iOS terrain adapter currently supports one OTC page only");
    if (!result.flat)
        throw std::runtime_error("first iOS terrain adapter requires Flat=1 until terrain collision is connected to vehicle physics");
    if (result.page_size < 3 || result.page_size > 65535)
        throw std::runtime_error("invalid OTC PageSize");
    if (result.page_config.empty())
        throw std::runtime_error("OTC has no PageFileFormat");

    const std::vector<std::string> page_lines = ReadDataLines(JoinPath(directory, result.page_config));
    if (page_lines.size() < 2)
        throw std::runtime_error("OTC page config is incomplete");
    const int layer_count = std::stoi(page_lines[1]);
    if (layer_count < 1)
        throw std::runtime_error("OTC page contains no terrain layers");
    if (static_cast<int>(page_lines.size()) < 2 + layer_count)
        throw std::runtime_error("OTC page layer list is truncated");

    for (int i = 0; i < layer_count; ++i)
    {
        const std::vector<std::string> fields = SplitComma(page_lines[2 + i]);
        if (fields.size() < 3)
            throw std::runtime_error("OTC terrain layer is malformed");
        TerrainLayerDefinition layer;
        layer.world_size = std::stof(fields[0]);
        layer.diffuse_specular = fields[1];
        layer.normal_height = fields[2];
        result.layers.push_back(std::move(layer));
    }

    return result;
}

class IOSRoRTerrainMaterialGenerator final : public Ogre::TerrainMaterialGenerator
{
public:
    explicit IOSRoRTerrainMaterialGenerator(std::string material_template)
        : m_material_template(std::move(material_template))
    {
        // Match the real RoR/OGRE terrain layer contract even though the first
        // Metal terrain pass samples only the diffuse/specular texture. Keeping
        // both sampler slots means OTC layer data remains structurally authentic.
        mLayerDecl = {
            Ogre::TerrainLayerSampler("albedo_specular", Ogre::PF_BYTE_RGBA),
            Ogre::TerrainLayerSampler("normal_height", Ogre::PF_BYTE_RGBA)
        };
    }

    bool isVertexCompressionSupported() const override { return false; }

    void requestOptions(Ogre::Terrain* terrain) override
    {
        terrain->_setMorphRequired(false);
        terrain->_setNormalMapRequired(false);
        terrain->_setLightMapRequired(false);
        terrain->_setCompositeMapRequired(false);
    }

    Ogre::MaterialPtr generate(const Ogre::Terrain* terrain) override
    {
        Ogre::MaterialPtr source = Ogre::MaterialManager::getSingleton().getByName(m_material_template);
        if (!source)
            OGRE_EXCEPT(Ogre::Exception::ERR_ITEM_NOT_FOUND,
                        "iOS terrain material template is missing: " + m_material_template,
                        "IOSRoRTerrainMaterialGenerator::generate");

        const Ogre::String target_name = terrain->getMaterialName();
        Ogre::MaterialPtr old = Ogre::MaterialManager::getSingleton().getByName(target_name);
        if (old) Ogre::MaterialManager::getSingleton().remove(target_name);
        return source->clone(target_name);
    }

    Ogre::MaterialPtr generateForCompositeMap(const Ogre::Terrain* terrain) override
    {
        return generate(terrain);
    }

    Ogre::uint8 getMaxLayers(const Ogre::Terrain*) const override { return 1; }

private:
    std::string m_material_template;
};

} // namespace

struct RoRTerrainScene::Impl
{
    Ogre::SceneManager* scene = nullptr;
    Ogre::TerrainGlobalOptions* globals = nullptr;
    Ogre::TerrainGroup* group = nullptr;
    bool owns_globals = false;
    bool ready = false;
    std::string error;
    TerrainDefinition definition;

    Impl(Ogre::SceneManager* scene_manager,
         const std::string& directory,
         const std::string& material_template)
        : scene(scene_manager)
    {
        try
        {
            if (!scene) throw std::runtime_error("terrain has no OGRE SceneManager");
            definition = ParseDefinition(directory);
            if (definition.layers.size() != 1)
                throw std::runtime_error("first Simple2 Metal material currently supports exactly one OTC layer");

            globals = Ogre::TerrainGlobalOptions::getSingletonPtr();
            if (!globals)
            {
                globals = OGRE_NEW Ogre::TerrainGlobalOptions();
                owns_globals = true;
            }

            Ogre::TerrainMaterialGeneratorPtr generator(
                OGRE_NEW IOSRoRTerrainMaterialGenerator(material_template));
            globals->setDefaultMaterialGenerator(generator);
            globals->setMaxPixelError(definition.max_pixel_error);
            globals->setLayerBlendMapSize(static_cast<Ogre::uint16>(definition.layer_blend_map_size));
            globals->setCompositeMapSize(static_cast<Ogre::uint16>(definition.composite_map_size));
            globals->setCompositeMapDistance(definition.composite_map_distance);
            globals->setSkirtSize(definition.skirt_size);
            globals->setLightMapSize(static_cast<Ogre::uint16>(definition.light_map_size));
            globals->setUseRayBoxDistanceCalculation(false);
            globals->setUseVertexCompressionWhenAvailable(false);
            globals->setDefaultResourceGroup(Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            globals->setDefaultLayerTextureWorldSize(definition.layers[0].world_size);

            const float world_size = std::max(definition.world_size_x, definition.world_size_z);
            group = OGRE_NEW Ogre::TerrainGroup(scene,
                                                 Ogre::Terrain::ALIGN_X_Z,
                                                 static_cast<Ogre::uint16>(definition.page_size),
                                                 world_size);
            group->setOrigin(Ogre::Vector3(definition.world_size_x * 0.5f,
                                           0.0f,
                                           definition.world_size_z * 0.5f));
            group->setResourceGroup(Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

            Ogre::Terrain::ImportData& import = group->getDefaultImportSettings();
            import.terrainSize = static_cast<Ogre::uint16>(definition.page_size);
            import.worldSize = world_size;
            import.inputScale = definition.world_size_y;
            import.minBatchSize = static_cast<Ogre::uint16>(definition.min_batch_size);
            import.maxBatchSize = static_cast<Ogre::uint16>(definition.max_batch_size);
            import.layerDeclaration = generator->getLayerDeclaration();
            import.layerList.resize(1);
            import.layerList[0].worldSize = definition.layers[0].world_size;
            import.layerList[0].textureNames.push_back(definition.layers[0].diffuse_specular);
            import.layerList[0].textureNames.push_back(definition.layers[0].normal_height);

            // Simple2 is authored Flat=1. This is exactly what desktop RoR's
            // TerrainGeometryManager::SetupGeometry() does for the same page.
            group->defineTerrain(0, 0, 0.0f);
            group->loadAllTerrains(true);
            if (!group->getTerrain(0, 0))
                throw std::runtime_error("OGRE Terrain failed to instantiate OTC page 0,0");
            group->freeTemporaryResources();
            ready = true;
        }
        catch (const Ogre::Exception& e)
        {
            error = e.getFullDescription();
        }
        catch (const std::exception& e)
        {
            error = e.what();
        }
    }

    ~Impl()
    {
        if (group)
        {
            OGRE_DELETE group;
            group = nullptr;
        }
        if (owns_globals && globals)
        {
            OGRE_DELETE globals;
            globals = nullptr;
        }
    }
};

RoRTerrainScene::RoRTerrainScene(Ogre::SceneManager* scene,
                                 const std::string& terrain_directory,
                                 const std::string& material_template_name)
    : m_impl(new Impl(scene, terrain_directory, material_template_name))
{
}

RoRTerrainScene::~RoRTerrainScene() = default;

bool RoRTerrainScene::Ready() const { return m_impl && m_impl->ready; }
const std::string& RoRTerrainScene::Error() const { return m_impl->error; }
const TerrainDefinition& RoRTerrainScene::Definition() const { return m_impl->definition; }
Ogre::Vector3 RoRTerrainScene::StartPosition() const { return m_impl->definition.start_position; }

} // namespace IOSOgre
} // namespace RoR
