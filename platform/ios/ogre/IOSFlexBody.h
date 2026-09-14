/*
    iOS OGRE flexbody bridge.

    This is intentionally a small platform adapter around the same locator
    algorithm used by RoR's source/main/physics/flex/FlexBody.cpp. It keeps the
    production node-and-beam simulation authoritative and only owns the dynamic
    OGRE mesh buffers needed by the Metal game view.
*/
#pragma once

#include "AuthoredVisualGeometry.h"
#include "Ogre.h"

#include <cstddef>
#include <string>
#include <vector>

namespace RoR {
namespace IOSOgre {

class IOSFlexBody
{
public:
    IOSFlexBody(Ogre::SceneManager* scene,
                const FlexBodyVisual& definition,
                const std::vector<Ogre::Vector3>& initial_nodes,
                const std::string& unique_suffix);
    ~IOSFlexBody();

    IOSFlexBody(const IOSFlexBody&) = delete;
    IOSFlexBody& operator=(const IOSFlexBody&) = delete;

    bool Ready() const { return m_ready; }
    const std::string& Error() const { return m_error; }
    const std::string& MeshName() const { return m_definition.mesh_name; }

    void Update(const std::vector<Ogre::Vector3>& nodes,
                const Ogre::Vector3& world_offset);

private:
    struct Locator
    {
        std::size_t ref = 0;
        std::size_t nx = 0;
        std::size_t ny = 0;
        Ogre::Vector3 coords = Ogre::Vector3::ZERO;
        Ogre::Vector3 normal_coords = Ogre::Vector3::ZERO;
    };

    struct BufferSpan
    {
        Ogre::HardwareVertexBufferSharedPtr positions;
        Ogre::HardwareVertexBufferSharedPtr normals;
        std::size_t count = 0;
        std::size_t first_vertex = 0;
    };

    bool Build(const std::vector<Ogre::Vector3>& initial_nodes,
               const std::string& unique_suffix);
    bool MakeLocator(const Ogre::Vector3& vertex,
                     const std::vector<Ogre::Vector3>& nodes,
                     Locator& locator) const;
    bool AttachmentFrame(const std::vector<Ogre::Vector3>& nodes,
                         Ogre::Vector3& position,
                         Ogre::Quaternion& orientation) const;

    Ogre::SceneManager* m_scene = nullptr;
    FlexBodyVisual m_definition;
    Ogre::MeshPtr m_mesh;
    Ogre::Entity* m_entity = nullptr;
    Ogre::SceneNode* m_scene_node = nullptr;
    std::vector<BufferSpan> m_spans;
    std::vector<Locator> m_locators;
    std::vector<Ogre::Vector3> m_positions;
    std::vector<Ogre::Vector3> m_normals;
    bool m_ready = false;
    std::string m_error;
};

} // namespace IOSOgre
} // namespace RoR