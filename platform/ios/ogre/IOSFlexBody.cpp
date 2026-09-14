/*
    iOS OGRE flexbody bridge. Locator construction and per-frame deformation are
    kept intentionally equivalent to source/main/physics/flex/FlexBody.cpp.
*/
#include "IOSFlexBody.h"

#include <cmath>
#include <limits>

namespace RoR {
namespace IOSOgre {
namespace {

constexpr float kEpsilon = 1.0e-10f;
constexpr float kOrthogonalityLimit = 0.70710678118654752440f;

bool ValidNode(std::size_t index, const std::vector<Ogre::Vector3>& nodes)
{
    return index < nodes.size();
}

} // namespace

IOSFlexBody::IOSFlexBody(Ogre::SceneManager* scene,
                         const FlexBodyVisual& definition,
                         const std::vector<Ogre::Vector3>& initial_nodes,
                         const std::string& unique_suffix)
    : m_scene(scene), m_definition(definition)
{
    m_ready = Build(initial_nodes, unique_suffix);
}

IOSFlexBody::~IOSFlexBody()
{
    if (!m_scene) return;
    if (m_scene_node)
    {
        m_scene_node->detachAllObjects();
        m_scene->destroySceneNode(m_scene_node);
        m_scene_node = nullptr;
    }
    if (m_entity)
    {
        m_scene->destroyEntity(m_entity);
        m_entity = nullptr;
    }
}

bool IOSFlexBody::AttachmentFrame(const std::vector<Ogre::Vector3>& nodes,
                                  Ogre::Vector3& position,
                                  Ogre::Quaternion& orientation) const
{
    if (!ValidNode(m_definition.node_ref, nodes) ||
        !ValidNode(m_definition.node_x, nodes) ||
        !ValidNode(m_definition.node_y, nodes))
        return false;

    const Ogre::Vector3& ref = nodes[m_definition.node_ref];
    const Ogre::Vector3 diff_x = nodes[m_definition.node_x] - ref;
    const Ogre::Vector3 diff_y = nodes[m_definition.node_y] - ref;
    if (diff_x.squaredLength() < kEpsilon || diff_y.squaredLength() < kEpsilon)
        return false;

    Ogre::Vector3 normal = diff_y.crossProduct(diff_x);
    if (normal.squaredLength() < kEpsilon) return false;
    normal.normalise();

    position = ref + m_definition.offset_x * diff_x + m_definition.offset_y * diff_y +
               m_definition.offset_z * normal;

    const Ogre::Vector3 ref_x = diff_x.normalisedCopy();
    const Ogre::Vector3 ref_y = ref_x.crossProduct(normal);
    const Ogre::Quaternion authored_rotation =
        Ogre::Quaternion(Ogre::Degree(m_definition.rot_z_degrees), Ogre::Vector3::UNIT_Z) *
        Ogre::Quaternion(Ogre::Degree(m_definition.rot_y_degrees), Ogre::Vector3::UNIT_Y) *
        Ogre::Quaternion(Ogre::Degree(m_definition.rot_x_degrees), Ogre::Vector3::UNIT_X);
    orientation = Ogre::Quaternion(ref_x, normal, ref_y) * authored_rotation;
    return true;
}

bool IOSFlexBody::MakeLocator(const Ogre::Vector3& vertex,
                              const std::vector<Ogre::Vector3>& nodes,
                              Locator& locator) const
{
    float closest_distance = std::numeric_limits<float>::max();
    bool found = false;
    for (std::size_t node_index : m_definition.node_indices)
    {
        if (!ValidNode(node_index, nodes)) continue;
        const float distance = vertex.squaredDistance(nodes[node_index]);
        if (distance < closest_distance)
        {
            closest_distance = distance;
            locator.ref = node_index;
            found = true;
        }
    }
    if (!found) return false;

    closest_distance = std::numeric_limits<float>::max();
    found = false;
    for (std::size_t node_index : m_definition.node_indices)
    {
        if (!ValidNode(node_index, nodes) || node_index == locator.ref) continue;
        const float distance = vertex.squaredDistance(nodes[node_index]);
        if (distance < closest_distance)
        {
            closest_distance = distance;
            locator.nx = node_index;
            found = true;
        }
    }
    if (!found) return false;

    const Ogre::Vector3 vx = (nodes[locator.nx] - nodes[locator.ref]).normalisedCopy();
    closest_distance = std::numeric_limits<float>::max();
    found = false;
    for (std::size_t node_index : m_definition.node_indices)
    {
        if (!ValidNode(node_index, nodes) || node_index == locator.ref || node_index == locator.nx)
            continue;
        const Ogre::Vector3 vt = (nodes[node_index] - nodes[locator.ref]).normalisedCopy();
        if (std::abs(vx.dotProduct(vt)) > kOrthogonalityLimit) continue;
        const float distance = vertex.squaredDistance(nodes[node_index]);
        if (distance < closest_distance)
        {
            closest_distance = distance;
            locator.ny = node_index;
            found = true;
        }
    }
    if (!found) return false;

    const Ogre::Vector3 diff_x = nodes[locator.nx] - nodes[locator.ref];
    const Ogre::Vector3 diff_y = nodes[locator.ny] - nodes[locator.ref];
    Ogre::Vector3 cross = diff_x.crossProduct(diff_y);
    if (cross.squaredLength() < kEpsilon) return false;
    cross.normalise();

    Ogre::Matrix3 basis;
    basis.SetColumn(0, diff_x);
    basis.SetColumn(1, diff_y);
    basis.SetColumn(2, cross);
    const Ogre::Matrix3 inverse = basis.Inverse();
    locator.coords = inverse * (vertex - nodes[locator.ref]);
    return true;
}

bool IOSFlexBody::Build(const std::vector<Ogre::Vector3>& initial_nodes,
                        const std::string& unique_suffix)
{
    if (!m_scene)
    {
        m_error = "missing OGRE scene manager";
        return false;
    }
    if (m_definition.node_indices.size() < 3)
    {
        m_error = "forset contains fewer than three nodes";
        return false;
    }
    if (m_definition.has_forvert_directives)
    {
        m_error = "forvert overrides are not implemented by the iOS bridge";
        return false;
    }

    Ogre::Vector3 attachment_position;
    Ogre::Quaternion attachment_orientation;
    if (!AttachmentFrame(initial_nodes, attachment_position, attachment_orientation))
    {
        m_error = "invalid authored flexbody attachment frame";
        return false;
    }

    try
    {
        Ogre::MeshPtr source = Ogre::MeshManager::getSingleton().load(
            m_definition.mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        if (!source)
        {
            m_error = "mesh resource did not load";
            return false;
        }
        const std::string mesh_name = "RoRIOSFlexMesh_" + unique_suffix;
        m_mesh = source->clone(mesh_name);

        bool has_uv = true;
        bool has_normal = true;
        auto inspect_vertex_data = [&](Ogre::VertexData* data)
        {
            if (!data) return;
            has_uv = has_uv && data->vertexDeclaration->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES);
            has_normal = has_normal && data->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
        };
        if (m_mesh->sharedVertexData) inspect_vertex_data(m_mesh->sharedVertexData);
        for (unsigned short i = 0; i < m_mesh->getNumSubMeshes(); ++i)
        {
            Ogre::SubMesh* submesh = m_mesh->getSubMesh(i);
            if (!submesh->useSharedVertices) inspect_vertex_data(submesh->vertexData);
        }
        if (!has_normal || !has_uv)
        {
            m_error = !has_normal ? "flexbody mesh has no normals" : "flexbody mesh has no texture coordinates";
            return false;
        }

        // Desktop RoR's FlexBody asks OGRE to reorganise position, normal and UV
        // into separate dynamic streams. OGRE 14 keeps the explicit usage-list
        // overload private, so use its public reorganisation API first and then
        // promote the two streams we actually rewrite every frame to CPU->GPU
        // buffers. UV remains static.
        Ogre::VertexDeclaration* optimal =
            Ogre::HardwareBufferManager::getSingleton().createVertexDeclaration();
        optimal->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
        optimal->addElement(1, 0, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
        optimal->addElement(2, 0, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);
        optimal->sort();
        optimal->closeGapsInSource();

        auto promote_dynamic_stream = [&](Ogre::VertexData* data,
                                          Ogre::VertexElementSemantic semantic)
        {
            const Ogre::VertexElement* element =
                data->vertexDeclaration->findElementBySemantic(semantic);
            if (!element) return;

            const unsigned short source_index = element->getSource();
            Ogre::HardwareVertexBufferSharedPtr original =
                data->vertexBufferBinding->getBuffer(source_index);
            Ogre::HardwareVertexBufferSharedPtr replacement =
                Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                    original->getVertexSize(), original->getNumVertices(),
                    Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE,
                    original->hasShadowBuffer());
            replacement->copyData(*original, 0, 0, original->getSizeInBytes(), true);
            data->vertexBufferBinding->setBinding(source_index, replacement);
        };

        auto reorganise_flex_vertex_data = [&](Ogre::VertexData* data,
                                                Ogre::VertexDeclaration* declaration)
        {
            if (!data) return;
            data->reorganiseBuffers(declaration);
            data->removeUnusedBuffers();
            data->closeGapsInBindings();
            promote_dynamic_stream(data, Ogre::VES_POSITION);
            promote_dynamic_stream(data, Ogre::VES_NORMAL);
        };

        if (m_mesh->sharedVertexData)
            reorganise_flex_vertex_data(m_mesh->sharedVertexData, optimal);
        for (unsigned short i = 0; i < m_mesh->getNumSubMeshes(); ++i)
        {
            Ogre::SubMesh* submesh = m_mesh->getSubMesh(i);
            if (!submesh->useSharedVertices)
                reorganise_flex_vertex_data(submesh->vertexData, optimal->clone());
        }

        std::size_t vertex_count = m_mesh->sharedVertexData ? m_mesh->sharedVertexData->vertexCount : 0;
        for (unsigned short i = 0; i < m_mesh->getNumSubMeshes(); ++i)
        {
            Ogre::SubMesh* submesh = m_mesh->getSubMesh(i);
            if (!submesh->useSharedVertices && submesh->vertexData)
                vertex_count += submesh->vertexData->vertexCount;
        }
        if (vertex_count == 0)
        {
            m_error = "flexbody mesh has no vertices";
            return false;
        }

        std::vector<Ogre::Vector3> source_positions(vertex_count);
        std::vector<Ogre::Vector3> source_normals(vertex_count);
        m_positions.resize(vertex_count);
        m_normals.resize(vertex_count);
        m_locators.resize(vertex_count);

        std::size_t cursor = 0;
        auto capture = [&](Ogre::VertexData* data)
        {
            if (!data || data->vertexCount == 0) return;
            const Ogre::VertexElement* position_element =
                data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
            const Ogre::VertexElement* normal_element =
                data->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
            BufferSpan span;
            span.count = data->vertexCount;
            span.first_vertex = cursor;
            span.positions = data->vertexBufferBinding->getBuffer(position_element->getSource());
            span.normals = data->vertexBufferBinding->getBuffer(normal_element->getSource());
            span.positions->readData(0, span.count * sizeof(Ogre::Vector3), source_positions.data() + cursor);
            span.normals->readData(0, span.count * sizeof(Ogre::Vector3), source_normals.data() + cursor);
            m_spans.push_back(span);
            cursor += span.count;
        };
        if (m_mesh->sharedVertexData) capture(m_mesh->sharedVertexData);
        for (unsigned short i = 0; i < m_mesh->getNumSubMeshes(); ++i)
        {
            Ogre::SubMesh* submesh = m_mesh->getSubMesh(i);
            if (!submesh->useSharedVertices) capture(submesh->vertexData);
        }
        if (cursor != vertex_count)
        {
            m_error = "flexbody vertex-buffer accounting mismatch";
            return false;
        }

        for (std::size_t i = 0; i < vertex_count; ++i)
        {
            const Ogre::Vector3 world_vertex = attachment_orientation * source_positions[i] + attachment_position;
            if (!MakeLocator(world_vertex, initial_nodes, m_locators[i]))
            {
                m_error = "unable to build RoR locator for vertex " + std::to_string(i);
                return false;
            }

            const Locator& locator = m_locators[i];
            const Ogre::Vector3 diff_x = initial_nodes[locator.nx] - initial_nodes[locator.ref];
            const Ogre::Vector3 diff_y = initial_nodes[locator.ny] - initial_nodes[locator.ref];
            Ogre::Vector3 cross = diff_x.crossProduct(diff_y).normalisedCopy();
            Ogre::Matrix3 basis;
            basis.SetColumn(0, diff_x);
            basis.SetColumn(1, diff_y);
            basis.SetColumn(2, cross);
            m_locators[i].normal_coords = basis.Inverse() * (attachment_orientation * source_normals[i]);
        }

        Ogre::AxisAlignedBox bounds = m_mesh->getBounds();
        const Ogre::Vector3 extent = bounds.getHalfSize();
        const float radius = std::max(5.0f, 4.0f * std::max(extent.x, std::max(extent.y, extent.z)));
        m_mesh->_setBounds(Ogre::AxisAlignedBox(-radius, -radius, -radius, radius, radius, radius), true);

        m_entity = m_scene->createEntity("RoRIOSFlexEntity_" + unique_suffix, m_mesh->getName());
        m_entity->setCastShadows(false);
        m_scene_node = m_scene->getRootSceneNode()->createChildSceneNode("RoRIOSFlexNode_" + unique_suffix);
        m_scene_node->attachObject(m_entity);
        Update(initial_nodes, Ogre::Vector3::ZERO);
        return true;
    }
    catch (const Ogre::Exception& e)
    {
        m_error = e.getFullDescription();
        return false;
    }
}

void IOSFlexBody::Update(const std::vector<Ogre::Vector3>& nodes,
                         const Ogre::Vector3& world_offset)
{
    if (!m_ready && !m_scene_node) return;

    Ogre::Vector3 center;
    Ogre::Quaternion ignored_orientation;
    if (!AttachmentFrame(nodes, center, ignored_orientation)) return;

    for (std::size_t i = 0; i < m_locators.size(); ++i)
    {
        const Locator& locator = m_locators[i];
        if (!ValidNode(locator.ref, nodes) || !ValidNode(locator.nx, nodes) || !ValidNode(locator.ny, nodes))
            return;
        const Ogre::Vector3 diff_x = nodes[locator.nx] - nodes[locator.ref];
        const Ogre::Vector3 diff_y = nodes[locator.ny] - nodes[locator.ref];
        Ogre::Vector3 cross = diff_x.crossProduct(diff_y);
        if (cross.squaredLength() < kEpsilon) return;
        cross.normalise();

        Ogre::Vector3 position = diff_x * locator.coords.x + diff_y * locator.coords.y + cross * locator.coords.z;
        m_positions[i] = position + nodes[locator.ref] - center;

        Ogre::Vector3 normal = diff_x * locator.normal_coords.x + diff_y * locator.normal_coords.y +
                               cross * locator.normal_coords.z;
        if (!normal.isZeroLength()) normal.normalise();
        m_normals[i] = normal;
    }

    for (const BufferSpan& span : m_spans)
    {
        span.positions->writeData(0, span.count * sizeof(Ogre::Vector3),
                                  m_positions.data() + span.first_vertex, true);
        span.normals->writeData(0, span.count * sizeof(Ogre::Vector3),
                                m_normals.data() + span.first_vertex, true);
    }
    if (m_scene_node) m_scene_node->setPosition(center + world_offset);
}

} // namespace IOSOgre
} // namespace RoR