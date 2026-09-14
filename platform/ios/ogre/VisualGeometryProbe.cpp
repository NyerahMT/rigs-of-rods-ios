#include "AuthoredVisualGeometry.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

bool VerifyFlexbodyMetadata()
{
    // Small parser-only fixture modelled after the real RoR flexbodies/forset
    // grammar. This deliberately covers an inclusive range plus a named/single
    // node so changes cannot silently sever the renderer from RoR's authored
    // deformation node set.
    const std::string truck = R"ROR(Flexbody parser probe
nodes
0, 0.0, 0.0, 0.0
1, 1.0, 0.0, 0.0
2, 0.0, 1.0, 0.0
3, 0.0, 0.0, 1.0
4, 1.0, 1.0, 0.0
5, 1.0, 0.0, 1.0
flexbodies
0, 1, 2, 0.10, 0.20, 0.30, 4.0, 5.0, 6.0, body.mesh
forset 0-4, 5
end
)ROR";

    const auto visual = RoR::IOSOgre::ParseAuthoredVisualGeometry(truck);
    if (visual.flexbodies.size() != 1)
    {
        std::cerr << "FAIL: expected one flexbody parser fixture, got "
                  << visual.flexbodies.size() << '\n';
        return false;
    }
    const auto& flex = visual.flexbodies.front();
    if (flex.node_ref != 0 || flex.node_x != 1 || flex.node_y != 2 ||
        flex.mesh_name != "body.mesh" || flex.node_indices.size() != 6)
    {
        std::cerr << "FAIL: authored flexbody attachment/forset data was not preserved\n";
        return false;
    }
    if (flex.offset_x != 0.10f || flex.offset_y != 0.20f || flex.offset_z != 0.30f ||
        flex.rot_x_degrees != 4.0f || flex.rot_y_degrees != 5.0f || flex.rot_z_degrees != 6.0f)
    {
        std::cerr << "FAIL: authored flexbody transform was not preserved\n";
        return false;
    }
    for (std::size_t i = 0; i < flex.node_indices.size(); ++i)
    {
        if (flex.node_indices[i] != i)
        {
            std::cerr << "FAIL: flexbody forset range resolved out of order at " << i << '\n';
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: visual_geometry_probe <truck-file>\n";
        return EXIT_FAILURE;
    }

    std::ifstream file(argv[1]);
    if (!file)
    {
        std::cerr << "FAIL: unable to open authored vehicle fixture\n";
        return EXIT_FAILURE;
    }
    std::ostringstream text;
    text << file.rdbuf();

    const RoR::IOSOgre::AuthoredVisualGeometry visual =
        RoR::IOSOgre::ParseAuthoredVisualGeometry(text.str());

    if (visual.cab_triangles.size() < 50)
    {
        std::cerr << "FAIL: expected production DAF cab geometry, got "
                  << visual.cab_triangles.size() << " triangles\n";
        return EXIT_FAILURE;
    }
    if (visual.wheels.size() != 4)
    {
        std::cerr << "FAIL: expected four authored DAF wheel visuals, got "
                  << visual.wheels.size() << '\n';
        return EXIT_FAILURE;
    }

    std::size_t textured = 0;
    for (const auto& triangle : visual.cab_triangles)
        if (triangle.has_uv) ++textured;

    if (textured < 100)
    {
        std::cerr << "FAIL: expected production DAF UV coverage, got "
                  << textured << '/' << visual.cab_triangles.size() << " textured triangles\n";
        return EXIT_FAILURE;
    }

    if (visual.props.size() != 5)
    {
        std::cerr << "FAIL: expected five stock DAF props, got " << visual.props.size() << '\n';
        return EXIT_FAILURE;
    }

    auto count_mesh = [&](const std::string& name)
    {
        std::size_t count = 0;
        for (const auto& prop : visual.props) if (prop.mesh_name == name) ++count;
        return count;
    };
    if (count_mesh("dashboard.mesh") != 1 || count_mesh("leftmirror.mesh") != 1 ||
        count_mesh("rightmirror.mesh") != 1 || count_mesh("seat.mesh") != 2)
    {
        std::cerr << "FAIL: stock DAF prop mesh set was not preserved\n";
        return EXIT_FAILURE;
    }

    if (!VerifyFlexbodyMetadata()) return EXIT_FAILURE;

    std::cout << "Authored OGRE visual fixture passed: "
              << visual.cab_triangles.size() << " cab triangles, "
              << textured << " UV-mapped, "
              << visual.wheels.size() << " wheels, "
              << visual.props.size() << " props; flexbody metadata probe passed.\n";
    return EXIT_SUCCESS;
}