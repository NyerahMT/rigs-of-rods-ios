#include "AuthoredVisualGeometry.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

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

    std::cout << "Authored OGRE visual fixture passed: "
              << visual.cab_triangles.size() << " cab triangles, "
              << textured << " UV-mapped, "
              << visual.wheels.size() << " wheels, "
              << visual.props.size() << " props.\n";
    return EXIT_SUCCESS;
}
