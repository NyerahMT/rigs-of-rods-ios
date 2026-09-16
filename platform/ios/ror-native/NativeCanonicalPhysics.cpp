#include "NativeCanonicalPhysics.h"

#include "NativeRigDefBridge.h"
#include "RigDef_Parser.h"

#include <sstream>

namespace RoR {
namespace IOSNative {

std::string CanonicalPhysicsRigDefExact(const std::string& truck_text)
{
    std::string canonical = CanonicalPhysicsRigDef(truck_text);
    if (canonical.empty() || truck_text.empty())
        return canonical;

    RigDef::Parser parser;
    parser.Prepare();
    std::istringstream input(truck_text);
    std::string line;
    while (std::getline(input, line))
        parser.ProcessRawLine(line.c_str());
    parser.Finalize();
    RigDef::DocumentPtr doc = parser.GetFile();
    if (!doc || !doc->root_module || doc->root_module->minimass.empty())
        return canonical;

    const RigDef::Minimass& mm = doc->root_module->minimass.back();
    if (mm.option != RigDef::MinimassOption::l_SKIP_LOADED)
        return canonical;

    const std::size_t section = canonical.find("\nminimass\n");
    if (section == std::string::npos)
        return canonical;
    const std::size_t value_begin = section + sizeof("\nminimass\n") - 1;
    const std::size_t value_end = canonical.find('\n', value_begin);
    if (value_end == std::string::npos)
        return canonical;

    // PortableRigDef now consumes the optional second minimass argument using
    // the same serialized letter as upstream MinimassOption::l_SKIP_LOADED.
    canonical.insert(value_end, ", l");
    return canonical;
}

} // namespace IOSNative
} // namespace RoR
