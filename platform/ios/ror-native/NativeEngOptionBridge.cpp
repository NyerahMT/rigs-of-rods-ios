#include "NativeRigDefBridge.h"

#include "RigDef_Parser.h"

#include <iomanip>
#include <sstream>

namespace RoR {
namespace IOSNative {
namespace {

RigDef::DocumentPtr ParseEngOptionDocument(const std::string& truck_text)
{
    if (truck_text.empty()) return RigDef::DocumentPtr();
    RigDef::Parser parser;
    parser.Prepare();
    std::istringstream input(truck_text);
    std::string line;
    while (std::getline(input, line))
        parser.ProcessRawLine(line.c_str());
    parser.Finalize();
    return parser.GetFile();
}

} // namespace

std::string CanonicalEngOptionSection(const std::string& truck_text)
{
    RigDef::DocumentPtr doc = ParseEngOptionDocument(truck_text);
    if (!doc || !doc->root_module || doc->root_module->engoption.empty())
        return std::string();

    const RigDef::Engoption& e = doc->root_module->engoption.back();
    std::ostringstream out;
    out << std::setprecision(9);
    out << "engoption\n"
        << e.inertia << ", " << static_cast<char>(e.type) << ", "
        << e.clutch_force << ", " << e.shift_time << ", "
        << e.clutch_time << ", " << e.post_shift_time << ", "
        << e.stall_rpm << ", " << e.idle_rpm << ", "
        << e.max_idle_mixture << ", " << e.min_idle_mixture << ", "
        << e.braking_torque << '\n';
    return out.str();
}

} // namespace IOSNative
} // namespace RoR
