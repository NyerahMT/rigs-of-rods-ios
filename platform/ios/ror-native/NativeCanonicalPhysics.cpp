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

    const std::string shock3 = CanonicalShock3Section(truck_text);
    if (!shock3.empty())
    {
        const std::size_t end_marker = canonical.rfind("end\n");
        if (end_marker != std::string::npos)
            canonical.insert(end_marker, shock3);
    }

    const std::string full_engoption = CanonicalEngOptionSection(truck_text);
    if (!full_engoption.empty())
    {
        const std::size_t section = canonical.find("engoption\n");
        if (section != std::string::npos)
        {
            const std::size_t value_begin = section + sizeof("engoption\n") - 1;
            const std::size_t value_end = canonical.find('\n', value_begin);
            if (value_end != std::string::npos)
                canonical.replace(section, value_end + 1 - section, full_engoption);
        }
        else
        {
            const std::size_t end_marker = canonical.rfind("end\n");
            if (end_marker != std::string::npos)
                canonical.insert(end_marker, full_engoption);
        }
    }

    // The legacy canonical serializer deliberately omitted torquecurve. Preserve
    // the exact upstream section now that the portable drivetrain implements the
    // same SimpleSpline custom-curve semantics. Named predefined models are kept
    // intact too so they remain auditable until torque_models.cfg is packaged.
    const std::string torque_curve = CanonicalTorqueCurveSection(truck_text);
    if (!torque_curve.empty())
    {
        const std::size_t end_marker = canonical.rfind("end\n");
        if (end_marker != std::string::npos)
            canonical.insert(end_marker, torque_curve);
    }

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

    canonical.insert(value_end, ", l");
    return canonical;
}

} // namespace IOSNative
} // namespace RoR
