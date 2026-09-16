#include "NativeRigDefBridge.h"

#include "RigDef_Parser.h"

#include <iomanip>
#include <sstream>

namespace RoR {
namespace IOSNative {
namespace {

RigDef::DocumentPtr ParseTorqueCurveDocument(const std::string& truck_text)
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

std::string CanonicalTorqueCurveSection(const std::string& truck_text)
{
    RigDef::DocumentPtr doc = ParseTorqueCurveDocument(truck_text);
    if (!doc || !doc->root_module || doc->root_module->torquecurve.empty())
        return std::string();

    const RigDef::TorqueCurve& curve = doc->root_module->torquecurve.front();
    std::ostringstream out;
    out << std::setprecision(9);
    out << "torquecurve\n";
    if (!curve.predefined_func_name.empty())
    {
        out << curve.predefined_func_name << '\n';
    }
    else
    {
        for (const RigDef::TorqueCurve::Sample& sample : curve.samples)
            out << sample.power << ", " << sample.torque_percent << '\n';
    }
    return out.str();
}

} // namespace IOSNative
} // namespace RoR
