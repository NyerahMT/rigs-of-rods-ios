#include "NativeRigDefBridge.h"

#include "RigDef_Parser.h"

#include <sstream>

namespace RoR {
namespace IOSNative {

RigDefSummary ParseRigDef(const std::string& truck_text)
{
    RigDefSummary out;
    if (truck_text.empty()) return out;

    RigDef::Parser parser;
    parser.Prepare();

    std::istringstream input(truck_text);
    std::string line;
    while (std::getline(input, line))
        parser.ProcessRawLine(line.c_str());

    parser.Finalize();
    RigDef::DocumentPtr doc = parser.GetFile();
    if (!doc || !doc->root_module) return out;

    const std::shared_ptr<RigDef::Document::Module>& m = doc->root_module;
    out.ready = true;
    out.name = doc->name;
    out.nodes = m->nodes.size();
    out.beams = m->beams.size();
    out.wheels = m->wheels.size() + m->wheels2.size() + m->meshwheels.size() +
                 m->meshwheels2.size() + m->flexbodywheels.size();
    out.engines = m->engine.size();
    out.axles = m->axles.size() + m->interaxles.size();
    out.shocks = m->shocks.size() + m->shocks2.size() + m->shocks3.size();
    out.commands = m->commands2.size();
    out.props = m->props.size();
    out.flexbodies = m->flexbodies.size();
    out.wings = m->wings.size();
    return out;
}

} // namespace IOSNative
} // namespace RoR
