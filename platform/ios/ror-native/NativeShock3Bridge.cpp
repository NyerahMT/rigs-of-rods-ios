#include "NativeRigDefBridge.h"

#include "RigDef_Parser.h"

#include <iomanip>
#include <sstream>

namespace RoR {
namespace IOSNative {
namespace {

RigDef::DocumentPtr ParseShockDocument(const std::string& truck_text)
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

std::string ShockRef(const RigDef::Node::Ref& ref)
{
    return ref.IsValidAnyState() ? ref.Str() : "-1";
}

std::string Shock3Options(unsigned int options)
{
    std::string out;
    if (options & RigDef::Shock3::OPTION_i_INVISIBLE)       out += 'i';
    if (options & RigDef::Shock3::OPTION_m_METRIC)          out += 'm';
    if (options & RigDef::Shock3::OPTION_M_ABSOLUTE_METRIC) out += 'M';
    return out;
}

void EmitShockBeamDefaults(std::ostream& out, const std::shared_ptr<RigDef::BeamDefaults>& d)
{
    if (!d) return;
    out << "set_beam_defaults_scale, "
        << d->scale.springiness << ", "
        << d->scale.damping_constant << ", "
        << d->scale.deformation_threshold_constant << ", "
        << d->scale.breaking_threshold_constant << '\n';
    out << "set_beam_defaults, "
        << d->springiness << ", "
        << d->damping_constant << ", "
        << d->deformation_threshold << ", "
        << d->breaking_threshold << '\n';
}

} // namespace

std::string CanonicalShock3Section(const std::string& truck_text)
{
    RigDef::DocumentPtr doc = ParseShockDocument(truck_text);
    if (!doc || !doc->root_module || doc->root_module->shocks3.empty())
        return std::string();

    std::ostringstream out;
    out << std::setprecision(9);
    out << "shocks3\n";
    for (const RigDef::Shock3& s : doc->root_module->shocks3)
    {
        EmitShockBeamDefaults(out, s.beam_defaults);
        out << ShockRef(s.nodes[0]) << ", " << ShockRef(s.nodes[1]) << ", "
            << s.spring_in << ", " << s.damp_in << ", "
            << s.damp_in_slow << ", " << s.split_vel_in << ", " << s.damp_in_fast << ", "
            << s.spring_out << ", " << s.damp_out << ", "
            << s.damp_out_slow << ", " << s.split_vel_out << ", " << s.damp_out_fast << ", "
            << s.short_bound << ", " << s.long_bound << ", " << s.precompression;
        const std::string options = Shock3Options(s.options);
        if (!options.empty()) out << ", " << options;
        out << '\n';
    }
    return out.str();
}

} // namespace IOSNative
} // namespace RoR
