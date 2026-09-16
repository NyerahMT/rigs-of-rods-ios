#include "NativeRigDefBridge.h"

#include "RigDef_Parser.h"

#include <iomanip>
#include <sstream>

namespace RoR {
namespace IOSNative {
namespace {

RigDef::DocumentPtr ParseDocument(const std::string& truck_text)
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

std::string Ref(const RigDef::Node::Ref& ref)
{
    return ref.IsValidAnyState() ? ref.Str() : "-1";
}

std::string NodeOptions(unsigned int options)
{
    std::string s;
    if (options & RigDef::Node::OPTION_m_NO_MOUSE_GRAB)      s += 'm';
    if (options & RigDef::Node::OPTION_f_NO_SPARKS)          s += 'f';
    if (options & RigDef::Node::OPTION_x_EXHAUST_POINT)      s += 'x';
    if (options & RigDef::Node::OPTION_y_EXHAUST_DIRECTION)  s += 'y';
    if (options & RigDef::Node::OPTION_c_NO_GROUND_CONTACT)  s += 'c';
    if (options & RigDef::Node::OPTION_h_HOOK_POINT)         s += 'h';
    if (options & RigDef::Node::OPTION_e_TERRAIN_EDIT_POINT) s += 'e';
    if (options & RigDef::Node::OPTION_b_EXTRA_BUOYANCY)     s += 'b';
    if (options & RigDef::Node::OPTION_p_NO_PARTICLES)       s += 'p';
    if (options & RigDef::Node::OPTION_L_LOG)                s += 'L';
    if (options & RigDef::Node::OPTION_l_LOAD_WEIGHT)        s += 'l';
    return s;
}

std::string BeamOptions(unsigned int options)
{
    std::string s;
    if (options & RigDef::Beam::OPTION_i_INVISIBLE) s += 'i';
    if (options & RigDef::Beam::OPTION_r_ROPE)      s += 'r';
    if (options & RigDef::Beam::OPTION_s_SUPPORT)   s += 's';
    return s;
}

std::string HydroOptions(unsigned int options)
{
    std::string s;
    if (options & RigDef::Hydro::OPTION_j_INVISIBLE)                 s += 'j';
    if (options & RigDef::Hydro::OPTION_s_DISABLE_ON_HIGH_SPEED)     s += 's';
    if (options & RigDef::Hydro::OPTION_a_INPUT_AILERON)             s += 'a';
    if (options & RigDef::Hydro::OPTION_r_INPUT_RUDDER)              s += 'r';
    if (options & RigDef::Hydro::OPTION_e_INPUT_ELEVATOR)            s += 'e';
    if (options & RigDef::Hydro::OPTION_u_INPUT_AILERON_ELEVATOR)    s += 'u';
    if (options & RigDef::Hydro::OPTION_v_INPUT_InvAILERON_ELEVATOR) s += 'v';
    if (options & RigDef::Hydro::OPTION_x_INPUT_AILERON_RUDDER)      s += 'x';
    if (options & RigDef::Hydro::OPTION_y_INPUT_InvAILERON_RUDDER)   s += 'y';
    if (options & RigDef::Hydro::OPTION_g_INPUT_ELEVATOR_RUDDER)     s += 'g';
    if (options & RigDef::Hydro::OPTION_h_INPUT_InvELEVATOR_RUDDER)  s += 'h';
    if (options & RigDef::Hydro::OPTION_n_INPUT_NORMAL)              s += 'n';
    return s;
}

std::string ShockOptions(unsigned int options)
{
    std::string s;
    if (options & RigDef::Shock::OPTION_i_INVISIBLE)    s += 'i';
    if (options & RigDef::Shock::OPTION_L_ACTIVE_LEFT)  s += 'L';
    if (options & RigDef::Shock::OPTION_R_ACTIVE_RIGHT) s += 'R';
    if (options & RigDef::Shock::OPTION_m_METRIC)       s += 'm';
    return s;
}

std::string Shock2Options(unsigned int options)
{
    std::string s;
    if (options & RigDef::Shock2::OPTION_i_INVISIBLE)         s += 'i';
    if (options & RigDef::Shock2::OPTION_s_SOFT_BUMP_BOUNDS)  s += 's';
    if (options & RigDef::Shock2::OPTION_m_METRIC)            s += 'm';
    if (options & RigDef::Shock2::OPTION_M_ABSOLUTE_METRIC)   s += 'M';
    return s;
}

void EmitBeamDefaults(std::ostream& out, const std::shared_ptr<RigDef::BeamDefaults>& d)
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

void EmitNodeDefaults(std::ostream& out, const std::shared_ptr<RigDef::NodeDefaults>& d)
{
    if (!d) return;
    // Per-element options are emitted separately; defaults here carry the four
    // physical scalars so generated wheel nodes inherit the exact friction.
    out << "set_node_defaults, " << d->load_weight << ", " << d->friction
        << ", " << d->volume << ", " << d->surface << '\n';
}

char WheelSideChar(RoR::WheelSide side)
{
    const int raw = static_cast<int>(side);
    if (raw == static_cast<int>('l') || raw == static_cast<int>('L')) return 'l';
    return 'r';
}

template <typename W>
void EmitBaseWheelPrefix(std::ostream& out, const W& w)
{
    out << w.width << ", " << w.num_rays << ", "
        << Ref(w.nodes[0]) << ", " << Ref(w.nodes[1]) << ", "
        << Ref(w.rigidity_node) << ", "
        << static_cast<int>(w.braking) << ", "
        << static_cast<int>(w.propulsion) << ", "
        << Ref(w.reference_arm_node) << ", " << w.mass;
}

} // namespace

RigDefSummary ParseRigDef(const std::string& truck_text)
{
    RigDefSummary out;
    RigDef::DocumentPtr doc = ParseDocument(truck_text);
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

std::string CanonicalPhysicsRigDef(const std::string& truck_text)
{
    RigDef::DocumentPtr doc = ParseDocument(truck_text);
    if (!doc || !doc->root_module) return std::string();
    const auto& m = doc->root_module;

    std::ostringstream out;
    out << std::setprecision(9);
    out << (doc->name.empty() ? std::string("RoR vehicle") : doc->name) << '\n';

    if (!m->globals.empty())
    {
        const RigDef::Globals& g = m->globals.back();
        out << "globals\n" << g.dry_mass << ", " << g.cargo_mass << ", " << g.material_name << '\n';
    }
    if (!m->minimass.empty())
        out << "minimass\n" << m->minimass.back().global_min_mass_Kg << '\n';

    if (!m->nodes.empty())
    {
        out << "nodes\n";
        for (const RigDef::Node& n : m->nodes)
        {
            EmitNodeDefaults(out, n.node_defaults);
            if (n.default_minimass)
                out << "set_default_minimass, " << n.default_minimass->min_mass_Kg << '\n';
            std::string opts = NodeOptions(n.options);
            out << n.id.Str() << ", " << n.position.x << ", " << n.position.y << ", " << n.position.z;
            if (!opts.empty() || n._has_load_weight_override)
                out << ", " << (opts.empty() ? std::string("n") : opts);
            if (n._has_load_weight_override)
                out << ", " << n.load_weight_override;
            out << '\n';
        }
    }

    if (!m->beams.empty())
    {
        out << "beams\n";
        for (const RigDef::Beam& b : m->beams)
        {
            EmitBeamDefaults(out, b.defaults);
            const std::string opts = BeamOptions(b.options);
            out << Ref(b.nodes[0]) << ", " << Ref(b.nodes[1]);
            if (!opts.empty()) out << ", " << opts;
            out << '\n';
        }
    }

    if (!m->hydros.empty())
    {
        out << "hydros\n";
        for (const RigDef::Hydro& h : m->hydros)
        {
            EmitBeamDefaults(out, h.beam_defaults);
            out << Ref(h.nodes[0]) << ", " << Ref(h.nodes[1]) << ", " << h.lenghtening_factor;
            const std::string opts = HydroOptions(h.options);
            if (!opts.empty()) out << ", " << opts;
            out << '\n';
        }
    }

    if (!m->shocks.empty())
    {
        out << "shocks\n";
        for (const RigDef::Shock& s : m->shocks)
        {
            EmitBeamDefaults(out, s.beam_defaults);
            out << Ref(s.nodes[0]) << ", " << Ref(s.nodes[1]) << ", "
                << s.spring_rate << ", " << s.damping << ", "
                << s.short_bound << ", " << s.long_bound << ", " << s.precompression;
            const std::string opts = ShockOptions(s.options);
            if (!opts.empty()) out << ", " << opts;
            out << '\n';
        }
    }

    if (!m->shocks2.empty())
    {
        out << "shocks2\n";
        for (const RigDef::Shock2& s : m->shocks2)
        {
            EmitBeamDefaults(out, s.beam_defaults);
            out << Ref(s.nodes[0]) << ", " << Ref(s.nodes[1]) << ", "
                << s.spring_in << ", " << s.damp_in << ", "
                << s.progress_factor_spring_in << ", " << s.progress_factor_damp_in << ", "
                << s.spring_out << ", " << s.damp_out << ", "
                << s.progress_factor_spring_out << ", " << s.progress_factor_damp_out << ", "
                << s.short_bound << ", " << s.long_bound << ", " << s.precompression;
            const std::string opts = Shock2Options(s.options);
            if (!opts.empty()) out << ", " << opts;
            out << '\n';
        }
    }

    if (!m->wheels.empty())
    {
        out << "wheels\n";
        for (const RigDef::Wheel& w : m->wheels)
        {
            EmitNodeDefaults(out, w.node_defaults);
            EmitBeamDefaults(out, w.beam_defaults);
            out << w.radius << ", ";
            EmitBaseWheelPrefix(out, w);
            out << ", " << w.springiness << ", " << w.damping << '\n';
        }
    }

    if (!m->wheels2.empty())
    {
        out << "wheels2\n";
        for (const RigDef::Wheel2& w : m->wheels2)
        {
            EmitNodeDefaults(out, w.node_defaults);
            EmitBeamDefaults(out, w.beam_defaults);
            out << w.rim_radius << ", " << w.tyre_radius << ", ";
            EmitBaseWheelPrefix(out, w);
            out << ", " << w.rim_springiness << ", " << w.rim_damping
                << ", " << w.tyre_springiness << ", " << w.tyre_damping << '\n';
        }
    }

    auto emit_meshwheels = [&out](const char* keyword, const auto& wheels)
    {
        if (wheels.empty()) return;
        out << keyword << '\n';
        for (const auto& w : wheels)
        {
            EmitNodeDefaults(out, w.node_defaults);
            EmitBeamDefaults(out, w.beam_defaults);
            out << w.tyre_radius << ", " << w.rim_radius << ", ";
            EmitBaseWheelPrefix(out, w);
            out << ", " << w.spring << ", " << w.damping << ", "
                << WheelSideChar(w.side) << ", " << w.mesh_name << ", " << w.material_name << '\n';
        }
    };
    emit_meshwheels("meshwheels", m->meshwheels);
    emit_meshwheels("meshwheels2", m->meshwheels2);

    if (!m->engine.empty())
    {
        const RigDef::Engine& e = m->engine.back();
        out << "engine\n" << e.shift_down_rpm << ", " << e.shift_up_rpm << ", "
            << e.torque << ", " << e.global_gear_ratio << ", "
            << e.reverse_gear_ratio << ", " << e.neutral_gear_ratio;
        for (float ratio : e.gear_ratios) out << ", " << ratio;
        out << '\n';
    }

    if (!m->engoption.empty())
    {
        const RigDef::Engoption& e = m->engoption.back();
        out << "engoption\n" << e.inertia << ", " << static_cast<char>(e.type)
            << ", " << e.clutch_force << ", " << e.shift_time
            << ", " << e.clutch_time << ", " << e.post_shift_time << '\n';
    }

    if (!m->brakes.empty())
    {
        const RigDef::Brakes& b = m->brakes.back();
        out << "brakes\n" << b.default_braking_force << ", " << b.parking_brake_force << '\n';
    }

    if (!m->contacters.empty())
    {
        out << "contacters\n";
        for (const RigDef::Node::Ref& ref : m->contacters) out << Ref(ref) << '\n';
    }

    // Deliberately leave unsupported physics sections intact as explicit audit
    // markers rather than silently pretending they were ported.
    if (!m->axles.empty())       out << "; upstream-audit: axles=" << m->axles.size() << '\n';
    if (!m->interaxles.empty())  out << "; upstream-audit: interaxles=" << m->interaxles.size() << '\n';
    if (!m->shocks3.empty())     out << "; upstream-audit: shocks3=" << m->shocks3.size() << '\n';
    if (!m->torquecurve.empty()) out << "; upstream-audit: torquecurve=" << m->torquecurve.size() << '\n';
    if (!m->transfercase.empty())out << "; upstream-audit: transfercase=" << m->transfercase.size() << '\n';
    if (!m->wings.empty())       out << "; upstream-audit: wings=" << m->wings.size() << '\n';
    if (!m->fusedrag.empty())    out << "; upstream-audit: fusedrag=" << m->fusedrag.size() << '\n';
    out << "end\n";
    return out.str();
}

} // namespace IOSNative
} // namespace RoR
