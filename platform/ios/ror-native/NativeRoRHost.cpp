#include "Application.h"
#include "Console.h"

#include <cstdarg>
#include <cstdio>
#include <string>

namespace RoR {

// CVar's tiny value-conversion hooks normally live in the desktop Console
// implementation. The native parser only needs two disabled diagnostic CVars.
std::string CVar::convertStr(float val)
{
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(val));
    return std::string(buf);
}

void CVar::logUpdate(const std::string&)
{
    // iOS parser host has no RoR.cfg/desktop console side effects.
}

namespace {
Console g_console;
CVar g_diag_node_import("diag_rig_log_node_import", "", CVAR_TYPE_BOOL);
CVar g_diag_node_stats("diag_rig_log_node_stats", "", CVAR_TYPE_BOOL);
}

namespace App {
CVar* diag_rig_log_node_import = &g_diag_node_import;
CVar* diag_rig_log_node_stats = &g_diag_node_stats;
Console* GetConsole() { return &g_console; }
} // namespace App

void Log(const char*)
{
    // Parser LOG() calls are diagnostic only; UIKit owns runtime presentation.
}

void LogFormat(const char*, ...)
{
    // Same as Log(): intentionally side-effect free in this narrow host layer.
}

} // namespace RoR

namespace RigDef {

// Desktop RoR keeps this diagnostic helper in Application.cpp. Pulling that
// translation unit into the parser-only iOS target would also drag in the
// desktop app/UI/network/scripting globals. Parsing behavior does not depend on
// the returned text; it is used only to label parser/importer diagnostics.
const char* KeywordToString(Keyword keyword)
{
    switch (keyword)
    {
        case Keyword::INVALID: return "invalid";
        case Keyword::NODES: return "nodes";
        case Keyword::NODES2: return "nodes2";
        case Keyword::BEAMS: return "beams";
        case Keyword::WHEELS: return "wheels";
        case Keyword::WHEELS2: return "wheels2";
        case Keyword::ENGINE: return "engine";
        case Keyword::HYDROS: return "hydros";
        case Keyword::SHOCKS: return "shocks";
        case Keyword::SHOCKS2: return "shocks2";
        case Keyword::SHOCKS3: return "shocks3";
        case Keyword::COMMANDS: return "commands";
        case Keyword::COMMANDS2: return "commands2";
        case Keyword::AXLES: return "axles";
        case Keyword::INTERAXLES: return "interaxles";
        case Keyword::FLEXBODIES: return "flexbodies";
        case Keyword::PROPS: return "props";
        case Keyword::WINGS: return "wings";
        case Keyword::CAB: return "cab";
        case Keyword::SUBMESH: return "submesh";
        default: return "rigdef-keyword";
    }
}

} // namespace RigDef
