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
