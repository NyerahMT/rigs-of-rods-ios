#pragma once

#include "CVar.h"

#include <string>

namespace RoR {

class Console;

// Narrow host surface used by RigDef_Parser.cpp / RigDef_SequentialImporter.cpp.
// The desktop Application.h also declares the entire game/message/scripting
// environment; none of that belongs in the parser-only iOS target.
void Log(const char* msg);
void LogFormat(const char* format, ...);
inline void LOG(const char* msg) { Log(msg); }
inline void LOG(const std::string& msg) { Log(msg.c_str()); }

namespace App {
extern CVar* diag_rig_log_node_import;
extern CVar* diag_rig_log_node_stats;
Console* GetConsole();
} // namespace App

} // namespace RoR
