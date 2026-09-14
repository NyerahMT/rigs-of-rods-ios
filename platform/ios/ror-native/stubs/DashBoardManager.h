#pragma once

// RigDef_Parser.cpp only needs the serialized dashboard data-type constants
// from the desktop dashboard header. Keep the UI/MyGUI dashboard manager out
// of the native iOS parser target while preserving the exact upstream values.
namespace RoR {

enum
{
    DC_MIN,

    DC_BOOL,
    DC_INT,
    DC_FLOAT,
    DC_CHAR,
    DC_INVALID,

    DC_MAX
};

} // namespace RoR
