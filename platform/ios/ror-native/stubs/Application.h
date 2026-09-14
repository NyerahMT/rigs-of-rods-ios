#pragma once

// Use RoR's real Application.h so RigDef sees the exact serialized enums,
// IDs, aliases, Keyword enum, and constants used by desktop RoR. The iOS
// parser target shadows only RefCountingObjectPtr.h to avoid pulling the
// AngelScript registration layer into this narrow parser build.
#include_next "Application.h"
