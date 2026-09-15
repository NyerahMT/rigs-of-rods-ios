/*
    This source file is part of Rigs of Rods.
*/

// Runtime source is restored from the last known-complete implementation by CI while
// loader parity work is in progress. This file must never be a placeholder: keeping a
// compilable translation unit here makes accidental source loss immediately visible.
//
// TODO(loader-parity): replace the historical restore step with the complete runtime
// after upstream-compatible node/beam mass construction lands.

#include "AuthoredVehicleRuntime.h"

namespace RoR {
namespace IOSVehicleCore {

// Deliberately no duplicate implementation here yet. The build workflow restores the
// complete historical implementation before CMake configures this target.

} // namespace IOSVehicleCore
} // namespace RoR
