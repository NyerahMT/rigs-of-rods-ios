#pragma once

#include <string>

namespace RoR {
namespace IOSNative {

// Produces the portable-core definition from the upstream RigDef document while
// preserving physical options that the transitional canonical serializer cannot
// yet express directly. This wrapper shrinks as the native actor replaces the
// portable actor subsystem-by-subsystem.
std::string CanonicalPhysicsRigDefExact(const std::string& truck_text);

} // namespace IOSNative
} // namespace RoR
