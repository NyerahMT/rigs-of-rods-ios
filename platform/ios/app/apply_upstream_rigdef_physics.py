#!/usr/bin/env python3
"""Route iOS simulation physics through upstream RigDef parsing.

The authored visual parser intentionally continues to receive the original truck
text. Only AuthoredVehicleRuntime receives the canonical physics definition
produced by RigDef::Parser + SequentialImporter.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_upstream_rigdef_physics.py <generated-game-source.mm>")

path = Path(sys.argv[1])
text = path.read_text()


def once(old: str, new: str, label: str) -> None:
    global text
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"upstream RigDef bridge anchor '{label}' expected once, found {count}")
    text = text.replace(old, new, 1)


once(
    '#include "AuthoredVehicleRuntime.h"\n',
    '#include "AuthoredVehicleRuntime.h"\n#include "NativeCanonicalPhysics.h"\n',
    "native RigDef include",
)

once(
    '''struct Snapshot
{
''',
    '''std::string UpstreamPhysicsDefinition(const std::string& truck_text)
{
    const std::string canonical = RoR::IOSNative::CanonicalPhysicsRigDefExact(truck_text);
    return canonical.empty() ? truck_text : canonical;
}

struct Snapshot
{
''',
    "canonical physics helper",
)

once(
    ': runtime(text), visual(RoR::IOSOgre::ParseAuthoredVisualGeometry(text))',
    ': runtime(UpstreamPhysicsDefinition(text)), visual(RoR::IOSOgre::ParseAuthoredVisualGeometry(text))',
    "simulation physics input",
)

path.write_text(text)
print('routed simulation physics through lossless upstream RigDef canonicalization; visuals retain raw authored text')
